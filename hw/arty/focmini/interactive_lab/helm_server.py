#!/usr/bin/env python3
"""Interactive EE Control Systems & RL Digital-Twin Workbench Server for Arty A7-35T FOCMini."""
import argparse, glob, json, math, os, pathlib, re, select, shutil, subprocess, sys, termios, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]


def discover_serial_port():
    env_port = os.environ.get("ARTY_PORT")
    if env_port and os.path.exists(env_port):
        return env_port
    candidates = (
        sorted(glob.glob("/dev/cu.usbserial-210319*1"))
        + sorted(glob.glob("/dev/ttyUSB*"))
        + sorted(glob.glob("/dev/serial/by-id/*Digilent*if01*"))
        + sorted(glob.glob("/dev/cu.usbserial-*1"))
    )
    if candidates:
        return candidates[0]
    return "/dev/cu.usbserial-210319A43D3E1"


HTTP_PORT = int(os.environ.get("HELM_HTTP_PORT", "8765"))

FIELDS = 'a v c i e s r b f m q n t p g x d z o h k y'.split()
PATTERN = re.compile(
    rb'C3 a=([0-9A-F]{3}) v=([01]) c=([01]) i=([01]) e=([0-9A-F]) s=([0-9A-F]{2}) '
    rb'r=([0-9A-F]{3}) b=([0-3]) f=([01]) m=([0-6]) q=([0-9A-F]) n=([0-9A-F]{4}) '
    rb't=([0-9A-F]{4}) p=([0-9A-F]{2}) g=([01]) x=([0-9A-F]{4}) d=([0-9A-F]{3}) '
    rb'z=([0-9A-F]{3}) o=([0-9A-F]{4}) h=([0-5]) k=([01]) y=([0-9A-F])'
    rb'(?: iq=([0-9A-F]{4}) vd=([0-9A-F]{4}) vq=([0-9A-F]{4}))?\r\n'
)


class MotorBridge:
    """Thread-safe UART bridge that forwards UI references and reads 40 Hz FPGA telemetry."""

    def __init__(self, port):
        self.port = port
        self.fd = None
        self.lock = threading.Lock()
        self.state = {
            "connected": False, "m": 0, "q": 0, "g": 0, "k": 0, "f": 1,
            "r": 0, "d": 0, "z": 0, "o": 0, "a": 0, "t": 0, "n": 0,
            "origin": None, "angle_deg": 0.0, "target_deg": 0.0, "error_deg": 0.0,
            "slew": 24, "auto_Wave": False, "rl_mode": 1, "h": 4,
            "fpga_rl": False, "rl_iq": 0.0, "rl_vd": 0.0, "rl_vq": 0.0
        }
        self.pending_target = None
        self.pending_slew = None
        self.want_active = False
        self.clear_fault = False
        self.fault_logged = False
        self.last_rx = 0.0
        self.running = True
        self._open_fd()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _open_fd(self):
        port = discover_serial_port() or self.port
        try:
            fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            t = termios.tcgetattr(fd)
            t[0] = 0; t[1] = 0
            t[2] = (t[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL)) | termios.CS8 | termios.CLOCAL | termios.CREAD
            t[3] = 0; t[4] = termios.B115200; t[5] = termios.B115200
            termios.tcsetattr(fd, termios.TCSANOW, t)
            termios.tcflush(fd, termios.TCIFLUSH)
            self.port = port
            self.fd = fd
            self.last_rx = time.monotonic()
            print(f"[helm_server] serial opened {port}", flush=True)
            return True
        except OSError as exc:
            self.fd = None
            self.state["connected"] = False
            print(f"[helm_server] serial open failed ({port}): {exc}", flush=True)
            return False

    def _close_fd(self):
        fd = self.fd
        self.fd = None
        self.state["connected"] = False
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _write_bytes(self, payload):
        fd = self.fd
        if fd is None:
            return False
        view = memoryview(payload)
        sent = 0
        for _ in range(5):
            try:
                n = os.write(fd, view[sent:])
                sent += n
                if sent >= len(payload):
                    return True
            except BlockingIOError:
                time.sleep(0.002)
            except OSError as exc:
                print(f"[helm_server] serial write error: {exc}", flush=True)
                self._close_fd()
                return False
        return False

    def send_raw(self, cmd):
        with self.lock:
            self._write_bytes((cmd + "\n").encode("ascii"))

    def set_helm(self, rel_deg, slew=None):
        with self.lock:
            origin = self.state.get("origin")
            if origin is None:
                origin = self.state.get("r", 0)
            counts = int(round(origin + (rel_deg * 4096.0 / 360.0))) & 0xFFF
            self.pending_target = counts
            if slew is not None:
                self.pending_slew = max(1, min(32, int(round(slew))))

    def arm_and_start(self):
        with self.lock:
            self.want_active = True
            self.clear_fault = True
            self.fault_logged = False
            self.pending_target = None
            self.state["origin"] = None

    def set_ctrl_mode(self, rl_mode):
        self.send_raw("M1" if rl_mode else "M0")

    def stop(self):
        with self.lock:
            self.want_active = False
            self.pending_target = None
            self._write_bytes(b"!\n")

    def _loop(self):
        buf = bytearray()
        last_ping = 0.0
        last_cmd = 0.0
        last_state_cmd = 0.0
        while self.running:
            try:
                if self.fd is None:
                    buf.clear()
                    time.sleep(0.25)
                    with self.lock:
                        self._open_fd()
                    continue
                try:
                    ready, _, _ = select.select([self.fd], [], [], 0.02)
                except (OSError, ValueError) as exc:
                    print(f"[helm_server] select error: {exc}", flush=True)
                    with self.lock:
                        self._close_fd()
                    continue
                now = time.monotonic()
                if ready:
                    try:
                        data = os.read(self.fd, 4096)
                    except OSError as exc:
                        print(f"[helm_server] serial read error: {exc}", flush=True)
                        with self.lock:
                            self._close_fd()
                        continue
                    if not data:
                        print("[helm_server] serial EOF; reopening", flush=True)
                        with self.lock:
                            self._close_fd()
                        continue
                    self.last_rx = now
                    buf.extend(data)
                    if len(buf) > 16384:
                        buf = buf[-4096:]
                while b"\n" in buf:
                    line, _, rem = buf.partition(b"\n")
                    buf = bytearray(rem)
                    m = PATTERN.fullmatch(line + b"\n")
                    if m:
                        groups = m.groups()
                        row = dict(zip(FIELDS, [int(x, 16) for x in groups[:22]]))
                        fpga_rl = groups[22] is not None
                        with self.lock:
                            if row["k"] == 1 and self.state["origin"] is None:
                                self.state["origin"] = row["d"]
                            orig = self.state["origin"] if self.state["origin"] is not None else row["r"]
                            rel_r = ((row["r"] - orig + 2048) % 4096) - 2048
                            rel_d = ((row["d"] - orig + 2048) % 4096) - 2048
                            err_z = row["z"] - 4096 if row["z"] >= 2048 else row["z"]
                            ph_o = row["o"] - 65536 if row["o"] >= 32768 else row["o"]
                            upd = {
                                "connected": True,
                                "m": row["m"], "q": row["q"], "g": row["g"], "k": row["k"], "f": row["f"],
                                "r": row["r"], "d": row["d"], "z": err_z, "o": ph_o, "a": row["a"],
                                "t": row["t"], "n": row["n"], "h": row["h"],
                                "fpga_rl": fpga_rl,
                                "rl_mode": (1 if row["h"] in (1, 4) else 0) if fpga_rl else (1 if row["h"] >= 4 else 0),
                                "angle_deg": round(rel_r * 360.0 / 4096.0, 2),
                                "target_deg": round(rel_d * 360.0 / 4096.0, 2),
                                "error_deg": round(err_z * 360.0 / 4096.0, 2),
                                "abs_angle_deg": round(row["r"] * 360.0 / 4096.0, 2),
                            }
                            if fpga_rl:
                                s16 = lambda b: (int(b, 16) - 65536) if int(b, 16) >= 32768 else int(b, 16)
                                upd["rl_iq"] = round(s16(groups[22]) / 16384.0 * 0.27, 4)
                                upd["rl_vd"] = round(s16(groups[23]) / 16384.0 * 4.5, 3)
                                upd["rl_vq"] = round(s16(groups[24]) / 16384.0 * 4.5, 3)
                            self.state.update(upd)
                            if row["m"] != 6:
                                self.fault_logged = False
                            if self.want_active and now - last_state_cmd >= 0.45:
                                if row["m"] == 0:
                                    self._write_bytes(b"ARM\n")
                                    last_state_cmd = now
                                elif row["m"] == 6:
                                    if self.clear_fault:
                                        self.clear_fault = False
                                        self._write_bytes(b"ARM\n")
                                        last_state_cmd = now
                                    else:
                                        self.want_active = False
                                        if not self.fault_logged:
                                            self.fault_logged = True
                                            print(f"[helm_server] FPGA latched FAULT (m=6 q={row['q']} f={row['f']} a={row['a']}); waiting for Arm", flush=True)
                                elif row["m"] == 3:
                                    self._write_bytes(b"TEST\n")
                                    last_state_cmd = now
                                    last_ping = now
                            elif row["m"] == 6 and not self.fault_logged:
                                self.fault_logged = True
                                print(f"[helm_server] FPGA latched FAULT (m=6 q={row['q']} f={row['f']} a={row['a']})", flush=True)

                now = time.monotonic()
                with self.lock:
                    if now - self.last_rx > 1.5 and self.state["connected"]:
                        self.state["connected"] = False
                    if now - self.last_rx > 3.0:
                        print("[helm_server] telemetry stalled >3s; reopening serial", flush=True)
                        self._close_fd()
                        continue
                    if self.state["m"] == 5 and now - last_cmd >= 0.035:
                        if self.pending_slew is not None and self.pending_slew != self.state["slew"]:
                            s_val = self.pending_slew
                            self.state["slew"] = s_val
                            self.pending_slew = None
                            self._write_bytes(f"S{s_val:02X}\n".encode("ascii"))
                            last_ping = now
                            last_cmd = now
                        elif self.pending_target is not None:
                            t_val = self.pending_target
                            self.pending_target = None
                            self._write_bytes(f"D{t_val:03X}\n".encode("ascii"))
                            last_ping = now
                            last_cmd = now
                        elif now - last_ping >= 0.15:
                            self._write_bytes(b"PING\n")
                            last_ping = now
            except Exception as exc:
                print(f"[helm_server] unexpected loop error: {exc}", flush=True)
                time.sleep(0.2)


bridge = None

RL_SHADOW = None
cm = None
_SHADOW_INIT = False
_CPU_RL_ENABLED = True


def _get_rl_shadow():
    """Load quantized CPU RL actor pair when running CPU RL inference + FPGA motor control."""
    global RL_SHADOW, cm, _SHADOW_INIT
    if _SHADOW_INIT or not _CPU_RL_ENABLED:
        return RL_SHADOW, cm
    _SHADOW_INIT = True
    try:
        import numpy as np  # noqa: F401
        sys.path.insert(0, str(HERE.parent / "rl_infer"))
        import quant, ctrl_model as _cm
        cm = _cm
        ckpt = REPO / "Vroom_2805_Experimental_Controller_Pair/selected_checkpoint.npz"
        if ckpt.exists():
            with np.load(ckpt, allow_pickle=False) as d:
                weights = {k: d[k].copy() for k in d.files if k != "metadata_json"}
            li = quant.quantize_actor(weights, "inner")
            lo = quant.quantize_actor(weights, "outer")
            RL_SHADOW = cm.Controller(li, lo, {"dir": -1, "off": 0, "zero_sum": 18960, "sign": -1, "gain": 0, "ph": 0})
            RL_SHADOW.reset_outer(0)
    except Exception:
        RL_SHADOW = None
    return RL_SHADOW, cm


class Handler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            data = (HERE / "ee_lab_page.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif self.path == "/api/state":
            with bridge.lock:
                st = dict(bridge.state)
            if not st.get("fpga_rl"):
                err_deg = float(st.get("error_deg", 0.0))
                rl_shadow, _cm = _get_rl_shadow()
                if rl_shadow is not None and _cm is not None:
                    err_rad_s = max(-10.0, min(10.0, err_deg * 0.25))
                    ref_n = int(round(err_rad_s / 18.0 * _cm.ONE))
                    _, tr = rl_shadow.step(int(st.get("r", 0)), 0, int(st.get("a", 1185)) * 16, ref_n)
                    st["rl_iq"] = round(tr["iq_ref"] / _cm.ONE * 0.27, 4)
                    st["rl_vd"] = round(tr["nd"] / _cm.ONE * 4.5, 3)
                    st["rl_vq"] = round(tr["nq"] / _cm.ONE * 4.5, 3)
                else:
                    st["rl_iq"] = round(max(-0.135, min(0.135, err_deg * 0.004)), 4)
                    st["rl_vd"] = 0.0
                    st["rl_vq"] = round(max(-1.5, min(1.5, err_deg * 0.04)), 3)
            payload = json.dumps(st).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(payload)
        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length) if length > 0 else b"{}"
        if self.path == "/api/helm":
            d = json.loads(body)
            bridge.set_helm(float(d.get("angle_deg", 0.0)), d.get("slew"))
        elif self.path == "/api/arm":
            bridge.arm_and_start()
        elif self.path == "/api/stop":
            bridge.stop()
        elif self.path == "/api/ctrl_mode":
            d = json.loads(body)
            bridge.set_ctrl_mode(bool(d.get("rl_mode", True)))
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"ok":true}')


def _resolve_bitstream(mode, bit_override=None):
    """Return bitstream path for cpu-rl (interactive_helm) or fpga-rl (latest rl_closed_loop)."""
    if bit_override:
        p = pathlib.Path(bit_override).expanduser().resolve()
        if not p.exists():
            raise FileNotFoundError(f"Bitstream not found: {p}")
        return p
    if mode == "cpu-rl":
        p = HERE / "interactive_helm.bit"
        if not p.exists():
            raise FileNotFoundError(f"Missing CPU-RL FPGA motor bitstream: {p}")
        return p
    if mode == "fpga-rl":
        cands = sorted((REPO / "results/arty_rl_infer").glob("*-crl-*/rl_closed_loop.bit"))
        if not cands:
            raise FileNotFoundError("No built rl_closed_loop.bit found under results/arty_rl_infer/")
        return cands[-1]
    return None


def _flash_bitstream(bit_path, ftdi_serial=None):
    """Program volatile Arty A7-35T SRAM via openFPGALoader and allow USB-UART to settle."""
    loader = shutil.which("openFPGALoader")
    if not loader:
        raise RuntimeError("openFPGALoader not found in PATH")
    cmd = [loader, "-c", "digilent", "--ftdi-serial", ftdi_serial, str(bit_path)] if ftdi_serial else [loader, "-b", "arty", str(bit_path)]
    print(f"[helm_server] flashing {bit_path} ...", flush=True)
    subprocess.run(cmd, check=True)
    time.sleep(0.35)


def main():
    """Single-command launcher for Arty A7-35T FPGA motor control + CPU or FPGA RL inference."""
    global bridge, _CPU_RL_ENABLED
    ap = argparse.ArgumentParser(description="Arty A7-35T Motor Bench Server (CPU RL or All-FPGA RL)")
    mx = ap.add_mutually_exclusive_group()
    mx.add_argument("--cpu-rl", action="store_true", help="Flash interactive_helm (motor control on FPGA) + run quantized RL inference on CPU")
    mx.add_argument("--fpga-rl", action="store_true", help="Flash latest rl_closed_loop.bit + run both RL inference and motor control on FPGA")
    ap.add_argument("--mode", choices=("auto", "cpu-rl", "fpga-rl"), default="auto", help="Execution mode (auto preserves loaded bitstream)")
    ap.add_argument("--bit", default=None, help="Explicit .bit path override")
    ap.add_argument("--ftdi-serial", default=None, help="Optional Digilent FTDI serial (uses -b arty auto-detect if omitted)")
    ap.add_argument("--no-flash", action="store_true", help="Skip JTAG SRAM programming even with --cpu-rl/--fpga-rl")
    ap.add_argument("--no-arm", action="store_true", help="Do not auto-arm on server start")
    ap.add_argument("--port", default=None, help="Override UART serial device path")
    ap.add_argument("--http-port", type=int, default=HTTP_PORT, help="HTTP UI/API port (default: 8765)")
    args = ap.parse_args()

    mode = "cpu-rl" if args.cpu_rl else ("fpga-rl" if args.fpga_rl else args.mode)
    if (args.bit or mode in ("cpu-rl", "fpga-rl")) and not args.no_flash:
        bit_path = _resolve_bitstream(mode, args.bit)
        if bit_path is not None:
            _flash_bitstream(bit_path, args.ftdi_serial)

    if mode == "cpu-rl":
        _CPU_RL_ENABLED = True
        rl_shadow, _ = _get_rl_shadow()
        if rl_shadow is None:
            raise RuntimeError("Failed to initialize CPU RL model (selected_checkpoint.npz / numpy)")
        print("[helm_server] CPU RL inference enabled + FPGA motor control (interactive_helm)", flush=True)
    elif mode == "fpga-rl":
        _CPU_RL_ENABLED = False
        print("[helm_server] All-FPGA RL inference + motor control enabled (CPU RL disabled)", flush=True)

    serial_port = args.port or discover_serial_port()
    bridge = MotorBridge(serial_port)
    if not args.no_arm:
        bridge.arm_and_start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    print(f"HELM_SERVER_READY mode={mode} http://127.0.0.1:{args.http_port} (serial={serial_port})", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[helm_server] stopping motor & closing...", flush=True)
    finally:
        bridge.stop()
        srv.server_close()


if __name__ == "__main__":
    main()
