#!/usr/bin/env python3
"""Interactive EE Control Systems & RL Digital-Twin Workbench Server for Arty A7-35T FOCMini."""
import glob, json, math, os, pathlib, re, select, sys, termios, threading, time
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


PORT = discover_serial_port()
HTTP_PORT = int(os.environ.get("HELM_HTTP_PORT", "8765"))

FIELDS = 'a v c i e s r b f m q n t p g x d z o h k y'.split()
PATTERN = re.compile(
    rb'C3 a=([0-9A-F]{3}) v=([01]) c=([01]) i=([01]) e=([0-9A-F]) s=([0-9A-F]{2}) '
    rb'r=([0-9A-F]{3}) b=([0-3]) f=([01]) m=([0-6]) q=([0-9A-F]) n=([0-9A-F]{4}) '
    rb't=([0-9A-F]{4}) p=([0-9A-F]{2}) g=([01]) x=([0-9A-F]{4}) d=([0-9A-F]{3}) '
    rb'z=([0-9A-F]{3}) o=([0-9A-F]{4}) h=([0-5]) k=([01]) y=([0-9A-F])\r\n'
)


class MotorBridge:
    def __init__(self, port):
        self.port = port
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        t = termios.tcgetattr(self.fd)
        t[0] = 0; t[1] = 0
        t[2] = (t[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL)) | termios.CS8 | termios.CLOCAL | termios.CREAD
        t[3] = 0; t[4] = termios.B115200; t[5] = termios.B115200
        termios.tcsetattr(self.fd, termios.TCSANOW, t)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        self.lock = threading.Lock()
        self.state = {
            "connected": True, "m": 0, "q": 0, "g": 0, "k": 0, "f": 1,
            "r": 0, "d": 0, "z": 0, "o": 0, "a": 0, "t": 0, "n": 0,
            "origin": None, "angle_deg": 0.0, "target_deg": 0.0, "error_deg": 0.0,
            "slew": 24, "auto_Wave": False, "rl_mode": 1, "h": 4
        }
        self.pending_target = None
        self.pending_slew = None
        self.want_active = False
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def send_raw(self, cmd):
        with self.lock:
            os.write(self.fd, (cmd + "\n").encode("ascii"))

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
            self.state["origin"] = None

    def set_ctrl_mode(self, rl_mode):
        self.send_raw("M1" if rl_mode else "M0")

    def stop(self):
        with self.lock:
            self.want_active = False
            self.pending_target = None
        self.send_raw("!")

    def _loop(self):
        buf = bytearray()
        last_ping = 0.0
        last_cmd = 0.0
        while self.running:
            if select.select([self.fd], [], [], 0.02)[0]:
                try:
                    data = os.read(self.fd, 4096)
                    buf.extend(data)
                except OSError:
                    pass
                while b"\n" in buf:
                    line, _, rem = buf.partition(b"\n")
                    buf = bytearray(rem)
                    m = PATTERN.fullmatch(line + b"\n")
                    if m:
                        row = dict(zip(FIELDS, [int(x, 16) for x in m.groups()]))
                        with self.lock:
                            if row["k"] == 1 and self.state["origin"] is None:
                                self.state["origin"] = row["d"]
                            orig = self.state["origin"] if self.state["origin"] is not None else row["r"]
                            rel_r = ((row["r"] - orig + 2048) % 4096) - 2048
                            rel_d = ((row["d"] - orig + 2048) % 4096) - 2048
                            err_z = row["z"] - 4096 if row["z"] >= 2048 else row["z"]
                            ph_o = row["o"] - 65536 if row["o"] >= 32768 else row["o"]
                            self.state.update({
                                "m": row["m"], "q": row["q"], "g": row["g"], "k": row["k"], "f": row["f"],
                                "r": row["r"], "d": row["d"], "z": err_z, "o": ph_o, "a": row["a"],
                                "t": row["t"], "n": row["n"], "h": row["h"], "rl_mode": 1 if row["h"] >= 4 else 0,
                                "angle_deg": round(rel_r * 360.0 / 4096.0, 2),
                                "target_deg": round(rel_d * 360.0 / 4096.0, 2),
                                "error_deg": round(err_z * 360.0 / 4096.0, 2),
                                "abs_angle_deg": round(row["r"] * 360.0 / 4096.0, 2),
                            })
                            if self.want_active:
                                if row["m"] in (0, 6):
                                    os.write(self.fd, b"ARM\n")
                                elif row["m"] == 3:
                                    os.write(self.fd, b"TEST\n")
                                    last_ping = time.monotonic()

            now = time.monotonic()
            with self.lock:
                if self.state["m"] == 5 and now - last_cmd >= 0.035:
                    if self.pending_slew is not None and self.pending_slew != self.state["slew"]:
                        s_val = self.pending_slew
                        self.state["slew"] = s_val
                        self.pending_slew = None
                        os.write(self.fd, f"S{s_val:02X}\n".encode("ascii"))
                        last_ping = now
                        last_cmd = now
                    elif self.pending_target is not None:
                        t_val = self.pending_target
                        self.pending_target = None
                        os.write(self.fd, f"D{t_val:03X}\n".encode("ascii"))
                        last_ping = now
                        last_cmd = now
                    elif now - last_ping >= 0.15:
                        os.write(self.fd, b"PING\n")
                        last_ping = now


bridge = MotorBridge(PORT)

RL_SHADOW = None
cm = None
try:
    import numpy as np
    sys.path.insert(0, str(HERE.parent / "rl_infer"))
    import quant, ctrl_model as _cm
    cm = _cm
    ckpt = REPO / "Vroom_2805_Experimental_Controller_Pair/selected_checkpoint.npz"
    if ckpt.exists():
        with np.load(ckpt, allow_pickle=False) as d:
            WEIGHTS = {k: d[k].copy() for k in d.files if k != "metadata_json"}
        LI = quant.quantize_actor(WEIGHTS, "inner")
        LO = quant.quantize_actor(WEIGHTS, "outer")
        RL_SHADOW = cm.Controller(LI, LO, {"dir": -1, "off": 0, "zero_sum": 18960, "sign": -1, "gain": 0, "ph": 0})
        RL_SHADOW.reset_outer(0)
except Exception:
    RL_SHADOW = None


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
            err_deg = float(st.get("error_deg", 0.0))
            if RL_SHADOW is not None and cm is not None:
                err_rad_s = max(-10.0, min(10.0, err_deg * 0.25))
                ref_n = int(round(err_rad_s / 18.0 * cm.ONE))
                _, tr = RL_SHADOW.step(int(st.get("r", 0)), 0, int(st.get("a", 1185)) * 16, ref_n)
                st["rl_iq"] = round(tr["iq_ref"] / cm.ONE * 0.27, 4)
                st["rl_vd"] = round(tr["nd"] / cm.ONE * 4.5, 3)
                st["rl_vq"] = round(tr["nq"] / cm.ONE * 4.5, 3)
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


if __name__ == "__main__":
    bridge.arm_and_start()
    srv = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    print(f"HELM_SERVER_READY http://127.0.0.1:{HTTP_PORT} (serial={PORT})", flush=True)
    srv.serve_forever()
