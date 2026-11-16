#!/usr/bin/env python3
"""On-silicon inference parity check for the P1 image (rl_parity_top).

Sends every golden observation vector of both actors over UART, records raw
bytes, and compares the device outputs with quant.py's integer model
(must be bit-exact) and with the FP32 golden outputs (quantization error).
Touches no motor signal: the P1 image holds all driver pins LOW.
No DTR/RTS toggles. No retries: a missing/invalid reply is recorded as failure.

Repro:
  python3 parity_host.py --run results/arty_rl_infer/<run> --port /dev/cu.usbserial-210319A43D3E1
"""
import argparse, datetime, json, os, pathlib, select, struct, termios, time

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('--run', type=pathlib.Path, required=True, help='run dir holding *_x.mem and *_y.txt')
ap.add_argument('--port', required=True)
ap.add_argument('--vectors', type=pathlib.Path,
                default=pathlib.Path(__file__).resolve().parents[4] / 'Vroom_2805_Experimental_Controller_Pair/golden_vectors.npz')
a = ap.parse_args()

out = a.run / ('bench_' + datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
out.mkdir()
fd = os.open(a.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
t = termios.tcgetattr(fd); t[0] = 0; t[1] = 0
t[2] = (t[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL)) | termios.CS8 | termios.CLOCAL | termios.CREAD
t[3] = 0; t[4] = termios.B115200; t[5] = termios.B115200
termios.tcsetattr(fd, termios.TCSANOW, t); termios.tcflush(fd, termios.TCIOFLUSH)
raw = (out / 'serial_rx.raw').open('wb'); txlog = (out / 'serial_tx.raw').open('wb')


def xor(bs):
    r = 0
    for b in bs:
        r ^= b
    return r


def read_exact(n, timeout=1.0):
    buf = bytearray(); end = time.monotonic() + timeout
    while len(buf) < n and time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], max(0, end - time.monotonic()))
        if r:
            chunk = os.read(fd, n - len(buf)); raw.write(chunk); buf += chunk
    return bytes(buf)


def infer(actor, xvec):
    frame = bytes([0xA5, actor]) + b''.join(struct.pack('<h', int(v)) for v in xvec)
    frame += bytes([xor(frame)])
    os.write(fd, frame); txlog.write(frame)
    r = read_exact(17)
    if len(r) != 17 or r[0] != 0x5A or r[1] != actor or r[16] != xor(r[:16]):
        raise RuntimeError(f'bad reply {r.hex()}')
    y0, y1, z0, z1, cyc = struct.unpack('<hhiiH', r[2:16])
    return y0, y1, z0, z1, cyc


result = {'port': a.port, 'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(), 'actors': {}}
try:
    g = np.load(a.vectors, allow_pickle=False)
    for actor, name, nin in ((0, 'inner', 25), (1, 'outer', 19)):
        x = np.array([int(s, 16) for s in (a.run / f"{name}_x.mem").read_text().split()], dtype=np.int64)
        x = np.where(x >= 32768, x - 65536, x).reshape(-1, nin)
        ref = np.loadtxt(a.run / f'{name}_y.txt', dtype=np.int64)
        rows = []
        t0 = time.monotonic()
        for n in range(len(x)):
            rows.append(infer(actor, x[n]))
        dev = np.array([r[:2] for r in rows], dtype=np.int64)
        np.savetxt(out / f'device_{name}_y.txt', np.array(rows, dtype=np.int64), fmt='%d',
                   header='y0 y1 z0 z1 cycles')
        fp = np.abs(dev / 32768.0 - g[f'{name}_output'])
        result['actors'][name] = {
            'vectors': len(x), 'mismatched_words_vs_integer_model': int((dev != ref).sum()),
            'max_abs_error_vs_fp32': float(fp.max()), 'mean_abs_error_vs_fp32': float(fp.mean()),
            'device_cycles': sorted(set(int(r[4]) for r in rows)),
            'wall_s': time.monotonic() - t0}
    result['bit_exact'] = all(v['mismatched_words_vs_integer_model'] == 0 for v in result['actors'].values())
    result['status'] = 'passed' if result['bit_exact'] else 'mismatch'
except Exception as e:
    result['status'] = 'failed'; result['failure'] = str(e)
finally:
    raw.close(); txlog.close(); os.close(fd)
    (out / 'parity.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2)); print(out)
raise SystemExit(0 if result.get('status') == 'passed' else 1)
