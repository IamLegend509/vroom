#!/usr/bin/env python3
"""Bench driver for helm_server --fpga-rl: arm, wait calibration, run a step sequence, log telemetry.

Usage: rl_bench.py OUT.jsonl [--mode raw|smooth] [--slew N] [--steps "0,30,-30,90,0"] [--dwell S]
Each step holds `dwell` seconds. Aborts immediately on FAULT (m=6) and sends STOP at the end.
"""
import argparse, json, time, urllib.request

B = "http://127.0.0.1:8765"


def get():
    return json.loads(urllib.request.urlopen(B + "/api/state", timeout=1).read())


def post(path, d=None):
    req = urllib.request.Request(B + path, data=json.dumps(d or {}).encode(), headers={"Content-Type": "application/json"})
    urllib.request.urlopen(req, timeout=1).read()


ap = argparse.ArgumentParser()
ap.add_argument("out")
ap.add_argument("--mode", choices=("raw", "smooth"), default="raw")
ap.add_argument("--slew", type=int, default=14)
ap.add_argument("--steps", default="0,30,-30,90,-90,0")
ap.add_argument("--dwell", type=float, default=3.0)
ap.add_argument("--no-arm", action="store_true")
a = ap.parse_args()

log = open(a.out, "w")
t0 = time.monotonic()


def sample(tag):
    s = get(); s["_t"] = round(time.monotonic() - t0, 3); s["_tag"] = tag
    log.write(json.dumps(s) + "\n")
    return s


status = "ok"
try:
    post("/api/ctrl_mode", {"rl_mode": a.mode == "smooth"})
    if not a.no_arm:
        post("/api/arm")
    deadline = time.monotonic() + 15
    while True:
        s = sample("cal")
        if s.get("m") == 5 and s.get("k") == 1:
            break
        if s.get("m") == 6 and time.monotonic() - t0 > 2:
            raise RuntimeError(f"fault during arm/cal q={s.get('q')}")
        if time.monotonic() > deadline:
            raise RuntimeError(f"no calibration (m={s.get('m')} k={s.get('k')})")
        time.sleep(0.05)
    post("/api/ctrl_mode", {"rl_mode": a.mode == "smooth"})
    time.sleep(0.3)
    for tgt in [float(x) for x in a.steps.split(",")]:
        post("/api/helm", {"angle_deg": tgt, "slew": a.slew})
        end = time.monotonic() + a.dwell
        while time.monotonic() < end:
            s = sample(f"step{tgt:g}")
            if s.get("m") == 6:
                raise RuntimeError(f"FAULT q={s.get('q')} at target {tgt}")
            time.sleep(0.025)
except Exception as e:
    status = f"fail: {e}"
finally:
    try:
        post("/api/stop")
    except Exception:
        pass
    log.close()
    print(status)
