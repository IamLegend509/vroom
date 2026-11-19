#!/usr/bin/env python3
"""Summarize a bench.py jsonl: per step, final error, RMS error over last half, overshoot, 2-deg settle time, chatter."""
import json, sys, math
rows = [json.loads(l) for l in open(sys.argv[1])]
steps = []
for r in rows:
    if r["_tag"].startswith("step"):
        if not steps or steps[-1][0] != r["_tag"]:
            steps.append((r["_tag"], []))
        steps[-1][1].append(r)
prev = 0.0
for tag, rs in steps:
    tgt = float(tag[4:])
    t0 = rs[0]["_t"]
    ang = [r["angle_deg"] for r in rs]
    err = [tgt - x for x in ang]
    half = err[len(err)//2:]
    rms = math.sqrt(sum(e*e for e in half)/len(half))
    d = tgt - prev
    over = 0.0
    if abs(d) > 1:
        over = max(((x - tgt) * (1 if d > 0 else -1) for x in ang), default=0)
    settle = None
    for i in range(len(err)):
        if all(abs(e) < 2 for e in err[i:]):
            settle = rs[i]["_t"] - t0; break
    vq = [r.get("rl_vq", 0) for r in rs]; iq = [r.get("rl_iq", 0) for r in rs]
    jit = max(ang[len(ang)//2:]) - min(ang[len(ang)//2:])
    print(f"{tag:>9} final_err={err[-1]:+6.2f} rms2={rms:5.2f} overshoot={over:5.2f} settle2deg={settle if settle is None else round(settle,2)} "
          f"p2p_tail={jit:5.2f} |vq|max={max(map(abs,vq)):.2f} |iq|max={max(map(abs,iq)):.3f} q={rs[-1]['q']} m={rs[-1]['m']}")
    prev = tgt
