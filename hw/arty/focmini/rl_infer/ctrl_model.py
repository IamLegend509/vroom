#!/usr/bin/env python3
"""Integer controller model for the C-RL image + approximate closed-loop plant.

Purpose
    Bit-exact specification of the fixed-point controller glue the RTL
    (``rl_ctrl.sv``) runs around the two quantized actors, and a closed-loop
    check of that exact arithmetic on an approximate PMSM. Mirrors
    ``InnerIO``/``OuterIO`` from the controller-pair package, with the declared
    single-current-sensor observer replacing measured id/iq.

Units (all integers, floor arithmetic: ``>>`` is arithmetic shift)
    current   i_n  : Q.14 of 0.27 A        (16384 = 0.27 A = trip level)
    speed     w_n  : Q.14 of 18 rad/s mech
    angle     e16  : 65536 per electrical turn;  th : AS5600 counts, 4096/mech turn
    normalized voltage n : Q.14 of Vdc/2  (|n| <= 0.9)
    trig      Q.14 from a 1024-entry sine table (index e16 >> 6)
    outer bias/cmd state: Q.24 of 0.27 A (feature = state >> 10)
    actor I/O : inputs Q.14 clipped to +/-16384, outputs Q.15 (quant.py)
    PWM       : 5000 counts/period (20 kHz), duty = 2500 + (phase * 2500 >> 14)

Plant (closed-loop check only) is the same approximate sinusoidal-EMF PMSM as
the single-sensor study: NOT GEM, used to compare designs, never to qualify.

Repro
    python3 hw/arty/focmini/rl_infer/ctrl_model.py --out <dir>
"""
import argparse, json, math, pathlib, sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import quant  # noqa: E402

# ---------------- fixed-point constants (shared with RTL) ----------------
Q = 14
ONE = 1 << Q
SIN = [round(math.sin(2 * math.pi * i / 1024) * ONE) for i in range(1024)]
POLE_PAIRS = 7
# Observer (nominal assumed-2805: R 12 ohm, L 4 mH, ke 0.0341 V s/rad, Vdc 9 V, dt 100 us)
R_NOM, L_NOM, KE_NOM, VDC_NOM, DT = 12.0, 0.004, 0.03410463066, 9.0, 1e-4
KV = round((VDC_NOM / 2) / (R_NOM * 0.27) * ONE)                 # n -> steady current
KW = round(POLE_PAIRS * 18 * (L_NOM / R_NOM) * ONE)             # cross-coupling
KE = round(KE_NOM * 18 / (R_NOM * 0.27) * ONE)                   # back-EMF current
KA = round(math.exp(-R_NOM / L_NOM * DT) * ONE)                  # exact decay per tick
KG = ONE // 2                                                    # innovation gain 0.5
K09 = round(0.9 * ONE)                                           # 14746
KS3 = round(math.sqrt(3) / 2 * ONE)                              # 14189
KC = 197                       # e16 per w_n per (age ~150 us) >> 14: 80.2*150e-6*2^14
ERR_TANH = 360                 # err/0.05 = err_n * 18/0.05
# IASA in Q.24 of 0.27 A: trial = bias + 0.005 * ai ; requested = 0.5 * ap + trial
IASA_LIM = 1 << 23             # 0.5 (= 0.135 A) in Q.24
PWM_HALF = 2500
PHASE_E16 = (0, 21845, 43691)   # sensed motor phase A / B / C
# ADC chain (nominal; must be measured): ACS724 400 mV/A, 18k over (18k || 3.32k), Arty 1/3.32,
# XADC 1 V / 4096 -> 66.5 codes/A; 16-sample sum -> 1064 per A -> i_n = sum * 57.0
ADC_GAIN = round(1 / (16 * 66.5) / 0.27 * ONE * 256)             # i_n per sum-LSB, Q.8
ENC_PERIOD = 200e-6            # AS5600 poll period (one 400 kHz status+angle read is ~145 us)
ENC_LATENCY = 60e-6            # angle register read -> frame accepted
SPEED_PER_COUNT = round((2 * math.pi / 4096) / ENC_PERIOD / 18 * ONE)  # w_n per count per poll


def clip(x, lo=-ONE, hi=ONE):
    return lo if x < lo else hi if x > hi else x


def mulq(a, b):
    return (a * b) >> Q


def sincos(e16):
    i = (e16 & 0xFFFF) >> 6
    return SIN[i], SIN[(i + 256) & 1023]


def tanh14(x14):
    """tanh of a Q.14 value, Q.14 result (quant.py table, Q.15 >> 1)."""
    z = clip(x14, -quant.ACT_MAX, quant.ACT_MAX)
    return int(quant.tanh_q(np.array([z]))[0]) >> 1


class Controller:
    """One instance = the RTL controller state after ARM/calibration."""

    def __init__(self, layers_inner, layers_outer, cal):
        self.li, self.lo = layers_inner, layers_outer
        self.cal = cal                      # dir (+1/-1), off (e16), zero_sum, sign (+1/-1), gain (Q.8)
        self.est = [0, 0]                   # observer id, iq (i_n)
        self.nprev = [0, 0]                 # normalized dq commanded last tick (applied next tick)
        self.napp = [0, 0]                  # normalized dq applied during the tick just ended
        self.prev_raw = [0, 0]              # InnerIO.previous[:2] (Q.14)
        self.ihist = [[0, 0] for _ in range(5)]
        self.ohist = [[0, 0, 0] for _ in range(20)]   # [w_n, err_n, cmd feature]
        self.bias = 0; self.cmd = 0         # Q.24
        self.iq_ref = 0                     # Q.14
        self.tick = 0

    def reset_outer(self, ref0):
        for h in self.ohist:
            h[:] = [0, ref0, 0]

    # ---- one 100 us inner tick ----
    def step(self, th, w_n, adc_sum, ref_n):
        c = self.cal
        w_n = clip(w_n, -(1 << 17) + 1, (1 << 17) - 1)   # RTL multiplier port bound
        ref_n = clip(ref_n, -(1 << 17) + 1, (1 << 17) - 1)
        e16 = (c['dir'] * POLE_PAIRS * 16 * th + c['off'] + ((w_n * KC) >> Q)) & 0xFFFF
        s, co = sincos(e16)
        ia = (c['sign'] * (adc_sum - c['zero_sum']) * c['gain']) >> 8
        # observer predict over the tick just ended: duties latch at tick boundaries, so the
        # voltage applied during it was commanded two ticks ago (napp)
        d, q = self.est
        ssd = mulq(self.napp[0], KV) + mulq(mulq(w_n, q), KW)
        ssq = mulq(self.napp[1], KV) - mulq(mulq(w_n, d), KW) - mulq(w_n, KE)
        d = ssd + mulq(d - ssd, KA)
        q = ssq + mulq(q - ssq, KA)
        # sensed phase: i_x = id cos(e - ph) - iq sin(e - ph), ph = 0 / +120 / -120 electrical degrees
        sp, cp = sincos(e16 - PHASE_E16[c.get('ph', 0)])
        innov = ia - (mulq(cp, d) - mulq(sp, q))
        gi = mulq(innov, KG)
        d = clip(d + mulq(cp, gi), -2 * ONE, 2 * ONE)
        q = clip(q - mulq(sp, gi), -2 * ONE, 2 * ONE)
        self.est = [d, q]
        trace = {'e16': e16, 'ia': ia, 'id': d, 'iq': q}
        # outer tick every 10 inner ticks (before the inner decision, like the pair evaluator)
        if self.tick % 10 == 0:
            err = ref_n - w_n
            xo = [w_n, ref_n, err, tanh14(err * ERR_TANH), d, q,
                  (self.bias >> 10) * 2, (self.cmd >> 10) * 2, s, co]
            for lag in (5, 10, 20):
                h = self.ohist[-lag]
                xo += [h[0], h[1], h[2]]
            xo = [clip(v) for v in xo]
            if self.tick:
                self.ohist = self.ohist[1:] + [[w_n, err, (self.cmd >> 10) * 2]]
            yo, _ = quant.forward_int(self.lo, np.array([xo]))
            ap, ai = int(yo[0][0]) >> 1, int(yo[0][1]) >> 1          # Q.14 raw, |.| <= 1
            trial = self.bias + ai * 82                               # 0.005 * 2^24 / 2^14 = 81.92
            req = (ap << 9) + trial                                   # 0.5 * ap in Q.24
            lim = clip(req, -IASA_LIM, IASA_LIM)
            self.bias = clip(trial + (((lim - req) * 1638) >> Q), -IASA_LIM, IASA_LIM)  # 0.1
            self.cmd = (self.cmd >> 1) + (lim >> 1)
            self.iq_ref = self.cmd >> 10
            trace['xo'] = xo; trace['yo'] = [int(v) for v in yo[0]]
        rq = self.iq_ref
        xi = [d, q, 0, rq, (0 - d) >> 1, (rq - q) >> 1, self.prev_raw[0], self.prev_raw[1], 0, 0]
        for h in self.ihist:
            xi += h
        xi += [w_n, s, co, 0, 0]
        xi = [clip(v) for v in xi]
        yi, _ = quant.forward_int(self.li, np.array([xi]))
        r0, r1 = int(yi[0][0]) >> 1, int(yi[0][1]) >> 1
        self.prev_raw = [r0, r1]
        self.ihist = self.ihist[1:] + [[d, q]]
        qd, qq = mulq(r0, K09), mulq(r1, K09)
        n2 = qd * qd + qq * qq
        if n2 > K09 * K09:
            sc = (K09 << Q) // math.isqrt(n2)
            qd = (abs(qd) * sc >> Q) * (1 if qd >= 0 else -1)
            qq = (abs(qq) * sc >> Q) * (1 if qq >= 0 else -1)
        self.napp = self.nprev
        self.nprev = [qd, qq]
        al = (co * qd - s * qq) >> Q
        be = (s * qd + co * qq) >> Q
        t = (be * KS3) >> Q
        hb = (-al) >> 1
        ph = [al, hb + t, hb - t]
        duty = [PWM_HALF + ((p * PWM_HALF) >> Q) for p in ph]
        trace.update({'xi': xi, 'yi': [int(v) for v in yi[0]], 'nd': qd, 'nq': qq, 'duty': duty,
                      'iq_ref': rq})
        self.tick += 1
        return duty, trace


# ---------------- approximate plant + sensors (closed-loop check) ----------------
P, R, L, KEp, J, FA, FB, VDC = 7, 12., .004, .03410463066, 1e-5, .0005, .00005, 9.0


def deriv(x, vdq, r=R, l=L, ke=KEp):
    i_d, i_q, w, th = x
    we = P * w
    return np.array([(vdq[0] - r * i_d + we * l * i_q) / l,
                     (vdq[1] - r * i_q - we * l * i_d - ke * w) / l,
                     (1.5 * ke * i_q - FA * np.tanh(w / .01) - FB * w) / J, w])


def closed_loop(profile, seconds, seed=0, enc_dir=-1, enc_off=1234, cur_sign=-1, zero_code=416, sensed_phase=0,
                plant_scale=(1., 1., 1.), adc_noise_codes=1.0, enc_noise_counts=0.5, record=None):
    """Simulate the integer controller against the plant. enc_dir/enc_off/cur_sign/zero model the
    physical unknowns; calibration is assumed perfect here (it is verified separately)."""
    rng = np.random.default_rng(seed)
    li = quant.quantize_actor(WEIGHTS, 'inner'); lo = quant.quantize_actor(WEIGHTS, 'outer')
    x = np.array([0., 0., 0., rng.uniform(0, 2 * np.pi)])
    # calibration that maps encoder -> electrical angle exactly (perfect alignment)
    th_counts = lambda mech: (enc_dir * mech * 4096 / (2 * np.pi) + enc_off) % 4096
    cal = {'dir': enc_dir, 'off': (-enc_dir * POLE_PAIRS * 16 * enc_off) & 0xFFFF,
           'zero_sum': 16 * zero_code, 'sign': cur_sign, 'gain': ADC_GAIN, 'ph': sensed_phase}
    ctl = Controller(li, lo, cal)
    ctl.reset_outer(int(round(profile(0) / 18 * ONE)))
    r_, l_, ke_ = R * plant_scale[0], L * plant_scale[1], KEp * plant_scale[2]
    duty = [PWM_HALF] * 3
    enc_th = int(th_counts(x[3])); w_f = 0; last_enc = enc_th; next_enc = 0.0; snap_t = 0.0; enc_snap = []
    log = []
    for k in range(int(seconds / DT)):
        t = k * DT
        # AS5600: one frame per ENC_PERIOD, visible ENC_LATENCY after the angle was read.
        # Plant state is only known at tick resolution here, so the read time is rounded to a tick.
        while snap_t <= t:
            enc_snap.append(x[3]); snap_t += ENC_PERIOD
        while next_enc + ENC_LATENCY <= t:
            new = int(round(th_counts(enc_snap.pop(0)) + rng.normal(0, enc_noise_counts))) % 4096
            dth = ((new - last_enc + 2048) % 4096) - 2048
            last_enc = new; enc_th = new
            w_f += ((enc_dir * dth * SPEED_PER_COUNT) - w_f) >> 3
            next_enc += ENC_PERIOD
        # XADC: 16 samples of phase A around the PWM centre at tick start
        ea = P * x[3]
        ea -= sensed_phase * 2 * np.pi / 3 if sensed_phase < 2 else -2 * np.pi / 3
        ia_true = x[0] * np.cos(ea) - x[1] * np.sin(ea)
        codes = np.clip(np.round(zero_code + cur_sign * ia_true * 66.5 + rng.normal(0, adc_noise_codes, 16)), 0, 4095)
        ref_n = int(round(profile(t) / 18 * ONE))
        new_duty, tr = ctl.step(enc_th, w_f, int(codes.sum()), ref_n)
        if record is not None:
            record.append({'th': enc_th, 'w': w_f, 'adc': int(codes.sum()), 'ref': ref_n, **tr})
        # apply previous duty during this tick (one-tick computation delay)
        ph = [(d - PWM_HALF) / PWM_HALF * VDC / 2 for d in duty]
        al = (2 * ph[0] - ph[1] - ph[2]) / 3; be = (ph[1] - ph[2]) / np.sqrt(3)
        for _ in range(10):
            th_e = P * x[3]
            v = (np.cos(th_e) * al + np.sin(th_e) * be, -np.sin(th_e) * al + np.cos(th_e) * be)
            h = DT / 10
            k1 = deriv(x, v, r_, l_, ke_); k2 = deriv(x + h / 2 * k1, v, r_, l_, ke_)
            k3 = deriv(x + h / 2 * k2, v, r_, l_, ke_); k4 = deriv(x + h * k3, v, r_, l_, ke_)
            x = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        duty = new_duty
        peak = float(np.hypot(x[0], x[1]))
        log.append((t, profile(t), x[2], peak, ctl.est[1] / ONE * .27, x[1]))
        if peak > .27:
            return {'tripped_at_s': t, 'peak_A': peak}, np.array(log)
    a = np.array(log)
    m = a[:, 0] > min(.2, a[-1, 0] / 2)
    err = a[m, 1] - a[m, 2]
    return {'speed_rmse': float(np.sqrt(np.mean(err ** 2))), 'max_abs_speed_err': float(np.abs(err).max()),
            'peak_current_A': float(a[:, 3].max()),
            'iq_est_rmse_A': float(np.sqrt(np.mean((a[:, 4] - a[:, 5]) ** 2)))}, a


WEIGHTS = None


def main():
    global WEIGHTS
    ap = argparse.ArgumentParser()
    ap.add_argument('--checkpoint', type=pathlib.Path, default=quant.PAIR / 'selected_checkpoint.npz')
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--trace-ticks', type=int, default=400)
    ap.add_argument('--trace-phase', type=int, default=1, help='sensed phase used for the RTL replay trace')
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    (a.out / 'sin.mem').write_text('\n'.join(format(v & 0xFFFF, '04x') for v in SIN) + '\n')
    (a.out / 'tanh.mem').write_text('\n'.join(format(int(v), '04x') for v in quant.TANH) + '\n')
    with np.load(a.checkpoint, allow_pickle=False) as d:
        WEIGHTS = {k: d[k].copy() for k in d.files if k != 'metadata_json'}
    steps = lambda t: 0. if t < .1 else 10. if t < .7 else -10. if t < 1.3 else 0.
    small = lambda t: 0. if t < .1 else 3. if t < .6 else -3. if t < 1.1 else 0.
    cases = [('steps10', steps, 1.6, {}), ('steps3', small, 1.3, {}), ('hold0', lambda t: 0., .6, {}),
             ('steps10_paramerr', steps, 1.6, {'plant_scale': (1.3, 1.2, .85)}),
             ('steps10_fwd_enc', steps, 1.6, {'enc_dir': 1, 'cur_sign': 1}),
             ('steps10_phaseB', steps, 1.6, {'sensed_phase': 1}), ('steps10_phaseC', steps, 1.6, {'sensed_phase': 2})]
    report = {'constants': {'KV': KV, 'KW': KW, 'KE': KE, 'KA': KA, 'KG': KG, 'K09': K09, 'KS3': KS3, 'KC': KC,
                            'ADC_GAIN_Q8': ADC_GAIN, 'SPEED_PER_COUNT': SPEED_PER_COUNT, 'ENC_PERIOD_s': ENC_PERIOD}, 'cases': {}}
    for name, prof, sec, kw in cases:
        for seed in range(2):
            r, _ = closed_loop(prof, sec, seed, **kw)
            report['cases'][f'{name}_s{seed}'] = r
            print(name, seed, json.dumps(r), flush=True)
    rec = []                                       # replay vectors from controller reset
    closed_loop(lambda t: 0. if t < .005 else 10., a.trace_ticks * DT, 0, record=rec, sensed_phase=a.trace_phase)
    report['trace'] = {'ticks': a.trace_ticks, 'sensed_phase': a.trace_phase}
    (a.out / 'ctrl_trace.json').write_text(json.dumps(rec))
    (a.out / 'ctrl_report.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
