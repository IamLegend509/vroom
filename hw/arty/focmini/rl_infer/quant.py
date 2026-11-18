#!/usr/bin/env python3
"""Fixed-point export + bit-exact integer model of the two vroom actors.

Purpose
    Convert the FP32 two-actor checkpoint (Linear/ReLU/Linear/ReLU/Linear/tanh,
    256x256 hidden) into the integer contract executed by ``rl_mlp.sv`` and
    provide the integer reference the RTL must match bit-for-bit.

Number formats (all two's complement integers)
    input  x   int16, Q.14  (x_real = x_int / 2**14; observations are pre-clipped to [-1, 1])
    weight w   per-row power-of-two scale: W_real[r, k] = w_int[r, k] * 2**e_r,
               |w_int| <= 2**(bits-1) - 1, e_r = ceil(log2(max_k |W[r, k]| / (2**(bits-1) - 1)))
    acc        sum_k w_int * a_int (exact; < 2**47 by construction)
    scale      s = floor((floor(acc / 2**(sh-1)) + 1) / 2)  with sh = 14 - (e_r + 14) = -e_r
               i.e. round-half-up of acc * 2**e_r (requires sh >= 1)
    bias       B_r = round(b_r * 2**14), added after scaling
    hidden     ReLU then saturate to [0, 2**24 - 1]            (Q.14, 25-bit signed storage)
    preact     saturate to [-(2**24 - 1), 2**24 - 1]            (Q.14)
    output     tanh via 513-entry table T[i] = min(32767, round(tanh(i/64) * 32768)),
               |z| = i/64 + f/16384 (i = |z| >> 8, f8 = |z| & 255),
               t = T[i] + floor(((T[i+1] - T[i]) * f8 + 128) / 256), |z| >= 8 -> 32767,
               sign restored; result int16 Q.15 (y_real = y_int / 32768).

Assumptions / limits
    Weight bit widths per layer are a declared resource/accuracy trade (BRAM on
    xc7a35t). Errors versus the FP32 golden vectors are measured and reported,
    not assumed. This is quantization for offline FPGA inference testing; it
    does not qualify any controller on hardware.

Repro
    python3 hw/arty/focmini/rl_infer/quant.py --out <dir>
"""
import argparse
import hashlib
import json
import pathlib

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[4]
PAIR = REPO / 'Vroom_2805_Experimental_Controller_Pair'
FA = 14                      # activation fraction bits
ACT_MAX = (1 << 24) - 1      # 25-bit signed storage, positive max
TANH_STEP_BITS = 6           # table step 1/64
TANH_N = 513

# name -> (inputs, lanes, weight bits for W0, W1, W2)
ACTORS = {'inner': (25, 16, (16, 16, 16)), 'outer': (19, 16, (16, 9, 16))}
H = 256


def tanh_table():
    t = np.round(np.tanh(np.arange(TANH_N) / (1 << TANH_STEP_BITS)) * 32768).astype(np.int64)
    return np.minimum(t, 32767)


TANH = tanh_table()


def tanh_q(z):
    z = np.asarray(z, dtype=np.int64)
    a = np.abs(z)
    i = a >> (FA - TANH_STEP_BITS)
    f = a & ((1 << (FA - TANH_STEP_BITS)) - 1)
    ic = np.minimum(i, TANH_N - 2)
    t = TANH[ic] + (((TANH[ic + 1] - TANH[ic]) * f + 128) >> 8)
    t = np.where(i >= TANH_N - 1, 32767, t)
    return np.where(z < 0, -t, t)


def quantize_actor(weights, name):
    nin, lanes, bits = ACTORS[name]
    layers = []
    for j in range(3):
        W = weights[f'{name}_W{j}'].astype(np.float64)
        b = weights[f'{name}_b{j}'].astype(np.float64)
        qmax = (1 << (bits[j] - 1)) - 1
        m = np.abs(W).max(axis=1)
        e = np.ceil(np.log2(np.maximum(m, 1e-30) / qmax)).astype(np.int64)
        w = np.clip(np.round(W / 2.0 ** e[:, None]), -qmax, qmax).astype(np.int64)
        sh = -e
        if sh.min() < 1:
            raise ValueError(f'{name} layer {j}: shift < 1 unsupported')
        B = np.round(b * (1 << FA)).astype(np.int64)
        if np.abs(B).max() > ACT_MAX:
            raise ValueError(f'{name} layer {j}: bias exceeds activation range')
        layers.append({'w': w, 'sh': sh, 'b': B, 'bits': bits[j]})
    return layers


def forward_int(layers, x_int):
    a = np.asarray(x_int, dtype=np.int64)
    for j, L in enumerate(layers):
        acc = a @ L['w'].T                          # exact int64 (fits: < 2**47)
        s = ((acc >> (L['sh'] - 1)) + 1) >> 1       # arithmetic shifts = floor
        v = s + L['b']
        if j < 2:
            a = np.clip(v, 0, ACT_MAX)
        else:
            z = np.clip(v, -ACT_MAX, ACT_MAX)
    return tanh_q(z), z


def quantize_input(x):
    x = np.clip(np.asarray(x, dtype=np.float64), -1.0, 1.0)
    return np.floor(x * (1 << FA) + 0.5).astype(np.int64)


def hexw(v, bits):
    return format(int(v) & ((1 << bits) - 1), f'0{(bits + 3) // 4}x')


def pack(values, bits):
    """values: lane-ordered list -> one hex word, lane 0 in the LSBs."""
    word = 0
    for lane, v in enumerate(values):
        word |= (int(v) & ((1 << bits) - 1)) << (lane * bits)
    return format(word, f'0{(len(values) * bits + 3) // 4}x')


SH_BITS = 6
BIAS_BITS = 25


def write_mems(layers, name, out):
    """Lane l owns rows r with r % lanes == l, local row r // lanes.

    W memory address = local_row * K + k; each word holds every lane's weight.
    Layer 2 has two rows: lanes 0 and 1, local row 0, other lanes zero.
    BS memory word per local row: lane fields of {shift[5:0], bias[24:0]}.
    """
    nin, lanes, _ = ACTORS[name]
    for j, L in enumerate(layers):
        rows, K = L['w'].shape
        npl = max(1, rows // lanes)
        wl, bl = [], []
        for lr in range(npl):
            for k in range(K):
                vals = [L['w'][lr * lanes + l, k] if lr * lanes + l < rows else 0 for l in range(lanes)]
                wl.append(pack(vals, L['bits']))
            fields = []
            for l in range(lanes):
                r = lr * lanes + l
                sh, b = (int(L['sh'][r]), int(L['b'][r])) if r < rows else (1, 0)
                fields.append((sh << BIAS_BITS) | (b & ((1 << BIAS_BITS) - 1)))
            bl.append(pack(fields, SH_BITS + BIAS_BITS))
        (out / f'{name}_w{j}.mem').write_text('\n'.join(wl) + '\n')
        (out / f'{name}_bs{j}.mem').write_text('\n'.join(bl) + '\n')


def write_combined_mems(all_layers, out):
    """Compact shared-engine tables for LANES=16 dual-actor RTL.

    w2_comb.mem  : 512 x 32b ({w2[row1, k], w2[row0, k]} for inner 0..255, outer 256..511)
    bs_comb.mem  : 64 x 480b (16 lanes x {shm1[4:0], bias[24:0]} for {actor, layer0/1, lr})
    bs2_comb.mem : 2 x 60b   (2 output lanes x {shm1[4:0], bias[24:0]} for inner/outer layer2)
    """
    w2_lines, bs_lines, bs2_lines = [], [], []
    for name in ('inner', 'outer'):
        layers = all_layers[name]
        for j in (0, 1):
            L = layers[j]
            for lr in range(H // 16):
                fields = []
                for l in range(16):
                    r = lr * 16 + l
                    shm1 = int(L['sh'][r]) - 1
                    if not (7 <= shm1 <= 30):
                        raise ValueError(f'{name} layer {j} row {r}: sh-1={shm1} out of [7, 30]')
                    b = int(L['b'][r])
                    fields.append((shm1 << BIAS_BITS) | (b & ((1 << BIAS_BITS) - 1)))
                bs_lines.append(pack(fields, 5 + BIAS_BITS))
        L2 = layers[2]
        for k in range(H):
            w2_lines.append(pack([L2['w'][0, k], L2['w'][1, k]], 16))
        f2 = []
        for r in (0, 1):
            shm1 = int(L2['sh'][r]) - 1
            if not (7 <= shm1 <= 30):
                raise ValueError(f'{name} layer 2 row {r}: sh-1={shm1} out of [7, 30]')
            b = int(L2['b'][r])
            f2.append((shm1 << BIAS_BITS) | (b & ((1 << BIAS_BITS) - 1)))
        bs2_lines.append(pack(f2, 5 + BIAS_BITS))
    (out / 'w2_comb.mem').write_text('\n'.join(w2_lines) + '\n')
    (out / 'bs_comb.mem').write_text('\n'.join(bs_lines) + '\n')
    (out / 'bs2_comb.mem').write_text('\n'.join(bs2_lines) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--checkpoint', type=pathlib.Path, default=PAIR / 'selected_checkpoint.npz')
    ap.add_argument('--vectors', type=pathlib.Path, default=PAIR / 'golden_vectors.npz')
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    with np.load(a.checkpoint, allow_pickle=False) as d:
        weights = {k: d[k].copy() for k in d.files if k != 'metadata_json'}
    report = {'checkpoint_sha256': hashlib.sha256(a.checkpoint.read_bytes()).hexdigest(),
              'vectors_sha256': hashlib.sha256(a.vectors.read_bytes()).hexdigest(),
              'activation_frac_bits': FA, 'activation_max': ACT_MAX, 'tanh_entries': TANH_N,
              'actors': {}}
    (a.out / 'tanh.mem').write_text('\n'.join(hexw(v, 16) for v in TANH) + '\n')
    all_layers = {}
    with np.load(a.vectors, allow_pickle=False) as g:
        for name, (nin, lanes, bits) in ACTORS.items():
            layers = quantize_actor(weights, name)
            all_layers[name] = layers
            write_mems(layers, name, a.out)
            xi = quantize_input(g[f'{name}_input'])
            y, z = forward_int(layers, xi)
            err = np.abs(y / 32768.0 - g[f'{name}_output'])
            np.savetxt(a.out / f'{name}_x.mem', xi.reshape(-1) & 0xFFFF, fmt='%04x')
            np.savetxt(a.out / f'{name}_y.txt', y, fmt='%d')
            report['actors'][name] = {
                'inputs': nin, 'lanes': lanes, 'weight_bits': list(bits), 'vectors': len(xi),
                'shift_ranges': [[int(L['sh'].min()), int(L['sh'].max())] for L in layers],
                'max_abs_error_vs_fp32': float(err.max()), 'mean_abs_error_vs_fp32': float(err.mean()),
                'max_abs_preact_q14': int(np.abs(z).max()),
                'cycles_estimate': (H // lanes) * nin + (H // lanes) * H + H}
    write_combined_mems(all_layers, a.out)
    (a.out / 'quant_report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
