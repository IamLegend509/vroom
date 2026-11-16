# RL actor inference RTL (Arty A7-35T)

Fixed-point hardware inference of the two learned vroom actors from
`Vroom_2805_Experimental_Controller_Pair/selected_checkpoint.npz`
(SHA256 `f2a8b76d…0b41`): inner 25→256→256→2 (10 kHz), outer 19→256→256→2 (1 kHz),
Linear/ReLU/Linear/ReLU/Linear/tanh, 144,388 parameters.

Status: **implemented and simulation-tested.** Device parity (P1) is pending a build
plus an operator-approved volatile programming step. No closed-loop motor control
uses these actors yet. The motor, sensing and drive are assumed rather than measured;
see the package README.

## Number contract (`quant.py` is the specification)

| Quantity | Format |
| --- | --- |
| Observation input | int16, Q.14, already clipped to [-1, 1] |
| Weights | per-row power-of-two scale `W = w·2^e_r`, symmetric; inner 16/16/16 bits, outer 16/9/16 bits |
| Accumulator | exact Σ w·a, < 2^47, DSP48 P register |
| Rescale | round-half-up `((acc >>> (sh-1)) + 1) >>> 1`, `sh = -e_r` |
| Hidden | + bias (Q.14), ReLU, saturate at 2^24−1 (25-bit) |
| Pre-tanh | saturate ±(2^24−1) |
| Output | tanh: 513-entry table at 1/64 steps + linear interpolation, int16 Q.15 |

The bit widths are a declared trade between BRAM and accuracy. Inner W1 at 8 or 9
bits gave 0.17–0.5 maximum output error against FP32, so the inner actor keeps 16 bits
throughout. The outer actor tolerates 9-bit W1.

Measured against the FP32 golden vectors (518 per actor):

| Actor | Max abs error | Mean abs error | Cycles @100 MHz |
| --- | --- | --- | --- |
| inner | 0.00560 | 0.00025 | 2,535 (25.4 µs of the 100 µs period) |
| outer | 0.00906 | 0.00074 | 4,687 (46.9 µs of the 1 ms period) |

The RTL output equals the `quant.py` integer model on every word: 0 mismatches
over 518 × 2 × 2 outputs (iverilog).

## Architecture (`rl_mlp.sv`)

The engine is a lock-step SIMD MAC array: inner has 32 lanes, outer 16. Lane `l` owns
hidden rows `r % LANES == l`. Each cycle every lane reads its slice of one wide
weight word and multiplies it by one broadcast activation. The pipeline runs:
issue → BRAM read → BRAM output register → multiply → accumulate → per-row
shift → bias/ReLU/saturate → write one wide hidden word. Layer 2 reuses lanes 0/1.
Hidden activations live in wide LUTRAM. The two actors are independent instances.

## P1 parity image (`rl_parity_top.sv`)

This image holds the motor-interface pins (`pwm_a/b/c`, `enable`, `reset_n`, `sleep_n`)
at constant LOW, so the DRI0058 stays asleep with its gates off. UART runs at 115200 8-N-1:

```
host → A5, actor(0 inner/1 outer), N×int16 LE (25/19), xor
FPGA → 5A, actor, y0 i16, y1 i16, z0 i32, z1 i32, cycles u16, xor      (17 B)
       EE, code(1 actor, 2 checksum), xor                                (error)
```

LEDs: LD4 heartbeat, LD5 toggles per reply, LD6 error latched, LD7 actor busy.

## Reproduce

```sh
python3 hw/arty/focmini/rl_infer/run.py                                   # export + full sims
python3 hw/arty/focmini/rl_infer/run.py --build-host lazybanana@10.249.214.176   # + Vivado 2024.2 remote
# volatile program (motor supply off), then:
python3 hw/arty/focmini/rl_infer/parity_host.py --run results/arty_rl_infer/<run> \
    --port /dev/cu.usbserial-210319A43D3E1
```

`run.py` never programs a device. Every run keeps its source snapshot, memories,
reports, logs and bitstream under ignored `results/arty_rl_infer/`.

## Limits

- Parity covers only the feed-forward networks. Controller I/O (histories, IASA
  adapter, action transform, sensing, timing, resets) is not implemented in RTL yet.
- The Arty has one current sensor, so the inner actor's id/iq observation cannot be
  measured directly. Any closed-loop use needs a declared estimator, which is a
  controller change and has not been validated against the trained contract.
