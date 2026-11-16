# Slow visible shaft sweep (O2)

A bounded open-loop demonstration requested after the small closed-loop position
trials were difficult to see. This is **not an accepted closed-loop position
controller**. It uses the proven O1 0.6 V modulation table and physical pins.
No current-regulation, electrical calibration or RL is added.

## Motion

At an assumed 9 V bus and seven pole pairs, 1.75 electrical turns correspond to
90 mechanical degrees. Actual travel must be read from encoder telemetry and
observed on the shaft; friction, load, weak magnetic field and the unvalidated
pole-pair assumption can change the result.

| Time | Drive command |
| --- | --- |
| 0–3 s | Hold field phase zero; rotor may align |
| 3–15 s | Decrease field phase by 1.75 electrical turns (expected positive encoder movement) |
| 15–20 s | Hold at the far end for five seconds |
| 20–32 s | Reverse the field trajectory back to phase zero |
| 32–35 s | Hold, then automatically disable |

Nominal shaft speed is 7.5 degrees/second. The 32-bit field accumulator changes
by 626349 counts/ms during movement. This approximates 1.75 turns in 12000 ms;
endpoint rounding is below 0.0004 electrical degree. Modulo phase at the far hold
is 0x40000000. Three-phase PWM remains 20 kHz and nominal 0.6 V amplitude.
The duty ROM is byte-identical to O1; no voltage increase or wiring change.
The bench supply is 9 V with a 0.10 A limit.

## Safety and interface

Boot/reset disarmed; ARM wakes the driver but does not enable PWM. Exact SWEEP
starts one 35-second sequence only after ARM. PING is required within 500 ms;
STOP or ! disables immediately. Repeated SWEEP cannot extend the timer, and ARM
during motion stops it. ARM expires after five seconds. Faults require BTN0 or
reset; no automatic retry. All six driver controls are low when idle or faulted.

The existing sensor and motion interlocks are retained: fresh unclipped ADC,
valid AS5600 reads younger than 100 ms, magnet detected, no excessive field,
maximum per-sample jump 228 counts (~20 degrees), and accumulated movement from
start within 1303 counts (~114.5 degrees). Weak-field ML is permitted and logged;
encoder accuracy remains unvalidated. No calibrated overcurrent trip exists.
The direct nFAULT kill and sticky capture remain, with the intentional CDC-7
preset warning reviewed rather than hidden. Movement-limit subtraction is
registered, adding 10 ns to that check. No multicycle timing exceptions.

Protocol is O2, 83 bytes at 115200 8N1, 10 Hz. Fields match O1; elapsed t is now
16 bits and ends at 0x88B8 (35000 ms). O1/C1/C2/C3 host programs cannot start O2.
LEDs and pins are unchanged: JA1/2/3 PWM, JA4 EN, JA7 RESET, JA8 SLEEP, JA9 nFAULT,
JB1 SCL, JB2 SDA, A0 current ADC, FPGA UART TX D10 / RX A9.

## Verification and operation

```
python3 hw/arty/focmini/open_loop_90/run.py
python3 hw/arty/focmini/open_loop_90/run.py --build
```

Tests cover the full 35000 ms profile and endpoint/midpoint phases, bounded
atomic PWM, all faults/stop paths, wraparound/travel limits, UART command parser,
independent I2C/UART top integration and host completion/fault/dropout behavior.
Each run retains immutable sources/hashes, tool logs, raw traces and metadata.
Builds do not program hardware. Previous failures remain preserved.

Use the built snapshot's bench.py with --run BUILD --port SERIAL. Default
--mode check never sends ARM. A single --mode sweep --motor-power-ready is only
for a coordinated test with the motor secured and shaft clear. The host logs all
traffic and sends STOP in cleanup. Completion proves the timed command sequence,
not a measured 90-degree position or a closed-loop tracking result.

C3 closed-loop source and its separate build are preserved in position_loop;
its longer-hold revision has not been physically tested. C1 and C2 failed full
tracking acceptance and remain recorded in VERIFICATION.md.

## Physical outcome (2026-11-16)

Two O2 runs completed the timed 35-second command sequence, with no active
driver fault or invalid/stale/clipped sensor packets. Both had weak magnetic
field throughout. The first reported +34.89° from initial to far hold and
another +3.60° from far to final hold. The requested repeat reported -2.64°
to the far hold and +2.81° back; its transient range was about -13.89° to +0.26°.
The operator saw the shaft mostly in one place with perhaps 5° of movement.
**A 90° shaft sweep was not demonstrated.** Timed host success is not motion
acceptance. Cause remains unproven; do not treat this as a calibrated controller.
