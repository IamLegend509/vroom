# Bounded encoder position controller (C3)

This implements a **mechanical position feedback loop** on Arty A7-35T using the
AS5600. It adjusts stator field phase in response to measured angle error. It is
an initial low-speed positioning experiment, not rotor-commutated dq FOC, speed
control, current regulation, or an indefinitely enabled servo. RL is untouched.

## Control contract

The proven open-loop three-phase duty table is unchanged: 20 kHz PWM,
`duty = round(5000 * (0.5 + (0.6/9) * sin(field + phase_offset)))` for phase
offsets 0, -120, +120 degrees. Voltage amplitude is nominal 0.6 V at a 9 V bus;
actual voltage is not measured. The bench supply remains limited to 0.10 A.
The one ACS724 supplies raw ADC health checks only; there is no calibrated ampere
reading, software overcurrent trip, or regulated current loop.

After explicit ARM and TEST:

| Time | Action |
| --- | --- |
| 0–2 s | Fixed field at phase zero, allow rotor to settle |
| 2–3 s | Capture and hold that mechanical position |
| 3–15 s | Target +57 encoder counts, approximately +5.01° |
| 15–27 s | Return to starting position |
| 27–39 s | Target -57 counts, approximately -5.01° |
| 15–51 s | Return, then automatically disable |

The last 0.5 s of initial settling needs at least 40 valid samples, each within
12 raw counts (1.055°) of the first sample in that window. The last sample becomes
the mechanical origin. This is **not electrical-zero calibration** and it does
not establish pole-pair count or encoder accuracy. Initial field application may
move the shaft farther than the later ±5° target offsets.

Feedback updates on each valid 100 Hz encoder sample. The relative reference
moves by at most one encoder count/sample (8.789°/s). Error is the signed shortest
12-bit difference `target - angle`. The error contribution has a ±4-count (0.352°) deadband; damping uses the
wrapped change in encoder angle since the previous sample:

```
error_term = 0 if abs(error_counts) <= 4 else -error_counts
delta_phase = clamp(error_term + 64 * measured_step_counts, -192, +192)
phase = clamp(phase + delta_phase, -21845, +21845)
```

Target capture, subtraction, correction, addition and clamping are registered
stages. A new encoder sample reaches the phase register five 100 MHz clocks
(50 ns) later; no multicycle timing exceptions are used. PWM still commits all
three duties together at its next period boundary.

Phase uses signed 16-bit units, 65536 per electrical turn. The high byte indexes
the existing 256-entry table. The phase slew limit is 105.47 electrical °/s, and
the total phase excursion is bounded to approximately ±120 electrical degrees.
The negative feedback direction follows the measured open-loop trial, where
increasing field phase decreased the encoder angle. No motor pole-pair value is
used by this controller. The assumed simulation plant uses seven pole pairs.

Each target's last 0.5 s needs at least 40 samples within ±12 counts (1.055°),
with the reference at the requested endpoint. All four checks must pass for
`completed`. A stationary but plausible encoder therefore cannot pass the test.
Saturation with error above 12 counts for 50 samples also aborts.

## Interlocks and interface

Boot/reset is disarmed. ARM wakes SLEEP for 10 ms with RESET low, releases RESET,
waits another 10 ms, then arms without enabling PWM. ARM expires after 5 s.
TEST starts one 51 s test; repeating it cannot extend the timer. ARM during a
run stops it. STOP or `!` disables immediately. A 500 ms keepalive timeout stops
on lost host control. Faults latch until BTN0/reset; no automatic retry.

The inherited nFAULT path immediately gates drive outputs and asynchronously
sets a sticky capture flip-flop. The state machine consumes its synchronized
value. This deliberate asynchronous preset remains a reviewed CDC-7 report item;
do not claim an entirely clean CDC report or suppress it. ADC stale/clipped,
I2C errors, missing magnet, excessive magnetic field, encoder age ≥100 ms,
single-sample movement >20°, or accumulated movement >114.5° aborts. After
origin capture, displacement >20° from that origin also aborts. AS5600 ML (weak
field) is allowed and recorded; it limits confidence in physical angle accuracy.

Wiring is identical to `../open_loop/open_loop.xdc`: JA1/2/3 PWM, JA4 enable,
JA7 RESET, JA8 SLEEP, JA9 nFAULT, JB1 SCL, JB2 SDA, A0 analog current,
UART TX D10 and RX A9. No power-on automatic movement. Pin constraints are fully
assigned with no DRC downgrades. Do not connect power until the operator is ready.

Exact uppercase commands: ARM, TEST, PING, STOP, `!`; CR or LF terminates a line.
SWEEP is deliberately rejected. Telemetry is the exclusive 114-byte C3 protocol,
115200 8N1, 10 Hz. C2, C1, O1 and D1 host programs cannot operate C3. Existing O1 fields
retain their meanings; elapsed time `t` now reaches 0xC738 (51000 ms). Extra fields:

| Field | Meaning (hex) |
| --- | --- |
| d | 12-bit target encoder angle |
| z | Signed 12-bit wrapped position error |
| o | Signed 16-bit field-phase offset |
| h | Profile stage 0–5 |
| k | Mechanical origin captured |
| y | Four retained settling-pass bits, F required for completion |

States: 0 idle, 1 wake, 2 reset settle, 3 armed, 4 PWM start, 5 active, 6 fault.
Result: 0 none, 1 completed, 2 stopped, 3 arm expired, 4 nFAULT, 5 ADC, 6 encoder,
7 movement limit, 8 keepalive, 9 UART, A initial settling, B tracking failure.
Target/error/phase are only control measurements while `k=1`; they reset after
disable. Result, elapsed time and pass bits remain observable until next ARM.

## Reproduce and inspect

```
python3 hw/arty/focmini/position_loop/run.py
python3 hw/arty/focmini/position_loop/run.py --build
```

Every run saves immutable source hashes, Git/submodule status, exact commands,
tool versions, raw traces, pass/failure logs, JSON metrics and a plot under
`results/arty_position_loop/`. Failure directories remain intact. Builds never
program hardware. The routed bitstream must pass setup/hold timing and DRC.

Tests run the actual RTL with an independent nonlinear damped rotor model,
including friction, a load disturbance, angle wrap, stuck encoder and reversed
direction. That plant is assumed and is not a validated model of this motor.
Separate tests exercise the production PWM period, safety trips, all UART
commands, full I2C/UART top integration, XADC health and host pseudo-terminal
completion/fault/dropout. The host's default `--mode check` never sends ARM.

Run the host from the built source snapshot with `--run BUILD_DIRECTORY --port
SERIAL_PORT`. `--mode test --motor-power-ready` is only for a coordinated powered
trial, after programming with 9 V off and checking disarmed telemetry. It issues
one ARM and one TEST, keeps timestamped raw traffic, and sends STOP in cleanup.
The C2 prefix identifies protocol, not the exact loaded bitstream; retain the
programming log and SHA alongside every hardware attempt.

Initial status: implemented and simulation-tested; physical closed-loop tracking
must be established in a separate recorded bench trial. Full voltage-mode FOC
still needs validated electrical alignment; current-mode FOC additionally needs
suitable phase-current measurements and calibration. See the primary
[SimpleFOC voltage-mode description](https://docs.simplefoc.com/voltage_torque_mode)
and [alignment workflow](https://docs.simplefoc.com/foc_implementation) for that
distinction; this implementation does not claim their FOC behavior.

## C1 hardware development result and C2 revision

C1 (18 seconds, error gain 2, no damping) was programmed and exercised once on
2026-11-16. It captured origin 301.37695°, requested 306.38672°, overshot to
about 309.8°, then latched tracking failure at 6999 ms. All sensor frames were
valid with weak-field status; no driver fault or ADC-health fault occurred.
The operator reported 0.004 A supply current and no significant visible motion;
CV/CC mode was not reported. Encoder readings do not establish independently
measured movement or accuracy. This is a failed development trial, not successful
closed-loop validation. Raw evidence is retained in
`results/arty_position_loop/20261115T235432-1sgyye_r/bench_20261116T000202021328Z/`.

C2 halves error-driven phase integration, adds damping from measured movement,
and extends target holds to 6–7 seconds. Voltage, phase bounds, ±5° targets and
±1.055° settling tolerance are unchanged. A higher-friction assumed plant case
is added for development verification; it is not a fitted or validated motor
model. C2 requires its own built image, disarmed check and powered trial.

C2 passed its first +5° hardware settling check (about -0.35° error), then
failed the return check at 16999 ms with +1.49° last active error. The operator
confirmed small movement out and back and switched 9 V off. C3 retains C2 gains
and all bounds but allows 12 seconds for each target (51 seconds total). Its
settling windows are 14.5–15, 26.5–27, 38.5–39 and 50.5–51 seconds. This changes
settling time, not tolerance. The testbench accelerates one simulated millisecond
to 50 clocks; plant integration and encoder updates remain 1 ms and 10 ms.
