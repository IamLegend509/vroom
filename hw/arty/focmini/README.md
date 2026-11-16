# FOCMini FPGA motor controller

Start with [the hardware handoff](HANDOFF.md) for the latest physical-test
outcome, MacBook continuation steps, and the distinction between built and
hardware-accepted images.

Synthesizable SystemVerilog classical FOC baseline for the DFRobot DRI0058
(DRV8313), using the physical setup in `IamLegend509/vroom:focMini`, commit
`44ffd63ab6ef8176c3fe562acc01b7aa8ca1bca6`. RTL tests and a Vivado routed
registered-boundary fixture pass at 100MHz on the actual 35T part. The nominal
closed-loop motor fixture passes; a heavier assumed motor misses one tracking
gate. The separate sensor diagnostic and open-loop images have now been tested
on the Arty, including visible forward/back motor movement. That does not yet
validate this two-current FOC core on hardware.
See [verification results](VERIFICATION.md) for measurements and scope.

The new [bounded position controller](position_loop/README.md) adds AS5600
feedback to the physical three-phase drive for a 51-second ±5° test. It adjusts
field phase to correct mechanical position, using the same nominal 0.6 V drive.
It is not dq FOC or current regulation. The verification log distinguishes its
simulation, implementation and physical-test status.

For visible motion, [the slow 90° sweep](open_loop_90/README.md) commands about
90 mechanical degrees over 12 seconds, holds for five seconds, then returns
over 12 seconds. This separate O2 image uses open-loop commutation and the same
nominal 0.6 V amplitude; actual travel is measured from the encoder afterward.

The top is `rtl/focmini_driver.sv`. It contains startup/interlocks, a velocity PI,
two current PIs, Clarke/Park and inverse transforms, centered sine modulation,
and atomic three-phase PWM. It accepts coherent calibrated sensor frames;
integration of the direct XADC/AS5600 acquisition into this two-current FOC core
remains pending. The diagnostic, open-loop and position-loop board tops use the
direct sensors separately, with the currently connected single current sensor.
[The board plan](board/README.md) now assigns JA driver pins, JB I2C pins and
A0/A1 current inputs for the Arty A7-35 Rev D/E, checked against Digilent's XDC.

## Parameters and origin

| Setting | Value | Origin |
| --- | --- | --- |
| Supply | 9 V | ESP32 Arduino source; operator-reported measurement |
| Pole pairs | 7 | ESP32 Arduino source; operator-verified |
| PWM | 20 kHz, three inputs N1/N2/N3 | ESP32 Arduino source |
| d/q voltage limits | each +/-1.50 V | SimpleFOC current PID output limit from `motor.voltage_limit` |
| q-current target | +/-0.5 A; d target = 0 | ESP32 `foc_current` profile |
| Mechanical speed target | +/-5 rad/s | ESP32 source |
| Current measurement | ACS724-05AB, 400 mV/A, two phases | ESP32 source; **not ACS714** |
| Divider in ESP32 reference circuit | 10k upper / 18k lower | ESP32 source; **not 10k/10k** |
| Rotor feedback | AS5600, 4096 counts/revolution | ESP32 source |
| Current PI | Kp=3, Ki=300, D=0 | pinned SimpleFOC 2.4.0 ESP32 defaults |
| Current filter | tau=1/(2*pi*300) seconds | pinned SimpleFOC 2.4.0 defaults |
| Speed PI / filter | Kp=0.5, Ki=10, D=0; tau=5 ms | pinned SimpleFOC 2.4.0 defaults |
| Current/speed update periods | 100 us / 1 ms | Explicit FPGA design choice |
| Stale frame / target watchdog | 300 us / 250 ms | New FPGA interlocks; configurable |

The Arduino application does not override its PI defaults. Its native IDF
alternative has different voltage/control settings; this port follows the
Arduino `esp32dev_as5600` FOC profile, not that alternative or the 44.4 V/21-pair
simulation plant. See [`profile.json`](profile.json) for the versioned record.

The original app's 0.30 V automatic rotor/current alignment is **not implemented**
here. `calibration_valid` must remain false until a measured electrical zero,
encoder direction and current-channel offsets/mapping/polarities are supplied.
ARM does not run alignment and never energizes the stage on its own.

SimpleFOC bounds each current PID's voltage independently. Accordingly the
1.50 V setting is not a hard bound on `sqrt(vd^2+vq^2)`. Centered phase voltages
are `4.5 + inverse_transform(vd,vq)` V and remain below the 9 V bus after
clamping. This implementation uses fixed point for the conventional controller;
it does not export, quantize or change any RL policy.

## Signal contract

All ports use the single `clk` domain except asynchronous active-low `fault_n`.
`reset` must be asserted at startup; outputs are disabled while reset is high.
Command and configuration levels must be synchronized by the future board adapter.

| Port | Meaning |
| --- | --- |
| `sample_request` | One clock pulse every 100 us; acquisition trigger/reference |
| `sample_valid` | One clock coherent current/angle frame; fixed 10 kHz cadence, +/-5 us tolerance |
| `sensor_ok` | This frame has valid ADC range, encoder/magnet status and transport/acquisition status |
| `phase_a`, `phase_b` | Signed 32-bit Q16.16 amperes, mapped to physical U/V with calibrated polarity; W=-(U+V) |
| `angle_raw` | AS5600 mechanical angle, unsigned 12 bits |
| `electrical_zero` | Electrical offset, unsigned turns/65536; theta = direction*7*angle - offset modulo one turn |
| `angle_reverse` | 1 reverses the mechanical angle and derived velocity direction |
| `calibration_valid` | Qualified configuration and sensor calibration; required to ARM and throughout operation |
| `phase_trip` | Explicit instantaneous phase-current threshold, Q16.16 A; >0 and <=2.5 A. No invented default |
| `arm` | One-clock request; drops any prior target, captures calibration, waits for a fresh nonzero target |
| `target_valid`, `target_velocity` | One-clock speed transaction in mechanical rad/s Q16.16; must be armed, finite integer representation, within +/-5 |
| `stop` | Immediate output gating followed by disarm; zero target also stops/disarms |
| `pwm_a/b/c`, `enable`, `sleep_n`, `reset_n` | DRI0058 N1/N2/N3, EN, SLEEP and RESET logic signals, respectively |
| telemetry ports | Q16.16 mechanical rad/s, filtered id/iq, iq target, vd/vq |

The 0.5 A target limit is separate from `phase_trip`. Select and record the
latter for the actual motor/driver setup. Phase checks use unfiltered U/V/W,
not filtered dq currents. External current/power protection remains necessary
for physical commissioning; this logic only sees the supplied sensor frames.

`focmini_current_scale.sv` converts ADC-calibrated **microvolts at the divider
node**, with independently measured per-channel zero offsets, to Q16.16 A:

```
I_A = (adc_uv - zero_uv) * 7 / 1,800,000
```

It implements the reference 10k/18k divider and ACS724 slope. It is not an ADC
driver. Do not reuse ESP32 ADC code/offset calibration on another converter.
Arty A0-A5 headers provide onboard scaling for 0-3.3V; bare/differential XADC
inputs have different limits. The ESP32 divider may be voltage-compatible with
A0/A1, but the Arty's input network adds attenuation/loading. Use measured
counts/A for the complete Arty path, not this nominal ESP32 voltage helper.

## Startup, commands, PWM and faults

1. Reset holds EN/PWM/SLEEP/RESET low. With reset released, keep EN low, wake
   for 10 ms, pulse RESET low for 10 ms, then settle for 10 ms.
2. Interpret nFAULT only after that sequence. A boot fault prevents motion.
3. With valid calibration and a fresh healthy sensor frame, pulse ARM. The
   gate remains disabled. Send a separate nonzero target to request motion.
4. The first computed duties commit at a PWM period boundary before EN rises.
   All three duties update together; no mid-period waveform changes occur.
5. Refresh the target transaction within 250 ms during motion. STOP/T0 disarms;
   starting again requires ARM followed by a fresh nonzero target.

A low nFAULT directly gates EN/SLEEP/PWM and asynchronously sets a capture
flip-flop, retaining pulses shorter than one core clock. A two-register
synchronizer feeds the sticky status latch. Other trips cut drive on the
following clock. Faults require reset; ARM cannot clear them. External EN
pull-down is required to hold the power stage off while the FPGA is unconfigured.
This controller emits three PWM signals; DRV8313 performs the half-bridge gate
drive/dead time. It does not emit six MOSFET gate signals.

| Fault code | Reason |
| --- | --- |
| 1 | DRI nFAULT, including a captured sub-clock pulse |
| 2 | Invalid sensor frame |
| 3 | U, V or reconstructed W exceeds the configured phase trip |
| 4 | Sensor frame timeout while armed |
| 5 | A frame overlaps unfinished control computation |
| 6 | Target transaction watchdog expires during motion |
| 7 | Calibration lost or configuration changes while armed |
| 8 | Sensor cadence outside the fixed-sample tolerance |

## Equations and numerical limits

Currents/voltages/speed use Q16.16 with saturating arithmetic. Transform and
current-filter multipliers use 20-bit values and 18-bit factors (the same 16
fractional bits), fitting one DSP48E1 each. There is no additional rounding.
At the top's maximum 2.5A phase trip, |alpha|<=2.5A, |beta|<=2.887A and
the dq current vector is <=2.887A; current-filter errors are <5.774A. These
fit the multiplier's [-8,8) value range. Voltage axes are bounded to 1.5V
and transformed voltages to 2.122V. The driver cancels out-of-range raw-current
jobs before those transforms can execute. Standalone core users must provide
the same raw-current protection; `focmini_driver` is the supported motor top.
Current PI errors use 22 bits (range +/-32A); the velocity PI retains 32 bits.
The velocity product, PI updates and PWM conversion have separate registered
steps. Three parallel 20-step unsigned dividers preserve exact PWM integer
division without a long combinational division path.
Sine/cosine use a 16-step rotation CORDIC with Q2.30 outputs and electrical
turns/65536 input. The Clarke convention is alpha=ia, beta=(ia+2*ib)/sqrt(3).
Park is id=alpha*cos+beta*sin, iq=beta*cos-alpha*sin. Inverse transforms use
the same phase order and centered SinePWM as SimpleFOC's default modulation.

Each PI uses trapezoidal integration: integral += Ki*Ts/2*(error+previous),
with both integral and output clamped independently, as the source PID with D=0.
The FPGA's fixed coefficients and sample schedule are explicit design choices;
this is not a bit-for-bit ESP32 float or variable-timestep implementation.

Mechanical velocity uses wrap-aware differences over ten 10 kHz frames, then
a 5 ms first-order filter. The first frame initializes the prior angle. The
12-bit encoder gives about 1.534 rad/s per count per 1 ms before filtering;
low-speed ripple/latency must be evaluated with the actual encoder acquisition.
Current filtering uses Ts/(tau+Ts). Histories, filters, integrals and pending
duties clear when motion stops. The FPGA sample clock is not evidence that an
I2C acquisition can deliver a fresh angle every 100 us: adapter throughput,
latency and coherent current/angle timestamps are unresolved board work.

## Reproduce tests

From the repository root, with Python 3, Icarus Verilog and Verilator:

```sh
python3 hw/arty/focmini/tests/run.py
# Adds actual RTL PWM -> assumed motor ODE -> sensors -> RTL closed-loop tests.
python3 hw/arty/focmini/tests/run.py --plant
# Synthesis, placement and routing of the registered diagnostic fixture.
python3 hw/arty/focmini/tools/build.py --vivado /path/to/vivado
```

The runner writes unique ignored `results/arty_focmini/` directories with raw
logs, immutable source snapshots/hashes, Git/submodule/tool identity,
floating-point vectors, and `metrics.json`. Failed runs remain and exit nonzero.

The base regression passes. The `--plant` regression currently returns nonzero
because the heavier fixture misses its reversal tracking gate; it is not a
successful qualification run. The build checks register-to-register paths,
including fixture launch/capture registers, and never creates a bitstream.
Physical IO constraints are absent and are explicitly outside its timing status.

Tests cover every 16-bit CORDIC angle, 10,001 bidirectional sensor-conversion
points, 65,536 bounded-multiply comparisons and 768 controller frames against an independent float model,
quadrants/offsets/reversal/wrap, PI and voltage limits, pipeline reset/overrun,
PWM atomic updates and high times, startup, ARM/STOP/T0, malformed targets,
sub-clock faults, third-phase overcurrent, bad/stale samples, calibration loss,
and command watchdog/refresh. Cancellation is checked at all 76 pipeline
offsets. Measured core latency is 67-74 clocks (0.67-0.74us at 100MHz), separately from
PWM commit delay, which can add up to one 50 us carrier period. Numerical tests
do not establish physical loop stability or real motor performance.

## Remaining board integration

- Implement the [assigned board interface](board/README.md): built-in XADC on
  A0/A1, AS5600 I2C on JB, driver signals on JA; add acquisition/synchronization
  and verify the physical board revision and complete IO timing.
- Measure current offsets/slopes, U/V mapping and polarity, encoder direction
  and electrical zero. Add a bounded operator-triggered alignment procedure.
- Prove sensor throughput/latency, then place/route the complete board top with
  actual pin/IO constraints; verify PWM/EN/FAULT on a scope before motor operation.
- Tune/validate the PI baseline on the measured motor, preserve traces, and
  only then add the RL observation/action adapter. The existing RL actor remains
  separate from this conventional baseline.

Source references:
[ESP32 implementation](https://github.com/IamLegend509/vroom/blob/44ffd63ab6ef8176c3fe562acc01b7aa8ca1bca6/hw/focmini/src/main.cpp),
[SimpleFOC 2.4.0 defaults](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/defaults.h),
[PI arithmetic](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/common/pid.cpp),
[modulation](https://github.com/simplefoc/Arduino-FOC/blob/v2.4.0/src/BLDCMotor.cpp).

## Single-sensor hardware diagnostic

See [sensor_diag](sensor_diag/README.md) for the separate physical Arty top with
A0 XADC, AS5600 I2C, and USB UART telemetry. It has no motor control and holds
all driver controls low. This does not complete the two-current FOC acquisition
or calibration/alignment work described above.

## First motor-drive stage

[open_loop](open_loop/README.md) adds a separate physical top for a fixed 0.6 V
nominal field sweep at a 9 V bus. It requires explicit ARM and SWEEP commands,
limits the profile to ten seconds, and disables on faults, invalid/stale sensors
or lost host keepalive. The single ACS724 remains raw telemetry. This stage
does not connect the two-current FOC core or establish electrical alignment.
Build and physical-test evidence are recorded separately under
`results/arty_open_loop`; see the open-loop source README for reproduction and limits.

## Driver wake isolation

[driver_diag](driver_diag/README.md) is a separate diagnostic image for checking
RESET/SLEEP/nFAULT after the arming failure. It structurally ties all three PWM
pins and EN low, exposes live nFAULT on LD6, and permits a bounded wake-only probe
independent of encoder status. Its D1 protocol is distinct from the motion
image's O1 protocol. Installing this image removes motor-drive capability until
the motion image is deliberately restored.
