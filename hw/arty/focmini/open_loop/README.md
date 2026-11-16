# First motor-drive stage: bounded open-loop field sweep

Separate physical top `open_loop_top`, Arty A7-35T, 100 MHz. Reuses the tested
AS5600 reader, A0 XADC acquisition and UART transmitter from `../sensor_diag`.
Sensor-only recovery firmware remains unchanged. No RL or ESP32 changes here.

**Implemented, not yet physically motion-tested:** three 20 kHz PWM inputs,
driver wake/reset sequence, arm/start/stop commands, 10-second one-shot timer,
keepalive and fault handling. Build results are recorded per immutable run.
This is voltage open-loop commutation, not current-regulated FOC or position control.
One ACS724 supplies raw telemetry; no calibrated overcurrent threshold exists.

## Fixed bench profile

The reference ESP32 test used a nominal 9 V bus and 0.6 V field amplitude.
`make_lut.py` creates 256 entries with
`duty[k] = round(5000 * (0.5 + (0.6 / 9) * sin(theta + offset[k])))`,
offsets `[0, -120, +120]` degrees. Average line-line voltages follow the
phase differences. Actual voltages scale with the real bus; there is no bus ADC.
13-bit duty counts commit together at the start of a 5000-clock PWM period.
A 32-bit phase accumulator advances by 1227134 per millisecond (one electrical
revolution in 3500 steps, rounding error <0.00015 degrees per revolution).
Eight high phase bits address the table, so angular quantization is 1.40625 degrees.

| Time from start | Electrical field |
|---|---|
| 0–1 s | Hold at zero |
| 1–4.5 s | One revolution forward |
| 4.5–5.5 s | Hold at zero |
| 5.5–9 s | One revolution backward |
| 9–10 s | Hold at zero, then disable |

This is **not** a full mechanical rotation. With seven actual pole pairs and
successful following, each electrical revolution corresponds to about 51.4
mechanical degrees. The initial hold may move the shaft to a nearby field position.
Pole pairs, electrical zero, direction and tracking are not established by this test.

## Commands and shutdown

115200 baud, 8-N-1. Uppercase exact ASCII lines, CR or LF terminated:

- `ARM`: from IDLE only; SLEEP high/reset low for 10 ms, then reset high for
  another 10 ms; check sensors/fault before ARMED. EN remains low. Expires in 5 s.
  ARM during a non-fault active state disarms it; cannot restart a sweep.
- `SWEEP`: only from ARMED. Starts at a PWM boundary. Repeats cannot extend motion.
- `PING`: required at least every 500 ms during RUN; host sends with each 10 Hz packet.
- `STOP`: disable. `!` disables without waiting for a newline; discard remainder
  of that line. BTN0 also resets/disarms. Unknown/overflow/framing-damaged/partial
  lines cannot reveal a valid command suffix. Interbyte timeout is 100 ms.

Any nFAULT low pulse immediately gates drive outputs and sets a sticky capture;
the state machine observes the capture through a synchronizer. FPGA fault latches
require BTN0/reset, never automatic retry. Pulse capture has physical device
minimum pulse-width limits; RTL sub-clock tests are not an oscilloscope proof.
Driver wake delay exceeds TI's approximately 1 ms wake guidance
([DRV8313 datasheet, section 7.4.1](https://www.ti.com/lit/ds/symlink/drv8313.pdf)).

Further RUN/ARMED abort conditions: stale or clipped ADC; failed I2C, missing
magnet, excessive field or encoder age >=100 ms; RUN keepalive expiry; UART
framing error. During RUN, wrapped sample step >228 encoder counts (~20 degrees)
or accumulated displacement >1303 counts (~114.5 degrees) also aborts.
These angle checks depend on the unvalidated sensor and are not a physical travel
limit. AS5600 is polled at 100 Hz; I2C transactions time out after 10 ms.

Weak-field ML is deliberately allowed and reported for this open-loop test.
The prior manual-turn capture had weak field throughout and one missing-magnet
sample: that condition will stop this image. No assertion that weak readings are
accurate. Lost magnet cannot be overridden with UART commands.

## Connections and observations

Same physical wiring as `../board/README.md`. FPGA UART TX **D10**, RX **A9**.
JA1/2/3 PWM, JA4 enable, JA7 reset, JA8 sleep, JA9 fault; JB1 SCL, JB2 SDA;
A0 the single ACS724 divider (18k upper +18k lower). A1 unused. Current sensor
5 V from Arty 5V0; AS5600/Pmod logic 3.3 V; common ground.

Boot, IDLE, FAULT and reset drive all six motor controls low. The firmware does
not govern pins before FPGA configuration; check EN low before applying motor power.
Fault protection in the driver is not a calibrated motor-current limit. The first
physical test requires a current-limited 9 V source and a clear shaft. The 0.6 V
setting is a duty calculation, not permission to raise the supply above 9 V.

LD4 heartbeat 5 Hz; LD5 ADC fresh; LD6 last I2C transaction passed; LD7 magnet
healthy (MD and neither ML nor MH). LD7 can stay off for the current weak magnet.

83-byte telemetry at 10 Hz (all values hexadecimal):
`O1 a=4B6 v=1 c=0 i=1 e=0 s=37 r=254 b=3 f=0 m=0 q=0 n=0000 t=0000 p=00 g=0 x=0000\r\n`

`a` raw 12-bit ADC, `v` fresh, `c` clip, `i/e` last I2C OK/error, `s` AS5600 status,
`r` raw angle, `b` SCL/SDA, `f` synchronized fault_n; `m` state, `q` result,
`n` syntactically accepted command count (not necessarily executed), `t` elapsed
milliseconds, `p` electrical phase table index, `g` enable logic, `x` rejection
counter. Both counters wrap at 65535. ADC is not amperes. Telemetry is a snapshot;
short-lived startup states need not appear in every capture.

States: 0 IDLE, 1 WAKE, 2 SETTLE, 3 ARMED, 4 STARTING, 5 RUN, 6 FAULT.
Results: 0 none, 1 completed, 2 stopped, 3 arm expired, 4 driver fault, 5 ADC,
6 encoder, 7 angle excursion, 8 keepalive, 9 UART.

## Reproduce and stage hardware work

From repo root:

```sh
python3 hw/arty/focmini/open_loop/run.py --build
```

Each run snapshots both folders, records source/git/submodule hashes, versions,
raw logs and LUT error metrics, then tests UART/parser, full controller schedule
and faults, a top-level UART/I2C exchange, and both sensor-diagnostic regression
suites. Tests scale timing for runtime; neither XADC mock nor static encoder
model simulates analog accuracy or motor dynamics. Vivado routes the actual
100 MHz/20 kHz design and checks timing, DRC, all package pins and fault kill paths.
No programming occurs in the runner. The intended asynchronous sticky fault latch
produces a CDC-7 finding at `control/fault_capture_reg/PRE`; six other external
inputs are reported as two-stage synchronizers. The capture directly disables
outputs, while state transitions observe its two-stage synchronized value.
Its clear path is enabled only while disarmed/startup or reset. This finding is
reviewed explicitly in each run's `review.json`, not hidden with a rule downgrade.
Physical short-pulse capture still needs validation.

Use the run's snapshot of `../sensor_diag/program.tcl` with the run's
`open_loop.bit` and USB serial `210319A43D3EA` for volatile programming only,
with motor power disconnected. That generic programmer retains its historical
sensor-diagnostic completion label; the supplied path determines the image.
Then use the snapshot host script with the stable USB serial port:

```sh
python3 RUN/source/open_loop/bench.py --run RUN --port PORT --mode check
```

This sends STOP and one unarmed SWEEP; it never sends ARM or raises outputs.
Logs verify state/command acknowledgments, not physical output voltages. After
checking EN low and the current-limited 9 V setup with the operator, one trial is:

```sh
python3 RUN/source/open_loop/bench.py --run RUN --port PORT --mode sweep --motor-power-ready
```

The host starts exactly once, logs every received byte and command, stops on
invalid/missing telemetry, and sends `!` on exit. FPGA has its own keepalive and
10-second limits if the host disappears. Completion means the commanded field
sequence completed, not that the shaft followed it. Preserve aborted trials.
After a fault, investigate before BTN0 and a separate new trial. No automatic retry.

Still remaining after this stage: physical PWM/enable and one powered sweep,
current offset/gain/polarity measurement, reliable rotor-angle characterization,
electrical alignment/direction, then voltage-mode closed-loop work. The existing
two-current FOC core requires a second phase measurement or a separately designed
and verified reconstruction scheme; it is not connected to this one-sensor top.
