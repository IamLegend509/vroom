# Driver wake diagnostic — motor outputs permanently low

This separate physical image isolates RESET/SLEEP/nFAULT behavior after the first
open-loop arming attempt failed. The next user-requested retry then failed its
magnet precheck (AS5600 status 0x17) before ARM. Neither attempt produced a PWM sweep.
The original motion firmware and its protections remain unchanged.

`driver_diag_top` has **no PWM generator and no motor-enable logic**. IN1, IN2,
IN3 and EN are literal constants 0, independent of commands, sensors or reset.
Vivado must prove that each of those four output buffers is driven by GND before
it can produce a diagnostic bitstream. This proves FPGA logic, not wire identity
or voltage at the physical motor driver. Motor power stays off during programming.

An ARM command starts a bounded wake probe: SLEEP high with RESET low for 10 ms,
then both high until ten seconds after ARM, then both low. SWEEP always rejects.
An additional ARM while active stops the probe; it never restarts the timer.
STOP or `!`, BTN0, UART framing error and a 500 ms keepalive loss turn both pins low.
PING resets the keepalive timer. No autonomous retries or automatic startup.

The diagnostic **observes** nFAULT, encoder status and raw ADC. They do not gate
wake-only operation, because the four motor-drive outputs are structurally low.
This does not authorize ignoring those signals in any image capable of motion.
Driver nFAULT may read low while asleep/unpowered; its response while awake is
what this test is designed to distinguish. Do not infer the cause from idle alone.

## Wiring and indication

Existing wiring remains: JA1/2/3 -> IN1/2/3, JA4 -> EN, JA7 -> RESET,
JA8 -> SLEEP, JA9 <- FAULT. Common ground; logic 3.3 V. Driver VCC/VM is the
9 V motor supply. Sensor connections remain unchanged.

LED mapping **changes in this diagnostic image**:

- LD4: 5 Hz heartbeat.
- LD5: diagnostic wake output active (SLEEP commanded high).
- LD6: live synchronized FAULT input; on means high.
- LD7: last AS5600 I2C read successful; it does not mean magnet healthy.

UART is 115200 8N1. Packets use **D1**, not the motion image's O1. Field order
matches O1; `m` is 0 IDLE, 1 WAKE/reset held low, 2 AWAKE/reset released.
`g` and `p` are always zero. `t` is milliseconds since ARM. `q`: 0 none,
1 finished, 2 stopped, 8 keepalive expired, 9 UART error. `f` is the live nFAULT
pin level, independently of state. `s` is raw encoder status. ADC is not amperes.
An exact 83-byte packet is emitted every 100 ms. Captured logic commands do not
prove voltage at the driver: meter measurements use a common Arty ground.

## Reproduce

```sh
python3 hw/arty/focmini/driver_diag/run.py --build
```

The runner snapshots this folder and its shared sensor/UART source folders in a
unique `results/arty_driver_diag` run, records hashes/versions, runs controller,
physical-top, UART/I2C and host pseudo-terminal tests, then optionally runs
Vivado 2024.2 on xc7a35ticsg324-1L. It checks all pin constraints, five input
synchronizers, four constant-GND motor outputs, routed timing and DRC.
No hardware programming or serial commands are issued by the runner.

Use the snapshot `source/sensor_diag/program.tcl` to program the verified
`driver_diag.bit` while the motor supply is OFF. The generic helper retains its
historical SENSOR_DIAG completion label; its argument and bit hash identify
what was loaded. Perform the default no-wake command check first:

```sh
python3 RUN/source/driver_diag/bench.py --run RUN --port PORT --mode check
```

After the operator enables the 9 V / 0.10 A supply, one diagnostic probe is:

```sh
python3 RUN/source/driver_diag/bench.py --run RUN --port PORT --mode wake
```

This raises only SLEEP/RESET, logs nFAULT for the bounded window, and stops.
Both host and FPGA stop on lost keepalive. Every serial byte and command is
retained. A completed wake sequence is not a pass verdict for driver health:
inspect the count/timing of nFAULT high while AWAKE and any physical pin readings.
The host accepts D1 only; the motion O1 host refuses this image. Do not attempt
motor movement until the motion image is deliberately restored and its driver
and sensor prerequisites are satisfied.
