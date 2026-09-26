# DFRobot SimpleFOCmini (DRI0058) + ESP32-WROOM

This folder contains a bounded motor bring-up application for the DRI0058
three-PWM driver. Open-loop operation does not require the optional AS5600.
This firmware is separate from the simulator and does not run an RL policy.

## Wiring

The DRI0058 supply range is 8–30 V, output rating is 2.5 A/channel, logic
accepts 3.3/5 V, and PWM must not exceed 200 kHz. The firmware pin map targets
the original ESP32-WROOM module:

| ESP32-WROOM GPIO | DRI0058 pin | Role |
| --- | --- | --- |
| 25 / 26 / 27 | IN1 / IN2 / IN3 | phase PWM A/B/C |
| 32 | EN | stage enable, active high |
| 33 | RESET | driver reset, active low |
| 14 | SLEEP | stage sleep, active low |
| 34 | FAULT | active-low fault input; external 10 kΩ pull-up to 3.3 V |
| GND | GND | common signal ground |
| 21 / 22 | AS5600 SDA / SCL | optional sensor mode only |

Connect motor phases only to M1/M2/M3 and motor supply only to VCC/GND. Do not
power the ESP32 from the DRI0058 3.3 V output, rated at 10 mA. Use USB or a
properly rated regulator and join signal grounds. GPIO 34 has no internal
pull-up, so fit the external FAULT pull-up. Verify the header order against
your board revision.

```text
Current-limited supply (+) ─────────────── DRI0058 VCC (8–30 V)
Current-limited supply (−) ────────┬────── DRI0058 GND
                                   └────── ESP32 GND
ESP32 GPIO 25 / 26 / 27 ───────────────── DRI0058 IN1 / IN2 / IN3
ESP32 GPIO 32 ──────────────────────────── DRI0058 EN
ESP32 GPIO 33 ──────────────────────────── DRI0058 RESET
ESP32 GPIO 14 ──────────────────────────── DRI0058 SLEEP
ESP32 GPIO 34 ───────────────┬──────────── DRI0058 FAULT
ESP32 3.3 V ─────────[10 kΩ]─┘
Motor phase U/V/W ───────────────────────── DRI0058 M1/M2/M3
ESP32 USB ───────────────────────────────── computer (serial + ESP32 power)
```

The ESP32 holds EN low, raises SLEEP once at boot, waits 10 ms, pulses RESET
low/high, waits another 10 ms, then checks FAULT before and after installing its
IRAM interrupt. TI specifies about 1 ms from nSLEEP rising until the DRV8313 is
operational; the firmware uses 10 ms before RESET and 10 ms after releasing it.
Healthy idle, ARM waiting, STOP, and T0 keep SLEEP high and EN low. Only a
latched hardware fault or startup/calibration failure drives both low. GPIOs
are high-impedance during ESP32 reset; confirm the driver's EN default is off
before applying motor supply. External pull-downs on EN and SLEEP hold that
state through reset.

## Firmware behavior

The Arduino profile has the user-confirmed 9 V supply and seven verified pole
pairs in source. These values were reported by the user; this workspace has not
independently measured the supply or motor. The native ESP-IDF Kconfig defaults
remain zero for bus voltage and pole pairs; its verified local build
configuration has 9000 mV, 7 pole pairs, 500 mV motor amplitude, and 5000
mrad/s. Pole pairs are verified motor data, not the number of poles.

Driver PWM range and motor phase amplitude are separate. The Arduino adapter
sets the driver range to the 9 V bus and the motor amplitude to 1.50 V; the
native ESP-IDF build retains its saved 0.50 V setting. AS5600 alignment uses
0.30 V. For centered sine modulation, the validation requires
`2 * amplitude <= driver range`; alignment amplitude must also be no greater
than the motor amplitude. This prevents clipping of the requested initial
waveform. It does not limit motor current: no phase-current sensor is present.

Startup is disarmed. `ARM` only unlocks commands in open-loop mode; it leaves
EN low while SLEEP stays high. Send a complete `T<rad/s>` command to enable the stage and run
the motor. For the first unloaded spin, the sequence is:

```text
ARM
T1
STOP
```

`STOP` and `T0` both set the target to zero, disarm, and drive EN low while
leaving the driver awake so FAULT remains observable.
Commands accept a finite signed target within ±5 rad/s and require a complete
line ending (CR, LF, or CRLF). Incomplete, overlong, malformed, or out-of-range
lines leave the prior target unchanged. Input parsing is limited to 64 bytes per
iteration. Avoid flooding commands because serial logging can still delay
control updates.

With AS5600 mode selected, `ARM` energizes the stage only for `initFOC()`
alignment. A failed alignment latches the shutdown until reboot; successful
alignment leaves the gate disabled while the armed controller waits for a
nonzero target.
Open-loop mode skips calibration. Both modes call `loopFOC()` and `move()` on
each active control iteration.

FAULT is active low. The DRI0058 schematic pulls FAULT to the driver's V3P3
rail. Firmware does not interpret the pin during sleep or the wake transition;
it checks the line after the wake/reset delays, then checks again with the ISR
installed. A low level after that settling sequence is treated as a real fault
and latches shutdown. On the original ESP32-WROOM, a falling-edge ISR uses
IRAM-resident direct GPIO register writes to clear both external EN and SLEEP,
including while alignment blocks the application loop. The fault latch cannot
be cleared by serial commands. Hardware FAULT, driver initialization failure,
motor initialization failure, or calibration failure requires inspection and
a reboot. The IDF adapter installs its own IRAM ISR service; Arduino does the
same instead of relying on the core's optional flash-resident dispatcher. The
direct GPIO bank mapping is guarded to the original ESP32 target.

These interlocks are software bring-up controls, not a safety system. Use a
current-limited supply, an unloaded motor, and a physical disconnect. The
firmware cannot enforce the DRI0058 current rating, detect all wiring faults, or
regulate speed under load in open-loop mode.

## Build

Build only; these commands do not upload or operate hardware. Run from the
repository root. PlatformIO Core 6.1.19 is pinned because pioarduino 55.03.37
expects SCons 4.8.1; Core 6.2.0 conflicts with this platform's SCons requirement.
This machine already has `.venv` configured. For a fresh checkout, create it
once; use the install command to update an existing environment:

```bash
uv venv --python 3.12 .venv  # Fresh checkout only; skip if .venv exists.
uv pip install --python .venv/bin/python -r hw/focmini/requirements-build.txt
```

Then use `.venv/bin/pio` for these builds:

```bash
# Arduino-ESP32 + SimpleFOC 2.4.0
./.venv/bin/pio run -d hw/focmini -e esp32dev_open_loop

# Optional AS5600 profile; calibration still waits for explicit ARM.
./.venv/bin/pio run -d hw/focmini -e esp32dev_as5600

# Native ESP-IDF profile. Kconfig defaults leave it disarmed.
./.venv/bin/pio run -d hw/focmini/idf -e esp32dev_idf -t menuconfig
./.venv/bin/pio run -d hw/focmini/idf -e esp32dev_idf
```

For native menu configuration, set the measured bus voltage and verified pole
pairs, retain a motor phase amplitude no greater than half the bus range, and
keep the command limit at or below 5000 mrad/s. The native project pins
`esp_simplefoc` 1.4.1; its dependency resolves Arduino-FOC 2.4.0. The Arduino
project uses pinned pioarduino 55.03.37 and Arduino-FOC 2.4.0. Build logs,
source hashes, and host regression evidence are recorded under ignored
`results/`. No upload or physical motor test is implied by a successful build.

After reviewing the wiring/configuration and building, the operator can upload
the first-spin Arduino profile. Keep the motor supply disconnected during USB
upload. The detected CP2102N port for this board is
`/dev/cu.usbserial-3120` (the port can change):

```bash
./.venv/bin/pio run -d hw/focmini -e esp32dev_open_loop -t upload \
  --upload-port /dev/cu.usbserial-3120
./.venv/bin/pio device monitor -d hw/focmini -b 115200 \
  --port /dev/cu.usbserial-3120
```

After upload, connect the measured 9 V motor supply with the motor unloaded and
current limit set, then press the ESP32 EN/reset button to boot while the driver
is powered. Open the serial monitor and wait for `READY DISARMED OPEN LOOP`;
opening the monitor may itself reset the ESP32. If the firmware reports an
active FAULT, inspect and clear the cause, then press EN/reset to reboot. Serial
commands cannot clear a FAULT latch. Once READY appears, enter `ARM`, `T1`, then
`STOP` as complete lines. The port shown is the user-confirmed CP2102N adapter
connected to ESP32-D0WD-V3, 4 MB flash; it can change when the adapter is moved.

On 2026-09-26, the Arduino build with a 1.00 V motor amplitude booted to
`READY DISARMED OPEN LOOP` on the ESP32-D0WD-V3. With the user-reported 9 V
driver supply and 0.5 A supply limit, a two-second `T1` command was acknowledged,
`STOP` was acknowledged, and no FAULT was reported. The user observed smooth
shaft rotation; speed was not independently measured because this setup has no
encoder. Raw serial evidence, configuration, hashes, build and upload status,
and the operator observation are in
[`focmini-amplitude-1000-test-20260926-o524qvwo`](../../results/focmini-amplitude-1000-test-20260926-o524qvwo).

With the actuator attached, the user reported that a 10-second `T1` at 1.00 V
twisted without smooth rotation, a 10-second `T0.2` stayed still, and a
two-second `T1` at 1.50 V still twisted. The user also reports that the actuator
moves freely when unpowered. Every run acknowledged `STOP` and reported no
FAULT; these are operator motion observations, not encoder, position, or current
measurements. Open-loop control does not sense whether the rotor follows its
command under load. Preserve the individual run traces under
[`results`](../../results/focmini-actuator-1500-test-20260926-cghl1iel).

Focused shared parser/interlock tests run on macOS without an ESP32 toolchain:

```bash
./hw/focmini/tests/run_control_tests.sh
```

## Optional AS5600 wiring

Power the sensor from ESP32 3.3 V; join grounds; wire SDA to GPIO 21 and SCL to
GPIO 22. Center its magnet over the shaft axis. Validate the I²C reading and
alignment before selecting the AS5600 profile. Alignment is energized only
after an explicit `ARM` command and is protected by the same FAULT cutoff.

## References

- [DFRobot DRI0058 specification and pinout](https://wiki.dfrobot.com/dri0058)
- [TI DRV8313 datasheet, §7.4.1](https://www.ti.com/lit/ds/symlink/drv8313.pdf)
- [TI DRV8313 control-signal sequencing guidance](https://e2e.ti.com/support/motor-drivers-group/motor-drivers/f/motor-drivers-forum/1008564/drv8313-switching-sequence-of-control-signals)
- [SimpleFOC 3PWM driver contract](https://docs.simplefoc.com/bldcdriver3pwm)
- [SimpleFOC 2.4.0 release](https://github.com/simplefoc/Arduino-FOC/releases)
