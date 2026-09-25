# DFRobot SimpleFOCmini (DRI0058) + ESP32

This is an **implemented, buildable target** with separate sensorless open-loop
bring-up and optional AS5600 closed-loop velocity modes. It deliberately does
not export, quantize, or execute an RL policy.

## Wiring

DRI0058 is an external 3-PWM driver, not an ESP32 board. Its documented supply
range is 8–30 V, output limit is 2.5 A/channel, logic accepts 3.3/5 V, and PWM
must not exceed 200 kHz. The ESP32 connection is:

| ESP32 DevKit GPIO | DRI0058 pin | Role |
| --- | --- | --- |
| 25 / 26 / 27 | IN1 / IN2 / IN3 | phase PWM A/B/C |
| 32 | EN | stage enable, active high |
| 33 | RESET | driver reset, active low |
| 14 | SLEEP | driver sleep, active low |
| 34 | FAULT | active-low fault input; external 10 kΩ pull-up to 3.3 V |
| GND | GND | required common signal ground |
| 21 / 22 | AS5600 SDA / SCL | used only by the `esp32dev_as5600` build |

Connect motor phases only to M1/M2/M3 and motor supply only to VCC/GND. Do not
power the ESP32 from DRI0058's 3.3-V output: it is rated at 10 mA. Use USB or a
properly rated regulator, while keeping signal grounds common. GPIO 34 has no
internal pull-up, so fit the external FAULT pull-up described above. Verify the
header order against your exact DRI0058 revision before connecting power.

```text
Current-limited DC supply (+) ───────────── DRI0058 VCC (8–30 V)
Current-limited DC supply (−) ───────┬───── DRI0058 GND
                                     └───── ESP32 GND
ESP32 GPIO 25 ──────────────────────────── DRI0058 IN1
ESP32 GPIO 26 ──────────────────────────── DRI0058 IN2
ESP32 GPIO 27 ──────────────────────────── DRI0058 IN3
ESP32 GPIO 32 ──────────────────────────── DRI0058 EN
ESP32 GPIO 33 ──────────────────────────── DRI0058 RESET
ESP32 GPIO 14 ──────────────────────────── DRI0058 SLEEP
ESP32 GPIO 34 ───────────────┬──────────── DRI0058 FAULT
ESP32 3.3 V ─────────[10 kΩ]─┘
Motor phase U/V/W ───────────────────────── DRI0058 M1/M2/M3
ESP32 USB ───────────────────────────────── computer (serial + ESP32 power)
```

For later AS5600 mode: power the sensor from ESP32 3.3 V; join grounds; wire
SDA to GPIO 21 and SCL to GPIO 22. Mount the magnet centered over the AS5600
shaft axis. Keep the motor supply off while checking sensor I²C and alignment.

## Configuration and safety

Before upload, edit `src/main.cpp` with measured data:

- `kMotorPolePairs` (the default `0` intentionally disarms the firmware);
- `kBusVoltageV` (measured supply voltage in V; board range is 8–30 V);
- conservative `kVoltageLimitV` (V; leave at 0.50 V for first spin);
- actual PWM, enable, reset, sleep, fault, ground, and motor-phase wiring.

The program starts with EN and SLEEP low, rejects an active fault, and latches a
falling FAULT edge by disabling the stage. This is a basic software interlock,
not a safety system: it cannot detect wiring errors or enforce current. Use a
current-limited supply, unloaded motor, and physical disconnect for first test.
The default build uses `velocity_openloop`, so it does not use the AS5600 and
cannot regulate speed under load. The optional AS5600 build uses closed-loop
velocity control. When ready, power the AS5600 from the ESP32's 3.3-V rail,
connect its ground to the common ground, connect SDA to GPIO 21 and SCL to GPIO
22, and mount its magnet concentrically with the motor shaft. Do not enable the
AS5600 build until the wiring and magnet alignment are verified.

## Build

From the repository root, install PlatformIO once if `.venv/bin/pio` is absent:

```bash
python3 -m venv .venv
./.venv/bin/pip install platformio
```

PlatformIO is installed in the repository virtual environment at
`.venv/bin/pio` (PlatformIO Core 6.2.0). The build uses the pinned pioarduino
platform release 55.03.37, which supplies Arduino-ESP32 3.3.7 required by
upstream [SimpleFOC Arduino-FOC](https://github.com/simplefoc/Arduino-FOC)
v2.4.0. Both profiles compiled successfully on 2026-09-26; no ESP32 has been
flashed or motor energized from this workspace.
Serial targets are mechanical rad/s and bounded by `kVelocityLimitRadPerSec`;
the initial 20 rad/s limit is approximately 191 RPM.

```bash
# First spin: no encoder required.
./.venv/bin/pio run -d hw/focmini -e esp32dev_open_loop

# Later, after AS5600 wiring/magnet validation:
./.venv/bin/pio run -d hw/focmini -e esp32dev_as5600

# Upload only after the matching configuration review:
./.venv/bin/pio run -d hw/focmini -e esp32dev_open_loop -t upload
./.venv/bin/pio device monitor -d hw/focmini -b 115200
```

Connect the ESP32 by USB before uploading. On Linux, the serial device is often
`/dev/ttyUSB0` or `/dev/ttyACM0`; select the detected device if auto-detection
does not find it. If upload cannot connect, hold BOOT while starting upload and
release it after the ESP32 enters its bootloader. Close the serial monitor
before uploading. For AS5600 firmware, use `esp32dev_as5600` in the upload
command.

Before upload, set `kBusVoltageV` to the measured voltage at the board and
`kMotorPolePairs` to the verified motor value. Zero values intentionally leave
the motor disarmed. Start with `kVoltageLimitV = 0.50` and a current-limited
supply; voltage-mode control cannot enforce the DRI0058 current rating.

No upload, hardware test, or measurement has been performed by this change.
Preserve source/firmware hashes, board revision, motor/supply details, serial
traces, and results under ignored `results/` before conclusions.

## Sources
- [DFRobot DRI0058 specification and pinout](https://wiki.dfrobot.com/dri0058)
- [SimpleFOC 3PWM driver contract](https://docs.simplefoc.com/bldcdriver3pwm)
- [SimpleFOC 2.4.0 release](https://github.com/simplefoc/Arduino-FOC/releases)
