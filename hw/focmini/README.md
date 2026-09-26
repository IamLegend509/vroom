# DFRobot SimpleFOCmini (DRI0058) + ESP32

This is an **implemented bring-up target** with separate sensorless open-loop
bring-up and optional AS5600 closed-loop velocity modes. The native ESP-IDF
project is in `idf/`; the PlatformIO Arduino project remains available in this
directory. Neither exports, quantizes, or executes an RL policy.

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

Before upload, set the parameters in the selected build system:

- PlatformIO: edit `src/main.cpp`; pole pairs default to `0` (disarmed), and
  `kBusVoltageV` must match the measured supply in V;
- ESP-IDF: configure verified pole pairs and measured bus voltage in
  `idf.py menuconfig`; both default to `0` (disarmed);
- both builds: retain a conservative `0.50 V` initial voltage limit and verify
  PWM, enable, reset, sleep, fault, ground, and motor-phase wiring.

Both programs start with EN and SLEEP low and reject an active fault. PlatformIO
latches a falling FAULT edge; ESP-IDF polls FAULT and latches a low level. These
are basic software interlocks, not a safety system: they cannot detect wiring
errors or enforce current. Use a current-limited supply, unloaded motor, and
physical disconnect for first test.
The default build uses `velocity_openloop`, so it does not use the AS5600 and
cannot regulate speed under load. The optional AS5600 build uses closed-loop
velocity control. When ready, power the AS5600 from the ESP32's 3.3-V rail,
connect its ground to the common ground, connect SDA to GPIO 21 and SCL to GPIO
22, and mount its magnet concentrically with the motor shaft. Do not enable the
AS5600 build until the wiring and magnet alignment are verified.

## Build

### Native ESP-IDF (recommended for teammates)

The IDF project uses Espressif's managed [`esp_simplefoc` component](https://docs.espressif.com/projects/esp-iot-solution/en/latest/motor/foc/esp_simplefoc.html),
based on Arduino-FOC APIs, rather than building Arduino-FOC directly as an
Arduino library. The project manifest pins component version 1.4.1 and accepts
ESP-IDF 5.3.x through 5.x (not IDF 6.x). ESP-IDF itself is installed per developer;
the component manager downloads the pinned FOC component and dependencies on
the first build. The default menu configuration leaves the motor disarmed.

From `hw/focmini/idf` in an ESP-IDF 5.x terminal:

```bash
idf.py set-target esp32
idf.py menuconfig
# FOCMini motor setup: enter measured bus voltage and verified pole pairs.
# Keep AS5600 disabled for the first open-loop spin.
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Use the serial command `T1` to request 1 rad/s, or `T0` to stop commanding
rotation. Commands are bounded by the configured velocity limit. Replace the
serial port with the one detected on your computer. The first build downloads
`espressif/esp_simplefoc` 1.4.1 through ESP-IDF's component manager; use an
internet connection. FreeRTOS tick rate is set to 1000 Hz because the control
loop schedules at 1 ms. To enable AS5600 later, configure
`FOCMINI_USE_AS5600=y` in menuconfig only after sensor wiring and magnet
alignment are checked. AS5600 uses SDA GPIO 21 and SCL GPIO 22.

ESP-IDF is not installed in the repository environment used to prepare this
change, so the native IDF project has **not yet been compiled or flashed**.
The PlatformIO build results below apply only to the Arduino implementation,
not this new IDF adapter. Review component/API compatibility with the chosen
ESP-IDF install before powering the motor.

### PlatformIO (existing alternative)

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
