# Hardware handoff — 16 November 2026

Branch: `fpga/focmini-driver`, fork: `IamLegend509/vroom`.
Includes the FPGA work and, at the user's request, the ESP32 single-current
monitor and encoder diagnostic changes.
RL training, checkpoints and inference have not been changed by this work.

## Current state

The last FPGA image programmed was **O2** (`open_loop_90`), a bounded open-loop
motion demonstration. Its last run finished with PWM and enable disabled, and
the host sent STOP. The operator was instructed to turn the 9 V output off;
verify it is off before disconnecting or reprogramming. Programming was volatile:
power cycling the Arty clears this image unless another image exists in flash.

**Neither an accurate closed-loop controller nor a physical 90° sweep has been
commissioned.** Do not confuse `sweep_completed` (timer/command completion) with
the shaft following that trajectory.

| Implementation | Verified status |
| --- | --- |
| `rtl/` two-current FOC baseline | Math/control RTL and nominal assumed-plant checks pass; a heavier assumed plant fails a tracking gate. Not integrated/commissioned with the current one-sensor setup. |
| `sensor_diag/` S1 | Direct AS5600, XADC A0 and UART tested on Arty; all motor controls low. Current gain/polarity and absolute encoder accuracy uncalibrated. |
| `driver_diag/` D1 | Wake-only test completed; PWM and enable structurally tied low. nFAULT was high when awake and low when asleep in this setup. |
| `open_loop/` O1 | Original 10-second field sweep visibly moved forward/back. Encoder-reported return differed by about 13.27°. |
| `position_loop/` C3 (current source) | Bounded mechanical position feedback via field phase, not dq/current FOC. Digital tests and Vivado pass; C3 has NOT been programmed or physically tested. |
| Earlier C1/C2 | C1 failed first +5° tracking check. C2 passed +5° (about -0.35° error), then failed return at 16.999 s (+1.49° last active error). Both are retained as failed full-profile tests. |
| `open_loop_90/` O2 | Digital tests/Vivado pass; two timed 35-second hardware sequences completed. A 90° physical out/back move was NOT achieved. |

O2 commands 1.75 electrical turns each way, assuming seven pole pairs: initial
hold 3 s, outward 12 s, far hold 5 s, return 12 s, final hold 3 s. Nominal phase
voltage is only 0.6 V at a 9 V bus. The first run's encoder hold differences were
+34.89° outward and +3.60° farther at the final hold. The requested repeat gave
-2.64° outward-hold difference and +2.81° back, with a transient excursion near
-13.89°. The operator reported mostly stationary motion, perhaps 5°. Weak-field
status persisted in every packet; its angle accuracy is not established. There
were no active driver-fault or sensor-health trips. Latest supply current/CV/CC
were not reported; the earlier C1 test's reported 0.004 A is not a C2/O2 measurement.

C3 retains C2's reduced integration and measured-motion damping, extending each
target hold to 12 s (51 s overall, ±5° targets, ±1.055° settling criterion). The
user requested the larger visible sweep before a C3 powered trial, so do not
describe C3 as physically accepted. No voltage increase beyond 0.6 V was made.

## Connected hardware and assumptions

- Arty A7-35T, 100 MHz clock, DFRobot DRI0058/DRV8313, bench supply 9 V with
  operator-set 0.10 A limit. Seven pole pairs is the configured assumption;
  revalidate it before treating electrical turns as mechanical degrees.
- AS5600 directly on JB1 SCL / JB2 SDA, 3.3 V and common ground. Module pull-ups
  are present. Magnet is detected but weak; the user cannot reduce the current
  gap further. A manual approximately 90° turn once reported 88.65°, which is
  not a full-range calibration or proof of accuracy under drive.
- One ACS724-05AB on A0, powered from Arty Arduino 5V0. Actual soldered divider
  is **18 kΩ upper / 18 kΩ lower**. ADC values are raw health telemetry, not
  calibrated amperes or an overcurrent trip. No A1 sensor is connected.
- JA1/2/3: driver IN1/2/3; JA4: enable; JA7: RESET; JA8: SLEEP; JA9: nFAULT.
  UART FPGA TX is **D10**, RX **A9**. See each image's XDC and board documents.
- USB serial `210319A43D3E`, JTAG target `210319A43D3EA`, FPGA `xc7a35t`.
  Linux UART was `/dev/serial/by-id/usb-Digilent_Digilent_USB_Device_210319A43D3E-if01-port0`.

## Continue from the MacBook

In an existing checkout of the fork:

```sh
git fetch origin
git switch --track origin/fpga/focmini-driver
git submodule update --init --recursive
```

If this local branch already exists, switch to it and pull with `--ff-only`
instead of creating another tracking branch. For a new clone, select branch
`fpga/focmini-driver` and include submodules.

Offline FPGA runners need Python 3, Icarus Verilog (`iverilog`/`vvp`), and
Verilator/C++ tools for the baseline harness. Position-loop plots additionally
need `matplotlib` (tested with 3.6.3). Existing runners create their results
directories on a fresh checkout. These commands never program a board:

```sh
python3 hw/arty/focmini/sensor_diag/run.py
python3 hw/arty/focmini/open_loop_90/run.py
python3 hw/arty/focmini/position_loop/run.py
```

The actual builds used Vivado 2024.2 on Linux. AMD's
[2024.2 supported OS list](https://docs.amd.com/r/2024.2-English/ug973-vivado-release-notes-install-license/Supported-Operating-Systems)
lists x86-64 Windows/Linux, not native macOS/Apple Silicon. Use the Linux build
machine (or another supported host) for the Vivado flow. Runners accept
`--vivado PATH`, otherwise finding `vivado` on PATH or under the build user's
`~/Vivado/2024.2/bin/`. Example on that supported host:

```sh
python3 hw/arty/focmini/open_loop_90/run.py --build --vivado /path/to/vivado
```

Programming remains a separate action with motor power off. Host scripts use
the specified serial path, not a fixed Linux device; discover the actual port
on the new machine. Default `--mode check` never sends ARM. Do not substitute
the O1, O2 or C3 host program for a different firmware protocol.

## ESP32 work included in this branch

`esp32dev_as5600_single_current` uses AS5600 speed feedback in voltage mode and
GPIO35 current telemetry. It does not regulate current or sample absent GPIO36.
`FOCMINI_PROGRAM=encoder_test` is read-only and holds EN/SLEEP/RESET low.
`select_program.py` now selects exactly one source file. See the ESP32 README
for its original GPIO map; do not use the Arty JA/JB mapping on ESP32.

```sh
python3 -m pip install -r hw/focmini/requirements-build.txt
FOCMINI_PROGRAM=main pio run -d hw/focmini -e esp32dev_as5600_single_current
FOCMINI_PROGRAM=encoder_test pio run -d hw/focmini -e esp32dev_as5600_single_current
```

The ESP32 monitor's conversion still assumes **10 kΩ / 18 kΩ**, whereas the
present Arty wiring is 18 kΩ / 18 kΩ. Reconcile the actual divider, ADC scaling,
zero and polarity before using any current reading as a calibrated value. Build
success does not validate that calibration or authorize an automatic upload.

## Evidence retained on the Linux computer

Git contains source, test harnesses, pin/calibration contracts and the verification
summary. Generated bitstreams, raw captures and bulk reports remain under ignored
`/home/suchit/vroom/results/`; they are **not included in a Git clone**. Copy them
separately if needed on the Mac. Prepared transfer ZIPs are under
`/home/suchit/Documents/Codex/2026-11-14/referenced-chatgpt-conversation-this-is-an/outputs/`.

| Image | Local result directory | Bitstream SHA256 |
| --- | --- | --- |
| O2, last programmed | `arty_open_loop_90/20261116T002454-p038gy93` | `c45f20a266d12a3842f9d8d51f2194e46e4b3a843baa299959933436d2a71ad4` |
| C3, built only | `arty_position_loop/20261116T001932-8m2uwbu6` | `c403c537b75b28de0c68fca4798884e7d059afed0a947c745e57b4dfec129681` |

Both hashes match the retained build metadata. Full timing, DRC and CDC reports
remain in those result directories. O2 timing passed at
+1.454 ns setup / +0.084 ns hold; C3 at +0.285 / +0.068 ns. Retained warnings
include the intentional asynchronous sticky-fault preset (CDC-7) and reviewed
router-created LUT buffer/constant pairs; zero DRC errors is not a clean CDC claim.

Latest O2 captures: `bench_20261116T012736709542Z` and
`bench_20261116T013022352149Z` within its build directory. Each retains raw bytes,
commands, packets and analysis. See [VERIFICATION.md](VERIFICATION.md) for the
earlier hardware results and preserved failed builds/trials.

## Next work

1. Establish trustworthy encoder movement/field strength and verify phase wiring
   and supply behavior under drive. The cause of the limited O2 motion is not
   proven; do not assume that changing targets or timers fixes it.
2. Validate pole pairs, electrical zero/direction and the actual current-measurement
   chain. The single uncalibrated sensor does not support the existing two-current
   FOC core as wired.
3. Commission conservative motion with measured supply current and encoder/visible
   agreement before accepting a larger closed-loop target. Preserve failures and
   keep bounded commands, watchdogs, fault latch and explicit operator readiness.
