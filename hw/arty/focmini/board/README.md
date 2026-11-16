# Arty A7-35 wiring and acquisition contract

A separate [sensor-only diagnostic](../sensor_diag/README.md) now implements
A0 XADC, AS5600 I2C and UART with all motor controls constant low. It uses the
reported single ACS724 and **18k/18k** divider. The two-current FOC board top
described below remains pending.

Pin assignments are prepared for the CSG324 Arty A7-35 Rev D/E, following
Digilent's official master XDC. Confirm the physical board model/revision when
hardware is available. The new XDC applies to the planned physical board top.
It does not apply to the controller's internal numeric ports or the earlier
registered implementation probe. ADC/I2C/command acquisition and the board top
remain to be implemented. Pin validation is not a complete board build.

## Connections

| Arty connection | Package pin | Signal / external connection |
| --- | --- | --- |
| 100MHz oscillator | E3 | clk |
| BTN0 | D9 | reset_button, to be synchronized/debounced |
| JA1 | G13 | pwm_a -> DRI IN1/N1 |
| JA2 | B11 | pwm_b -> DRI IN2/N2 |
| JA3 | A11 | pwm_c -> DRI IN3/N3 |
| JA4 | D12 | enable -> DRI EN |
| JA7 | D13 | reset_n -> DRI RESET |
| JA8 | B18 | sleep_n -> DRI SLEEP |
| JA9 | A18 | fault_n <- DRI FAULT |
| JB1 | E15 | AS5600 SCL, open-drain |
| JB2 | E16 | AS5600 SDA, open-drain |
| A0 header | C6/C5 analog path | Divided ACS724 U output -> VAUX4 |
| A1 header | A6/A5 analog path | Divided ACS724 V output -> VAUX5 |
| Onboard USB UART | A9 / D10 | uart_rx into FPGA / uart_tx from FPGA |
| LED0..3 | H5/J5/T9/T10 | Planned ready / armed / fault / calibrated status |

Use the **header label A0/A1**, not the FPGA package coordinates, for analog
wiring. Each board input internally feeds both positive/negative analog nets.
The negative analog port is part of the onboard reference circuit, not a second
external sensor signal. The separate A0/A1 digital nets F5/D8 must not be driven
or enabled while those headers are analog inputs.

JA5/JA11 and JB5/JB11 are ground; the 6/12 positions are 3.3V. Use common signal
ground. AS5600 supply/pull-ups are 3.3V; SCL/SDA must only drive low or release.
Use existing module I2C pull-ups if present and suitable; verify their 3.3V rail.
DFRobot's published DRI0058 schematic already has 10k pull-ups on FAULT, RESET
and SLEEP. Do not add a 10k pull-down on SLEEP: it would form a divider with the
onboard pull-up. EN has a series resistor and chip-internal pull-downs; verify
its startup/configuration voltage before motor-powered operation. The first
sensor diagnostic requires motor power disconnected. ACS724 can use the Arty
5V0 power header when the board is properly powered; Pmod supplies are 3.3V.
No motor phase or 9V supply is connected to FPGA logic/analog input pins.

## Raw analog values and the built-in ADC

The ACS724 outputs analog voltage. FPGA logic consumes **digital ADC samples**:

```
ACS724 U/V voltages -> external divider -> Arty A0/A1 board analog network
 -> built-in XADC -> channel-tagged raw counts -> offset/gain/polarity conversion
 -> amperes Q16.16 -> FOC controller

AS5600 SDA/SCL -> I2C register reads + status -> mechanical angle + timestamp
 -> electrical angle after direction/pole-pair/zero correction
```

An external ADC is not required just to obtain analog samples: the Arty already
has XADC. The FPGA still needs logic to configure conversions/channels, read
results, trigger near a suitable PWM point, and reject invalid/stale samples.
The future frame assembler must account for U/V sampling skew and angle age;
using A0/A1 does not itself establish simultaneous acquisition or 10kHz timing.

**Correction to the earlier blanket voltage warning:** A0-A5 headers have
onboard scaling for 0-3.3V. Other XADC connections (including A6-A11 differential
inputs) have different limits; the native ADC range is 0-1V. The existing ESP32
10k/18k divider can produce a header-compatible voltage, but its complete gain
must be recalibrated with the Arty connected. The board's resistor network loads
the external divider and adds attenuation. Do not assume the ESP32 conversion
coefficient survives this change. Do not connect an unattenuated 5V sensor
output to the 0-3.3V header.

The AS5600 can also output an analog angle voltage. This pin plan follows the
ESP32 application's I2C connection so angle and magnet/status checks remain
available. Switching to analog angle would be a separate acquisition choice.

## Calibration and alignment to implement

These steps are a specification, not an implemented calibration state machine.
The current controller already requires `calibration_valid`, direction, zero
and a phase-trip threshold; the future board adapter must supply measured data.

| Step | Action | Result / acceptance |
| --- | --- | --- |
| Sensor readiness | Initialize XADC and AS5600; check ranges, magnet/status, sample age | Healthy sensor frames, no missing conversions |
| Current zero | With shaft stationary and zero phase current guaranteed, EN low, average both ADC channels | Separate zero counts and noise statistics; reject clipping/unstable readings |
| Current gain | Apply independently measured known currents of both signs to each sensing path | Measured counts/A through the complete divider + board + ADC chain |
| Phase mapping/polarity | Under a bounded explicit alignment request, use low test vectors and verify expected current patterns | Map sensed U/V and their signs to IN1/IN2; no blind sign assumptions |
| Encoder direction | Slowly sweep a limited stator field forward/back and observe unwrapped mechanical angle | Direction sign and plausible repeatable motion; stuck rotor is failure |
| Electrical zero | Hold a known field vector at the ESP32's initial 0.30V setting, within configured current/time limits; sample settled angle | Electrical phase offset for FOC, checked for repeatability |
| Commit | Disable drive; check every prerequisite and record the measurement/configuration identity | Set calibration_valid; still wait for a new motion command |

The field sweep and hold energize the motor. They need a separate explicit
operator command, valid current conversion/protection first, bounded duration,
and STOP/FAULT/stale-sensor aborts throughout. The 0.30V setting is the source
profile's starting value, not proof that a particular motor will align at it.

For each current channel, use a measured signed conversion:

```
I_A = (raw_count_A - zero_count_A) / counts_per_amp_A
I_B = (raw_count_B - zero_count_B) / counts_per_amp_B
I_C = -(I_A + I_B)
theta_e = wrap(7 * direction * theta_mechanical - electrical_zero)
```

Phase C is reconstructed, so its zero sum is **not an independent** check of
sensor health or phase mapping. Current offsets and gains are different from
electrical zero: the latter aligns rotor angle to the stator's magnetic field.
Mechanical encoder zero alone does not establish electrical zero.

Record ADC channel/configuration, measured gains/offsets/noise, phase ordering,
polarity, encoder direction, electrical zero, alignment settings, board/wiring
identity and a version/checksum. Capture fresh zero-current offsets at startup
only under the guaranteed-zero condition. Revalidate after wiring, sensor,
mounting or ADC changes. `calibration_template.json` deliberately contains null
measurements and cannot authorize drive.

## Validate the constraints without hardware

```sh
python3 hw/arty/focmini/tools/check_pins.py
```

The checker verifies the pinned official XDC hash and every selected source
port/pin pair, then uses Vivado's RTL elaboration and device package database to
check all 21 planned ports, pin uniqueness, directions, IO standards, analog pin
functions and clock period. Its fixture is only a port contract. It does not
instantiate the sensor/controller system, implement/place/route the board, or
generate/program a bitstream. Raw evidence is retained in unique ignored
`results/arty_focmini_pins/` directories. `--master-xdc PATH` uses a local copy.

Latest check: **21/21 pass**, including corrected UART directions, on Vivado
2024.2, evidence `results/arty_focmini_pins/20261115T220949-mge_qhi6/metrics.json`.
The fixture's constant-disabled motor outputs and released I2C lines produce
expected constant/unconnected warnings; no critical warnings/errors occurred
in this passing check. The earlier failed validation attempt is preserved.

Sources:
- [Official Arty A7-35 master XDC](https://raw.githubusercontent.com/Digilent/digilent-xdc/master/Arty-A7-35-Master.xdc)
- [Official Arty A7 schematic, Rev E](https://digilent.com/reference/_media/reference/programmable-logic/arty-a7/arty_a7_sch.pdf)
- [AS5600 operation/I2C guide](https://look.ams-osram.com/m/4f8342513a447495/original/AS5600_UG000254_2-00.pdf)
- [DRI0058 interface](https://wiki.dfrobot.com/dri0058/)

UART correction (16 November 2026): the original plan swapped FPGA RX/TX.
The official net `uart_txd_in` is input A9; `uart_rxd_out` is output D10.
The original package/pair validation did not catch this semantic direction
error. The validator now explicitly checks the UART direction contract.
