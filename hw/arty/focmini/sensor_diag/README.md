# Arty A7-35T sensor-only diagnostic

This separate physical top reads one ACS724 analog channel on A0/VAUX4 and an
AS5600 on JB1=SCL, JB2=SDA. It sends ASCII telemetry at 115200 baud, 8-N-1 over
the Arty FTDI UART (interface 1). There is no UART command receiver. PWM A/B/C,
EN, RESET and SLEEP are constant zero in RTL and checked as constant GND in the
synthesized netlist. This guarantees those logic values only after configuration;
it is not a guarantee about unconfigured-pin voltage or physical wiring.

Keep the motor's 9 V supply disconnected for this diagnostic. FPGA programming
is volatile JTAG configuration, not a persistent flash write. It does not spin
the motor, align the rotor, measure electrical zero, or calibrate current gain.
The existing two-current FOC controller is not included or modified.

## Actual reported wiring

- Arty A7-35T (CSG324; physical board revision still to be confirmed).
- One ACS724-05AB: VCC from Arty power-header 5V0; common GND.
- OUT -> **18k** -> A0, A0 -> **18k** -> GND. The ideal external ratio is 1/2;
  the Arty input network loads this divider. Do not reuse the old ESP32 10k/18k
  coefficient. Raw XADC counts are deliberately reported without amperes.
- AS5600 supply 3.3V, SCL JB1, SDA JB2; existing soldered module/wiring retained.
  No FPGA weak I2C pull-ups are enabled; both pins only pull low or release.
  Verify module pull-ups and levels if communications fail.
- Driver pins match the main board pin map; no added EN/SLEEP pull-downs.
  DFRobot's published schematic has 10k pull-ups on RESET, SLEEP and FAULT;
  therefore blindly adding a 10k SLEEP pull-down is incorrect. EN has a board
  series resistor and chip-internal pull-downs. Motor-powered startup needs its
  own verification; an ESP32 working previously does not establish FPGA startup.
- A1 and the analog header's separate digital nets are unused.

## Sampling and protocol

XADC: single channel VAUX4, unipolar, extended acquisition (10 ADCCLK acquisition
cycles), 1MHz ADCCLK from 100MHz/100, continuous ~31.25ksps. INIT_40=0114,
INIT_41=3FFF (single channel, alarms disabled, calibration correction enabled),
INIT_42=6400. This is deliberately slow diagnostic acquisition, not the final
PWM-synchronous controller input. Read DRP address 14h after EOC with matching
channel tag; capture on DRDY. Mark stale at 10ms and rail-near codes <=4 or >=4091.
XADC correction is not calibration of the ACS724/divider/board signal chain.

AS5600: nominal 100kHz (actual slightly slower due to state transitions), one
transaction per 100ms, combined pointer write 0Bh followed by repeated-start
read of STATUS, RAW_ANGLE_H, RAW_ANGLE_L. ACK first two read bytes, NACK final byte,
then STOP. No configuration, EEPROM, or OTP writes. SCL/SDA pass two-flop
synchronizers. Wait for actual SCL high (clock stretching), abort/release lines
if a whole transaction exceeds 10ms; no automatic bus-clear clocks.

Example (all numeric fields hexadecimal):

```
S1 a=654 v=1 c=0 i=1 e=0 s=20 r=ABC b=3 f=0
```

- a: latest 12-bit XADC code; v: fresh; c: near a converter rail.
- i: completed I2C transaction succeeded; e: 0 success, 1 bus not idle,
  2 write-address NACK, 3 register-pointer NACK, 4 read-address NACK,
  5 timeout, 6 invalid state.
- s: AS5600 STATUS; bit5 magnet detected, bit4 too weak, bit3 too strong.
- r: raw 12-bit mechanical angle; degrees = code * 360 / 4096. On failure,
  s/r retain the previous successful reading and must be ignored when i=0.
- b: SCL/SDA levels at packet capture, bit1=SCL, bit0=SDA; idle=3.
- f: synchronized FAULT input, informational only. With driver power absent,
  this is not a diagnosis of driver health.

Physical LD4 (led[0]) toggles each poll, LD5 indicates fresh ADC, LD6 last I2C
transaction OK, LD7 last successful transaction with magnet detected and neither weak nor strong.
BTN0 resets diagnostic logic. Packets contain one consistent latched snapshot;
ADC and angle measurements are not simultaneous and not suitable as FOC frames.

## Reproduce

```
python3 hw/arty/focmini/sensor_diag/run.py --build
```

Runs from an immutable source snapshot under a unique ignored
`results/arty_sensor_diag/` directory. Records source hashes, Git/submodule
identities, commands, raw simulation and Vivado logs, timing, utilization, I/O,
CDC, DRC, constant-motor-output proof, and bitstream hash. Build never programs.
Read `metadata.json` and accept only `status=bitstream_built` without critical
warnings. Program the same hashed bitstream with `program.tcl` only after review
and matching USB serial / xc7a35t identity. Never use an earlier rejected build.

The I2C testbench emulates an independent open-drain slave and tests data,
repeated start, ACK/NACK, stretching, stuck buses, timeout/recovery, and reset.
The top testbench decodes the actual UART waveform and checks output constants,
ADC channel filtering, clipping, freshness timeout, and reset. Its XADC model is
only a digital handshake model: it cannot establish analog accuracy/settling.

Timing exceptions end only at first synchronizer inputs for asynchronous board
inputs. Output paths get a 20ns propagation bound, well below 100kHz I2C and
115200 baud periods; there is no synchronous external clock relationship.
Constant motor outputs and analog ports do not have ordinary synchronous I/O
setup/hold requirements. Check all reported exceptions and CDC findings.

Sources:
- AMD UG480 control registers: https://docs.amd.com/r/en-US/ug480_7Series_XADC/Control-Registers
- Digilent Arty A7 Rev E schematic: https://digilent.com/reference/_media/reference/programmable-logic/arty-a7/arty_a7_sch.pdf
- AS5600 register map: https://look.ams-osram.com/m/7059eac7531a86fd/original/AS5600-DS000365.pdf
- DRI0058 schematic: https://img.dfrobot.com.cn/wikicn/5ea64bf6cf1d8c7738ad2881/547246ee565b6a3b7ac28bd5878fd061.jpg

Hardware integration correction: FPGA UART TX is **D10**, connected to FTDI
RXD; A9 is FTDI TXD / FPGA RX and is unused by this diagnostic. The earlier
pin plan reversed these; the first programmed image produced no UART data.
A build-time direction assertion now guards the corrected TX assignment.
