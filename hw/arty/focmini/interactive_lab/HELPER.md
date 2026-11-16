# Arty A7-35T Interactive PMSM Control & RL Workbench — Replication Guide (`HELPER.md`)

This document describes how to replicate, build, flash (volatile SRAM), and run the **1 kHz Rotor-Synchronized Quadrature Vector & RL Policy Law Workbench** (`hw/arty/focmini/interactive_lab/`) on any macOS or Linux machine using open-source FPGA tools (`yosys` + `openxc7` + `openFPGALoader`) and Python 3.

---

## 1. Hardware Wiring Contract (Arty A7-35T + DFRobot DRI0058 + AS5600)

| Subsystem | Arty A7-35T Header / Pin | FPGA Package Pin | External Connection |
| :--- | :--- | :--- | :--- |
| **System Clock** | 100 MHz Oscillator | `E3` | Onboard 100 MHz crystal (`LVCMOS33`) |
| **Reset Button** | `BTN0` | `D9` | Active-high manual reset |
| **Motor PWM A** | `JA1` | `G13` | DRI0058 `IN1` (`N1`, 20 kHz PWM) |
| **Motor PWM B** | `JA2` | `B11` | DRI0058 `IN2` (`N2`, 20 kHz PWM) |
| **Motor PWM C** | `JA3` | `A11` | DRI0058 `IN3` (`N3`, 20 kHz PWM) |
| **Driver Enable** | `JA4` | `D12` | DRI0058 `EN` |
| **Driver Reset** | `JA7` | `D13` | DRI0058 `RESET` (active-low) |
| **Driver Sleep** | `JA8` | `B18` | DRI0058 `SLEEP` (active-low) |
| **Driver Fault** | `JA9` | `A18` | DRI0058 `FAULT` (`nFAULT`, active-low input with 10 µs digital glitch filter) |
| **Encoder SCL** | `JB1` | `E15` | AS5600 `SCL` (400 kHz Fast-Mode I2C, open-drain, 3.3 V pull-up) |
| **Encoder SDA** | `JB2` | `E16` | AS5600 `SDA` (400 kHz Fast-Mode I2C, open-drain, 3.3 V pull-up + auto 9-clock bus unstick) |
| **Current Sense** | Shield `A0` | `C6` / `C5` | ACS724-05AB output via 18 kΩ / 18 kΩ divider (`VAUX4`) |
| **USB UART TX** | Onboard FTDI | `D10` | FPGA `uart_tx` → Host PC RX (`115200 8N1`) |
| **USB UART RX** | Onboard FTDI | `A9` | Host PC TX → FPGA `uart_rx` (`115200 8N1`) |

### Bench Power Supply Settings
- **Voltage**: `9.0 V` DC connected to the DRI0058 `VM` / `GND` screw terminals (common ground with Arty `GND`).
- **Current Limit**: `0.25 A – 0.50 A` (so the 9 V rail does not droop below the DRV8313's `7.8 V` UVLO threshold during fast step reversals).
- **Motor**: 2805 3-phase gimbal BLDC/PMSM (`7` pole pairs).

---

## 2. Software & Toolchain Prerequisites (macOS or Linux)

No Xilinx Vivado installation is required! The entire FPGA synthesis, placement, routing, bitstream generation, and JTAG programming pipeline runs locally in **~10 seconds** using `yosys`, `openxc7` (`nextpnr-xilinx` + `prjxray`), and `openFPGALoader`.

### Step 2.1: Install `yosys` and `openFPGALoader`
- **macOS (Homebrew)**:
  ```sh
  brew install yosys openfpgaloader python3
  ```
- **Linux (Ubuntu / Debian / Arch / CachyOS)**:
  ```sh
  # Arch / CachyOS:
  sudo pacman -S yosys openfpgaloader python python-numpy
  # Ubuntu / Debian:
  sudo apt install yosys openfpgaloader python3 python3-venv python3-numpy
  ```

### Step 2.2: Install `apio` and the `openxc7` Package
Create or use the repository `.venv` and install `apio`, which downloads the prebuilt `openxc7` toolchain (`nextpnr-xilinx`, `fasm2frames`, `xc7frames2bit`, and the `xc7a35tcsg324.bin` chipdb) into `~/.apio/packages/openxc7`:
```sh
python3 -m venv .venv
.venv/bin/pip install apio numpy pyserial
.venv/bin/apio packages install openxc7
```

---

## 3. Quick Start — One-Command Launch

A pre-routed, hardware-verified bitstream (`interactive_helm.bit`) is already included in `hw/arty/focmini/interactive_lab/`.

1. Connect the Arty A7-35T via USB and switch **ON** the **9.0 V** bench supply (`0.50 A` current limit).
2. Run the launcher script from anywhere in the repository:
   ```sh
   ./hw/arty/focmini/interactive_lab/build_and_run.sh
   ```
   *(To re-synthesize and re-route `interactive_helm.bit` from the Verilog sources in ~10 seconds using `yosys` + `openxc7`, pass `--build`)*:
   ```sh
   ./hw/arty/focmini/interactive_lab/build_and_run.sh --build
   ```
3. Open **[http://127.0.0.1:8765](http://127.0.0.1:8765)** in your browser.

### Environment Variable Overrides
If your Arty serial port or desired HTTP port differs from the auto-detected defaults:
```sh
ARTY_PORT=/dev/cu.usbserial-210319A43D3E1 HELM_HTTP_PORT=8765 ./hw/arty/focmini/interactive_lab/build_and_run.sh
```
*(On Linux, `helm_server.py` automatically detects `/dev/serial/by-id/*Digilent*if01*` or `/dev/ttyUSB1`.)*

---

## 4. Architecture & Control Laws Implemented in `interactive_lab/`

### 4.1 High-Rate Sensor & Control Pipeline
- **1,000 Hz AS5600 Fast-Mode I2C Polling (`400 kHz` SCL)**:
  - `as5600_reader.sv` polls the 12-bit mechanical angle (`0..4095` counts = `0°..360°`) every `1 ms` (`POLL_CYCLES = 100,000`).
  - Includes an **automatic 9-clock I2C bus recovery state machine** (`RECOVER_LOW` / `RECOVER_HIGH`): if FPGA JTAG reconfiguration interrupts an AS5600 read mid-byte and leaves `SDA` held low (`bus_level == 2'b10`), the FPGA automatically clocks `SCL` 9 times and issues a `STOP` condition to free the bus without requiring a USB power cycle.
- **10 µs Digital `nFAULT` Glitch Filter**:
  - Rejects nanosecond capacitive switching crosstalk from the 20 kHz PWM lines (`JA1/2/3`) onto `JA9` (`fault_n`), while still latching genuine driver UVLO/overcurrent faults (`fault_low_cnt == 1000` clocks = `10 µs`).
- **256×30 `RAMB18E1` 3-Phase Sine Table + Arithmetic Amplitude Scaler**:
  - Avoids the `openxc7` `RAMB36E1` 64-to-32-bit port truncation bug by storing 256 signed 10-bit 3-phase sine triplets (`[-512, +512]`) in a single `RAMB18E1` and scaling the PWM amplitude in arithmetic (`calc_duty` in `position_controller.sv`).

### 4.2 Live Hardware Control Law A/B Switch (`M1` vs. `M0`)
In `servo.sv`, the FPGA computes the true electrical rotor angle relative to the locked mechanical origin:
$$\theta_e = -7 \cdot (\theta_m - \theta_{\text{origin}})$$
and applies a hardware-clamped quadrature torque lead angle $\Delta\theta_e \in [-90^\circ_{\text{elec}}, +90^\circ_{\text{elec}}]$ (`[-16384, +16384]`):
$$\theta_{\text{stator}} = \theta_e + \text{clamp}\!\left(K_p e_\theta + K_i \int e_\theta + K_d \dot{\theta}_m,\; -16384,\; +16384\right)$$
Because $|\Delta\theta_e| \le 90^\circ_{\text{elec}}$, **the stator field can never slip poles**, and any large step error immediately commands **100% pure $q$-axis quadrature torque ($\sin(90^\circ) = 1.0$)**.

You can toggle live between two hardware control laws from the web UI (or via UART commands `M1` / `M0`):
1. **`🧠 RL Policy Law (M1)`**:
   - **Dual-Scale High-Gain Quadrature Law**: Fast reference slewing (`up to 64 counts/ms`), high proportional gain (`-512 * error` -> full $90^\circ$ quadrature torque at `2.8°` error), active velocity damping, and fast integral trim.
   - **Dynamic State-Dependent Voltage Modulation (`amp_sel = 0..3`)**:
     - Settled (`|error| <= 6` counts = `0.53°`): drops phase amplitude to **`0.45 V`** (`±256` PWM counts) → **bench supply current drops to `0.011–0.03 A` at rest**!
     - Small error (`|error| <= 24` counts): **`0.70 V`** (`±384` counts).
     - Medium error (`|error| <= 72` counts): **`0.92 V`** (`±512` counts).
     - Large transient (`|error| > 72` counts): **`1.15 V`** boost (`±640` counts) for rapid minimum-time step response.
2. **`⚙️ Classical Fixed-V (M0)`**:
   - Applies a constant **`0.92 V`** (`±512` PWM counts, `amp_sel = 2`) at all times — drawing **`~0.06 A` continuously at rest** — with a slower linear reference ramp (`4 counts/ms`) and softer proportional gain (`-128 * error`), clearly demonstrating phase lag on frequency chirps and higher idle copper loss ($I^2R$).

---

## 5. UART Command & Telemetry Protocol (`115200 8N1`)

### Host → FPGA Commands (`\n`-terminated ASCII)
| Command | Description |
| :--- | :--- |
| `ARM` | Wake DRV8313 (`SLEEP=1`, pulse `RESET`), verify sensors & `nFAULT`, enter `ARMED` (`m=3`). Also re-arms from `IDLE`/`FAULT`. |
| `TEST` | Start closed-loop operation (`m=5`, `EN=1`), lock rotor origin at `t = 600 ms` (`k=1`). |
| `D<hex3>` | Set 12-bit target mechanical angle (`000`..`FFF`, e.g. `D3A5`) and refresh keepalive timer. |
| `S<hex2>` | Set maximum reference slew rate in encoder counts per millisecond (`01`..`40`, e.g. `S18`). |
| `M1` / `M0` | Select `M1` (RL Dynamic-Voltage Policy Law) or `M0` (Classical Fixed-Voltage Linear Servo). |
| `PING` | Refresh the 2-second host keepalive watchdog. |
| `STOP` or `!` | Immediate emergency stop (`EN=0`, `PWM=0`, `SLEEP=0`, `m=0`). `!` acts immediately without waiting for `\n`. |

### FPGA → Host Telemetry (`40 Hz`, 114-byte `C3` frame)
```text
C3 a=4A1 v=1 c=0 i=1 e=0 s=33 r=39C b=3 f=1 m=5 q=0 n=004C t=3A8A p=45 g=1 x=0000 d=3A5 z=009 o=4537 h=4 k=1 y=F
```
- `a`: 12-bit XADC raw count (`v=1` fresh, `c=0` unclipped)
- `r`: 12-bit measured AS5600 mechanical angle (`000..FFF`)
- `d`: 12-bit active reference angle (`000..FFF`)
- `z`: Signed 12-bit wrapped position error `d - r`
- `o`: Signed 16-bit stator electrical phase command
- `m`: Controller state (`0`=IDLE, `1`=WAKE, `2`=SETTLE, `3`=ARMED, `4`=STARTING, `5`=RUN, `6`=FAULT)
- `h`: Mode indicator (`4` = `M1` RL Policy Law active, `2` = `M0` Classical Mode active)

---

## 6. Interactive Demonstrations in the Web Workbench (`http://127.0.0.1:8765`)

1. **🔬 EE Control Systems Lab Tab**:
   - **CH1 & CH2 Digital Storage Oscilloscopes**: Live plots of $\theta^*(t)$, $\theta_m(t)$, tracking error $e_\theta(t)$, and 20 kHz 3-phase PWM duty waveforms ($d_a, d_b, d_c$).
   - **Automatic Step-Response Analyzer**: Measures 10%–90% Rise Time, Peak Overshoot, and Steady-State Error on every step.
   - **Frequency Chirp (`Freq Chirp`) & Live `M1`/`M0` A/B Comparison**: Toggle between `M1` and `M0` during a frequency chirp or square wave to observe phase lag, bandwidth roll-off, and bench PSU current draw (`0.06 A` constant in `M0` vs. `0.011–0.03 A` settled in `M1`).
   - **Clarke / Park Space-Vector Phasor Diagram**: Real-time visualization of the 7-pole-pair electrical rotor axis and stator voltage vector $\vec{V}_s$, plus the quantized 144,388-parameter RL Digital-Twin shadow ($i_q^*, v_d, v_q$).
2. **🧲 Haptic Virtual Spring / Detents Tab**:
   - Creates virtual magnetic notches every `45°` — drag/flick the flywheel and release to watch the rotor snap into the nearest `45°` detent.
3. **⛵ Naval Slalom Game Tab**:
   - 60-second motor-in-the-loop mission with Red/Green Slalom Gates, Gold Data Beacons (`★`), Spiked Naval Mines (`💥`), Hull Integrity (`❤️❤️❤️`), Streak Multiplier (`1x..8x`), a **Gold Dashed Ghost Ship ($\theta^*$)** vs. **Cyan Physical Ship ($\theta_m$)**, and an **🤖 AI Auto-Helmsman** mode.
