#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
OPENXC7="${HOME}/.apio/packages/openxc7"

if [ "${1:-}" = "--build" ] || [ ! -f interactive_helm.bit ]; then
  echo "[1/3] Synthesizing with Yosys + OpenXC7..."
  yosys -q -p "read_verilog -sv as5600_reader.sv uart_tx.sv xadc_a0.sv uart_rx.sv command_rx.sv servo.sv position_controller.sv position_loop_top.sv; synth_xilinx -arch xc7 -top position_loop_top; write_json interactive.json"
  "${OPENXC7}/bin/nextpnr-xilinx" --chipdb "${OPENXC7}/chipdb/xc7a35tcsg324.bin" --xdc pos.xdc --json interactive.json --fasm interactive.fasm -q
  "${OPENXC7}/bin/fasm2frames" --part xc7a35tcsg324-1 --db-root "${OPENXC7}/share/nextpnr/external/prjxray-db/artix7" interactive.fasm > interactive.frames
  "${OPENXC7}/bin/xc7frames2bit" --part_file "${OPENXC7}/share/nextpnr/external/prjxray-db/artix7/xc7a35tcsg324-1/part.yaml" --part_name xc7a35tcsg324-1 --frm_file interactive.frames --output_file interactive_helm.bit
  rm -f interactive.json interactive.fasm interactive.frames
fi

echo "[2/3] Programming Arty A7-35T SRAM (volatile)..."
openFPGALoader -c digilent --ftdi-serial 210319A43D3E interactive_helm.bit

echo "[3/3] Starting EE Control Systems & RL Workbench at http://127.0.0.1:8765 ..."
python3 helm_server.py
