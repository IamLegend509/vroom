#!/usr/bin/env bash
# Run the on-FPGA RL motor controller (C-RL image) on the Arty A7-35T + DRI0058 + AS5600.
# No LLM or agent is involved: this flashes a prebuilt bitstream and starts helm_server.py.
#
#   ./run_rl.sh            flash the pinned RL bitstream, start the server disarmed, open the UI
#   ./run_rl.sh --no-flash start the server on the image already in FPGA SRAM
#   ./run_rl.sh test       scripted step test against a running server (arms the motor!)
#   ./run_rl.sh build      rebuild: bit-exact sim + trip/calibration TB + remote Vivado
#   ./run_rl.sh stop       stop any running helm_server (FPGA cuts the drive within 0.5 s)
#
# Environment overrides:
#   BIT=/path/to/rl_closed_loop.bit   bitstream (default: pinned build below, else newest *-crl-*)
#   BUILD_HOST=user@host              Vivado host for `build`
#   PORT=8765                         HTTP port
#
# Bench setup: PSU 9 V with a 0.3 A current limit. In the UI: Arm (4 s calibration with a
# ~51 deg sweep), choose "Raw NN (M0)" (trained policy unmodified; "Smoothed NN (M1)" is a 1.6 ms
# output filter: quieter, worse tracking), slew ~14, then step buttons / drag the wheel.
#
# Fault codes (q= in the log / UI): 4 driver nFAULT (supply sag or driver OCP), 6 encoder lost,
# 8 host keepalive > 500 ms, 9 UART framing, 11 actor overrun, 12 observer current > 0.27 A,
# 13 speed > 25 rad/s, 14 calibration refused (rotor did not follow the sweep). Press Arm to retry.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
PINNED="$REPO/results/arty_rl_infer/20261118T154206-crl-h2k3lakq/rl_closed_loop.bit"
PORT="${PORT:-8765}"
BUILD_HOST="${BUILD_HOST:-lazybanana@10.249.214.176}"
cd "$REPO"

pick_bit() {
  if [ -n "${BIT:-}" ]; then echo "$BIT"; return; fi
  if [ -f "$PINNED" ]; then echo "$PINNED"; return; fi
  ls -1d results/arty_rl_infer/*-crl-*/rl_closed_loop.bit 2>/dev/null | sort | tail -1
}

stop_server() {
  if pgrep -f "helm_server.py" >/dev/null; then
    echo "[run_rl] stopping running helm_server"
    pkill -f "helm_server.py" || true
    sleep 1
  fi
}

case "${1:-run}" in
  stop)
    stop_server
    ;;

  test)
    shift || true
    curl -sf "http://127.0.0.1:$PORT/api/state" >/dev/null \
      || { echo "[run_rl] no server on port $PORT; start it first with ./run_rl.sh"; exit 1; }
    out="results/arty_rl_bench/$(date +%Y%m%dT%H%M%S)_steps.jsonl"
    mkdir -p "$(dirname "$out")"
    echo "[run_rl] ARMING THE MOTOR for a step test (PSU 9 V / 0.3 A, keep clear). Ctrl+C sends STOP."
    sleep 2
    python3 "$HERE/rl_bench.py" "$out" --mode "${MODE:-raw}" --slew "${SLEW:-14}" \
      --steps "${STEPS:-0,20,-20,45,-45,0}" --dwell "${DWELL:-2.5}"
    python3 "$HERE/rl_bench_summary.py" "$out"
    echo "[run_rl] trace: $out"
    ;;

  build)
    python3 hw/arty/focmini/rl_infer/run_closed_loop.py --build-host "$BUILD_HOST"
    echo "[run_rl] new bitstream: $(ls -1d results/arty_rl_infer/*-crl-*/rl_closed_loop.bit | sort | tail -1)"
    echo "[run_rl] run it with: BIT=<that path> ./run_rl.sh"
    ;;

  run|--no-flash)
    command -v python3 >/dev/null || { echo "python3 not found"; exit 1; }
    python3 -c "import numpy" 2>/dev/null || { echo "numpy missing: pip3 install numpy"; exit 1; }
    stop_server
    if [ "${1:-run}" = "--no-flash" ]; then
      flash_args=(--no-flash)
      echo "[run_rl] using the image already in FPGA SRAM"
    else
      command -v openFPGALoader >/dev/null || { echo "openFPGALoader not found (brew install openfpgaloader)"; exit 1; }
      bit="$(pick_bit)"
      [ -n "$bit" ] && [ -f "$bit" ] || { echo "[run_rl] no RL bitstream found; run ./run_rl.sh build or set BIT="; exit 1; }
      flash_args=(--bit "$bit")
      echo "[run_rl] bitstream: $bit"
    fi
    ( for _ in $(seq 1 60); do
        curl -sf "http://127.0.0.1:$PORT/api/state" >/dev/null 2>&1 && { open "http://127.0.0.1:$PORT" 2>/dev/null || true; exit 0; }
        sleep 0.5
      done ) &
    echo "[run_rl] starting server disarmed on http://127.0.0.1:$PORT  (Ctrl+C stops the motor and exits)"
    exec python3 "$HERE/helm_server.py" --fpga-rl --no-arm --http-port "$PORT" "${flash_args[@]}"
    ;;

  -h|--help|help)
    sed -n '2,22p' "$HERE/run_rl.sh" | sed 's/^# \{0,1\}//'
    ;;

  *)
    echo "unknown command: $1 (try --help)"; exit 1
    ;;
esac
