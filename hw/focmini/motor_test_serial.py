#!/usr/bin/env python3
"""Safely request and monitor one bounded motor-test run over serial.

This helper is only for src/motor_test.cpp. It waits for the firmware's READY
message, refuses to command motion if startup reports DISARMED/FAULT, and asks
for a separate operator confirmation before sending RUN10. It does not measure
or guarantee motor speed/current. Run from the repository's pinned build venv.
"""

from __future__ import annotations

import argparse
import sys
import time

import serial


BAUD_RATE = 115200
READY_TOKEN = "READY DISARMED MOTOR TEST"
STARTUP_TIMEOUT_S = 45.0
RUN_TIMEOUT_S = 13.0


def read_line(port: serial.Serial) -> str:
    return port.readline().decode("utf-8", errors="replace").rstrip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="ESP32 serial port")
    args = parser.parse_args()

    print(f"Opening {args.port} at {BAUD_RATE} baud; serial may reset the ESP32.")
    print("Waiting up to 45 s for READY. If needed, press the ESP32 EN/reset button.")
    try:
        with serial.Serial(args.port, BAUD_RATE, timeout=0.25, write_timeout=1) as port:
            start = time.monotonic()
            ready_seen = False
            while time.monotonic() - start < STARTUP_TIMEOUT_S:
                line = read_line(port)
                if not line:
                    continue
                print(line, flush=True)
                if READY_TOKEN in line:
                    ready_seen = True
                    break
                if "DISARMED:" in line or "FAULT active" in line:
                    print("Startup is not healthy; RUN10 was not sent.", file=sys.stderr)
                    return 1

            if not ready_seen:
                print("Timed out waiting for READY; RUN10 was not sent.", file=sys.stderr)
                return 1

            print("\nCheck that the motor is unloaded, secured, and the shaft is clear.")
            print("Confirm the current-limited supply and wiring are correct.")
            answer = input("Type y and press Enter to run for at most 10 seconds [y/N]: ")
            if answer.strip().lower() not in {"y", "yes"}:
                print("Cancelled; motor command not sent.")
                return 0

            port.write(b"RUN10\n")
            port.flush()
            run_start = time.monotonic()
            try:
                while time.monotonic() - run_start < RUN_TIMEOUT_S:
                    line = read_line(port)
                    if not line:
                        continue
                    print(line, flush=True)
                    if "RUN10 complete" in line:
                        return 0
                    if "LATCHED:" in line or "RUN10 rejected:" in line:
                        print("The firmware stopped/refused the run; inspect the driver.", file=sys.stderr)
                        return 1
            except KeyboardInterrupt:
                port.write(b"STOP\n")
                port.flush()
                print("\nInterrupted; STOP sent. Verify EN is low.", file=sys.stderr)
                return 130

            port.write(b"STOP\n")
            port.flush()
            print("No completion message; STOP sent. Verify EN is low before retrying.", file=sys.stderr)
            return 1
    except KeyboardInterrupt:
        print("\nCancelled before RUN10 was sent.", file=sys.stderr)
        return 130
    except (serial.SerialException, OSError) as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
