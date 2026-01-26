#!/usr/bin/env python3
"""
Capture KF CSV stream from ESP32 over Serial and save to a .csv file.

Stops automatically when it sees the line: ---DONE---
"""

from __future__ import annotations
import argparse
import time
from pathlib import Path

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="roll_kf_partc.csv", help="Output CSV filename")
    ap.add_argument("--timeout", type=float, default=2.0)
    args = ap.parse_args()

    out_path = Path(args.out).expanduser().resolve()
    print(f"[INFO] Writing to: {out_path}")

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    time.sleep(1.0)  # allow board reset / settle

    # Optional: clear old buffered bytes
    ser.reset_input_buffer()

    with out_path.open("w", encoding="utf-8") as f:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line == "---DONE---":
                print("[INFO] DONE received. Closing file.")
                break

            # Ignore accidental banners
            if line.startswith("---") and "DONE" not in line:
                continue

            f.write(line + "\n")

    ser.close()
    print("[INFO] Saved successfully.")


if __name__ == "__main__":
    main()
