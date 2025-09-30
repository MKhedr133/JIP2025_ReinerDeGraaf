#!/usr/bin/env python3
import serial
from datetime import datetime

PORT = "/dev/ttyUSB0"
BAUD = 19200   # adjust if needed

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print(f"Reading raw data from {PORT} at {BAUD} baud")
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                # decode everything, replace errors so we don't crash
                line = raw.decode("utf-8", errors="replace").strip()
                print(f"{datetime.now().strftime('%H:%M:%S')} RAW -> {line}")
        except KeyboardInterrupt:
            print("\nStopped.")

if __name__ == "__main__":
    main()
