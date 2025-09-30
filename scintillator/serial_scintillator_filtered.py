#!/usr/bin/env python3
import serial
from datetime import datetime

PORT = "/dev/ttyUSB0"
BAUD = 19200
THRESHOLD = 20   # cps threshold

def main():
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print(f"Monitoring CPS on {PORT} (threshold={THRESHOLD})")
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                parts = line.split()
                
                # find all "cps" tokens and take the number just before them
                cps_values = []
                for i, token in enumerate(parts):
                    if token.lower() == "cps" and i > 0:
                        try:
                            cps_values.append(float(parts[i-1]))
                        except ValueError:
                            pass

                # only process if we found at least one cps value
                if cps_values:
                    # pick the second cps (the "real" one)
                    if len(cps_values) >= 2:
                        cps = cps_values[1]
                    else:
                        cps = cps_values[0]

                    status = "OK ->" if cps < THRESHOLD else "ALERT ->"
                    print(f"{datetime.now().strftime('%H:%M:%S')} {status} CPS={cps}")
        except KeyboardInterrupt:
            print("\nStopped.")

if __name__ == "__main__":
    main()
