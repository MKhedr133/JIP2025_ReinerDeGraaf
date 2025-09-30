#!/usr/bin/env python3
import serial, csv, time
from datetime import datetime

PORT = "/dev/ttyUSB0"
BAUD = 19200
THRESHOLD = 20

# Try common query commands (with different line endings)
CANDIDATES = [
    "\r", "?\r", "D\r", "Q\r", "S\r", "R\r",
    "READ\r", "MEAS?\r", "PRINT\r", "PRN\r",
]
# Also try CRLF versions
CANDIDATES += [c.replace("\r", "\r\n") for c in CANDIDATES]

def extract_cps(line: str):
    parts = line.split()
    cps_vals = []
    for i, tok in enumerate(parts):
        if tok.lower() == "cps" and i > 0:
            try:
                cps_vals.append(float(parts[i-1]))
            except ValueError:
                pass
    # prefer the second cps if present (your “real” CPS)
    if cps_vals:
        return cps_vals[1] if len(cps_vals) >= 2 else cps_vals[0]
    return None

def find_working_command(ser: serial.Serial, trial_time=6.0):
    """Try candidate commands until one yields a line with CPS."""
    for cmd in CANDIDATES:
        # clear input buffer
        ser.reset_input_buffer()
        start = time.time()
        last_send = 0
        while time.time() - start < trial_time:
            # send command once per second
            if time.time() - last_send > 1.0:
                ser.write(cmd.encode("ascii", errors="ignore"))
                last_send = time.time()
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            cps = extract_cps(line)
            if cps is not None:
                return cmd  # success
    return None

def main():
    out_name = f"scint_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with serial.Serial(PORT, BAUD, timeout=1, xonxoff=False, rtscts=False) as ser, \
         open(out_name, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "cps"])

        print(f"Trying to find a query command on {PORT} @ {BAUD}…")
        cmd = find_working_command(ser)
        if not cmd:
            print("Couldn’t discover a polling command. "
                  "Try enabling continuous output in the device menu, "
                  "or we can try toggling handshakes (xon/xoff, rts/cts).")
            return
        print(f"✅ Using polling command: {repr(cmd)}")
        print(f"Logging to {out_name}; threshold={THRESHOLD} cps")

        try:
            last_send = 0
            while True:
                # poll once per second
                if time.time() - last_send > 1.0:
                    ser.write(cmd.encode("ascii", errors="ignore"))
                    last_send = time.time()

                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                cps = extract_cps(line)
                if cps is None:
                    continue

                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                status = "OK ->" if cps < THRESHOLD else "ALERT ->"
                print(f"{now} {status} CPS={cps:.2f}")
                w.writerow([now, cps])
                f.flush()
        except KeyboardInterrupt:
            print("\nStopped.")

if __name__ == "__main__":
    main()
