#!/usr/bin/env python3
"""
Simple gpiozero test for a 90° servo on GPIO18 (Docker-only, no host daemon).
Forces the lgpio backend. Moves 0° -> 45° -> 90°, then detaches.

Wiring:
- Signal -> BCM GPIO18 (physical pin 12)
- Servo V+ -> external 6–7.4 V supply (NOT the Pi’s 5V)
- GNDs common: servo GND <-> Pi GND
"""
from time import sleep
import sys

from gpiozero import Device, AngularServo

# Force lgpio backend (no pigpio, no host)
try:
    from gpiozero.pins.lgpio import LGPIOFactory
    Device.pin_factory = LGPIOFactory()  # will fail if /dev/gpiochip* isn't available
except Exception as e:
    print("Failed to load lgpio backend:", e)
    print("Fix:\n"
          "  1) apt install python3-lgpio inside the container\n"
          "  2) start container with --device=/dev/gpiomem and --device=/dev/gpiochipN\n")
    sys.exit(1)

# Define the servo (per your spec: 1500–2500 µs across 0–90°)
servo = AngularServo(
    18,                      # BCM pin
    min_angle=0, max_angle=90,
    min_pulse_width=0.0015,  # 1500 µs
    max_pulse_width=0.0025,  # 2500 µs
    frame_width=0.020        # 20 ms
)

try:
    print("→ angle = 0°");   servo.angle = 0;   sleep(1.0)
    print("→ angle = 45°");  servo.angle = 45;  sleep(1.0)
    print("→ angle = 90°");  servo.angle = 90;  sleep(1.0)
    print("Done. Detaching PWM.")
finally:
    try:
        servo.detach()
        servo.close()
    except Exception:
        pass
