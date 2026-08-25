"""
Moves one servo to its zero pose + a trim offset over serial, so alignment
can be checked by eye and dialed in one value at a time.

Usage (from this directory):
  venv\\Scripts\\python.exe calibrate.py COM5 14 5
  venv\\Scripts\\python.exe calibrate.py COM5 14 -3

Valid pins: 14, 12, 13, 15, 16, 5, 4, 2 (the 8 servo GPIOs).

NOTE: this moves the servo live so the effect can be checked, but does NOT
persist the offset anywhere -- matches the existing S,<pin>,<offset> serial
command's behavior (bench-test-only), which is different from the EEPROM-
backed calibration the web API's /api/v1/calibration endpoint writes. Once
the right offsets are known, making them stick permanently needs a small
separate firmware addition -- ask if that's wanted once values are dialed in.
"""
import argparse
import time

import serial

VALID_PINS = {14, 12, 13, 15, 16, 5, 4, 2}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("pin", type=int, choices=sorted(VALID_PINS))
    parser.add_argument("offset", type=int)
    parser.add_argument("--baud", type=int, default=9600)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(2.0)  # let the board finish its post-open reset/boot
    ser.reset_input_buffer()

    ser.write(f"S,{args.pin},{args.offset}\n".encode("utf-8"))

    deadline = time.time() + 2.0
    while time.time() < deadline:
        reply = ser.readline().decode("utf-8", errors="ignore").strip()
        if reply:
            print(reply)
            break
    else:
        print("(no reply within 2s)")

    ser.close()


if __name__ == "__main__":
    main()
