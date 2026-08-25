"""
Moves one or more servos to their zero pose + a trim offset over serial,
so alignment can be checked by eye. All pairs given in one call are sent
over a single serial connection, so they land together without the board
rebooting (and undoing earlier moves) between them -- each new connection
to this board toggles DTR and resets it, so this only works within one
open connection, not across separate script runs.

Usage (from this directory):
  venv\\Scripts\\python.exe calibrate.py COM5 14 5
  venv\\Scripts\\python.exe calibrate.py COM5 12 8 13 8 5 -8 4 -8

Valid pins: 14, 12, 13, 15, 16, 5, 4, 2 (the 8 servo GPIOs).

NOTE: this moves the servos live so the effect can be checked, but does NOT
persist the offsets anywhere -- matches the existing S,<pin>,<offset> serial
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
    parser.add_argument("values", nargs="+", type=int, help="pin offset [pin offset ...]")
    parser.add_argument("--baud", type=int, default=9600)
    args = parser.parse_args()

    if len(args.values) % 2 != 0:
        raise SystemExit("expected pairs: <pin> <offset> [<pin> <offset> ...]")
    pairs = list(zip(args.values[0::2], args.values[1::2]))
    for pin, _ in pairs:
        if pin not in VALID_PINS:
            raise SystemExit(f"pin {pin} not in {sorted(VALID_PINS)}")

    ser = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(2.0)  # let the board finish its post-open reset/boot
    ser.reset_input_buffer()

    for pin, offset in pairs:
        ser.write(f"S,{pin},{offset}\n".encode("utf-8"))
        deadline = time.time() + 2.0
        replied = False
        while time.time() < deadline:
            reply = ser.readline().decode("utf-8", errors="ignore").strip()
            if reply:
                print(reply)
                replied = True
                break
        if not replied:
            print(f"(no reply for pin {pin} within 2s)")
        time.sleep(0.1)

    ser.close()


if __name__ == "__main__":
    main()
