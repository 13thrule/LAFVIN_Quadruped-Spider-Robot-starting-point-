"""
Sends an action command to the real robot over serial and prints its reply.

Usage (from this directory):
  venv\\Scripts\\python.exe send_action.py COM5 forward
  venv\\Scripts\\python.exe send_action.py COM5 dance1
  venv\\Scripts\\python.exe send_action.py COM5 stop

Action names match the web API's ?action= values / getActionName():
standby, forward, backward, leftshift, rightshift, turnleft, turnright,
lie, hello, fighting, pushup, sleep, dance1, dance2, dance3, avoid,
center, zero, stop (stop is calibration()'s own special case, not a
real program id).
"""
import argparse
import time

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("action")
    parser.add_argument("--baud", type=int, default=9600)
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(0.3)  # let the line settle after opening

    ser.write(f"A,{args.action}\n".encode("utf-8"))

    deadline = time.time() + 2.0
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            print(line)
        if line.startswith("A,") or line.startswith("ERR,"):
            break

    ser.close()


if __name__ == "__main__":
    main()
