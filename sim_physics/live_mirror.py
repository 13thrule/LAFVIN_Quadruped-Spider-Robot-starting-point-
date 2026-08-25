"""
Reads the real robot's serial telemetry (POS:<8 angles>,<ms> per keyframe,
added to spiderbot_motion.cpp's Servo_PROGRAM_Run) and drives the PyBullet
sim to mirror it live -- same interpolation approach as interactive_sim.py,
just fed by the real robot instead of a local gait table.

Requires the firmware with the POS: telemetry line (see
firmware/0.SpiderBot/spiderbot_motion.cpp) already flashed and the board
connected over USB, with Serial idle otherwise (same TX/RX-sharing
constraint as everything else on this board -- don't have the ultrasonic
sensor's calibration serial commands fighting this at the same time).

Usage (from this directory):
  venv\\Scripts\\python.exe live_mirror.py COM5
  venv\\Scripts\\python.exe live_mirror.py COM5 --baud 9600
"""
import argparse
import math
import queue
import re
import threading
import time

import pybullet as p
import pybullet_data
import serial

import robot_config as cfg
from robot_model import build_spider_robot, LEG_COLOR_NAMES
from servo_map import LEGS, LEG_CHANNELS, SERVO_CENTER_DEG
from run_sim import SERVO_MAX_TORQUE_NM, SERVO_MAX_VELOCITY_RAD_S, servo_deg_to_joint_rad
from interactive_sim import apply_frame, foot_world_position

LEG_LABELS = {"RF": "right-front", "LF": "left-front", "RR": "right-rear", "LR": "left-rear"}
COLOR_KEY = "  ".join(f"{leg}={LEG_COLOR_NAMES[leg]}({LEG_LABELS[leg]})" for leg in LEGS)

POS_LINE = re.compile(r"POS:([\d,]+)")
FALL_TILT_DEG = 45
TELEMETRY_INTERVAL_S = 0.15


def serial_reader(port, baud, out_queue, stop_event):
    ser = serial.Serial(port, baud, timeout=1)
    print(f"Listening on {port} @ {baud} baud...")
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except serial.SerialException as exc:
            print(f"serial error: {exc}")
            break
        if not line:
            continue
        match = POS_LINE.search(line)
        if not match:
            continue
        parts = [int(v) for v in match.group(1).split(",") if v != ""]
        if len(parts) != 9:
            continue
        out_queue.put((parts[:8], parts[8]))
    ser.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("--baud", type=int, default=9600)
    args = parser.parse_args()

    incoming = queue.Queue()
    stop_event = threading.Event()
    reader = threading.Thread(target=serial_reader, args=(args.port, args.baud, incoming, stop_event), daemon=True)
    reader.start()

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setRealTimeSimulation(0)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.resetDebugVisualizerCamera(0.40, 35, -25, [0, 0, 0.04])

    body_id, joints, links = build_spider_robot()
    tibia_link = {leg: links[f"{leg}_tibia_fixed"] for leg in LEGS}

    status_id = p.addUserDebugText("waiting for robot...", [0, 0, 0.14], textColorRGB=[1, 1, 0.6], textSize=1.3)
    p.addUserDebugText(
        f"mirroring real robot on {args.port} -- move it with the IR remote or app",
        [0, 0, -0.02], textColorRGB=[0.6, 0.7, 0.9], textSize=1.0,
    )
    p.addUserDebugText(COLOR_KEY, [0, 0, -0.035], textColorRGB=[0.85, 0.85, 0.85], textSize=1.0)

    PHYSICS_DT = 1.0 / 240.0
    accumulator = 0.0
    last_time = time.time()
    current_target = None  # (angles, ms)
    frame_start_angles = [SERVO_CENTER_DEG] * 8
    elapsed = 0.0
    last_telemetry = 0.0
    lines_seen = 0

    while p.isConnected():
        now = time.time()
        frame_dt = min(now - last_time, 0.25)
        last_time = now
        accumulator += frame_dt

        try:
            while True:
                angles, ms = incoming.get_nowait()
                lines_seen += 1
                if current_target is not None:
                    frame_start_angles = current_target[0]
                current_target = (angles, ms)
                elapsed = 0.0
        except queue.Empty:
            pass

        while accumulator >= PHYSICS_DT:
            if current_target is not None:
                target_angles, ms = current_target
                elapsed += PHYSICS_DT * 1000.0
                t = min(1.0, elapsed / ms) if ms > 0 else 1.0
                blended = [
                    frame_start_angles[i] + (target_angles[i] - frame_start_angles[i]) * t
                    for i in range(8)
                ]
                apply_frame(body_id, joints, blended + [ms])
            p.stepSimulation()
            accumulator -= PHYSICS_DT

        pos, orn = p.getBasePositionAndOrientation(body_id)
        euler = p.getEulerFromQuaternion(orn)
        tilt_deg = max(abs(math.degrees(euler[0])), abs(math.degrees(euler[1])))

        if now - last_telemetry > TELEMETRY_INTERVAL_S:
            last_telemetry = now
            grounded = sum(1 for leg in LEGS if p.getContactPoints(bodyA=body_id, linkIndexA=tibia_link[leg]))
            state_txt = "no data yet" if current_target is None else f"{lines_seen} keyframes received"
            fall_txt = " FELL" if tilt_deg > FALL_TILT_DEG else ""
            text = (
                f"{state_txt}  tilt:{tilt_deg:4.1f}deg  height:{pos[2]*1000:5.1f}mm  "
                f"feet down:{grounded}{fall_txt}"
            )
            color = [1, 0.4, 0.4] if tilt_deg > FALL_TILT_DEG else [1, 1, 1]
            status_id = p.addUserDebugText(
                text, [pos[0] - 0.06, pos[1], pos[2] + 0.10],
                textColorRGB=color, textSize=1.3, replaceItemUniqueId=status_id,
            )

        time.sleep(max(0.0, 1.0 / 240.0 - (time.time() - now)))

    stop_event.set()


if __name__ == "__main__":
    main()
