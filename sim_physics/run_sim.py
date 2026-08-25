"""
Loads the spider robot into PyBullet with real gravity/ground contact and
drives it through a chosen gait's keyframes, reporting whether it stays
upright.

robot_config.py's dimensions are informed estimates measured from a
same-family open-source design's real STL files, not exact measurements of
this specific unit -- see robot_config.py's docstring for what's measured
vs. estimated, and for how to swap in real measurements later.

Usage (from this directory):
  venv\\Scripts\\python.exe run_sim.py wave
  venv\\Scripts\\python.exe run_sim.py trot_original
  venv\\Scripts\\python.exe run_sim.py wave --headless --cycles 5
"""

import argparse
import math
import time

import pybullet as p
import pybullet_data

import robot_config as cfg
from robot_model import build_spider_robot
from servo_map import LEGS, LEG_CHANNELS, SERVO_CENTER_DEG
from gaits import GAITS

# MG90S real stall torque is ~2.2 kgf-cm at 4.8V (~0.216 N-m) -- NOT the
# arbitrary force=2.0 N-m this script used to simulate with, which was
# roughly 10x stronger than the real actuator and would mask real tracking
# error under load. Loaded-servo speed is ~2.5ms/degree per the earlier
# gait research (real-world CircuitDigest measurement, not a spec-sheet
# no-load number), which is ~6.98 rad/s; used a bit under that (5.0) since
# a walking robot's load varies through the gait rather than being constant.
SERVO_MAX_TORQUE_NM = 0.22
SERVO_MAX_VELOCITY_RAD_S = 5.0


def servo_deg_to_joint_rad(servo_deg, sign):
    return (servo_deg - SERVO_CENTER_DEG) * (math.pi / 180.0) * sign


def apply_frame(body_id, joints, frame):
    for leg in LEGS:
        ch = LEG_CHANNELS[leg]
        hip_target = servo_deg_to_joint_rad(frame[ch["hip_idx"]], ch["swing_sign"])
        knee_target = servo_deg_to_joint_rad(frame[ch["knee_idx"]], ch["lift_sign"])
        p.setJointMotorControl2(
            body_id, joints[f"{leg}_hip"], p.POSITION_CONTROL,
            targetPosition=hip_target,
            force=SERVO_MAX_TORQUE_NM, maxVelocity=SERVO_MAX_VELOCITY_RAD_S,
        )
        p.setJointMotorControl2(
            body_id, joints[f"{leg}_knee"], p.POSITION_CONTROL,
            targetPosition=knee_target,
            force=SERVO_MAX_TORQUE_NM, maxVelocity=SERVO_MAX_VELOCITY_RAD_S,
        )


def run(gait_name, cycles, headless):
    frames = GAITS[gait_name]

    p.connect(p.DIRECT if headless else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")

    body_id, joints = build_spider_robot()

    # Let the robot settle onto its feet under gravity before walking.
    for _ in range(120):
        p.stepSimulation()
        if not headless:
            time.sleep(1.0 / 240.0)

    fell = False
    fall_reason = None
    max_tilt_deg = 0.0
    min_height = 999.0
    completed_cycles = 0

    for cycle in range(cycles):
        for frame in frames:
            apply_frame(body_id, joints, frame)
            steps = max(1, int(frame[8] / 1000.0 * 240))
            for _ in range(steps):
                p.stepSimulation()
                if not headless:
                    time.sleep(1.0 / 240.0)

                pos, orn = p.getBasePositionAndOrientation(body_id)
                euler = p.getEulerFromQuaternion(orn)
                tilt_deg = max(abs(math.degrees(euler[0])), abs(math.degrees(euler[1])))
                max_tilt_deg = max(max_tilt_deg, tilt_deg)
                min_height = min(min_height, pos[2])

                if tilt_deg > 45 or pos[2] < cfg.TIBIA_LENGTH_M * 0.3:
                    fell = True
                    fall_reason = (
                        f"tilt={tilt_deg:.1f}deg height={pos[2] * 1000:.0f}mm "
                        f"during cycle {cycle + 1}"
                    )
                    break
            if fell:
                break
        if fell:
            break
        completed_cycles = cycle + 1

    print(f"gait={gait_name}")
    print(f"cycles_completed={completed_cycles}/{cycles}")
    print(f"max_tilt_deg={max_tilt_deg:.1f}  min_height_mm={min_height * 1000:.1f}")
    if fell:
        print(f"RESULT: FELL OVER -- {fall_reason}")
    else:
        print("RESULT: stayed upright for the full run")

    p.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("gait", choices=list(GAITS.keys()))
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    run(args.gait, args.cycles, args.headless)
