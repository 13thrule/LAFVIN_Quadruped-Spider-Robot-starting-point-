"""
Interactive test suite: drive every gait/pose/performance program live in
the PyBullet GUI, with on-screen telemetry and one-key fall recovery.

Run (from this directory):
  venv\\Scripts\\python.exe interactive_sim.py

Controls (click the PyBullet window first so it has keyboard focus):
  Locomotion (hold to keep going, matches the firmware's hold-to-repeat):
    W forward   S backward   A left-shift   D right-shift
    Q turn-left E turn-right T original trot (comparison, hold-capable)
  Poses / performance (one press, plays once, returns to standby):
    1 standby   2 lie       3 hello      4 fighting  5 push-up
    6 sleep     7 dance 1   8 dance 2    9 dance 3   0 center
    - zero
  R  reset (re-spawns the robot at the start pose/position -- use after a
     fall, or any time you want a clean restart)
  ESC / close window to quit
"""
import math
import time

import pybullet as p
import pybullet_data

import robot_config as cfg
from robot_model import build_spider_robot
from servo_map import LEGS, LEG_CHANNELS, SERVO_CENTER_DEG
from gaits import GAITS, HOLDABLE
from run_sim import SERVO_MAX_TORQUE_NM, SERVO_MAX_VELOCITY_RAD_S, servo_deg_to_joint_rad

# ASCII codes for the locomotion (hold-capable) keys.
LOCOMOTION_KEYS = {
    ord("w"): "forward",
    ord("s"): "backward",
    ord("a"): "leftshift",
    ord("d"): "rightshift",
    ord("q"): "turnleft",
    ord("e"): "turnright",
    ord("t"): "trot_original",
}

# One-shot pose/performance keys.
ONE_SHOT_KEYS = {
    ord("1"): "standby",
    ord("2"): "lie",
    ord("3"): "hello",
    ord("4"): "fighting",
    ord("5"): "pushup",
    ord("6"): "sleep",
    ord("7"): "dance1",
    ord("8"): "dance2",
    ord("9"): "dance3",
    ord("0"): "center",
    ord("-"): "zero",
}

FALL_TILT_DEG = 45
STATUS_TEXT_ID = None
LEGEND_TEXT_ID = None


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


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.40, cameraYaw=35, cameraPitch=-25,
        cameraTargetPosition=[0, 0, 0.04],
    )

    body_id, joints = build_spider_robot()
    start_pos, start_orn = p.getBasePositionAndOrientation(body_id)

    print(__doc__)

    state = {
        "gait": "standby",
        "frame_index": 0,
        "elapsed": 0.0,
        "holding": False,
        "fell": False,
        "last_status_update": 0.0,
    }

    def reset_robot():
        p.resetBasePositionAndOrientation(body_id, start_pos, start_orn)
        p.resetBaseVelocity(body_id, [0, 0, 0], [0, 0, 0])
        state["gait"] = "standby"
        state["frame_index"] = 0
        state["elapsed"] = 0.0
        state["holding"] = False
        state["fell"] = False

    def switch_gait(name):
        if state["gait"] != name:
            state["gait"] = name
            state["frame_index"] = 0
            state["elapsed"] = 0.0

    last_time = time.time()
    global STATUS_TEXT_ID, LEGEND_TEXT_ID
    STATUS_TEXT_ID = p.addUserDebugText("", [0, 0, 0.12], textColorRGB=[1, 1, 1], textSize=1.3)
    LEGEND_TEXT_ID = p.addUserDebugText(
        "WASD/QE=walk  T=trot  1-9,0,-=poses  R=reset",
        [0, 0, -0.02], textColorRGB=[0.6, 0.7, 0.9], textSize=1.0,
    )

    while p.isConnected():
        now = time.time()
        dt = now - last_time
        last_time = now

        keys = p.getKeyboardEvents()

        if keys.get(ord("r"), 0) & p.KEY_WAS_TRIGGERED:
            reset_robot()

        held_locomotion = None
        for key_code, gait_name in LOCOMOTION_KEYS.items():
            if keys.get(key_code, 0) & p.KEY_IS_DOWN:
                held_locomotion = gait_name
                break

        if held_locomotion:
            state["holding"] = True
            switch_gait(held_locomotion)
        else:
            if state["holding"]:
                # Just released -- let the current stride finish naturally,
                # matching the firmware's own hold-to-repeat behavior rather
                # than freezing mid-step.
                state["holding"] = False

        for key_code, gait_name in ONE_SHOT_KEYS.items():
            if keys.get(key_code, 0) & p.KEY_WAS_TRIGGERED:
                state["holding"] = False
                switch_gait(gait_name)

        frames = GAITS[state["gait"]]
        state["elapsed"] += dt * 1000.0  # ms
        frame = frames[state["frame_index"]]

        while state["elapsed"] >= frame[8]:
            state["elapsed"] -= frame[8]
            state["frame_index"] += 1
            if state["frame_index"] >= len(frames):
                if state["holding"] and state["gait"] in HOLDABLE:
                    state["frame_index"] = 0
                else:
                    # One-shot action finished (or button released) --
                    # settle back to standby, same as the firmware returning
                    # to a resting pose after a program completes.
                    if state["gait"] != "standby":
                        switch_gait("standby")
                    else:
                        state["frame_index"] = len(frames) - 1
                    break
            frame = frames[state["frame_index"]]

        apply_frame(body_id, joints, frame)
        p.stepSimulation()

        pos, orn = p.getBasePositionAndOrientation(body_id)
        euler = p.getEulerFromQuaternion(orn)
        tilt_deg = max(abs(math.degrees(euler[0])), abs(math.degrees(euler[1])))
        if tilt_deg > FALL_TILT_DEG:
            state["fell"] = True

        if now - state["last_status_update"] > 0.15:
            state["last_status_update"] = now
            status = "FELL OVER -- press R to reset" if state["fell"] else "OK"
            text = (
                f"gait: {state['gait']}   tilt: {tilt_deg:4.1f} deg   "
                f"height: {pos[2] * 1000:5.1f} mm   [{status}]"
            )
            color = [1, 0.3, 0.3] if state["fell"] else [1, 1, 1]
            STATUS_TEXT_ID = p.addUserDebugText(
                text, [pos[0] - 0.05, pos[1], pos[2] + 0.10],
                textColorRGB=color, textSize=1.3, replaceItemUniqueId=STATUS_TEXT_ID,
            )

        time.sleep(max(0.0, 1.0 / 240.0 - (time.time() - now)))


if __name__ == "__main__":
    main()
