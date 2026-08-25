"""
Interactive test suite: drive every gait/pose/performance program live in
the PyBullet GUI, with real contact-based foot telemetry, a live support-
polygon view, camera auto-follow, speed control, and one-key fall recovery.

Run (from this directory):
  venv\\Scripts\\python.exe interactive_sim.py

Controls (click the PyBullet window first so it has keyboard focus):
  Locomotion (hold to keep going, matches the firmware's hold-to-repeat):
    W forward   S backward   A left-shift   D right-shift
    Q turn-left E turn-right T original trot (comparison, hold-capable)
  Poses / performance (one press, plays once, returns to standby):
    1 standby   2 lie       3 hello      4 fighting  5 push-up
    6 sleep     7 dance 1   8 dance 2    9 dance 3   0 center
    - zero       G frog-hop (crouch, explosive launch, land -- one-shot)
  View:
    C front view   V side view   B top-down view   N reset/iso view
    F toggle camera auto-follow (on by default; turn off to orbit freely
      with the mouse -- auto-follow fights manual orbiting otherwise)
  Playback:
    ]  speed up gait playback    [  slow down    \\  reset speed to 1x
    SPACE  pause / resume
  R  reset (re-spawns the robot at the start pose/position -- use after a
     fall, or any time you want a clean restart)
  ESC / close window to quit

The foot markers and support-polygon lines are driven by real PyBullet
contact points (getContactPoints), not a guessed lift threshold -- so what
you see is genuinely which feet are touching the ground plane, and the
polygon is the actual support region, live.
"""
import math
import time

import pybullet as p
import pybullet_data

import robot_config as cfg
from robot_model import build_spider_robot, LEG_COLOR_NAMES
from servo_map import LEGS, LEG_CHANNELS, SERVO_CENTER_DEG
from gaits import GAITS, HOLDABLE
from run_sim import SERVO_MAX_TORQUE_NM, SERVO_MAX_VELOCITY_RAD_S, servo_deg_to_joint_rad

LEG_LABELS = {"RF": "right-front", "LF": "left-front", "RR": "right-rear", "LR": "left-rear"}
COLOR_KEY = "  ".join(f"{leg}={LEG_COLOR_NAMES[leg]}({LEG_LABELS[leg]})" for leg in LEGS)

LOCOMOTION_KEYS = {
    ord("w"): "forward",
    ord("s"): "backward",
    ord("a"): "leftshift",
    ord("d"): "rightshift",
    ord("q"): "turnleft",
    ord("e"): "turnright",
    ord("t"): "trot_original",
}

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
    ord("g"): "froghop",
}

# yaw, pitch, distance
CAMERA_PRESETS = {
    ord("c"): (0, -10, 0.30),     # front
    ord("v"): (90, -10, 0.30),    # side
    ord("b"): (0, -89, 0.32),     # top-down
    ord("n"): (35, -25, 0.40),    # default iso
}

FALL_TILT_DEG = 45
TELEMETRY_INTERVAL_S = 0.15
FOOT_MARK_HALF_M = 0.008


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


def foot_world_position(body_id, tibia_link_index):
    link_state = p.getLinkState(body_id, tibia_link_index, computeForwardKinematics=True)
    link_pos, link_orn = link_state[4], link_state[5]
    tip, _ = p.multiplyTransforms(link_pos, link_orn, [0, 0, -cfg.TIBIA_LENGTH_M / 2], [0, 0, 0, 1])
    return tip


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # PyBullet's GUI connection can otherwise step physics on its own
    # internal clock in addition to our explicit stepSimulation() calls
    # below (double-stepping) -- a known cause of "fine headless, falls
    # over live" that bit this exact script once already.
    p.setRealTimeSimulation(0)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    body_id, joints, links = build_spider_robot()
    start_pos, start_orn = p.getBasePositionAndOrientation(body_id)
    tibia_link = {leg: links[f"{leg}_tibia_fixed"] for leg in LEGS}

    print(__doc__)

    cam = {"yaw": 35.0, "pitch": -25.0, "distance": 0.40, "follow": True,
           "target": [0.0, 0.0, 0.04]}
    p.resetDebugVisualizerCamera(cam["distance"], cam["yaw"], cam["pitch"], cam["target"])

    state = {
        "gait": "standby", "frame_index": 0, "elapsed": 0.0,
        "holding": False, "fell": False, "paused": False, "speed": 1.0,
        "last_telemetry": 0.0,
    }

    def reset_robot():
        p.resetBasePositionAndOrientation(body_id, start_pos, start_orn)
        p.resetBaseVelocity(body_id, [0, 0, 0], [0, 0, 0])
        state.update(gait="standby", frame_index=0, elapsed=0.0, holding=False, fell=False)

    def switch_gait(name):
        if state["gait"] != name:
            state["gait"] = name
            state["frame_index"] = 0
            state["elapsed"] = 0.0

    text_ids = {
        "status": p.addUserDebugText("", [0, 0, 0.14], textColorRGB=[1, 1, 1], textSize=1.3),
        "servos": p.addUserDebugText("", [0, 0, 0.11], textColorRGB=[0.7, 0.85, 1], textSize=1.0),
        "legend": p.addUserDebugText(
            "WASD/QE walk  T trot  G froghop  1-9,0,- poses  CVBN camera  F follow  []=speed  SPACE pause  R reset",
            [0, 0, -0.02], textColorRGB=[0.6, 0.7, 0.9], textSize=1.0,
        ),
        "color_key": p.addUserDebugText(
            COLOR_KEY, [0, 0, -0.035], textColorRGB=[0.85, 0.85, 0.85], textSize=1.0,
        ),
    }
    foot_line_ids = {leg: (
        p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0], lineWidth=4),
        p.addUserDebugLine([0, 0, 0], [0, 0, 0], [0, 1, 0], lineWidth=4),
    ) for leg in LEGS}
    polygon_line_ids = [p.addUserDebugLine([0, 0, 0], [0, 0, 0], [1, 0.7, 0.1], lineWidth=2) for _ in range(4)]

    last_time = time.time()
    PHYSICS_DT = 1.0 / 240.0
    accumulator = 0.0

    while p.isConnected():
        now = time.time()
        frame_dt = min(now - last_time, 0.25)
        last_time = now

        keys = p.getKeyboardEvents()

        if keys.get(ord("r"), 0) & p.KEY_WAS_TRIGGERED:
            reset_robot()
        if keys.get(ord(" "), 0) & p.KEY_WAS_TRIGGERED:
            state["paused"] = not state["paused"]
        if keys.get(ord("]"), 0) & p.KEY_WAS_TRIGGERED:
            state["speed"] = min(4.0, round(state["speed"] + 0.25, 2))
        if keys.get(ord("["), 0) & p.KEY_WAS_TRIGGERED:
            state["speed"] = max(0.25, round(state["speed"] - 0.25, 2))
        if keys.get(ord("\\"), 0) & p.KEY_WAS_TRIGGERED:
            state["speed"] = 1.0
        if keys.get(ord("f"), 0) & p.KEY_WAS_TRIGGERED:
            cam["follow"] = not cam["follow"]
        for key_code, preset in CAMERA_PRESETS.items():
            if keys.get(key_code, 0) & p.KEY_WAS_TRIGGERED:
                cam["yaw"], cam["pitch"], cam["distance"] = preset

        held_locomotion = None
        for key_code, gait_name in LOCOMOTION_KEYS.items():
            if keys.get(key_code, 0) & p.KEY_IS_DOWN:
                held_locomotion = gait_name
                break

        if held_locomotion:
            state["holding"] = True
            switch_gait(held_locomotion)
        else:
            state["holding"] = False

        for key_code, gait_name in ONE_SHOT_KEYS.items():
            if keys.get(key_code, 0) & p.KEY_WAS_TRIGGERED:
                state["holding"] = False
                switch_gait(gait_name)

        if not state["paused"]:
            accumulator += frame_dt * state["speed"]

        # Fixed-timestep accumulator: gait state and physics advance
        # together in fixed PHYSICS_DT increments, however many are needed
        # to catch up -- keeps real-time framerate variance (and now the
        # speed multiplier) from ever letting commanded servo targets get
        # ahead of what the physics has actually simulated.
        while accumulator >= PHYSICS_DT:
            frames = GAITS[state["gait"]]
            state["elapsed"] += PHYSICS_DT * 1000.0
            frame = frames[state["frame_index"]]

            while state["elapsed"] >= frame[8]:
                state["elapsed"] -= frame[8]
                state["frame_index"] += 1
                if state["frame_index"] >= len(frames):
                    if state["holding"] and state["gait"] in HOLDABLE:
                        state["frame_index"] = 0
                    else:
                        if state["gait"] != "standby":
                            switch_gait("standby")
                        else:
                            state["frame_index"] = len(frames) - 1
                        break
                frame = frames[state["frame_index"]]

            apply_frame(body_id, joints, frame)
            p.stepSimulation()
            accumulator -= PHYSICS_DT

        pos, orn = p.getBasePositionAndOrientation(body_id)
        euler = p.getEulerFromQuaternion(orn)
        tilt_deg = max(abs(math.degrees(euler[0])), abs(math.degrees(euler[1])))
        if tilt_deg > FALL_TILT_DEG:
            state["fell"] = True

        if cam["follow"]:
            cam["target"][0] += (pos[0] - cam["target"][0]) * min(1.0, frame_dt * 4.0)
            cam["target"][1] += (pos[1] - cam["target"][1]) * min(1.0, frame_dt * 4.0)
            p.resetDebugVisualizerCamera(cam["distance"], cam["yaw"], cam["pitch"], cam["target"])

        if now - state["last_telemetry"] > TELEMETRY_INTERVAL_S:
            state["last_telemetry"] = now

            grounded = {}
            foot_pos = {}
            for leg in LEGS:
                contacts = p.getContactPoints(bodyA=body_id, linkIndexA=tibia_link[leg])
                grounded[leg] = len(contacts) > 0
                foot_pos[leg] = foot_world_position(body_id, tibia_link[leg])

            for leg in LEGS:
                fx, fy, fz = foot_pos[leg]
                color = [0.25, 0.9, 0.35] if grounded[leg] else [0.9, 0.35, 0.25]
                a_id, b_id = foot_line_ids[leg]
                p.addUserDebugLine(
                    [fx - FOOT_MARK_HALF_M, fy, fz + 0.001], [fx + FOOT_MARK_HALF_M, fy, fz + 0.001],
                    color, lineWidth=4, replaceItemUniqueId=a_id,
                )
                p.addUserDebugLine(
                    [fx, fy - FOOT_MARK_HALF_M, fz + 0.001], [fx, fy + FOOT_MARK_HALF_M, fz + 0.001],
                    color, lineWidth=4, replaceItemUniqueId=b_id,
                )

            grounded_legs = [leg for leg in LEGS if grounded[leg]]
            edges = []
            if len(grounded_legs) >= 2:
                cx = sum(foot_pos[leg][0] for leg in grounded_legs) / len(grounded_legs)
                cy = sum(foot_pos[leg][1] for leg in grounded_legs) / len(grounded_legs)
                ordered = sorted(
                    grounded_legs,
                    key=lambda leg: math.atan2(foot_pos[leg][1] - cy, foot_pos[leg][0] - cx),
                )
                n = len(ordered)
                edges = [(ordered[i], ordered[(i + 1) % n]) for i in range(n)] if n >= 3 else [(ordered[0], ordered[1])]
            for i, line_id in enumerate(polygon_line_ids):
                if i < len(edges):
                    a, b = edges[i]
                    ax, ay, az = foot_pos[a]
                    bx, by, bz = foot_pos[b]
                    p.addUserDebugLine([ax, ay, az + 0.002], [bx, by, bz + 0.002],
                                        [1, 0.7, 0.1], lineWidth=2, replaceItemUniqueId=line_id)
                else:
                    p.addUserDebugLine([0, 0, -1], [0, 0, -1], [1, 0.7, 0.1], replaceItemUniqueId=line_id)

            status = "FELL -- press R" if state["fell"] else ("PAUSED" if state["paused"] else f"{len(grounded_legs)} feet down")
            color = [1, 0.3, 0.3] if state["fell"] else [1, 1, 1]
            status_text = (
                f"gait: {state['gait']:14s} tilt: {tilt_deg:4.1f}deg  height: {pos[2] * 1000:5.1f}mm  "
                f"speed: {state['speed']:.2f}x  [{status}]"
            )
            text_ids["status"] = p.addUserDebugText(
                status_text, [pos[0] - 0.06, pos[1], pos[2] + 0.10],
                textColorRGB=color, textSize=1.3, replaceItemUniqueId=text_ids["status"],
            )

            frame = GAITS[state["gait"]][state["frame_index"]]
            servo_text = "G14={:3d} G12={:3d} G13={:3d} G15={:3d} G16={:3d} G5={:3d} G4={:3d} G2={:3d}".format(*frame[:8])
            text_ids["servos"] = p.addUserDebugText(
                servo_text, [pos[0] - 0.06, pos[1], pos[2] + 0.075],
                textColorRGB=[0.7, 0.85, 1], textSize=1.0, replaceItemUniqueId=text_ids["servos"],
            )

        time.sleep(max(0.0, 1.0 / 240.0 - (time.time() - now)))


if __name__ == "__main__":
    main()
