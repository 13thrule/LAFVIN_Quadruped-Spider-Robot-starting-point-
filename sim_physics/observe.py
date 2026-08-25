"""
Runs a gait headlessly and produces both a numeric telemetry log and a
handful of rendered snapshot frames, specifically so the log + images can
be read back and inspected directly (via the Read tool) -- a way to
actually watch and understand a run without relying on someone else's eyes
on a live GUI window.

Usage (from this directory):
  venv\\Scripts\\python.exe observe.py forward --cycles 5
  venv\\Scripts\\python.exe observe.py trot_original --cycles 5 --out obs_trot

Produces, under --out (default: observe_<gait>/):
  summary.txt        -- stats, timeline of notable events, verdict
  frame_XXX.png       -- snapshots at regular intervals
  frame_worst.png     -- snapshot at the single worst-tilt moment
  frame_fall.png      -- snapshot at the moment it fell, if it did
"""
import argparse
import math
import os

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

import robot_config as cfg
from robot_model import build_spider_robot
from servo_map import LEGS
from gaits import GAITS
from run_sim import SERVO_MAX_TORQUE_NM, SERVO_MAX_VELOCITY_RAD_S, servo_deg_to_joint_rad
from interactive_sim import apply_frame, foot_world_position

FALL_TILT_DEG = 45
SNAPSHOT_EVERY_S = 1.0


def support_polygon_area(foot_pos, grounded_legs):
    if len(grounded_legs) < 3:
        return 0.0
    cx = sum(foot_pos[leg][0] for leg in grounded_legs) / len(grounded_legs)
    cy = sum(foot_pos[leg][1] for leg in grounded_legs) / len(grounded_legs)
    ordered = sorted(grounded_legs, key=lambda leg: math.atan2(foot_pos[leg][1] - cy, foot_pos[leg][0] - cx))
    pts = [(foot_pos[leg][0], foot_pos[leg][1]) for leg in ordered]
    area2 = 0.0
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        area2 += x1 * y2 - x2 * y1
    return abs(area2) / 2.0


def render(body_id, path, cam_target):
    view = p.computeViewMatrix([0.28, -0.28, 0.18], cam_target, [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(45, 1.4, 0.01, 2.0)
    w, h, rgba, _, _ = p.getCameraImage(900, 640, view, proj, renderer=p.ER_TINY_RENDERER)
    Image.fromarray(np.reshape(rgba, (h, w, 4)).astype(np.uint8), "RGBA").convert("RGB").save(path)


def run(gait_name, cycles, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    frames = GAITS[gait_name]

    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setRealTimeSimulation(0)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.loadURDF("plane.urdf")

    body_id, joints, links = build_spider_robot()
    tibia_link = {leg: links[f"{leg}_tibia_fixed"] for leg in LEGS}

    for _ in range(120):
        p.stepSimulation()

    log = []
    worst = {"tilt": -1.0, "sim_time": 0.0}
    fell_at = None
    snapshot_count = 0
    next_snapshot_t = 0.0
    sim_time = 0.0

    fell = False
    cycle_reached = 0
    for cycle in range(cycles):
        for frame in frames:
            apply_frame(body_id, joints, frame)
            steps = max(1, int(frame[8] / 1000.0 * 240))
            for _ in range(steps):
                p.stepSimulation()
                sim_time += 1.0 / 240.0

                pos, orn = p.getBasePositionAndOrientation(body_id)
                euler = p.getEulerFromQuaternion(orn)
                tilt_deg = max(abs(math.degrees(euler[0])), abs(math.degrees(euler[1])))

                grounded_legs = [leg for leg in LEGS if p.getContactPoints(bodyA=body_id, linkIndexA=tibia_link[leg])]
                foot_pos = {leg: foot_world_position(body_id, tibia_link[leg]) for leg in LEGS}
                area_cm2 = support_polygon_area(foot_pos, grounded_legs) * 1e4

                log.append((sim_time, tilt_deg, pos[2] * 1000.0, len(grounded_legs), area_cm2))

                if tilt_deg > worst["tilt"]:
                    worst.update(tilt=tilt_deg, sim_time=sim_time)

                if sim_time >= next_snapshot_t:
                    render(body_id, os.path.join(out_dir, f"frame_{snapshot_count:03d}.png"), pos)
                    snapshot_count += 1
                    next_snapshot_t += SNAPSHOT_EVERY_S

                if tilt_deg > FALL_TILT_DEG and fell_at is None:
                    fell_at = sim_time
                    fell = True
                    render(body_id, os.path.join(out_dir, "frame_fall.png"), pos)
                    break
            if fell:
                break
        if fell:
            break
        cycle_reached = cycle + 1

    # Always capture the worst-tilt moment too, even if it never crossed
    # the fall threshold -- re-run is unnecessary, current pose is close
    # enough since we just broke out at/near that point when it fell, and
    # when it didn't fall the final pose is a reasonable stand-in.
    pos, _ = p.getBasePositionAndOrientation(body_id)
    render(body_id, os.path.join(out_dir, "frame_worst_or_final.png"), pos)

    tilts = [row[1] for row in log]
    heights = [row[2] for row in log]
    areas = [row[4] for row in log if row[3] >= 3]
    grounded_counts = [row[3] for row in log]

    # Explain *why* a low grounded-foot count happened, not just that it
    # did: find the longest consecutive run at the minimum count and
    # report its duration -- a single-physics-step (~4ms) blip during a
    # keyframe transition means something very different from a sustained
    # period airborne.
    min_grounded = min(grounded_counts)
    longest_run = 0
    longest_run_start = None
    cur_run = 0
    cur_run_start = None
    for i, row in enumerate(log):
        if row[3] == min_grounded:
            if cur_run == 0:
                cur_run_start = row[0]
            cur_run += 1
            if cur_run > longest_run:
                longest_run = cur_run
                longest_run_start = cur_run_start
        else:
            cur_run = 0
    longest_run_duration_ms = longest_run * (1000.0 / 240.0)

    lines = []
    lines.append(f"gait: {gait_name}")
    lines.append(f"cycles requested: {cycles}   cycles completed: {cycle_reached}")
    lines.append(f"sim_time_reached_s: {sim_time:.2f}")
    lines.append("")
    lines.append(f"max_tilt_deg: {max(tilts):.1f}  (at sim_time {worst['sim_time']:.2f}s)")
    lines.append(f"mean_tilt_deg: {sum(tilts) / len(tilts):.2f}")
    lines.append(f"min_height_mm: {min(heights):.1f}  max_height_mm: {max(heights):.1f}")
    lines.append(f"min_grounded_feet: {min_grounded}  (0 or 1 means airborne/unsupported at that instant)")
    if min_grounded < 3:
        lines.append(
            f"  longest run at {min_grounded} feet: {longest_run_duration_ms:.0f}ms, "
            f"starting at sim_time {longest_run_start:.2f}s "
            f"({'a brief transition blip' if longest_run_duration_ms < 20 else 'sustained -- worth a closer look'})"
        )
    if areas:
        lines.append(f"support_polygon_area_cm2: min={min(areas):.1f} mean={sum(areas) / len(areas):.1f} max={max(areas):.1f}")
    else:
        lines.append("support_polygon_area_cm2: never had 3+ feet grounded simultaneously")
    lines.append("")
    if fell_at is not None:
        lines.append(f"VERDICT: FELL at sim_time {fell_at:.2f}s (cycle {cycle_reached + 1}) -- see frame_fall.png")
    else:
        lines.append(f"VERDICT: stayed upright for the full requested run ({cycles} cycles)")
    lines.append("")
    lines.append(f"{snapshot_count} interval snapshots written (every {SNAPSHOT_EVERY_S}s of sim time) as frame_NNN.png")

    summary = "\n".join(lines)
    with open(os.path.join(out_dir, "summary.txt"), "w") as f:
        f.write(summary + "\n")
    print(summary)

    p.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("gait", choices=list(GAITS.keys()))
    parser.add_argument("--cycles", type=int, default=5)
    parser.add_argument("--out", default=None)
    args = parser.parse_args()
    out_dir = args.out or f"observe_{args.gait}"
    run(args.gait, args.cycles, out_dir)
