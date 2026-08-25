"""Renders a still image of the robot at rest (standby pose, settled under
gravity) to a PNG, so geometry/stance correctness can be checked visually
without a screen-capture tool."""
import sys

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

from robot_model import build_spider_robot

out_path = sys.argv[1] if len(sys.argv) > 1 else "snapshot.png"

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)
p.loadURDF("plane.urdf")

body_id, joints = build_spider_robot()

for _ in range(240):
    p.stepSimulation()

view = p.computeViewMatrix(
    cameraEyePosition=[0.22, -0.22, 0.16],
    cameraTargetPosition=[0, 0, 0.03],
    cameraUpVector=[0, 0, 1],
)
proj = p.computeProjectionMatrixFOV(fov=45, aspect=1.4, nearVal=0.01, farVal=2.0)
w, h, rgba, _, _ = p.getCameraImage(980, 700, view, proj, renderer=p.ER_TINY_RENDERER)

img = Image.fromarray(np.reshape(rgba, (h, w, 4)).astype(np.uint8), "RGBA").convert("RGB")
img.save(out_path)
print(f"saved {out_path}")
