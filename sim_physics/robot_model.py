"""
Procedurally builds the spider robot as a PyBullet multibody -- no separate
URDF file, so updating robot_config.py and re-running is the entire model
update path (no XML to keep in sync).

Leg model: each leg is a simple 2-DOF serial chain -- hip (swing) joint,
revolute about Z, mounted on the body; a fixed-length coxa link; a knee
(lift) joint, revolute about Y, at the end of the coxa; a fixed-length
tibia link down to the foot tip. This is the standard approximation for a
hobby quadruped leg and matches how sim/gait_lab.html already models the
same two servos per leg (swing direction + lift/retract). The real
mechanism may be a linkage rather than a bare hinge; this is an
approximation, not a measured kinematic model.
"""

import pybullet as p

import robot_config as cfg
from servo_map import LEGS

# (x_sign, y_sign) for each leg's hip mount position relative to body
# center, in a +X = forward, +Y = left, +Z = up frame.
HIP_SIGNS = {
    "RF": (+1, -1),
    "LF": (+1, +1),
    "RR": (-1, -1),
    "LR": (-1, +1),
}


def _box_shapes(half_extents, color):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    return col, vis


def build_spider_robot(start_height=None):
    """Builds the robot in the currently-connected PyBullet client.

    Returns (body_id, joint_index_by_name), where joint_index_by_name maps
    e.g. "RF_hip" / "RF_knee" to the PyBullet joint index for
    setJointMotorControl2.
    """
    body_half = [cfg.BODY_LENGTH_M / 2, cfg.BODY_WIDTH_M / 2, cfg.BODY_HEIGHT_M / 2]
    body_col, body_vis = _box_shapes(body_half, [0.2, 0.5, 0.9, 1])

    coxa_half = [cfg.COXA_LENGTH_M / 2, 0.006, 0.006]
    tibia_half = [0.006, 0.006, cfg.TIBIA_LENGTH_M / 2]
    coxa_col, coxa_vis = _box_shapes(coxa_half, [0.85, 0.85, 0.85, 1])
    tibia_col, tibia_vis = _box_shapes(tibia_half, [0.3, 0.3, 0.3, 1])

    link_masses, link_col, link_vis = [], [], []
    link_pos, link_orn = [], []
    link_inertial_pos, link_inertial_orn = [], []
    link_parent, link_joint_type, link_joint_axis = [], [], []
    joint_names = []

    for leg in LEGS:
        xs, ys = HIP_SIGNS[leg]
        hip_x = xs * cfg.HIP_X_OFFSET_M
        hip_y = ys * cfg.HIP_Y_OFFSET_M

        # Hip (swing) joint -- massless connector mounted on the body.
        link_masses.append(0.0001)
        link_col.append(-1)
        link_vis.append(-1)
        link_pos.append([hip_x, hip_y, cfg.HIP_Z_OFFSET_M])
        link_orn.append([0, 0, 0, 1])
        link_inertial_pos.append([0, 0, 0])
        link_inertial_orn.append([0, 0, 0, 1])
        link_parent.append(0)  # base
        link_joint_type.append(p.JOINT_REVOLUTE)
        link_joint_axis.append([0, 0, 1])
        joint_names.append(f"{leg}_hip")
        hip_link_index = len(link_masses)

        # Coxa -- fixed-length link, child of the hip joint.
        link_masses.append(cfg.LEG_MASS_KG)
        link_col.append(coxa_col)
        link_vis.append(coxa_vis)
        link_pos.append([cfg.COXA_LENGTH_M / 2, 0, 0])
        link_orn.append([0, 0, 0, 1])
        link_inertial_pos.append([0, 0, 0])
        link_inertial_orn.append([0, 0, 0, 1])
        link_parent.append(hip_link_index)
        link_joint_type.append(p.JOINT_FIXED)
        link_joint_axis.append([0, 0, 0])
        joint_names.append(f"{leg}_coxa_fixed")
        coxa_link_index = len(link_masses)

        # Knee (lift) joint -- at the end of the coxa.
        link_masses.append(0.0001)
        link_col.append(-1)
        link_vis.append(-1)
        link_pos.append([cfg.COXA_LENGTH_M / 2, 0, 0])
        link_orn.append([0, 0, 0, 1])
        link_inertial_pos.append([0, 0, 0])
        link_inertial_orn.append([0, 0, 0, 1])
        link_parent.append(coxa_link_index)
        link_joint_type.append(p.JOINT_REVOLUTE)
        link_joint_axis.append([0, 1, 0])
        joint_names.append(f"{leg}_knee")
        knee_link_index = len(link_masses)

        # Tibia -- fixed-length link down to the foot tip.
        link_masses.append(cfg.LEG_MASS_KG)
        link_col.append(tibia_col)
        link_vis.append(tibia_vis)
        link_pos.append([0, 0, -cfg.TIBIA_LENGTH_M / 2])
        link_orn.append([0, 0, 0, 1])
        link_inertial_pos.append([0, 0, 0])
        link_inertial_orn.append([0, 0, 0, 1])
        link_parent.append(knee_link_index)
        link_joint_type.append(p.JOINT_FIXED)
        link_joint_axis.append([0, 0, 0])
        joint_names.append(f"{leg}_tibia_fixed")

    if start_height is None:
        start_height = cfg.TIBIA_LENGTH_M + cfg.BODY_HEIGHT_M / 2 + 0.01

    body_id = p.createMultiBody(
        baseMass=cfg.BODY_MASS_KG,
        baseCollisionShapeIndex=body_col,
        baseVisualShapeIndex=body_vis,
        basePosition=[0, 0, start_height],
        linkMasses=link_masses,
        linkCollisionShapeIndices=link_col,
        linkVisualShapeIndices=link_vis,
        linkPositions=link_pos,
        linkOrientations=link_orn,
        linkInertialFramePositions=link_inertial_pos,
        linkInertialFrameOrientations=link_inertial_orn,
        linkParentIndices=link_parent,
        linkJointTypes=link_joint_type,
        linkJointAxis=link_joint_axis,
    )

    joint_index_by_name = {name: i for i, name in enumerate(joint_names)}
    return body_id, joint_index_by_name
