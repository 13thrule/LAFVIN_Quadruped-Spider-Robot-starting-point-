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

Stance geometry checked against docs/source/_static/2.spider.png (LAFVIN's
own product photo): legs splay outward diagonally from the body (not
aligned with the body's length/width axes) and angle down-and-out to the
ground -- a wide, low spider stance, not a boxy right-angle one. Each hip
joint's zero-orientation is rotated to point outward from its body corner,
and the coxa/tibia links carry a fixed downward tilt, matching that stance.
This isn't just cosmetic: getting the leg-splay angle right changes the
actual support-polygon shape, which is what the stability numbers in
run_sim.py depend on.
"""

import math

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

# How far the coxa (first segment) and tibia (second segment) tilt downward
# from horizontal at their neutral/zero commanded position, matching the
# photo's wide, low stance. Purely a fixed visual/geometric offset -- baked
# into the link transforms below, so it doesn't change how gait commands
# are interpreted in run_sim.py.
COXA_DOWN_TILT_RAD = math.radians(12)
TIBIA_DOWN_TILT_RAD = math.radians(45)

BODY_COLOR = [0.06, 0.06, 0.07, 1]
HEAD_COLOR = [0.10, 0.10, 0.12, 1]
LEG_COLOR = [0.04, 0.04, 0.05, 1]
EYE_COLOR = [0.75, 0.78, 0.8, 1]


def _shapes(shape_type, color, **kwargs):
    col = p.createCollisionShape(shape_type, **kwargs)
    vis = p.createVisualShape(shape_type, rgbaColor=color, **kwargs)
    return col, vis


def build_spider_robot(start_height=None):
    """Builds the robot in the currently-connected PyBullet client.

    Returns (body_id, joint_index_by_name), where joint_index_by_name maps
    e.g. "RF_hip" / "RF_knee" to the PyBullet joint index for
    setJointMotorControl2.
    """
    body_half = [cfg.BODY_LENGTH_M / 2, cfg.BODY_WIDTH_M / 2, cfg.BODY_HEIGHT_M / 2]
    body_col, body_vis = _shapes(p.GEOM_BOX, BODY_COLOR, halfExtents=body_half)

    coxa_half = [cfg.COXA_LENGTH_M / 2, 0.006, 0.006]
    tibia_half = [0.005, 0.005, cfg.TIBIA_LENGTH_M / 2]
    coxa_col, coxa_vis = _shapes(p.GEOM_BOX, LEG_COLOR, halfExtents=coxa_half)
    tibia_col, tibia_vis = _shapes(p.GEOM_BOX, LEG_COLOR, halfExtents=tibia_half)

    link_masses, link_col, link_vis = [], [], []
    link_pos, link_orn = [], []
    link_inertial_pos, link_inertial_orn = [], []
    link_parent, link_joint_type, link_joint_axis = [], [], []
    joint_names = []

    for leg in LEGS:
        xs, ys = HIP_SIGNS[leg]
        hip_x = xs * cfg.HIP_X_OFFSET_M
        hip_y = ys * cfg.HIP_Y_OFFSET_M
        outward_yaw = math.atan2(ys, xs)  # points away from body center

        # Hip (swing) joint -- massless connector mounted on the body,
        # zero-oriented to face outward from this corner (see module
        # docstring) so the swing servo's commanded delta is relative to a
        # realistic splayed stance, not straight ahead.
        link_masses.append(0.0001)
        link_col.append(-1)
        link_vis.append(-1)
        link_pos.append([hip_x, hip_y, cfg.HIP_Z_OFFSET_M])
        link_orn.append(p.getQuaternionFromEuler([0, 0, outward_yaw]))
        link_inertial_pos.append([0, 0, 0])
        link_inertial_orn.append([0, 0, 0, 1])
        link_parent.append(0)  # base
        link_joint_type.append(p.JOINT_REVOLUTE)
        link_joint_axis.append([0, 0, 1])
        joint_names.append(f"{leg}_hip")
        hip_link_index = len(link_masses)

        # Coxa -- fixed-length link, child of the hip joint, tilted down
        # slightly from horizontal.
        link_masses.append(cfg.LEG_MASS_KG)
        link_col.append(coxa_col)
        link_vis.append(coxa_vis)
        link_pos.append([
            math.cos(COXA_DOWN_TILT_RAD) * cfg.COXA_LENGTH_M / 2,
            0,
            -math.sin(COXA_DOWN_TILT_RAD) * cfg.COXA_LENGTH_M / 2,
        ])
        link_orn.append(p.getQuaternionFromEuler([0, COXA_DOWN_TILT_RAD, 0]))
        link_inertial_pos.append([0, 0, 0])
        link_inertial_orn.append([0, 0, 0, 1])
        link_parent.append(hip_link_index)
        link_joint_type.append(p.JOINT_FIXED)
        link_joint_axis.append([0, 0, 0])
        joint_names.append(f"{leg}_coxa_fixed")
        coxa_link_index = len(link_masses)

        # Knee (lift) joint -- at the end of the coxa, inheriting its
        # downward tilt, then tilted further down again so the tibia's
        # neutral direction angles out-and-down to the ground like the
        # reference photo, not straight down.
        #
        # NOTE the negated sign here: the tibia link's shape hangs along
        # its own local -Z (see below), and empirically (verified via
        # sim_physics/snapshot.py renders) a *positive* Y-pitch rotates
        # that -Z direction toward -X (back under the body) rather than
        # +X (outward) -- confirmed by comparing snapshot_tibiaonly.png
        # (positive sign, legs curl inward) against the corrected render.
        # Negating fixes it without touching the tibia's own position.
        link_masses.append(0.0001)
        link_col.append(-1)
        link_vis.append(-1)
        link_pos.append([cfg.COXA_LENGTH_M / 2, 0, 0])
        link_orn.append(p.getQuaternionFromEuler([0, -TIBIA_DOWN_TILT_RAD, 0]))
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

    # Decorative-only links matching the photo's raised "head" block and
    # the twin ultrasonic-sensor "eyes" on the front face -- both fixed to
    # the base, no physics role, just so the sim is recognizable as this
    # robot and forward-facing is obvious at a glance.
    # Centered (was 0.15*BODY_LENGTH forward with a 0.22 half-width, so its
    # near edge almost touched the front hips -- looked lopsided toward one
    # leg pair instead of centered between all four, as in the reference
    # photo. Smaller and centered reads much closer to the real proportions.
    head_half = [cfg.BODY_LENGTH_M * 0.15, cfg.BODY_WIDTH_M * 0.30, cfg.BODY_HEIGHT_M * 0.9]
    head_col, head_vis = _shapes(p.GEOM_BOX, HEAD_COLOR, halfExtents=head_half)
    link_masses.append(0.0001)
    link_col.append(head_col)
    link_vis.append(head_vis)
    link_pos.append([0, 0, cfg.BODY_HEIGHT_M / 2 + head_half[2]])
    link_orn.append([0, 0, 0, 1])
    link_inertial_pos.append([0, 0, 0])
    link_inertial_orn.append([0, 0, 0, 1])
    link_parent.append(0)
    link_joint_type.append(p.JOINT_FIXED)
    link_joint_axis.append([0, 0, 0])
    joint_names.append("head_fixed")

    eye_radius = cfg.BODY_WIDTH_M * 0.10
    eye_col, eye_vis = _shapes(p.GEOM_SPHERE, EYE_COLOR, radius=eye_radius)
    for eye_y_sign in (-1, +1):
        link_masses.append(0.0001)
        link_col.append(eye_col)
        link_vis.append(eye_vis)
        link_pos.append([
            head_half[0] * 0.95,
            eye_y_sign * cfg.BODY_WIDTH_M * 0.16,
            cfg.BODY_HEIGHT_M / 2 + head_half[2],
        ])
        link_orn.append([0, 0, 0, 1])
        link_inertial_pos.append([0, 0, 0])
        link_inertial_orn.append([0, 0, 0, 1])
        link_parent.append(0)
        link_joint_type.append(p.JOINT_FIXED)
        link_joint_axis.append([0, 0, 0])
        joint_names.append(f"eye_fixed_{eye_y_sign}")

    if start_height is None:
        rise = cfg.COXA_LENGTH_M * math.sin(COXA_DOWN_TILT_RAD) + cfg.TIBIA_LENGTH_M * math.cos(TIBIA_DOWN_TILT_RAD)
        start_height = rise + cfg.BODY_HEIGHT_M / 2 + 0.015

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

    # Only the revolute hip/knee joints are ever driven; the "_fixed" links
    # (coxa, tibia, decorative head/eyes) have no motor to control, but
    # e.g. the tibia links are still useful to look up for foot-contact
    # checks and support-polygon visualization.
    joint_index_by_name = {
        name: i for i, name in enumerate(joint_names)
        if name.endswith("_hip") or name.endswith("_knee")
    }
    link_index_by_name = {name: i for i, name in enumerate(joint_names)}

    return body_id, joint_index_by_name, link_index_by_name
