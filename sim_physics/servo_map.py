"""
Servo channel roles and sign conventions, matching sim/gait_lab.html's
CHANNELS table -- both were reverse-engineered and cross-validated against
actionarray.h's own row comments earlier this session (e.g. "leg1,4 up",
"leg2 fw"), not guessed independently in two places.

Column/index order matches the firmware exactly:
  0=G14 (RF knee/lift)   1=G12 (RF hip/swing)
  2=G13 (RR hip/swing)   3=G15 (RR knee/lift)
  4=G16 (LF knee/lift)   5=G5  (LF hip/swing)
  6=G4  (LR hip/swing)   7=G2  (LR knee/lift)
"""

LEGS = ["RF", "RR", "LF", "LR"]

# lift_sign:  +1 means increasing the knee channel's servo angle raises the
#             foot; -1 means decreasing it does.
# swing_sign: +1 means increasing the hip channel's servo angle swings the
#             leg in the "forward/recovery" direction (right-side
#             convention); -1 is the mirrored left-side convention.
LEG_CHANNELS = {
    "RF": dict(knee_idx=0, hip_idx=1, lift_sign=+1, swing_sign=+1),
    "RR": dict(knee_idx=3, hip_idx=2, lift_sign=-1, swing_sign=+1),
    "LF": dict(knee_idx=4, hip_idx=5, lift_sign=-1, swing_sign=-1),
    "LR": dict(knee_idx=7, hip_idx=6, lift_sign=+1, swing_sign=-1),
}

SERVO_CENTER_DEG = 90
