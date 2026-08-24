"""
Single source of truth for the spider bot's physical dimensions/mass.

Every value below is a PLACEHOLDER guess, not a measurement -- generate_urdf.py
reads this file, so updating these numbers to real measurements and re-running
`python generate_urdf.py` is the entire update path. Nothing else needs to
change by hand.

What to actually measure on the robot (all in meters, i.e. mm / 1000):
  BODY_LENGTH / BODY_WIDTH / BODY_HEIGHT -- the main chassis block the 4 hips
      mount to. Skip the acrylic corner tabs/antennae, just the core body.
  COXA_LENGTH  -- the fixed stub from the hip joint to the knee/lift joint
      (the segment whose *direction* changes with the "leg" swing servo).
  TIBIA_LENGTH -- the segment from the knee/lift joint to the foot tip (the
      segment whose *angle* changes with the "foot" lift servo).
  HIP_X_OFFSET / HIP_Y_OFFSET -- how far each hip sits from the body's
      center, along the body's length/width. If all 4 hips sit flush at the
      body's corners, this can just be BODY_LENGTH/2 and BODY_WIDTH/2.
  BODY_MASS_KG -- whole-robot weight (batteries included) is fine as a
      starting point; individual leg mass matters much less.
"""

# --- Body -------------------------------------------------------------
BODY_LENGTH_M = 0.14   # PLACEHOLDER -- front-to-back
BODY_WIDTH_M = 0.10    # PLACEHOLDER -- left-to-right
BODY_HEIGHT_M = 0.03   # PLACEHOLDER -- top-to-bottom of the chassis block
BODY_MASS_KG = 0.30    # PLACEHOLDER -- whole robot, batteries included

# --- Legs (identical for all 4) ----------------------------------------
COXA_LENGTH_M = 0.025   # PLACEHOLDER -- hip (swing) joint to knee (lift) joint
TIBIA_LENGTH_M = 0.045  # PLACEHOLDER -- knee (lift) joint to foot tip
LEG_MASS_KG = 0.015     # PLACEHOLDER -- per leg segment (coxa or tibia), small
                         # relative to BODY_MASS_KG so a rough guess is fine

# --- Hip mount points, relative to body center --------------------------
# Defaults assume all 4 hips sit at the body's corners. Override if the
# real mounts sit inboard of the chassis edges.
HIP_X_OFFSET_M = BODY_LENGTH_M / 2
HIP_Y_OFFSET_M = BODY_WIDTH_M / 2
HIP_Z_OFFSET_M = 0.0  # hip height relative to body center, usually 0

# --- Servo angle range (from actionarray.h: PWMRES_Min/Max) -------------
SERVO_MIN_DEG = 1
SERVO_MAX_DEG = 180
SERVO_CENTER_DEG = 90
