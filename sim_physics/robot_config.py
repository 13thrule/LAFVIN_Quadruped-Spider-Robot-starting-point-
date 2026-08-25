"""
Single source of truth for the spider bot's physical dimensions/mass.

Values below are now INFORMED ESTIMATES, not blind guesses: measured
directly (bounding-box, in sim_physics/reference_stl/measure.py) from real
STL files published at github.com/xinlitech/quadruped-spider-for-esp8266 --
a same-family open-source "quadruped spider ESP8266" design sold under
several rebrands (this LAFVIN kit almost certainly among them, given the
identical servo count/layout and matching AliExpress listings for the same
underlying design). Two honest caveats, still: (1) not verified identical
to this exact physical unit -- that repo's parts are 3D-printed, while
LAFVIN's own BOM describes acrylic panels, so it may be a variant; (2) the
hip-bracket cluster's (leg1_1/2/3) contribution to COXA_LENGTH is estimated
from overlapping bounding boxes, not a traced assembly.

Updating these to real measurements of the actual robot and re-running any
script here is the entire update path -- nothing else needs to change.

What to actually measure to replace an estimate below (in meters, mm/1000):
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
# From bodyBottom.stl's measured bounding box (83.76mm x 49mm x 14mm).
# Axis-to-robot-orientation (which measured axis is front-back vs
# left-right) is an assumption, not verified: took the longer axis as
# front-to-back since these chassis plates are typically elongated to fit
# 4 hip mounts symmetrically front/rear.
BODY_LENGTH_M = 0.084   # measured (bodyBottom.stl long axis) -- front-to-back
BODY_WIDTH_M = 0.049    # measured (bodyBottom.stl short axis) -- left-to-right
BODY_HEIGHT_M = 0.014   # measured (bodyBottom.stl thickness)
BODY_MASS_KG = 0.40     # still an estimate -- AliExpress package weight for
                         # this design is 1.0kg (packaging+manual+screws
                         # included, not the assembled robot), used only as
                         # a sanity check that the robot is well under 1kg

# --- Legs (identical for all 4) ----------------------------------------
# leg2.stl (the long structural arm) measures 65mm; the leg1_1/1_2/1_3
# hip-bracket cluster that connects it to the hip servo horn is compact --
# estimated at ~20mm of added reach from its overlapping bounding boxes,
# not a traced assembly.
COXA_LENGTH_M = 0.020    # estimated (leg1_1/1_2/1_3 bracket cluster)
TIBIA_LENGTH_M = 0.065   # measured (leg2.stl long axis)
LEG_MASS_KG = 0.015      # still an estimate -- small relative to
                          # BODY_MASS_KG so a rough guess matters less

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
