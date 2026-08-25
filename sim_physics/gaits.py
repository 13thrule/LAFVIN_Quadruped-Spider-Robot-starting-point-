"""
Every gait/pose/performance program from the firmware, transcribed verbatim
from firmware/0.SpiderBot/actionarray.h (and spiderbot_motion.cpp for the
composite ones -- hello and sleep each chain two matrices back to back,
matching what SpiderBotMotion::hello()/sleep() actually do). Same 8-channel
+ ms format: [G14, G12, G13, G15, G16, G5, G4, G2, ms].
"""

STANDBY = [
    [90, 90, 90, 90, 90, 90, 90, 90, 500],
    [60, 90, 90, 120, 120, 90, 90, 60, 500],
]

# The original, shipped diagonal-pair trot -- kept only as a comparison
# baseline against the current wave-gait FORWARD. NOT what's currently in
# firmware/0.SpiderBot/actionarray.h (that file has WAVE_FORWARD below).
TROT_ORIGINAL = [
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
    [90, 90, 90, 110, 110, 90, 45, 90, 200],
    [70, 90, 90, 110, 110, 90, 45, 70, 200],
    [70, 90, 90, 90, 90, 90, 45, 70, 200],
    [70, 39, 141, 90, 90, 90, 90, 70, 200],
    [70, 39, 141, 110, 110, 90, 90, 70, 200],
    [90, 90, 141, 110, 110, 90, 90, 90, 200],
    [90, 90, 90, 110, 110, 135, 90, 90, 200],
    [70, 90, 90, 110, 110, 135, 90, 70, 200],
    [70, 90, 90, 110, 90, 135, 90, 70, 200],
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
]

# Current firmware FORWARD gait -- statically-stable wave/crawl (one leg
# lifts at a time, alternating RF -> LF -> LR -> RR). Matches
# firmware/0.SpiderBot/actionarray.h's Servo_Prg_2 exactly.
WAVE_FORWARD = [
    [70, 90, 75, 110, 110, 105, 105, 70, 90],
    [90, 90, 75, 110, 110, 105, 105, 70, 90],
    [90, 135, 75, 110, 110, 105, 105, 70, 90],
    [70, 135, 75, 110, 110, 105, 105, 70, 90],

    [70, 120, 60, 110, 110, 105, 120, 70, 90],
    [70, 120, 60, 110, 90, 105, 120, 70, 90],
    [70, 120, 60, 110, 90, 60, 120, 70, 90],
    [70, 120, 60, 110, 110, 60, 120, 70, 90],

    [70, 105, 45, 110, 110, 75, 120, 70, 90],
    [70, 105, 45, 110, 110, 75, 120, 90, 90],
    [70, 105, 45, 110, 110, 75, 75, 90, 90],
    [70, 105, 45, 110, 110, 75, 75, 70, 90],

    [70, 90, 45, 110, 110, 90, 90, 70, 90],
    [70, 90, 45, 90, 110, 90, 90, 70, 90],
    [70, 90, 90, 90, 110, 90, 90, 70, 90],
    [70, 90, 90, 110, 110, 90, 90, 70, 90],
]

BACKWARD = [
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
    [90, 45, 90, 110, 110, 90, 90, 90, 200],
    [70, 45, 90, 110, 110, 90, 90, 70, 200],
    [70, 45, 90, 90, 90, 90, 90, 70, 200],
    [70, 90, 90, 90, 90, 135, 45, 70, 200],
    [70, 90, 90, 110, 110, 135, 45, 70, 200],
    [90, 90, 90, 110, 110, 135, 90, 90, 200],
    [90, 90, 135, 110, 110, 90, 90, 90, 200],
    [70, 90, 135, 110, 110, 90, 90, 70, 200],
    [70, 90, 135, 90, 110, 90, 90, 70, 200],
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
]

LEFTSHIFT = [
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
    [70, 90, 45, 90, 90, 90, 90, 70, 200],
    [70, 90, 45, 110, 110, 90, 90, 70, 200],
    [90, 90, 45, 110, 110, 90, 90, 90, 200],
    [90, 135, 90, 110, 110, 45, 90, 90, 200],
    [70, 135, 90, 110, 110, 45, 90, 70, 200],
    [70, 135, 90, 90, 90, 90, 90, 70, 200],
    [70, 90, 90, 90, 90, 90, 135, 70, 200],
    [70, 90, 90, 110, 110, 90, 135, 70, 200],
    [70, 90, 90, 110, 110, 90, 135, 90, 200],
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
]

RIGHTSHIFT = [
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
    [70, 90, 90, 90, 90, 45, 90, 70, 200],
    [70, 90, 90, 110, 110, 45, 90, 70, 200],
    [90, 90, 90, 110, 110, 45, 90, 90, 200],
    [90, 90, 45, 110, 110, 90, 135, 90, 200],
    [70, 90, 45, 110, 110, 90, 135, 70, 200],
    [70, 90, 90, 90, 90, 90, 135, 70, 200],
    [70, 135, 90, 90, 90, 90, 90, 70, 200],
    [70, 135, 90, 110, 110, 90, 90, 70, 200],
    [90, 135, 90, 110, 110, 90, 90, 70, 200],
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
]

TURNLEFT = [
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
    [90, 90, 90, 110, 110, 90, 90, 90, 200],
    [90, 135, 90, 110, 110, 90, 135, 90, 200],
    [70, 135, 90, 110, 110, 90, 135, 70, 200],
    [70, 135, 90, 90, 90, 90, 135, 70, 200],
    [70, 135, 135, 90, 90, 135, 135, 70, 200],
    [70, 135, 135, 110, 110, 135, 135, 70, 200],
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
]

TURNRIGHT = [
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
    [70, 90, 90, 90, 90, 90, 90, 70, 200],
    [70, 90, 45, 90, 90, 45, 90, 70, 200],
    [70, 90, 45, 110, 110, 45, 90, 70, 200],
    [90, 90, 45, 110, 110, 45, 90, 90, 200],
    [90, 45, 45, 110, 110, 45, 45, 90, 200],
    [70, 45, 45, 110, 110, 45, 45, 70, 200],
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
]

LIE = [
    [110, 90, 90, 70, 70, 90, 90, 110, 500],
]

HELLO_WAVE = [
    [60, 90, 125, 80, 120, 82, 90, 50, 280],
    [60, 90, 125, 80, 92, 74, 90, 50, 220],
    [60, 90, 125, 80, 38, 62, 90, 50, 240],
    [60, 90, 125, 80, 34, 118, 90, 50, 260],
    [60, 90, 125, 80, 46, 58, 90, 50, 260],
    [60, 90, 125, 80, 34, 118, 90, 50, 260],
    [60, 90, 125, 80, 46, 58, 90, 50, 260],
    [60, 90, 125, 80, 44, 88, 90, 50, 220],
    [60, 90, 125, 80, 92, 78, 90, 50, 220],
    [60, 90, 125, 80, 120, 82, 90, 50, 280],
]
HELLO = HELLO_WAVE + STANDBY  # matches SpiderBotMotion::hello()

FIGHTING = [
    [120, 90, 90, 110, 60, 90, 90, 70, 500],
    [120, 70, 70, 110, 60, 70, 70, 70, 500],
    [120, 110, 110, 110, 60, 110, 110, 70, 500],
    [120, 70, 70, 110, 60, 70, 70, 70, 500],
    [120, 110, 110, 110, 60, 110, 110, 70, 500],
    [70, 90, 90, 70, 110, 90, 90, 110, 500],
    [70, 70, 70, 70, 110, 70, 70, 110, 500],
    [70, 110, 110, 70, 110, 110, 110, 110, 500],
    [70, 70, 70, 70, 110, 70, 70, 110, 500],
    [70, 110, 110, 70, 110, 110, 110, 110, 500],
    [70, 90, 90, 70, 110, 90, 90, 110, 500],
    [60, 90, 90, 120, 120, 90, 90, 60, 500],
]

PUSHUP = [
    [70, 90, 90, 120, 120, 90, 90, 70, 500],
    [100, 90, 90, 80, 80, 90, 90, 100, 600],
    [60, 90, 90, 120, 120, 90, 90, 60, 700],
    [100, 90, 90, 80, 80, 90, 90, 100, 800],
    [60, 90, 90, 120, 120, 90, 90, 60, 900],
    [100, 90, 90, 80, 80, 90, 90, 100, 1000],
    [60, 90, 90, 120, 120, 90, 90, 60, 1100],
    [100, 90, 90, 80, 80, 90, 90, 100, 2000],
    [60, 90, 90, 120, 120, 90, 90, 60, 2500],
    [135, 90, 90, 45, 45, 90, 90, 135, 200],
    [70, 90, 90, 45, 60, 90, 90, 135, 800],
    [70, 90, 90, 45, 110, 90, 90, 135, 800],
    [70, 90, 90, 110, 110, 90, 90, 70, 800],
]

SLEEP_POSE = [
    [30, 90, 90, 150, 150, 90, 90, 30, 500],
    [30, 45, 135, 150, 150, 135, 45, 30, 500],
]
SLEEP = STANDBY + SLEEP_POSE  # matches SpiderBotMotion::sleep()

DANCE1 = [
    [90, 90, 90, 90, 90, 90, 90, 90, 400],
    [50, 90, 90, 90, 90, 90, 90, 90, 400],
    [90, 90, 90, 130, 90, 90, 90, 90, 400],
    [90, 90, 90, 90, 90, 90, 90, 50, 400],
    [90, 90, 90, 90, 130, 90, 90, 90, 400],
    [50, 90, 90, 90, 90, 90, 90, 90, 400],
    [90, 90, 90, 130, 90, 90, 90, 90, 400],
    [90, 90, 90, 90, 90, 90, 90, 50, 400],
    [90, 90, 90, 90, 130, 90, 90, 90, 400],
    [90, 90, 90, 90, 90, 90, 90, 90, 400],
]

DANCE2 = [
    [70, 45, 135, 110, 110, 135, 45, 70, 320],
    [95, 45, 135, 85, 110, 135, 45, 70, 260],
    [70, 45, 135, 110, 85, 135, 45, 95, 260],
    [95, 45, 135, 85, 110, 135, 45, 70, 260],
    [70, 45, 135, 110, 85, 135, 45, 95, 260],
    [95, 45, 135, 85, 110, 135, 45, 70, 260],
    [70, 45, 135, 110, 85, 135, 45, 95, 260],
    [95, 45, 135, 85, 110, 135, 45, 70, 260],
    [70, 45, 135, 110, 85, 135, 45, 95, 260],
    [70, 45, 135, 110, 110, 135, 45, 70, 320],
]

DANCE3 = [
    [70, 45, 45, 110, 110, 135, 135, 70, 400],
    [110, 45, 45, 60, 70, 135, 135, 70, 400],
    [70, 45, 45, 110, 110, 135, 135, 70, 400],
    [110, 45, 45, 110, 70, 135, 135, 120, 400],
    [70, 45, 45, 110, 110, 135, 135, 70, 400],
    [110, 45, 45, 60, 70, 135, 135, 70, 400],
    [70, 45, 45, 110, 110, 135, 135, 70, 400],
    [110, 45, 45, 110, 70, 135, 135, 120, 400],
    [70, 45, 45, 110, 110, 135, 135, 70, 400],
    [70, 90, 90, 110, 110, 90, 90, 70, 400],
]

CENTER = [[90, 90, 90, 90, 90, 90, 90, 90, 300]]  # Servo_Act_1-ish neutral hold
ZERO = [[135, 45, 135, 45, 45, 135, 45, 135, 300]]  # Servo_Act_0

# Frog-hop / lunge: real research (MSU one-motor jumping robot, U. Illinois
# SG90 spring-charge project, arXiv jumping-robot reviews) says bare
# direct-drive MG90S servos with no spring/elastic storage physically
# cannot deliver genuine liftoff -- peak launch power needs energy released
# in milliseconds, and a position-controlled PWM servo can't front-load
# force the way a released spring does. That's a power-delivery ceiling,
# not a keyframe-timing problem, and matches what real-hardware serial
# previews showed (flat, no launch) even after retuning the old
# crouch/launch/land version. Earlier PyBullet sim runs showed apparent air
# time for that version, but PyBullet's servo joints track commanded
# angles closer to ideally than a real loaded MG90S can, which is exactly
# why the sim result didn't hold up on hardware.
#
# So this is deliberately NOT a jump: it's a real, physically grounded
# lunge. All 4 feet stay planted and swing backward together -- a
# synchronized power stroke, the same mechanism that makes a walking
# gait's power stroke move the body, just done on all 4 legs at once
# instead of one at a time -- while the knees extend from a deep crouch.
# That's a genuine forward weight-shift with a sudden, explosive-looking
# snap, plus a brief hold at full extension to sell the "sprung" look.
# ONE_SHOT like hello/dances, not HOLDABLE. All angles (55/65/115/125,
# 45/135 hips) reuse ranges already proven safe elsewhere in this firmware
# (sleep/dance2/dance3/turn gaits use the same extremes).
FROGHOP = [
    [55, 90, 90, 125, 125, 90, 90, 55, 70],     # crouch: all 4 feet compress/plant deeper together, fast
    [55, 135, 45, 125, 125, 45, 135, 55, 70],   # drag: ONLY hips power-stroke (front opposite rear), knees stay at crouch depth
    [115, 135, 45, 65, 65, 45, 135, 115, 90],   # rise: knees extend to standing height, hips hold the stroke
    [115, 135, 45, 65, 65, 45, 135, 115, 100],  # hold at full extension -- sells the "sprung" look
    [70, 90, 90, 110, 110, 90, 90, 70, 180],    # recover -- settle back to standing pose
]

# Sit: rear feet tuck up under the body (RR/LR knees driven toward their
# lift/raise direction), front feet stay planted at the standing pose --
# a real sit silhouette (haunches down, front up), not just "lie" (which
# is a single diagonal-pair tilt, not a sit). Hips stay centered.
# Self-contained: the first row re-establishes the 70/110 standing pose
# this whole action was designed from, regardless of whatever pose the
# robot was actually in when triggered -- found the hard way on real
# hardware: PROGRAM_STANDBY settles at a different 60/120 pose, not
# 70/110, so chaining these off of "standby" silently halved bow's dip
# and leaked unintended rear-leg motion.
SIT = [
    [70, 90, 90, 110, 110, 90, 90, 70, 300],  # re-establish the standing pose this action assumes
    [70, 90, 90, 70, 110, 90, 90, 110, 500],  # settle into sit -- rear knees tuck up, front stays standing
]

# Bow: front feet extend down (RF/LF knees driven toward their lower/extend
# direction) while rear feet hold the standing pose -- dips the front of
# the body down and back up, a personality flourish adapted from the
# research pass's survey of similar hobby-quadruped move sets (OpenCat/
# miniKame). Reuses the same knee range as the (now-removed) lunge's
# crouch (55/130, already proven safe). Self-contained -- see SIT's
# comment above for why the leading standing-pose row matters.
BOW = [
    [70, 90, 90, 110, 110, 90, 90, 70, 300],  # re-establish the standing pose this action assumes
    [30, 90, 90, 110, 150, 90, 90, 70, 450],  # dip -- front knees extend down, rear unchanged
    [30, 90, 90, 110, 150, 90, 90, 70, 400],  # hold the bow
    [70, 90, 90, 110, 110, 90, 90, 70, 450],  # recover -- back to standing
]

# Shake / wiggle: a small-amplitude hip-only twist (knees stay at the
# standing pose throughout) -- deliberately different from dance2, which
# is knee-based. This is a body wiggle in place, not a locomotion gait, so
# (unlike the lunge) front/rear hip direction doesn't need to be mirrored
# for net progress -- there's no net-progress goal here. Self-contained --
# see SIT's comment above for why the leading standing-pose row matters.
SHAKE = [
    [70, 90, 90, 110, 110, 90, 90, 70, 300],    # re-establish the standing pose this action assumes
    [60, 105, 75, 120, 120, 75, 105, 60, 150],  # twist one way + knees compress together
    [80, 75, 105, 100, 100, 105, 75, 80, 150],  # twist the other way + knees extend together
    [60, 105, 75, 120, 120, 75, 105, 60, 150],
    [80, 75, 105, 100, 100, 105, 75, 80, 150],
    [70, 90, 90, 110, 110, 90, 90, 70, 200],    # recover -- back to standing
]

GAITS = {
    "standby": STANDBY,
    "trot_original": TROT_ORIGINAL,
    "forward": WAVE_FORWARD,
    "froghop": FROGHOP,
    "sit": SIT,
    "bow": BOW,
    "shake": SHAKE,
    "backward": BACKWARD,
    "leftshift": LEFTSHIFT,
    "rightshift": RIGHTSHIFT,
    "turnleft": TURNLEFT,
    "turnright": TURNRIGHT,
    "lie": LIE,
    "hello": HELLO,
    "fighting": FIGHTING,
    "pushup": PUSHUP,
    "sleep": SLEEP,
    "dance1": DANCE1,
    "dance2": DANCE2,
    "dance3": DANCE3,
    "center": CENTER,
    "zero": ZERO,
}

# Locomotion gaits loop indefinitely while selected; poses/performance moves
# play once and return to standby, matching the firmware's own IR behavior
# (isHoldableLocomotion() in 0.SpiderBot.ino).
HOLDABLE = {"forward", "trot_original", "backward", "leftshift", "rightshift", "turnleft", "turnright"}
