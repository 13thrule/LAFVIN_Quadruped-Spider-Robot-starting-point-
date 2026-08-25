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
    [70, 90, 75, 110, 110, 105, 105, 70, 200],
    [90, 90, 75, 110, 110, 105, 105, 70, 200],
    [90, 135, 75, 110, 110, 105, 105, 70, 200],
    [70, 135, 75, 110, 110, 105, 105, 70, 200],

    [70, 120, 60, 110, 110, 105, 120, 70, 200],
    [70, 120, 60, 110, 90, 105, 120, 70, 200],
    [70, 120, 60, 110, 90, 60, 120, 70, 200],
    [70, 120, 60, 110, 110, 60, 120, 70, 200],

    [70, 105, 45, 110, 110, 75, 120, 70, 200],
    [70, 105, 45, 110, 110, 75, 120, 90, 200],
    [70, 105, 45, 110, 110, 75, 75, 90, 200],
    [70, 105, 45, 110, 110, 75, 75, 70, 200],

    [70, 90, 45, 110, 110, 90, 90, 70, 200],
    [70, 90, 45, 90, 110, 90, 90, 70, 200],
    [70, 90, 90, 90, 110, 90, 90, 70, 200],
    [70, 90, 90, 110, 110, 90, 90, 70, 200],
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

# Frog-hop: a real frog doesn't crawl with its front legs planted while the
# back legs paddle -- it crouches to load up, then explodes ALL FOUR legs
# at once and actually leaves the ground, lands, and stops. That's a
# discrete one-shot leap, not a continuous locomotion gait -- two earlier
# versions here got the category wrong (a "front anchored, rear paddling"
# loop, then just a faster version of the same loop). This one is a real
# crouch -> launch -> land sequence instead, ONE_SHOT like hello/dances,
# not HOLDABLE.
#
# All 4 feet extend together on the launch row, plus the rear legs' swing
# servos snap through their propulsion direction for forward thrust while
# airborne. Every angle used (55/65/115/125) stays within ranges already
# proven safe elsewhere in this firmware (sleep/dance2/dance3 use the same
# 45/135 extremes) -- only the *timing* is new and aggressive: the 80ms
# launch row is faster than the ~2.5ms/degree loaded-MG90S floor from
# earlier research, which is inherent to what an explosive launch actually
# requires. That's fine for a single one-off trigger; do NOT make this
# HOLDABLE/repeating without separately re-checking servo heat/stall risk
# under repeated rapid-fire use.
FROGHOP = [
    [55, 90, 90, 125, 125, 90, 90, 55, 60],     # crouch: all 4 feet compress down together, fast
    [115, 90, 45, 65, 65, 90, 135, 115, 60],    # launch: all 4 feet extend hard + rear legs snap, fast
    [70, 90, 90, 110, 110, 90, 90, 70, 150],    # land -- settle back to standing pose
]

GAITS = {
    "standby": STANDBY,
    "trot_original": TROT_ORIGINAL,
    "forward": WAVE_FORWARD,
    "froghop": FROGHOP,
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
