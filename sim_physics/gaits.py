"""
Keyframe gaits transcribed verbatim from the firmware / sim history, in the
same 8-channel + ms format as actionarray.h:
  [G14, G12, G13, G15, G16, G5, G4, G2, ms]
"""

# The original, shipped diagonal-pair trot -- kept here only as a comparison
# baseline. This is NOT what's currently in firmware/0.SpiderBot/actionarray.h
# (that file now has WAVE_GAIT below); this is what git history shows before
# the wave-gait redesign.
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

# Current firmware gait -- statically-stable wave/crawl (one leg lifts at a
# time, alternating RF -> LF -> LR -> RR, the other three shift weight and
# stay grounded). Matches firmware/0.SpiderBot/actionarray.h exactly.
WAVE_GAIT = [
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

GAITS = {
    "trot_original": TROT_ORIGINAL,
    "wave": WAVE_GAIT,
}
