# =============================================================================
# BLEND / CONNECT CORNER TEST PATH  —  blend_corner_001
# =============================================================================
# Geometry for C19 (`src/test_blend_corner.py`): does `connect=1` plus a blend
# radius actually hold speed through a corner, or does the tool decelerate at
# every one?
#
# EDIT THIS FILE, NOT THE TEST. The test reads the points, the sequence and
# the settings below; it generates nothing. Move a point, change an angle,
# add a corner — no source change needed.
#
# TWO PROPERTIES OF THIS PATH ARE LOAD-BEARING. Change them knowingly:
#
#   ORIENTATION IS IDENTICAL AT EVERY POINT. A segment whose tool rotation
#   implies more than TCP_ANGULAR_VELOCITY is time-scaled by the controller
#   (H67), and that looks exactly like a corner deceleration without being
#   one. With zero rotation everywhere, omega is zero on every segment and
#   nothing can throttle for that reason, so any dip measured at a corner IS
#   the corner. Give any point a different rotation and the test stops
#   measuring what it claims to.
#
#   SEGMENTS ARE 100 mm AND THE SPEED IS THE FACTORY DEFAULT 0.25 m/s.
#   Ramp distance is v^2/2a: 0.25 m/s against line_acc 1.60 is ~20 mm each
#   way, so 100 mm is 5x the ramp and there is a real cruise plateau to
#   lose at a corner. Raising the speed WITHOUT lengthening the segments
#   destroys the test — at 0.45 m/s the ramp is 63 mm and a 120 mm segment
#   is nothing but ramp, so it would "confirm" a deceleration problem
#   whatever the controller did. Keep segment >= ~4x ramp.
#
#   Testing at the default also means the run needs NO limit changes: no
#   ratcheting to undo (reset_limits.py), and nothing that runs against
#   RealMan's advice to leave the TCP values alone (H62).
#
#   THE PATH FOLDS BACK on itself (turns alternate left/right) so it stays
#   inside 559 mm of the base — 53 % of reach. A zigzag that always turns
#   the same way spirals out past 1200 mm, into the near-singular region
#   where elbow demand explodes, which would contaminate a corner test.
#
#   IT IS ALSO SCREENED FOR ELBOW DEMAND, and that is not a formality: an
#   earlier version of this path had reachable endpoints but drove J4 to
#   319 % of its limit BETWEEN two of them. Endpoint reachability is not
#   enough. Worst segment here is 34 % of the J4 limit. Re-screen after ANY
#   edit:
#     python3 orientation_cost.py --segments ../paths/blend_corner_001.py \
#             --tool L_glove_2 --speed 0.25
#
# The corner angles are the interior turn at each vertex: 15 and 30 are the
# cases reported as already decelerating, 90 is the reference for "must".
#
# Points are in the CONTROLLER World frame, mm and radians, tool L_glove_2.
# Free space in front of the LEFT arm; every point verified to solve IK
# within joint limits. Re-verify if the cell changes (see above).
# =============================================================================

# =============================================================================
# MOTION SETTINGS
# =============================================================================

DEFAULT_SPEED = 100

# Blend radius percentage. The test OVERRIDES this per case — it sweeps
# BLEND_SWEEP below — but this is what a single manual run would use.
BLEND = 25

# The blend radii the test compares. r=0 is the no-blending control: if
# r=0 and r=50 measure the same, the radius is not being applied at all.
#
# r=10 IS DELIBERATELY ABSENT, and that is a gap worth naming: 10 is what
# `stage_runner` actually dispatches on every cleaning stroke, so this sweep
# does not cover the operating configuration. It cannot, on this geometry —
# an r=10 blend on a 65 mm segment is a 6.5 mm dip, and after the analysis
# excludes the samples whose window straddles the vertex there is nothing
# left to take a minimum over. Adding it made `verify_blend_measure` fail by
# 21 points against a KNOWN input, which is the self-test doing its job.
#
# To measure r=10 the SEGMENTS must be longer, not the speed lower: at the
# 214 mm segments a real cleaning stroke averages, r=10 is a 21 mm dip and
# resolves comfortably. A two-corner path at 214 mm fits inside the same
# proven box (389 mm of x). That is the follow-up, not a change here.
BLEND_SWEEP = [0, 25, 50]

# Interior turn angle at each corner, degrees, in traversal order. Used to
# LABEL the results; the geometry below must match it.
CORNER_ANGLES = [90, 30, 60, 15, 45]

STARTPOSE_SPEED = 30


# =============================================================================
# TCP SPEED LIMITS
# =============================================================================
# Configured in the ARM controller, printed by the test for the record.
# 0.250 / 1.600 / 0.600 / 4.000 are the confirmed factory defaults (H64).

# 0.20 until 2026-08-13. The corner measurement is resolution-limited by how
# many 100 Hz samples land inside the blend, and the UDP position field is
# not synchronous with the push, so single samples cannot be trusted and the
# analysis smooths 70 ms. At 0.20 m/s an r=25 blend on a 65 mm segment is
# only ~8 samples wide and the smoothing plus the corner-straddle exclusion
# swallow it: recovered retention came back 10-30 points high against a
# KNOWN input. At 0.10 m/s the same blend is ~16 samples and the worst error
# over a prescribed 100/70/50/30/20 % falls to 9 points.
#
# It also stays well under the 0.250 m/s factory default, so no limit is
# raised to run this.
#
# TO MEASURE AT AN OPERATING SPEED, pass --speed. Be aware what it costs: at
# 0.45 m/s the ramp is 63 mm against a 65 mm segment, so the tool never
# reaches cruise and there is no plateau to retain — retention is undefined
# there, and the test says so rather than printing a number. Real cleaning
# strokes average 214 mm segments and DO reach cruise at 0.45, which is why
# the mechanism measured here transfers to them.
TCP_LINEAR_VELOCITY = 0.10
TCP_LINEAR_ACCELERATION = 1.60

TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00


# =============================================================================
# CARTESIAN POSES  (mm, radians)
# =============================================================================
# Planar zigzag, 100 mm segments, constant orientation. Turns ALTERNATE
# left/right by CORNER_ANGLES at each interior vertex so the path folds back
# instead of spiralling out of the workspace.
#
#   c1 .. c5 are the corners under test; "start" and "end" are the straight
#   run-in and run-out that give the first and last corner a proper approach.

POSES_MM = {
    "start":   [ 515.000,   32.000, -323.628, -3.117, -0.4, 0.077],
    "c1":      [ 515.000,  -33.000, -323.628, -3.117, -0.4, 0.077],
    "c2":      [ 580.000,  -33.000, -323.628, -3.117, -0.4, 0.077],
    "c3":      [ 636.292,  -65.500, -323.628, -3.117, -0.4, 0.077],
    "c4":      [ 692.583,  -33.000, -323.628, -3.117, -0.4, 0.077],
    "c5":      [ 755.368,  -16.177, -323.628, -3.117, -0.4, 0.077],
    "end":     [ 787.868,   40.115, -323.628, -3.117, -0.4, 0.077],
}


# =============================================================================
# SEQUENCE
# =============================================================================
# One pass. Every case re-runs this identical traversal from "start", so the
# arm configuration is the same for each blend radius.

SEQUENCE = [
    "start",
    "c1",
    "c2",
    "c3",
    "c4",
    "c5",
    "end",
]


# =============================================================================
# PER-SEGMENT SPEEDS
# =============================================================================
# All 100: the test is about corner behaviour, so nothing should be throttled
# by a per-segment speed. Lower one only to isolate a corner deliberately.

SEGMENT_SPEEDS = {
    ("start", "c1"): 100,
    ("c1", "c2"): 100,
    ("c2", "c3"): 100,
    ("c3", "c4"): 100,
    ("c4", "c5"): 100,
    ("c5", "end"): 100,
}
