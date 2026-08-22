# =============================================================================
# CUT-COEFFICIENT ANGLE SWEEP, 100-160 deg  (c_theta_sweep_001)
# =============================================================================
# QUESTION (EMULATOR_ROADMAP gap 1): what is c(theta) between the two
# measured anchors? The pooled c(r) — c(10)=1.70, c(25)=1.57, c(50)=1.33,
# fitted 2026-08-19 over 1454 corners — is POOLED over the corpus angle
# mix, while 90-deg-class corners are separately measured at c ~= 0.70
# (MOTION_FINDINGS 9.3d, exact). Between 90 and 165 deg the blend model
# interpolates blindly; this path measures five points on that curve.
#
# GEOMETRY: seven 90 mm segments, six corners — one sacrificial 90 deg
# corner (the chain's first corner NEVER blends, 18/18) followed by the five
# measured corners at 100 / 115 / 130 / 145 / 160 deg. Turn signs alternate
# so the path folds back inside the hardware-proven box (x 515-788,
# y -65..+40, min box margin 11.3 mm); a same-sign zigzag would spiral into
# the near-singular region and contaminate the measurement with elbow
# demand. Segments are uniform 90 mm so min(L_in, L_out) is the same 90 mm
# at every measured corner — c falls straight out of the measured cut.
#
# TWO LOAD-BEARING PROPERTIES (same as blend_corner_001 — change knowingly):
#   ORIENTATION IS IDENTICAL AT EVERY POINT — zero rotation on every
#   segment, so nothing angular-throttles (H67) and any speed dip at a
#   corner IS the corner.
#   SPEED IS THE FACTORY 0.25 m/s — blend geometry is speed-independent
#   (identical cuts 0.10-0.35 m/s in SIM), so one rung suffices, needs
#   no limit changes, and keeps J4 in the proven range on this box.
#
# PREDICTED SIGNATURES per corner and r (cut = c * (r/100) * 90 mm;
# measurement resolution ~4 mm, analyse_coverage.py):
#
#   if the POOLED c(r) held at every angle:
#       r=10 -> 15.3 mm   r=25 -> 35.3 mm   r=50 -> 59.9 mm   (all corners)
#   if the OLD angle model held (0.70 at 90-120, 1.4 above 120):
#       100 deg: r=10 ->  6.3 mm | 115 deg: same 0.70 class
#       130-160 deg: r=10 -> 12.6 mm  r=25 -> 31.5 mm  r=50 -> 63.0 mm
#   the truth is expected BETWEEN these: c rising monotonically with theta
#   from ~0.70 toward the reversal class. Whatever is measured slots
#   directly into rm_emulator._corner_model as a (theta, r) table entry:
#       c_measured = cut_measured / ((r/100) * 0.090)
#
#   sacrificial P1 (90 deg): 0 cut at every r (first-corner exemption).
#
# ANALYSIS: python3 analyse_coverage.py ../runs/<run_dir>  (per-corner
# entry/exit cut + vmiss). SIM position channel is exact on blend geometry
# (72/72 runs, MOTION_FINDINGS 9) — SIM runs settle this; no REAL needed.
#
# Points are in the CONTROLLER World frame, mm and radians, tool L_glove_4,
# same z-plane and orientation as blend_corner_001. Screen after ANY edit:
#   python3 orientation_cost.py --segments ../paths/c_theta_sweep_001.py \
#           --tool L_glove_4 --speed 0.25
# RUN (one invocation sweeps r = 10 / 25 / 50):
#   python3 test_blend_corner.py --side left --mode SIM \
#           --path ../paths/c_theta_sweep_001.py
# =============================================================================

DEFAULT_SPEED = 100

# Uniform blend radius per pass; the driver runs the sweep one r at a time.
BLEND = 10
BLEND_SWEEP = [10, 25, 50]

TOOL_FRAME = "L_glove_4"

# ONE rung: blend geometry is speed-independent, and 0.25 keeps J4 in
# the proven range on this box (J4 crosses its limit near 0.385 m/s here).
SPEED_LADDER = [0.25]

TCP_LINEAR_VELOCITY = 0.25
TCP_LINEAR_ACCELERATION = 1.60
TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00

# Interior turn at each vertex P1..P6. P1 is the sacrificial first corner.
CORNER_ANGLES = [90, 100, 115, 130, 145, 160]

POSES_MM = {
    "P0": [755.000,  20.000, -323.628, -3.117, -0.4, 0.077],
    "P1": [691.360, -43.640, -323.628, -3.117, -0.4, 0.077],
    "P2": [627.721,  20.000, -323.628, -3.117, -0.4, 0.077],
    "P3": [576.099, -53.724, -323.628, -3.117, -0.4, 0.077],
    "P4": [531.099,  24.219, -323.628, -3.117, -0.4, 0.077],
    "P5": [619.732,   8.590, -323.628, -3.117, -0.4, 0.077],
    "P6": [538.164, -29.445, -323.628, -3.117, -0.4, 0.077],
    "P7": [627.821, -21.601, -323.628, -3.117, -0.4, 0.077],
}

SEQUENCE = ["P0", "P1", "P2", "P3", "P4", "P5", "P6", "P7"]
