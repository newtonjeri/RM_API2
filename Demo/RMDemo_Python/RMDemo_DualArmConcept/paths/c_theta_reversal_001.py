# =============================================================================
# CUT-COEFFICIENT AT NEAR-REVERSALS, 166-176 deg  (c_theta_reversal_001)
# =============================================================================
# QUESTION (d9's open point on contract A.2, 2026-08-19): the contract's
# c(r) — c(10)=1.70, c(25)=1.57, c(50)=1.33 — is POOLED over the corpus
# angle mix, and A.2 records that 82-95 % of all blend loss lives at
# reversals > 165 deg while 90-deg-class corners are separately measured at
# c ~= 0.70. If the pool contains 0.70-class corners, the true coefficient
# AT REVERSALS must sit ABOVE the pooled 1.70 at r=10 — meaning even the
# retuned emulator still understates the cut at exactly the angle class
# that dominates the loss. Measured exposure in the commode_c corpus: 82 of
# 533 shared-vertex corners (15.4 %) are > 165 deg. This path measures c at
# four reversal angles directly.
#
# GEOMETRY: six 120 mm segments, five corners — one sacrificial 90 deg
# corner (first corner never blends, 18/18), then near-reversals at
# 166 / 170 / 173 / 176 deg. The path is a folding fan: each reversal
# returns nearly parallel to the previous stroke, so it sits naturally
# inside the hardware-proven box (x 515-788, y -65..+40, min box margin
# 10.0 mm). Segments uniform 120 mm -> min(L_in, L_out) = 120 mm at every
# measured corner. NO angle is exact 180: A.2 measures exact retraces as
# ZERO cut (collinearity protects) while 172-176 deg near-reversals of the
# same length cut at the FULL rate — 176 is the closest safe probe.
#
# TWO LOAD-BEARING PROPERTIES (same as blend_corner_001 — change knowingly):
#   ORIENTATION IS IDENTICAL AT EVERY POINT — zero rotation on every
#   segment, so nothing angular-throttles (H67).
#   SPEED IS THE FACTORY 0.25 m/s — blend geometry is speed-independent
#   (A.2), one rung suffices, no limit changes, J4 stays in the proven
#   range on this box.
#
# PREDICTED SIGNATURES per corner and r (cut = c * (r/100) * 120 mm;
# resolution ~4 mm, analyse_coverage.py):
#
#   if A.2's POOLED c(r) held at reversals:
#       r=10 -> 20.4 mm    r=25 -> 47.1 mm    r=50 -> 79.8 mm
#   if d9's pooling argument is right, c(10) at reversals > 1.70:
#       r=10 -> ABOVE 20.4 mm, by the amount the 0.70-class drags the pool
#   MECHANISM CHECK (A.2): at r=25/50 expect EARLY TURNAROUND, not corner
#   rounding — the vertex never approached within ~30 mm; at r=50 on the
#   2026-08-14 geometries reversals removed ~250 mm per vertex on
#   300-400 mm strokes, i.e. c ~= 250/(0.5*380) ~= 1.32 pooled-consistent.
#   analyse_coverage's entry/exit split is the discriminator.
#
#   sacrificial P1 (90 deg): 0 cut at every r (first-corner exemption).
#
# WHY THIS MATTERS FOR REGENERATION: a generated cleaning task's coverage
# accounting subtracts predicted cuts. Understating c at reversals
# over-reports coverage on the ~15 % of corners that carry 82-95 % of the
# loss — the unsafe direction (cleaned band eaten silently). The measured
# value slots into rm_emulator._corner_model and the coverage math both.
#
# ANALYSIS: python3 analyse_coverage.py ../runs/<run_dir>. SIM position
# channel is exact on blend geometry (72/72, MOTION_FINDINGS 9) — SIM
# settles this; no REAL needed.
#
# Points are in the CONTROLLER World frame, mm and radians, tool L_glove_4,
# same z-plane and orientation as blend_corner_001. Screen after ANY edit:
#   python3 orientation_cost.py --segments ../paths/c_theta_reversal_001.py \
#           --tool L_glove_4 --speed 0.25
# RUN (one invocation sweeps r = 10 / 25 / 50):
#   python3 test_blend_corner.py --side left --mode SIM \
#           --path ../paths/c_theta_reversal_001.py
# =============================================================================

DEFAULT_SPEED = 100

# Uniform blend radius per pass; the driver runs the sweep one r at a time.
BLEND = 10
BLEND_SWEEP = [10, 25, 50]

TOOL_FRAME = "L_glove_4"

# ONE rung: blend geometry is speed-independent (A.2); 0.25 keeps J4 in the
# proven range on this box.
SPEED_LADDER = [0.25]

TCP_LINEAR_VELOCITY = 0.25
TCP_LINEAR_ACCELERATION = 1.60
TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00

# Interior turn at each vertex P1..P5. P1 is the sacrificial first corner.
CORNER_ANGLES = [90, 166, 170, 173, 176]

POSES_MM = {
    "P0": [715.000, -55.000, -323.628, -3.117, -0.4, 0.077],
    "P1": [630.147,  29.853, -323.628, -3.117, -0.4, 0.077],
    "P2": [545.294, -55.000, -323.628, -3.117, -0.4, 0.077],
    "P3": [648.154,   6.805, -323.628, -3.117, -0.4, 0.077],
    "P4": [536.125, -36.200, -323.628, -3.117, -0.4, 0.077],
    "P5": [652.560,  -7.169, -323.628, -3.117, -0.4, 0.077],
    "P6": [534.383, -28.007, -323.628, -3.117, -0.4, 0.077],
}

SEQUENCE = ["P0", "P1", "P2", "P3", "P4", "P5", "P6"]
