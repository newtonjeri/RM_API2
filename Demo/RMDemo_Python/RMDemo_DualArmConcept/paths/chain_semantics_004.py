# =============================================================================
# CHAIN SEMANTICS TEST 1 — per-command BLEND RADIUS  (chain_semantics_004)
# =============================================================================
# QUESTION (MOTION_FINDINGS 9.3c, unverified list): `rm_movel` takes r on
# every call — does a chained program HONOR a different r per move, or does
# the chain latch one radius for the whole program? Everything the redesigned
# cleaning path needs (small r at coverage-critical stroke ends, large r
# elsewhere) rides on this answer, so it is screened in SIM before any path
# is rebuilt around it.
#
# GEOMETRY: a serpentine of FOUR IDENTICAL 90 deg corners — 200 mm strokes,
# 45 mm steps — inside the same hardware-proven box, z-plane, constant
# orientation and tool as blend_corner_001. Identical corners are the point:
# any per-corner difference in the result is the per-move PARAMETER, not the
# geometry. Constant orientation means nothing is angular-throttled (H67).
#
# THIS IS THE 0.45 VARIANT, AND IT IS MARGINAL BY DESIGN: J4 scales
# linearly with line speed on this box — 52 pct at 0.25 screens to ~94 pct
# at 0.45. That is the experiment: SIM either runs it (and answers the
# semantics question at operating speed) or refuses it safely with the
# pendant "Joint4overspeed". If it refuses, that itself bounds where 0.45
# chains can run. Do NOT run this variant in REAL before SIM passes it.
#
#     P0(530,-60) ──m1 200──> P1(730,-60)          corner A (90) end of m1
#                              │ m2 45
#     P3(530,-15) <──m3 200── P2(730,-15)          corner B (90) end of m2
#      │ m4 45                                     corner C (90) end of m3
#     P4(530, 30) ──m5 200──> P5(730, 30)          corner D (90) end of m4
#
# R_LIST — one blend radius PER MOVE, in dispatch order (m1..m5). The test
# sends move i with r=R_LIST[i]; the LAST move always closes the chain with
# r=0, connect=0 regardless (chain-close rule).
#
#     R_LIST = [50, 0, 25, 50, 0]
#
# ⚠ THE PREDICTIONS BELOW ARE HISTORICAL AND UNVALIDATED.
# They use cut = 1.4*(r/100)*minL — a single flat coefficient. A later fit
# over 1454 corners / 43 runs found c DECLINES with r: c(10)=1.70,
# c(25)=1.57, c(50)=1.33. At r=10 — the radius used on dense geometry —
# 1.70 lies ABOVE the old 1.3-1.5 band, so everything below UNDERSTATES
# the cut, i.e. errs toward predicting the tool passes closer to the
# commanded polyline than it does. NEITHER form is hardware-validated.
# Kept as the historical prediction record; do not read these numbers as
# current. Two junction terms nothing here accounts for, both advisory:
# v_arc <= sqrt(a_lat*R) and cut >= |v_out^2 - v_in^2|/(2a)
# (src/junction_limits.py).
# PREDICTED SIGNATURES at the four corners (cut ~= 1.4*(r/100)*minL, minL =
# the 45 mm step — MOTION_FINDINGS 9.2; resolution of the cut measurement is
# ~4 mm, analyse_coverage.py):
#
#   per-command HONORED:  A: 0 (first corner never blends — 9.2, 18/18)
#                         B: 0 cut + FULL STOP  (r=0 honored mid-chain)
#                         C: ~16 mm cut, continuous
#                         D: ~31 mm cut, continuous
#   chain LATCHES FIRST r (50):   B/C/D all ~31 mm, B does NOT stop
#   chain LATCHES LAST r (0):     B/C/D all 0 + stop at every corner
#   uniform-something-else:       B/C/D equal cuts at some other size
#
# ANALYSIS: python3 analyse_coverage.py ../runs/<run_dir>   (per-corner
# entry/exit cut + vmiss), plus the stall table for the stop-vs-continuous
# discrimination. SIM position channel is exact; run this in SIM — SIM
# reproduced blend geometry, cuts, and chain stalls faithfully on 72/72
# runs (MOTION_FINDINGS 9).
#
# Points are in the CONTROLLER World frame, mm and radians, tool L_glove_4,
# same z-plane and orientation as blend_corner_001 (hardware-proven box,
# x 515-788, y -65..+40). Screen after ANY edit:
#   python3 orientation_cost.py --segments ../paths/chain_semantics_004.py \
#           --tool L_glove_4 --speed 0.45
# =============================================================================
# 0.45 VARIANT (2026-08-15): same protocol as its 0.25 twin, run at the
# 0.45 baseline to test whether the chain semantics and the cut geometry
# generalise to operating speed. This box screens J4 ~94 pct at 0.45
# (52 pct at 0.25, linear) — marginal by design; SIM refuses safely if
# over. 005 plateaus if honored: ~250 / ~450 / ~350 mm/s (steps are
# accel-limited and never reach 450 — expected, not a failure).

DEFAULT_SPEED = 100

# Uniform base radius — R_LIST OVERRIDES this per move; kept for the meta
# record and for any tool that reads only BLEND.
BLEND = 25
BLEND_SWEEP = [25]

# Per-move blend radius, dispatch order m1..m5. len == number of moves.
R_LIST = [50, 0, 25, 50, 0]

TOOL_FRAME = "L_glove_4"

# ONE rung at 0.45 — the marginal-by-design point (see header). The harness
# applies line_acc 1.6 (scale_for), which satisfies the >=3x law.
SPEED_LADDER = [0.45]

TCP_LINEAR_VELOCITY = 0.45
TCP_LINEAR_ACCELERATION = 1.60
TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00

# All four corners are 90 deg by construction.
CORNER_ANGLES = [90, 90, 90, 90]

POSES_MM = {
    "P0": [530.000, -60.000, -323.628, -3.117, -0.4, 0.077],
    "P1": [730.000, -60.000, -323.628, -3.117, -0.4, 0.077],
    "P2": [730.000, -15.000, -323.628, -3.117, -0.4, 0.077],
    "P3": [530.000, -15.000, -323.628, -3.117, -0.4, 0.077],
    "P4": [530.000,  30.000, -323.628, -3.117, -0.4, 0.077],
    "P5": [730.000,  30.000, -323.628, -3.117, -0.4, 0.077],
}

SEQUENCE = ["P0", "P1", "P2", "P3", "P4", "P5"]
