# =============================================================================
# CHAIN SEMANTICS TEST 3 — CHAINED APPROACH consumes the exemption
#                                                       (chain_semantics_003)
# =============================================================================
# QUESTION (MOTION_FINDINGS 9.3c, unverified list): the FIRST corner of a
# chain never blends (9.2 — 18/18 task runs, both directions, moves with the
# direction). If the 50 mm APPROACH move is dispatched as part of the chain,
# does the exemption land on the approach->P0 corner — outside the cleaned
# area — leaving every PATH corner free to blend?
#
# SAME GEOMETRY AS chain_semantics_001 (see its header; one rung at 0.25).
# Uniform r=25, uniform v.
#
# CHAIN_APPROACH = True changes the ENTRY, not the path: the test stops the
# entry sequence at PRESTART (P0 + 50 mm world Z, reached by movej_p and
# VERIFIED by pose readback), starts the recorder, then dispatches
#
#     prestart --movel 50 mm, v=TRANSIT_V, r=25, connect=1--> P0
#     P0 -> P1 -> ... -> P5      (r=25 each, last move r=0 connect=0)
#
# as ONE chain. The recording therefore starts at PRESTART and the commanded
# polyline in run.json starts there too — the approach is measured, on
# purpose.
#
# ⚠ SUPERSEDED by COMMODE_C_CLEANING_CONTRACT A.2 (frozen 2026-08-19).
# The predictions below use cut = 1.4*(r/100)*minL — a single flat
# coefficient. A.2 re-measured c over 1454 corners / 43 runs and it
# DECLINES with r: c(10)=1.70, c(25)=1.57, c(50)=1.33. At r=10 — the
# radius the freeze rule mandates everywhere — 1.70 lies ABOVE the old
# 1.3-1.5 band, so everything below UNDERSTATES the cut, i.e. errs
# toward predicting the tool passes closer to the commanded polyline
# than it does. Kept as the historical prediction record; do not read
# these numbers as current. A.2 also adds two junction terms nothing
# here accounts for: A.2.1 v_arc <= sqrt(a_lat*R) and A.2.2
# cut >= |v_out^2 - v_in^2|/(2a).
# PREDICTED SIGNATURES (cut ~= 1.4*(r/100)*minL; corner A minL = 45 mm step,
# corner P0 minL = 50 mm approach):
#
#   exemption CONSUMED by the approach (the design bet):
#       corner at P0: 0 cut (it is now the chain's first corner; a stop
#                     here is fine — it is the touchdown, not the path)
#       corner A(P1): ~16 mm cut and NO stop  <-- vs 0 cut + stop in
#                     chain_semantics_001/002, same geometry, same r
#   approach REFUSES to chain (chain starts at P0 as before):
#       full stop at P0, corner A: 0 cut + stop (exemption still on A)
#   exemption follows the PATH not the chain (unexpected):
#       corner at P0: ~17 mm cut; corner A: 0 cut + stop
#
# All three outcomes are distinct in one run, and 001/002 double as the
# control (same corner A, unchained entry).
#
# Screen after ANY edit:
#   python3 orientation_cost.py --segments ../paths/chain_semantics_003.py \
#           --tool L_glove_4 --speed 0.25
# =============================================================================

DEFAULT_SPEED = 100

BLEND = 25
BLEND_SWEEP = [25]

# The entry protocol switch this file exists for (read by test_blend_corner).
CHAIN_APPROACH = True

TOOL_FRAME = "L_glove_4"

SPEED_LADDER = [0.25]

TCP_LINEAR_VELOCITY = 0.25
TCP_LINEAR_ACCELERATION = 1.60
TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00

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
