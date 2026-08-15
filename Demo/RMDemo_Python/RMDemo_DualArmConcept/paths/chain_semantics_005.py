# =============================================================================
# CHAIN SEMANTICS TEST 2 — per-command SPEED  (chain_semantics_005)
# =============================================================================
# QUESTION (MOTION_FINDINGS 9.3c, unverified list): `rm_movel` takes v (a
# PERCENT of the arm's line_speed baseline — movel-speed-semantics, settled)
# on every call. Does a chained program honor a different v per move, or does
# the chain latch one speed? If per-move v holds, toplid can run fast
# everywhere and slow ONLY into `point13->point12` — the one segment that
# killed both 0.45 runs — instead of capping the whole task.
#
# SAME GEOMETRY AS chain_semantics_004 (see ITS header for why 0.45 is
# marginal-by-design on this box, ~94 pct J4). Uniform r=25 keeps every
# corner blended and continuous so the chain state is the same at each
# corner; only v varies.
#
# V_LIST — one v% PER MOVE, dispatch order m1..m5, of the 0.45 m/s baseline:
#
#     V_LIST = [56, 56, 100, 100, 78]
#     -> stroke1 (m1) 0.252 m/s | stroke2 (m3) 0.45 | stroke3 (m5) 0.351
#
# Ramp check (acc 1.6, what scale_for applies): 0.45 needs ~63 mm each
# way, so the 200 mm strokes keep a ~74 mm plateau at full speed — thin
# but measurable; 0.252 and 0.351 keep >=120 mm. Plateau median over the
# CENTRAL 70 mm of each stroke (windowed speed, 70 ms, the aliasing rule).
#
# PREDICTED PLATEAUS (mm/s) on strokes 1/2/3:
#   per-command HONORED:        ~250 / ~450 / ~350   (three DIFFERENT values)
#   chain LATCHES FIRST v (56): ~250 / ~250 / ~250
#   chain LATCHES LAST v (78):  ~350 / ~350 / ~350
# Three distinct signatures — one run separates all three hypotheses.
# Also check corners B/C/D stay continuous through the v transitions
# (r=25 measured 26 mm/s minima on task U-turns; these 90 deg corners ran
# 61-85 mm/s minima — a stop at a v-change corner means a v change BREAKS
# the chain, which is its own answer).
#
# NOTE the run.json `speed_achieved.pct_of_cap` will read LOW on this run by
# design — the run is not supposed to cruise at cap. Read per-stroke
# plateaus, not the whole-run number.
#
# Screen after ANY edit:
#   python3 orientation_cost.py --segments ../paths/chain_semantics_005.py \
#           --tool L_glove_4 --speed 0.45
# =============================================================================
# 0.45 VARIANT (2026-08-15): same protocol as its 0.25 twin, run at the
# 0.45 baseline to test whether the chain semantics and the cut geometry
# generalise to operating speed. This box screens J4 ~94 pct at 0.45
# (52 pct at 0.25, linear) — marginal by design; SIM refuses safely if
# over. 005 plateaus if honored: ~250 / ~450 / ~350 mm/s (steps are
# accel-limited and never reach 450 — expected, not a failure).

DEFAULT_SPEED = 100

BLEND = 25
BLEND_SWEEP = [25]

# Per-move v%, dispatch order m1..m5. len == number of moves.
V_LIST = [56, 56, 100, 100, 78]

TOOL_FRAME = "L_glove_4"

SPEED_LADDER = [0.45]

TCP_LINEAR_VELOCITY = 0.45
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
