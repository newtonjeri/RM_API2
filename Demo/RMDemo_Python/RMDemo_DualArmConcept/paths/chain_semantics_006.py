# =============================================================================
# CHAIN SEMANTICS TEST 6 — rm_movec ARCS IN A CHAIN  (chain_semantics_006)
# =============================================================================
# QUESTION (CLEANING_MOTION_SPEC §3): `rm_movec(pose_via, pose_to, v, r,
# loop, connect, block)` exists and has never been dispatched. The wiping
# program wants arcs for turnarounds and for the annular regions
# (seat_ring, bowl rims — circles currently authored as polygons whose
# every vertex fights the blend engine and the angular cap). Before any
# path is designed around arcs, ONE run answers:
#
#   1. does movec ACCEPT connect=1 inside a movel chain (ret=0)?
#      A ret=1 at move 2 is itself the answer — the harness prints which
#      move refused.
#   2. is the line->arc junction CONTINUOUS? The arcs here are semicircles
#      TANGENT to the strokes, so there is no geometric corner — any stop
#      at P1/P2/P3/P4 is chain behaviour, not geometry.
#   3. does the tool TRACE the commanded circle? (SIM position channel is
#      exact; compare against the ideal semicircle through the via.)
#   4. does the FIRST-move exemption (movel chains never blend their first
#      corner) have a movec analogue? Compare arc 1 vs arc 2.
#
# GEOMETRY: three 150 mm strokes joined by two semicircular turnarounds of
# radius 22.5 mm (half the 45 mm row spacing), vias at the arc apex.
# Same proven box / z-plane / constant orientation / tool as
# chain_semantics_001 (J4 screens ~52 % of limit at 0.25 on these strokes;
# arcs stay inside x 515-730). Constant orientation keeps the angular cap
# out of the measurement.
#
#     P0(545,-60) ──150──> P1(695,-60) ⌒V1(717.5,-37.5)⌒ P2(695,-15)
#     P3(545,-15) <──150── P2          (arc bulges +x, tangent both ends)
#     P3 ⌒V2(522.5,7.5)⌒ P4(545,30) ──150──> P5(695,30)
#
# ARC_LIST — one entry PER MOVE: None = movel, or a VIA_MM key = movec
# through that via. loop is ALWAYS dispatched 0; the loop parameter is
# deliberately untested here (unknown chain interaction — do not add it to
# this file without its own screen).
#
# Chord-vs-arc note: the harness's arc-length sanity check measures against
# the waypoint polyline; two semicircles inflate the traced path ~9 % over
# the chords, inside its 15 % tolerance — expected, not an error.
#
# ANALYSIS: python3 analyse_run.py <run> for the state machine (arm_status
# MOVE_C?), plus the dedicated arc check:
#   the traced points during each turnaround vs the circle of radius
#   22.5 mm centred between the stroke ends — report max radial error.
# Run in SIM first:
#   python3 test_blend_corner.py --side left --mode SIM \
#           --path ../paths/chain_semantics_006.py
# =============================================================================

DEFAULT_SPEED = 100
BLEND = 25
BLEND_SWEEP = [25]

# Per-move: m1 line, m2 ARC via V1, m3 line, m4 ARC via V2, m5 line (close).
ARC_LIST = [None, "V1", None, "V2", None]

TOOL_FRAME = "L_glove_4"

SPEED_LADDER = [0.25]

TCP_LINEAR_VELOCITY = 0.25
TCP_LINEAR_ACCELERATION = 1.60
TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00

POSES_MM = {
    "P0": [545.000, -60.000, -323.628, -3.117, -0.4, 0.077],
    "P1": [695.000, -60.000, -323.628, -3.117, -0.4, 0.077],
    "P2": [695.000, -15.000, -323.628, -3.117, -0.4, 0.077],
    "P3": [545.000, -15.000, -323.628, -3.117, -0.4, 0.077],
    "P4": [545.000,  30.000, -323.628, -3.117, -0.4, 0.077],
    "P5": [695.000,  30.000, -323.628, -3.117, -0.4, 0.077],
}

# Arc via poses (apex of each semicircle), same orientation as the strokes.
VIA_MM = {
    "V1": [717.500, -37.500, -323.628, -3.117, -0.4, 0.077],
    "V2": [522.500,   7.500, -323.628, -3.117, -0.4, 0.077],
}

SEQUENCE = ["P0", "P1", "P2", "P3", "P4", "P5"]
