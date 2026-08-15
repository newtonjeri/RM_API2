# =============================================================================
# TOPLID LEFT, REDESIGNED, GLOVE-COVERAGE-COMPLETE  —  toplid_left_002
# =============================================================================
# Revision 2 (2026-08-15): stroke density now set by the GLOVE, not by the
# original waypoints. glove_frames.yaml gives L_glove_frame_2 a 35 mm brush
# width (W, perpendicular to the wipe); with Newton's 1.5 cm dimensional
# tolerance the guaranteed band per stroke is 20 mm. The original task's
# 7 rows are 24-42 mm apart, so THE ORIGINAL NEVER GUARANTEED COVERAGE —
# and neither did revision 1, which reused its rows.
#
# THIS revision interpolates the fan: 14 stroke rows (positions linear,
# orientations SLERPED between the proven neighbouring rows), max gap
# 19.5 mm, plus a short wedge row where rows 14/15 converge at point12.
# Numerically verified: 100.00 pct of the original-area convex hull lies
# within 10 mm of a cleaning centerline (17 130 samples, 2 mm grid).
# Overlap everywhere, as advised.
#
# Everything else is revision 1's verified structure (MOTION_FINDINGS
# 9.2/9.3b/9.3d/9.3e):
#   * serpentine, touchdown 25 mm outside the top-right corner
#     (CHAIN_APPROACH consumes the first-corner exemption + its stop);
#   * LEFT turns padded 20 mm past the rim — strokes carry r=35, the short
#     hops carry r=15 (below the r>=25 chain-hazard band seen at the hinge
#     termination and the point13 freeze; cuts land in padding at any r);
#   * RIGHT edge unpadded (the lid edge IS the reach boundary, 898 mm:
#     +20 mm there screens J4 at 2-5x) — r=12 corners, ~2-4 mm cuts that
#     the glove's 80 mm along-wipe length covers anyway;
#   * ONE r=0 stop at the point12 fan apex (row 14''s end);
#   * wedge row U-turns INSIDE already-covered ground (r=35, clip harmless)
#     then transits back to the rim start;
#   * rim pass (13-11-7-5-3-1) and top edge (1-20-2) at r=10, v=60;
#   * orientation follows the lid crown along every stroke (H67 throttles).
#
# J4 screen: worst segment ~71 pct at 0.25 (matches the original's measured
# 56-73 pct family; interpolated rows sit between proven neighbours).
# LENGTH: 6.91 m (original: 6.20 m — but under-covered; rev 1: 3.43 m,
# also under-covered). Estimated stroke time dominates; at a 0.45 baseline
# cap stroke V_LIST entries at 65 first (9.3e) and run chain_semantics_004/
# 005 before any 0.45 attempt.
#
# RUN (SIM first; left arm, tool L_glove_2):
#   python3 test_blend_corner.py --side left --mode SIM \
#           --path ../paths/toplid_left_002.py
# Screen after ANY edit:
#   python3 orientation_cost.py --segments ../paths/toplid_left_002.py \
#           --tool L_glove_2 --speed 0.25
# =============================================================================

DEFAULT_SPEED = 100
BLEND = 35
BLEND_SWEEP = [35]

CHAIN_APPROACH = True

R_LIST = [12, 35, 15, 12, 12, 35, 15, 12, 12, 35, 15, 12, 12, 35, 15, 12, 12, 35, 15, 12, 12, 35, 15, 12, 12, 35, 15, 0, 10, 15, 35, 15, 10, 10, 10, 10, 10, 10, 10, 0]
V_LIST = [60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 100, 60, 100, 60, 100, 60, 60, 60, 60, 60, 60, 60]

TOOL_FRAME = "L_glove_2"

SPEED_LADDER = [0.25]

TCP_LINEAR_VELOCITY = 0.25
TCP_LINEAR_ACCELERATION = 1.60
TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00

POSES_MM = {
    "t00_touch": [  838.581,    72.888,  -269.892, +3.010077, -0.544059, +0.208759],
    "t01_r01R": [  842.500,    48.200,  -269.500, +3.010077, -0.544059, +0.208759],
    "t02_r01L": [  530.003,    43.392,  -272.705, +3.113933, -0.029752, +0.003812],
    "t03_r02L": [  517.502,    34.504,  -272.529, +3.113866, -0.047205, +0.003814],
    "t04_r02R": [  844.167,    37.700,  -269.333, +3.043604, -0.549119, +0.140944],
    "t05_r03R": [  845.833,    27.200,  -269.167, +3.077541, -0.552222, +0.072798],
    "t06_r03L": [  505.001,    25.606,  -272.354, +3.113799, -0.064658, +0.003818],
    "t07_r04L": [  492.501,    16.700,  -272.179, +3.113732, -0.082111, +0.003823],
    "t08_r04R": [  847.500,    16.700,  -269.000, +3.111692, -0.553346, +0.004479],
    "t09_r05R": [  847.000,     2.700,  -269.167, +3.094603, -0.553032, +0.038650],
    "t10_r05L": [  480.834,     4.639,  -271.987, +3.117443, -0.087821, -0.054562],
    "t11_r06L": [  469.168,    -7.428,  -271.797, +3.121484, -0.093306, -0.112998],
    "t12_r06R": [  846.500,   -11.300,  -269.333, +3.077542, -0.552222, +0.072798],
    "t13_r07R": [  846.000,   -25.300,  -269.500, +3.060534, -0.550917, +0.106903],
    "t14_r07L": [  457.503,   -19.502,  -271.609, +3.125842, -0.098546, -0.171485],
    "t15_r08L": [  447.009,   -33.219,  -271.856, +3.121891, -0.090320, -0.127560],
    "t16_r08R": [  845.500,   -44.800,  -269.750, +3.101436, -0.553217, +0.024983],
    "t17_r09R": [  845.000,   -64.300,  -270.000, -3.140741, -0.552658, -0.057022],
    "t18_r09L": [  436.518,   -46.951,  -272.103, +3.118310, -0.081929, -0.083699],
    "t19_r10L": [  431.277,   -59.015,  -272.050, +3.121831, -0.099119, -0.110135],
    "t20_r10R": [  847.000,   -80.550,  -271.000, +3.075788, -0.553267, +0.000664],
    "t21_r11R": [  849.000,   -96.800,  -272.000, +3.009244, -0.550607, +0.058234],
    "t22_r11L": [  426.037,   -71.086,  -272.000, +3.125814, -0.116210, -0.136681],
    "t23_r12L": [  424.885,   -80.031,  -271.850, +3.106830, -0.107046, -0.134871],
    "t24_r12R": [  847.833,  -110.467,  -271.500, +2.988812, -0.549498, +0.068915],
    "t25_r13R": [  846.667,  -124.133,  -271.000, +2.968410, -0.548204, +0.079559],
    "t26_r13L": [  423.735,   -88.977,  -271.700, +3.087881, -0.097850, -0.133238],
    "t27_r14L": [  422.588,   -97.922,  -271.549, +3.068963, -0.088625, -0.131781],
    "t28_r14R": [  845.500,  -137.800,  -270.500, +2.948043, -0.546725, +0.090161],
    "t29_row15": [  441.000,  -128.300,  -271.500, +3.058216, +0.139305, -0.124477],
    "t30_wedgeL": [  421.785,  -112.876,  -271.549, +3.063389, +0.025330, -0.127828],
    "t31_wedgeP": [  611.325,  -124.025,  -271.080, +3.032534, -0.217666, -0.048168],
    "t32_wedgeT": [  612.206,  -109.051,  -271.080, +3.034480, -0.206156, -0.052211],
    "t33_rim13": [  441.000,  -128.300,  -271.500, +3.058216, +0.139305, -0.124477],
    "t34_rim11": [  442.500,   -99.800,  -271.500, +3.068963, -0.088625, -0.131781],
    "t35_rim7": [  456.500,   -47.800,  -272.000, +3.118310, -0.081929, -0.083699],
    "t36_rim5": [  477.500,   -19.800,  -271.500, +3.125842, -0.098546, -0.171485],
    "t37_rim3": [  512.500,    16.700,  -272.000, +3.113732, -0.082111, +0.003823],
    "t38_rim1": [  550.000,    43.700,  -272.500, +3.113933, -0.029752, +0.003812],
    "t39_edge20": [  675.500,    56.200,  -276.000, -3.127306, -0.302294, +0.159851],
    "t40_edge2": [  842.500,    48.200,  -269.500, +3.010077, -0.544059, +0.208759],
}

SEQUENCE = [
    "t00_touch",
    "t01_r01R",
    "t02_r01L",
    "t03_r02L",
    "t04_r02R",
    "t05_r03R",
    "t06_r03L",
    "t07_r04L",
    "t08_r04R",
    "t09_r05R",
    "t10_r05L",
    "t11_r06L",
    "t12_r06R",
    "t13_r07R",
    "t14_r07L",
    "t15_r08L",
    "t16_r08R",
    "t17_r09R",
    "t18_r09L",
    "t19_r10L",
    "t20_r10R",
    "t21_r11R",
    "t22_r11L",
    "t23_r12L",
    "t24_r12R",
    "t25_r13R",
    "t26_r13L",
    "t27_r14L",
    "t28_r14R",
    "t29_row15",
    "t30_wedgeL",
    "t31_wedgeP",
    "t32_wedgeT",
    "t33_rim13",
    "t34_rim11",
    "t35_rim7",
    "t36_rim5",
    "t37_rim3",
    "t38_rim1",
    "t39_edge20",
    "t40_edge2",
]
