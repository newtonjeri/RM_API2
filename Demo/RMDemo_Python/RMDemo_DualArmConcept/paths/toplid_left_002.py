# =============================================================================
# TOPLID LEFT, REDESIGNED, TWO-PASS  —  toplid_left_002  (rev 3, 2026-08-15)
# =============================================================================
# Rev 3 exists because of the BLEND FLOOR measured in rev 2's SIM run
# (20260815T144323, MOTION_FINDINGS 9.3f): the controller declines to blend
# segments shorter than ~25-30 mm at ANY commanded r and full-stops instead —
# rev 2's single-pass serpentine had 9-25 mm hops and collected 29 brief
# stops at its turnarounds.
#
# THE FIX — TWO-PASS SERPENTINE: odd rows 1,3,5,..,13 top->bottom (pass A),
# then even rows 2,4,..,12,14 (pass B). Same 14 interpolated fan rows, same
# verified 100.00 pct area coverage at the 20 mm effective glove band
# (L_glove_frame_2, 35 mm brush, 1.5 cm tolerance) — but consecutive
# strokes are now TWO row-gaps apart, so the turn hops are 18-39 mm and
# most sit ABOVE the blend floor. Remaining below-floor moves: 5 mid-path
# (18-27.5 mm — the fan's bottom convergence is geometric) + the rim-entry
# hop; those may still stop briefly. This run also MEASURES the floor
# precisely (EMULATOR_ROADMAP gap: which of 18/21/22/24.6/27.5 mm blend).
#
# STRUCTURAL BONUSES vs rev 2:
#   * the pass-A -> pass-B transition runs UP the RIGHT EDGE as one 171 mm
#     cleaning move — the right boundary strip now has its own pass;
#   * row 14 approaches the apex through a ~90 deg corner (blendable),
#     not a reversal; the ONE designed r=0 reversal stop is at the apex
#     between row 15 and the wedge row;
#   * the wedge row runs apex -> left in ONE stroke (no U-turn needed);
#   * touchdown moved to the TOP-LEFT padding (reach-safe side), 25 mm
#     outside the area; CHAIN_APPROACH spends the first-corner exemption
#     and its stop there.
#
# Per-move programs: strokes ending LEFT r=35 (cuts land in the 20 mm
# padding), strokes ending RIGHT r=12 (lid edge = reach boundary, 898 mm,
# no padding room — cuts ~2 mm), 2-gap hops r=25, rim/edge r=10, apex and
# chain close r=0. v: strokes 100, everything else 60.
# J4 screen: worst ~71 pct at 0.25 (proven family). 7.33 m total.
# At a 0.45 baseline: cap stroke V_LIST entries at 65 first (9.3e).
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

R_LIST = [12, 12, 25, 35, 25, 12, 25, 35, 25, 12, 25, 35, 25, 12, 12, 35, 25, 12, 25, 35, 25, 12, 25, 35, 25, 12, 25, 35, 25, 0, 35, 10, 10, 10, 10, 10, 10, 10, 0]
V_LIST = [60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 60, 100, 100, 60, 60, 60, 60, 60, 60, 60, 60]

TOOL_FRAME = "L_glove_2"

SPEED_LADDER = [0.25]

TCP_LINEAR_VELOCITY = 0.25
TCP_LINEAR_ACCELERATION = 1.60
TCP_ANGULAR_VELOCITY = 0.60
TCP_ANGULAR_ACCELERATION = 4.00

POSES_MM = {
    "u00_touch": [  505.008,    43.008,  -272.961, +3.113933, -0.029752, +0.003812],
    "u01_p01L": [  530.003,    43.392,  -272.705, +3.113933, -0.029752, +0.003812],
    "u02_p01R": [  842.500,    48.200,  -269.500, +3.010077, -0.544059, +0.208759],
    "u03_p03R": [  845.833,    27.200,  -269.167, +3.077541, -0.552222, +0.072798],
    "u04_p03L": [  505.001,    25.606,  -272.354, +3.113799, -0.064658, +0.003818],
    "u05_p05L": [  480.834,     4.639,  -271.987, +3.117443, -0.087821, -0.054562],
    "u06_p05R": [  847.000,     2.700,  -269.167, +3.094603, -0.553032, +0.038650],
    "u07_p07R": [  846.000,   -25.300,  -269.500, +3.060534, -0.550917, +0.106903],
    "u08_p07L": [  457.503,   -19.502,  -271.609, +3.125842, -0.098546, -0.171485],
    "u09_p09L": [  436.518,   -46.951,  -272.103, +3.118310, -0.081929, -0.083699],
    "u10_p09R": [  845.000,   -64.300,  -270.000, -3.140741, -0.552658, -0.057022],
    "u11_p11R": [  849.000,   -96.800,  -272.000, +3.009244, -0.550607, +0.058234],
    "u12_p11L": [  426.037,   -71.086,  -272.000, +3.125814, -0.116210, -0.136681],
    "u13_p13L": [  423.735,   -88.977,  -271.700, +3.087881, -0.097850, -0.133238],
    "u14_p13R": [  846.667,  -124.133,  -271.000, +2.968410, -0.548204, +0.079559],
    "u16_edgeup": [  844.167,    37.700,  -269.333, +3.043604, -0.549119, +0.140944],
    "u16_q02L": [  517.502,    34.504,  -272.529, +3.113866, -0.047205, +0.003814],
    "u17_q04L": [  492.501,    16.700,  -272.179, +3.113732, -0.082111, +0.003823],
    "u18_q04R": [  847.500,    16.700,  -269.000, +3.111692, -0.553346, +0.004479],
    "u19_q06R": [  846.500,   -11.300,  -269.333, +3.077542, -0.552222, +0.072798],
    "u20_q06L": [  469.168,    -7.428,  -271.797, +3.121484, -0.093306, -0.112998],
    "u21_q08L": [  447.009,   -33.219,  -271.856, +3.121891, -0.090320, -0.127560],
    "u22_q08R": [  845.500,   -44.800,  -269.750, +3.101436, -0.553217, +0.024983],
    "u23_q10R": [  847.000,   -80.550,  -271.000, +3.075788, -0.553267, +0.000664],
    "u24_q10L": [  431.277,   -59.015,  -272.050, +3.121831, -0.099119, -0.110135],
    "u25_q12L": [  424.885,   -80.031,  -271.850, +3.106830, -0.107046, -0.134871],
    "u26_q12R": [  847.833,  -110.467,  -271.500, +2.988812, -0.549498, +0.068915],
    "u28_apex1": [  845.500,  -137.800,  -270.500, +2.948043, -0.546725, +0.090161],
    "u29_q14L": [  422.588,   -97.922,  -271.549, +3.068963, -0.088625, -0.131781],
    "u30_r15L": [  421.006,  -127.830,  -271.549, +3.058216, +0.139305, -0.124477],
    "u31_apex2": [  845.500,  -137.800,  -270.500, +2.948043, -0.546725, +0.090161],
    "u32_wedgeL": [  421.785,  -112.876,  -271.549, +3.063389, +0.025330, -0.127828],
    "u33_rim13": [  441.000,  -128.300,  -271.500, +3.058216, +0.139305, -0.124477],
    "u33_rim11": [  442.500,   -99.800,  -271.500, +3.068963, -0.088625, -0.131781],
    "u34_rim7": [  456.500,   -47.800,  -272.000, +3.118310, -0.081929, -0.083699],
    "u35_rim5": [  477.500,   -19.800,  -271.500, +3.125842, -0.098546, -0.171485],
    "u36_rim3": [  512.500,    16.700,  -272.000, +3.113732, -0.082111, +0.003823],
    "u37_rim1": [  550.000,    43.700,  -272.500, +3.113933, -0.029752, +0.003812],
    "u39_edge20": [  675.500,    56.200,  -276.000, -3.127306, -0.302294, +0.159851],
    "u40_edge2": [  842.500,    48.200,  -269.500, +3.010077, -0.544059, +0.208759],
}

SEQUENCE = [
    "u00_touch",
    "u01_p01L",
    "u02_p01R",
    "u03_p03R",
    "u04_p03L",
    "u05_p05L",
    "u06_p05R",
    "u07_p07R",
    "u08_p07L",
    "u09_p09L",
    "u10_p09R",
    "u11_p11R",
    "u12_p11L",
    "u13_p13L",
    "u14_p13R",
    "u16_edgeup",
    "u16_q02L",
    "u17_q04L",
    "u18_q04R",
    "u19_q06R",
    "u20_q06L",
    "u21_q08L",
    "u22_q08R",
    "u23_q10R",
    "u24_q10L",
    "u25_q12L",
    "u26_q12R",
    "u28_apex1",
    "u29_q14L",
    "u30_r15L",
    "u31_apex2",
    "u32_wedgeL",
    "u33_rim13",
    "u33_rim11",
    "u34_rim7",
    "u35_rim5",
    "u36_rim3",
    "u37_rim1",
    "u39_edge20",
    "u40_edge2",
]
