# =============================================================================
# PLANAR SPEED-RAMP TEST  —  planar_speed_ramp_001   (2026-08-19, Newton)
# =============================================================================
# Ramp the linear cap 0.45 -> 1.00 m/s in +0.1 steps with the angular cap
# COUPLED by C2 (omega = 1.25 * v), on a purpose-built planar surface, to
# settle what the contract has been asserting without evidence. Procedure is
# SCREEN -> SIM -> REAL per rung; REAL is never entered if SIM fails.
#
# WHY NOT toplid_left_002 (Newton): toplid carries the fixture's real geometry
# -- fan rows, a converging apex, 10 of 39 moves under the ~25-30 mm blend
# floor -- so a failure there is ambiguous between SPEED and GEOMETRY. This is
# a clean planar rectangle of the same size in the same proven volume, so a
# failure is attributable to speed.
#
# GEOMETRY (contract C4 / C4b / A.1 / A.2 / A.3)
#   surface   planar, Z = -272 mm (toplid measures 17 mm of z-span over
#             413 x 361 mm -- this is that surface, idealised)
#   extent    380 x 180 mm, against toplid_left's own 408 x 194 mm, inside the
#             measured reach of toplid_left_002 (X 421..849, Y -137.8..+56.2)
#   glove     L_glove_frame_2 -- 35 x 80 mm pad, t = 20 mm, presses +Z
#   spacing   20 mm rows = 35 mm brush - 15 mm dimensional tolerance
#   strokes   380 mm. A.1: L_min(1.0 m/s) = v/3 = 333 mm, so EVERY stroke can
#             reach the top rung -- on toplid only 14 of 39 moves clear that.
#   turns     rm_movec semicircles, R = 20 mm, vias at the apex, bulging into
#             the padding OUTSIDE the cleaned band. An arc is geometry, not a
#             blended corner, so it takes no A.2 cut at all.
#   two-pass  odd rows then even rows (toplid_left_002 rev 3's fix) so the turn
#             hop is 40 mm, ABOVE the blend floor. A one-pass 20 mm serpentine
#             would full-stop at every turn and measure nothing.
#
# THE TILT PROFILE IS MANDATORY, AND ITS MAGNITUDE IS NOT A FREE PARAMETER.
# Measured on this path with the J4 screen at 0.45 m/s:
#     constant straight-down press ....... 682 %   (j4 3191 deg/m)
#     + 27 deg roll about the normal ..... 430 %   (j4 1841 deg/m)
#     toplid tilt scaled to 0.67 ......... 432 %   (j4 1773 deg/m)
#     FULL toplid tilt profile ............ 99 %   (j4  355 deg/m)
#     tilt x1.20 (theta 38.1, f 0.405) ..... 41 %   (Newton's f>=0.40 gate)
#     (toplid_left_002 itself ............ 129 %, and it completed on hardware)
# A 380 mm straight sweep in X held at a constant orientation drives the elbow
# through a near-singular configuration -- the cost is the TRANSLATION, not the
# rotation. toplid's varying tilt is what escapes it, and a PARTIAL tilt is as
# bad as none: the arm needs the whole sweep to stay conditioned. So rx/ry
# track X exactly as toplid's do.
#
# THE CONSEQUENCE, which is the finding this test exists to confirm:
#   the conditioning tilt is 36.03 deg over 380 mm, i.e. kappa = 1.655 rad/m.
#   CORRECTED 2026-08-19 (contract A.6). This block previously read 40.5 deg
#   / 1.86 rad/m: that divided the rotation across the 420 mm PADDED span
#   (39.82 deg) by the 380 mm STROKE. Measured on these poses, read in the
#   Euler-RPY convention the controller actually uses (Rz*Ry*Rx,
#   orientation_cost._Rmat -- NOT a rotation vector, which would give 23.5
#   deg / 1.08): theta runs 2.33 deg at x=445 to 37.92 deg at x=825, geodesic
#   sweep 36.03 deg, f = 0.407 at theta_max.
#   A segment demands omega = kappa * v, so at 1.0 m/s it demands 1.655 rad/s
#   -- ABOVE the 1.25 this ladder commanded. omega hits 1.25 at v = 0.755 m/s,
#   so every rung above that was asking for more angular rate than allowed,
#   at the far-reach end where the tilt is steepest. That is where J4 failed.
#   With angular_acc held at 4.0 the vendor 3x ratio caps omega at 1.333, so
#       v_stroke(max) = 1.333 / 1.86 = 0.72 m/s   -- REGARDLESS of the linear cap
#   and under C2's ratio coupling (omega_cap = 1.25 v) every stroke runs at
#       v_eff = 1.25 v / 1.86 = 0.67 * v          -- at EVERY rung.
#   THAT PREDICTION IS SUPERSEDED (Newton, 2026-08-19). He moved the contact
#   gate from f >= 0.5 to f >= 0.40 rather than cap the tilt, on the grounds
#   that 4 mm of contact band is cheap and rotation buys manipulability. He
#   was right, and by more than he claimed: at theta = 38.1 deg the SAME ramp
#   screens 41 % at 0.45 m/s, 73 % at 0.80 and 91 % at 1.00, where the 30-deg
#   geometry blocked everything above 0.50. The 1.0 m/s target is plausible
#   again — on the J4 screen, which is J4-ONLY and cannot see that J1 binds on
#   11 of 24 tasks. Only the all-joint abort in SIM/REAL settles it.
#
# PER-MOVE PROGRAM
#   strokes  v 100 % of the rung, r 10 (A.2 freeze rule)
#   arcs     v 25 % -- lateral accel on a 20 mm radius is v^2/R; at the top rung
#            25 % is 0.25 m/s = 3.1 m/s^2, about the linear accel. The arcs are
#            not under test; the straights are.
#   entry    r 0 at touchdown in the padding -- CHAIN_APPROACH spends the
#            first-corner exemption there (A.2: the first corner never blends).
#   blend    at a stroke/arc junction A.2 takes min(L_in, L_out) = the 40 mm arc
#            chord, so at r=10 the cut is 1.70*0.10*40 = 6.8 mm, inside the
#            25 mm padding. Nothing lands in the cleaned band.
#
# RUN -- use the driver, which enforces screen -> SIM -> REAL and the abort rule:
#   python3 src/run_speed_ramp.py --side left
# =============================================================================

DEFAULT_SPEED = 100
BLEND = 10
BLEND_SWEEP = [10]

CHAIN_APPROACH = True

TOOL_FRAME = "L_glove_2"

SPEED_LADDER = [0.45, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

TCP_LINEAR_VELOCITY = 0.45
TCP_LINEAR_ACCELERATION = 1.60      # = max(1.6, 3v) at the first rung
TCP_ANGULAR_VELOCITY = 0.5625       # = 1.25 * 0.45, the C2 coupling
TCP_ANGULAR_ACCELERATION = 4.00     # the SHIPPED default = the FLOOR, not a cap.
                                    # The driver raises it per rung as
                                    # max(4.0, 3*omega) — the 1.33 rad/s
                                    # ceiling was REMOVED (Newton, 2026-08-19),
                                    # so acceleration scales with the cap
                                    # instead of pinning it. Cost is H62: a
                                    # higher value lengthens the stop.

R_LIST = [0, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10]
V_LIST = [60, 100, 25, 100, 25, 100, 25, 100, 25, 100, 25, 100, 25, 100, 25, 100, 25, 100, 25, 100]
ARC_LIST = [None, None, 'V01', None, 'V02', None, 'V03', None, 'V04', None, 'V05', None, 'V06', None, 'V07', None, 'V08', None, 'V09', None]

POSES_MM = {
    "t00_touch": [  425.000, -130.000,   -272.0, +3.120501, +0.002711, +0.000000],
    "r00_in": [  445.000, -130.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r00_out": [  825.000, -130.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r02_in": [  825.000,  -90.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r02_out": [  445.000,  -90.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r04_in": [  445.000,  -50.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r04_out": [  825.000,  -50.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r06_in": [  825.000,  -10.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r06_out": [  445.000,  -10.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r08_in": [  445.000,   30.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r08_out": [  825.000,   30.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r01_in": [  825.000, -110.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r01_out": [  445.000, -110.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r03_in": [  445.000,  -70.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r03_out": [  825.000,  -70.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r05_in": [  825.000,  -30.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r05_out": [  445.000,  -30.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r07_in": [  445.000,   10.000,   -272.0, +3.113933, -0.029752, +0.000000],
    "r07_out": [  825.000,   10.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r09_in": [  825.000,   50.000,   -272.0, +2.989133, -0.646552, +0.000000],
    "r09_out": [  445.000,   50.000,   -272.0, +3.113933, -0.029752, +0.000000],
}

VIA_MM = {
    "V01": [  845.000, -110.000,   -272.0, +2.982565, -0.679015, +0.000000],
    "V02": [  425.000,  -70.000,   -272.0, +3.120501, +0.002711, +0.000000],
    "V03": [  845.000,  -30.000,   -272.0, +2.982565, -0.679015, +0.000000],
    "V04": [  425.000,   10.000,   -272.0, +3.120501, +0.002711, +0.000000],
    "V05": [  845.000,  -40.000,   -272.0, +2.982565, -0.679015, +0.000000],
    "V06": [  425.000,  -90.000,   -272.0, +3.120501, +0.002711, +0.000000],
    "V07": [  845.000,  -50.000,   -272.0, +2.982565, -0.679015, +0.000000],
    "V08": [  425.000,  -10.000,   -272.0, +3.120501, +0.002711, +0.000000],
    "V09": [  845.000,   30.000,   -272.0, +2.982565, -0.679015, +0.000000],
}
