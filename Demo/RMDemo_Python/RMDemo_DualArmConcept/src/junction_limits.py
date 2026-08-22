"""Arc speed and junction speed-step feasibility, offline. ADVISORY.

Two geometric checks on a path's corners, computable with no hardware:

    ARC   v_arc <= sqrt(a_lat * R)            R = (c^2/4 + s^2) / (2s)
    STEP  cut   >= |v_out^2 - v_in^2| / (2a)
    cut    = c(r) * (r/100) * min(L_in, L_out)
             c(10) = 1.70   c(25) = 1.57   c(50) = 1.33   [1454-corner fit]

`r` IS A PERCENTAGE 0-100, NOT MILLIMETRES (vendor: jiao rong ban jing bai fen bi).

*** NOTHING HERE GATES. *** This module PREDICTS; it never refuses a rung.
Every verdict it prints is a hypothesis to be scored against what the arm
actually does, and the ladder runs every rung regardless of what it says.
That is deliberate — see WHY ADVISORY below.

WHY THESE CHECKS EXIST. The 2026-08-19 hardware ramp failed at v = 1.00 with
`Out Of Reach, reason: Joint4overspeed`, and the code was on the GUI only:
`err1..err7` and `lift_err` were zero in all 23 recordings and `arm_status`
read IDLE straight through the event. Nothing in-band saw it. STEP is one
candidate explanation — that the failure was a GEOMETRY defect, exact and
computable offline. The J4 screen could not have caught it (it reads 10-15 %
LOW, and averages a 3-sample event across 1220 samples); a question asked
about a single junction can.

WHAT HARDWARE HAS SAID SO FAR — and it points both ways:

  * FOR the STEP mechanism (2026-08-21). The controller REFUSES a chain
    outright (`ret=1`) when a blend cut cannot carry its speed step: the arm
    stopped 12.3 mm short of `point15` against a 12.8 mm cut that needed
    90.2 mm. The mechanism is real and the controller enforces it itself.
  * AGAINST this module's CALIBRATION. Run over `planar_speed_ramp_001` on
    the commanded speeds, STEP flags 19 of 19 junctions at rung 0.45 — a rung
    that completed on hardware with J4 at 47.8 % and 0 ms of dwell. It also
    ranks the one junction that DID fail (`r01_in`, the 145.6 mm arc into a
    full-speed stroke) as the LEAST severe of the nine arc->stroke entries at
    every rung, because its longer approach raises `min(L_in, L_out)` and so
    credits it with more cut:

        rung 0.90   r01_in   deficit 115.0 mm   implied 15.1 m/s^2 ( 5.6x cmd)
                    r02_in   deficit 129.4 mm   implied 35.5 m/s^2 (13.1x cmd)
                    r09_in   deficit 129.4 mm   implied 35.5 m/s^2 (13.1x cmd)

    Measurement says the opposite: `r01_in` was the SOLE source of all J4 load
    above 51 %, and the eight 56.6 mm entries never passed 51 %.

WHY ADVISORY. A right mechanism with a wrong calibration must not be allowed
to decide, and the two bullets above are exactly that. Gating on it would
refuse rungs the arm demonstrably completes — which would suppress the very
measurements that could fix the calibration. So: the ladder runs, this module
predicts, the recordings judge. Do NOT re-tune the coefficients to make the
flags come out right; measure c(theta) directly (the sweep paths are built)
and let the measurement land.

WHERE A FLAG DOES POINT SOMEWHERE. When a junction is infeasible at every r,
the resolution is to LOWER THE SPEED, never to raise r — a bigger blend on
dense geometry cuts the corner further off the commanded polyline. So this
module reports a speed to drop to and never an r to raise, as a suggestion.

The two checks belong together: the composite fix for the ramp failure is to
lift the approach arc to its own ARC limit first, which shrinks the residual
step to one STEP can afford. Neither alone is sufficient.
"""

import math

# --- blend-cut law ------------------------------------------------------
# Measured domain is r ∈ {10, 25, 50} ONLY (72 runs, 2026-08-14, two
# geometries, both arms, both directions, 0.25 m/s baseline). Blend geometry
# is speed-independent, so this table transfers across rungs; it does NOT
# transfer past r = 50, which is why `max_cut` stops there.
_C_OF_R = ((10.0, 1.70), (25.0, 1.57), (50.0, 1.33))
R_MEASURED_MAX = 50.0


def c_of_r(r_pct):
    """Cut coefficient c(r). Declines with r — it is NOT the flat 1.4.

    The flat-in-r form (0 / 0.70 / 1.4 by turn angle) was retired from
    `rm_emulator.py:_corner_model` in 2fbc9f4; the PREDICTED SIGNATURE headers
    of `paths/chain_semantics_00{1,3,4}.py` keep it as annotated history. At
    r = 10 — the radius used on dense geometry — this 1.70 sits ABOVE the old
    band, so the old form understated the cut where this project operates.

    *** NEITHER FORM IS HARDWARE-VALIDATED. *** This table is a fit pooled
    over a mixed turn-angle corpus, and the loss is concentrated at reversals,
    so a per-corner coefficient is expected to differ from it — above 1.70 at
    reversals, below it near 90 degrees. That is what the c(theta) sweep
    measures. Until it runs, treat these as the best available estimate and
    not as measured truth.
    """
    if r_pct <= _C_OF_R[0][0]:
        return _C_OF_R[0][1]
    if r_pct >= _C_OF_R[-1][0]:
        return _C_OF_R[-1][1]
    for (r0, c0), (r1, c1) in zip(_C_OF_R, _C_OF_R[1:]):
        if r_pct <= r1:
            return c0 + (c1 - c0) * (r_pct - r0) / (r1 - r0)
    return _C_OF_R[-1][1]


def blend_cut(r_pct, l_in, l_out):
    """Total arc-length removed (entry + exit) at a blended vertex [m]."""
    if r_pct <= 0.0:
        return 0.0
    return c_of_r(r_pct) * (r_pct / 100.0) * min(l_in, l_out)


def max_cut(l_in, l_out, r_limit=R_MEASURED_MAX):
    """Largest cut this junction can offer, inside the MEASURED r domain.

    The ceiling is evaluated at r = 50, not r = 100: c(r) past 50 has never
    been measured, and large r on dense geometry cuts the corner too far off
    the commanded polyline anyway — an infeasibility verdict must not be
    talked away with an extrapolated coefficient.
    """
    return blend_cut(min(r_limit, R_MEASURED_MAX), l_in, l_out)


# --- arc speed --------------------------------------------------------
def arc_radius_chord_sagitta(chord, sagitta):
    """R from chord c and sagitta s [m]."""
    if sagitta <= 0.0:
        return float("inf")
    return (chord * chord / 4.0 + sagitta * sagitta) / (2.0 * sagitta)


def arc_radius_3pt(a, via, b):
    """Exact circumradius through three 3-D points [m].

    Preferred over chord/sagitta when the via is known, because it needs no
    estimate: the via IS on the arc, so the circumcircle is determined. The
    two agree only when the via sits at the arc midpoint.
    """
    ax, ay, az = a
    vx, vy, vz = via
    bx, by, bz = b
    p = math.dist((ax, ay, az), (vx, vy, vz))
    q = math.dist((vx, vy, vz), (bx, by, bz))
    r = math.dist((bx, by, bz), (ax, ay, az))
    ux, uy, uz = vx - ax, vy - ay, vz - az
    wx, wy, wz = bx - ax, by - ay, bz - az
    cx = uy * wz - uz * wy
    cy = uz * wx - ux * wz
    cz = ux * wy - uy * wx
    area2 = math.sqrt(cx * cx + cy * cy + cz * cz)   # = 2 * triangle area
    if area2 < 1e-12:
        return float("inf")                          # collinear → straight
    return p * q * r / (2.0 * area2)


def arc_length_3pt(a, via, b):
    """Length along the circular arc a → via → b [m]."""
    R = arc_radius_3pt(a, via, b)
    if not math.isfinite(R):
        return math.dist(a, via) + math.dist(via, b)

    def sweep(p, q):
        d = math.dist(p, q)
        return 2.0 * math.asin(min(1.0, d / (2.0 * R)))

    return R * (sweep(a, via) + sweep(via, b))


def v_arc_max(R, a_lat):
    """Lateral-acceleration ceiling on an arc [m/s]. a_lat = commanded line_acc."""
    if not math.isfinite(R):
        return float("inf")
    return math.sqrt(max(0.0, a_lat) * R)


# --- junction speed step ---------------------------------------------
def required_cut(v_in, v_out, a):
    """Blend length the commanded speed CHANGE needs [m]."""
    if a <= 0.0:
        return float("inf")
    return abs(v_out * v_out - v_in * v_in) / (2.0 * a)


def feasible_speed(v_other, cut, a):
    """Largest speed reachable across `cut` from `v_other` [m/s].

    The "lower speed binds" resolution, solved for the speed rather than the
    cut: v^2 = v_other^2 +/- 2*a*cut.
    """
    return math.sqrt(max(0.0, v_other * v_other + 2.0 * a * cut))


# --- path-level check -------------------------------------------------------
# Path-module convention (test_blend_corner.py:214): R_LIST/V_LIST/ARC_LIST
# hold one value PER MOVE in dispatch order, len == len(POSES_MM) - 1.
# `rm_movel`'s r blends at the vertex the move ENDS on, so move i's r governs
# the junction at pose[i+1]. A CHAIN_APPROACH prestart move is prepended by
# the driver and is NOT modelled here; it carries the first-corner exemption.

def _mm_to_m(p):
    return (p[0] / 1000.0, p[1] / 1000.0, p[2] / 1000.0)


def moves_of(mod, rung):
    """[(name_from, name_to, length_m, v_mps, via_or_None, R_m_or_None)]."""
    names = list(mod.POSES_MM)
    poses = [_mm_to_m(mod.POSES_MM[n]) for n in names]
    vias = getattr(mod, "VIA_MM", {}) or {}
    arc_list = getattr(mod, "ARC_LIST", None) or [None] * (len(names) - 1)
    v_list = getattr(mod, "V_LIST", None) or [100] * (len(names) - 1)
    out = []
    for i in range(len(poses) - 1):
        key = arc_list[i] if i < len(arc_list) else None
        via = _mm_to_m(vias[key]) if key else None
        if via:
            length = arc_length_3pt(poses[i], via, poses[i + 1])
            radius = arc_radius_3pt(poses[i], via, poses[i + 1])
        else:
            length = math.dist(poses[i], poses[i + 1])
            radius = None
        v = rung * (float(v_list[i]) / 100.0)
        out.append((names[i], names[i + 1], length, v, key, radius))
    return out


def check_path(mod, rung, line_acc):
    """Full ARC + STEP audit of a path module at one rung.

    Returns (arcs, junctions, worst) — `worst` is the lowest speed either check
    PREDICTS the path is limited to, or None when everything clears. Advisory:
    no caller should refuse a rung on it. ARC and STEP are evaluated together,
    never separately.
    """
    mv = moves_of(mod, rung)
    r_list = getattr(mod, "R_LIST", None) or [0] * len(mv)

    arcs = []
    for i, (a, b, L, v, key, R) in enumerate(mv):
        if key is None:
            continue
        vmax = v_arc_max(R, line_acc)
        arcs.append({"i": i, "at": "%s->%s" % (a, b), "via": key, "R": R,
                     "L": L, "v": v, "v_max": vmax, "ok": v <= vmax + 1e-9})

    junctions = []
    for i in range(len(mv) - 1):
        a_in, b_in, l_in, v_in, key_in, R_in = mv[i]
        a_out, b_out, l_out, v_out, key_out, _ = mv[i + 1]
        r = float(r_list[i]) if i < len(r_list) else 0.0
        cut = blend_cut(r, l_in, l_out)
        need = required_cut(v_in, v_out, line_acc)
        ceiling = max_cut(l_in, l_out)
        # The resolution when infeasible: LOWER SPEED. Never a larger r.
        v_bind = feasible_speed(min(v_in, v_out), cut, line_acc)
        rec = {"i": i, "vertex": b_in, "r": r, "l_in": l_in, "l_out": l_out,
               "v_in": v_in, "v_out": v_out, "cut": cut, "need": need,
               "ceiling": ceiling, "v_bind": v_bind,
               "ok": cut + 1e-12 >= need,
               "infeasible_any_r": ceiling + 1e-12 < need}
        # The composite fix (ARC + STEP together): if the approach is
        # an arc it may be allowed to run faster than commanded, which shrinks
        # the step this junction has to absorb.
        if key_in is not None and R_in is not None:
            v_arc = min(v_arc_max(R_in, line_acc), max(v_in, v_out))
            rec["v_in_lifted"] = v_arc
            rec["need_lifted"] = required_cut(v_arc, v_out, line_acc)
            rec["lifted_ok"] = cut + 1e-12 >= rec["need_lifted"]
        junctions.append(rec)

    binds = [a["v_max"] for a in arcs if not a["ok"]]
    binds += [j["v_bind"] for j in junctions if not j["ok"]]
    return arcs, junctions, (min(binds) if binds else None)


def report(mod, rung, line_acc, name=""):
    """Human-readable audit. Returns (text, worst_binding_speed_or_None)."""
    arcs, junctions, worst = check_path(mod, rung, line_acc)
    L = []
    L.append("JUNCTION AUDIT (ADVISORY)  %s  rung %.2f m/s  line_acc %.2f m/s^2"
             % (name, rung, line_acc))
    bad_a = [a for a in arcs if not a["ok"]]
    L.append("  ARC arcs: %d checked, %d over the lateral-acc limit"
             % (len(arcs), len(bad_a)))
    for a in bad_a:
        L.append("    OVER  %-22s R %5.1f mm  v %.3f > sqrt(a*R) = %.3f m/s"
                 % (a["at"], a["R"] * 1000, a["v"], a["v_max"]))
    bad_j = [j for j in junctions if not j["ok"]]
    L.append("  STEP junctions: %d checked, %d short of the cut they need"
             % (len(junctions), len(bad_j)))
    for j in bad_j:
        tag = "INFEASIBLE AT EVERY r" if j["infeasible_any_r"] else "short at this r"
        L.append("    %-21s %-10s r=%2.0f  L_in %6.1f  L_out %6.1f mm"
                 % (j["vertex"], tag, j["r"], j["l_in"] * 1000, j["l_out"] * 1000))
        L.append("        v %.3f -> %.3f needs %5.1f mm, r=%2.0f delivers %5.1f mm, "
                 "r=50 ceiling %5.1f mm"
                 % (j["v_in"], j["v_out"], j["need"] * 1000, j["r"],
                    j["cut"] * 1000, j["ceiling"] * 1000))
        L.append("        lower speed suggested -> v_out <= %.3f m/s (never a larger r)"
                 % j["v_bind"])
        if "v_in_lifted" in j:
            L.append("        ARC composition: lift the approach arc to %.3f m/s "
                     "-> step needs %5.1f mm -> %s"
                     % (j["v_in_lifted"], j["need_lifted"] * 1000,
                        "CLEARS" if j["lifted_ok"] else "still short"))
    a_bind = min([a["v_max"] for a in arcs if not a["ok"]], default=None)
    if a_bind is None:
        L.append("  ARC (advisory): arcs clear this rung.")
    else:
        L.append("  ARC (advisory): arcs predict this rung limited to %.3f m/s."
                 % a_bind)
    j_bind = min([j["v_bind"] for j in junctions if not j["ok"]], default=None)
    if j_bind is None:
        L.append("  STEP (advisory): junctions clear this rung.")
    else:
        L.append("  STEP (advisory): %d/%d junctions short on commanded "
                 "speeds; the" % (len(bad_j), len(junctions)))
        L.append("      lower speed would put this rung at %.3f m/s." % j_bind)
        L.append("      NOT ENFORCED. This predictor is known to over-flag: run")
        L.append("      verbatim it refuses rungs hardware has completed "
                 "(19/19 at 0.45,")
        L.append("      J4 47.8 %, 0 ms dwell). Score it against the recording; "
                 "do not obey it,")
        L.append("      and do not re-tune it to fit — measure c(theta).")
    return "\n".join(L), worst


if __name__ == "__main__":
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from run_speed_ramp import load_path, line_acc_for

    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "paths", "planar_speed_ramp_001.py")
    mod = load_path(path)
    rungs = ([float(x) for x in sys.argv[2].split(",")] if len(sys.argv) > 2
             else list(getattr(mod, "SPEED_LADDER", [0.45])))
    for rung in rungs:
        txt, _ = report(mod, rung, line_acc_for(rung), os.path.basename(path))
        print(txt)
        print()
