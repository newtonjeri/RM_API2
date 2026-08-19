"""A.2.1 / A.2.2 — arc speed and junction speed-step feasibility, offline.

Contract: `COMMODE_C_CLEANING_CONTRACT.md` (frozen 2026-08-19, sha256
09c20c2a…f1e3), Appendix A.2, A.2.1, A.2.2. Where this module and the
contract disagree, THE CONTRACT WINS and this file is the defect.

WHY THIS MODULE EXISTS. The 2026-08-19 hardware ramp failed at v = 1.00 with
`Out Of Reach, reason: Joint4overspeed`, and C9 records that the code was on
the GUI only — `err1..err7` and `lift_err` were zero in all 23 recordings and
`arm_status` read IDLE straight through the event. Nothing in-band saw it.
A.2.2 is the contract's answer: the failure was a GEOMETRY defect, exact and
computable with no hardware at all. A.4's J4 screen could not have caught it
(it reads 10–15 % LOW, and it averages a 3-sample event across 1220 samples);
this check can, because it asks a question about one junction.

    A.2.1  v_arc  <= sqrt(a_lat * R)          R = (c²/4 + s²) / (2s)
    A.2.2  cut    >= |v_out² - v_in²| / (2a)
    A.2    cut     = c(r) * (r/100) * min(L_in, L_out)
           c(10) = 1.70   c(25) = 1.57   c(50) = 1.33     [M, 1454 corners]

`r` IS A PERCENTAGE 0–100, NOT MILLIMETRES (vendor: 交融半径百分比系数).

⚠ A.2.2 IS BINDING, AND THERE IS AN OPEN CALIBRATION DISPUTE (for Newton).
An earlier revision of this module printed A.2.2 as ADVISORY on the strength
of one peer result (rm-api2-b4). That was a unilateral suspension of a
frozen clause on an unratified finding — reversed 2026-08-19 (alix-ws-61's
argument): A.4's withdrawal handed the gating job to A.2.1 + A.2.2 ("the
screen ranks, geometry gates"), so with A.2.2 advisory NOTHING gated a
junction offline. A clause that over-flags costs speed; a clause that is
absent costs an arm. A.2.2 gates until Newton rules otherwise.

THE DISPUTE, recorded verbatim so the refusals below are not mistaken for a
clean bill: run over `planar_speed_ramp_001` on the commanded speeds, A.2.2
flags **19 of 19** junctions at rung 0.45 — a rung that completed on
hardware with J4 at 47.8 % and 0 ms of dwell — so THIS GATE REFUSES RUNGS
C2 RECORDS AS COMPLETED. It also ranks the one junction that DID fail
(`r01_in`, the 145.6 mm arc into a full-speed stroke) as the LEAST severe of
the nine arc→stroke entries at every rung, because its longer approach
raises `min(L_in, L_out)` and therefore the cut it is credited with:

    rung 0.90   r01_in   deficit 115.0 mm   implied  15.1 m/s²  ( 5.6× cmd)
                r02_in   deficit 129.4 mm   implied  35.5 m/s²  (13.1× cmd)
                r09_in   deficit 129.4 mm   implied  35.5 m/s²  (13.1× cmd)

A.7 records the opposite: `r01_in` is "the SOLE source of all J4 load above
51 %" and the eight 56.6 mm entries "never passed 51 %". Whether that
anti-correlation is real or an artifact of comparing WHOLE-PATH outcomes to
PER-JUNCTION verdicts (J4 passed 80 % only inside a 40 mm window at ONE
junction — a path statistic averages the event away, exactly how A.4's band
failed) is under independent verification (alix-ws-54, 2026-08-19). The
verdict goes to Newton; neither outcome is a code default's to settle. Do
not quietly re-tune this module either way — the contradiction is loud on
purpose. A.2.1 is undisputed: per-arc, needs no speed pairing, and nothing
in the corpus contradicts it.

THE RULE WHEN A JUNCTION IS INFEASIBLE AT EVERY r (A.2.2, verbatim): **the
LOWER SPEED binds — never a larger r, and never "run it anyway".** This
module therefore reports a speed to drop to; it never reports an r to raise.

A.2.1 AND A.2.2 ARE CHECKED TOGETHER OR NOT AT ALL — "neither alone is
sufficient". The contract's own fix for the ramp failure is compositional:
raise the arc to its own A.2.1 limit first, and the residual step is one
A.2.2 can afford.
"""

import math

# --- A.2 blend-cut law ------------------------------------------------------
# Measured domain is r ∈ {10, 25, 50} ONLY (72 runs, 2026-08-14, two
# geometries, both arms, both directions, 0.25 m/s baseline). Blend geometry
# is speed-independent, so this table transfers across rungs; it does NOT
# transfer past r = 50, which is why `max_cut` stops there.
_C_OF_R = ((10.0, 1.70), (25.0, 1.57), (50.0, 1.33))
R_MEASURED_MAX = 50.0


def c_of_r(r_pct):
    """Cut coefficient c(r). Declines with r — it is NOT the flat 1.4.

    The refuted flat-in-r form (0 / 0.70 / 1.4 by turn angle) was retired
    from `rm_emulator.py:_corner_model` in 2fbc9f4; the PREDICTED SIGNATURE
    headers of `paths/chain_semantics_00{1,3,4}.py` keep it as annotated
    history. At r = 10 — the radius the freeze rule mandates on dense
    geometry — the true 1.70 is ABOVE the old band, so the old form
    UNDERSTATED the cut exactly where this project operates.
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

    The contract's worked example evaluates the ceiling at r = 50, not at
    r = 100, and so does this. c(r) past 50 has never been measured, and the
    freeze rule forbids large r on dense geometry anyway — an infeasibility
    verdict must not be talked away with an extrapolated coefficient.
    """
    return blend_cut(min(r_limit, R_MEASURED_MAX), l_in, l_out)


# --- A.2.1 arc speed --------------------------------------------------------
def arc_radius_chord_sagitta(chord, sagitta):
    """R from chord c and sagitta s — the contract's stated form [m]."""
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


# --- A.2.2 junction speed step ---------------------------------------------
def required_cut(v_in, v_out, a):
    """Blend length the commanded speed CHANGE needs [m]."""
    if a <= 0.0:
        return float("inf")
    return abs(v_out * v_out - v_in * v_in) / (2.0 * a)


def feasible_speed(v_other, cut, a):
    """Largest speed reachable across `cut` from `v_other` [m/s].

    This is the "LOWER SPEED binds" resolution of A.2.2, solved for the speed
    rather than the cut: v² = v_other² ± 2·a·cut.
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
    """Full A.2.1 + A.2.2 audit of a path module at one rung.

    Returns (arcs, junctions, worst) — `worst` is the lowest speed any clause
    binds the path to, or None when everything clears. A.2.1 and A.2.2 are
    evaluated together, never separately.
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
        # A.2.2's resolution: the LOWER SPEED binds. Never a larger r.
        v_bind = feasible_speed(min(v_in, v_out), cut, line_acc)
        rec = {"i": i, "vertex": b_in, "r": r, "l_in": l_in, "l_out": l_out,
               "v_in": v_in, "v_out": v_out, "cut": cut, "need": need,
               "ceiling": ceiling, "v_bind": v_bind,
               "ok": cut + 1e-12 >= need,
               "infeasible_any_r": ceiling + 1e-12 < need}
        # The compositional fix (A.2.1 + A.2.2 together): if the approach is
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
    L.append("A.2.1/A.2.2 JUNCTION AUDIT  %s  rung %.2f m/s  line_acc %.2f m/s^2"
             % (name, rung, line_acc))
    bad_a = [a for a in arcs if not a["ok"]]
    L.append("  A.2.1 arcs: %d checked, %d over the lateral-acc limit"
             % (len(arcs), len(bad_a)))
    for a in bad_a:
        L.append("    OVER  %-22s R %5.1f mm  v %.3f > sqrt(a*R) = %.3f m/s"
                 % (a["at"], a["R"] * 1000, a["v"], a["v_max"]))
    bad_j = [j for j in junctions if not j["ok"]]
    L.append("  A.2.2 junctions: %d checked, %d short of the cut they need"
             % (len(junctions), len(bad_j)))
    for j in bad_j:
        tag = "INFEASIBLE AT EVERY r" if j["infeasible_any_r"] else "short at this r"
        L.append("    %-21s %-10s r=%2.0f  L_in %6.1f  L_out %6.1f mm"
                 % (j["vertex"], tag, j["r"], j["l_in"] * 1000, j["l_out"] * 1000))
        L.append("        v %.3f -> %.3f needs %5.1f mm, r=%2.0f delivers %5.1f mm, "
                 "r=50 ceiling %5.1f mm"
                 % (j["v_in"], j["v_out"], j["need"] * 1000, j["r"],
                    j["cut"] * 1000, j["ceiling"] * 1000))
        L.append("        LOWER SPEED BINDS -> v_out <= %.3f m/s (never a larger r)"
                 % j["v_bind"])
        if "v_in_lifted" in j:
            L.append("        A.2.1 composition: lift the approach arc to %.3f m/s "
                     "-> step needs %5.1f mm -> %s"
                     % (j["v_in_lifted"], j["need_lifted"] * 1000,
                        "CLEARS" if j["lifted_ok"] else "still short"))
    a_bind = min([a["v_max"] for a in arcs if not a["ok"]], default=None)
    if a_bind is None:
        L.append("  VERDICT (A.2.1, BINDING): arcs clear this rung.")
    else:
        L.append("  VERDICT (A.2.1, BINDING): arcs bind this rung to %.3f m/s."
                 % a_bind)
    j_bind = min([j["v_bind"] for j in junctions if not j["ok"]], default=None)
    if j_bind is None:
        L.append("  VERDICT (A.2.2, BINDING): junctions clear this rung.")
    else:
        L.append("  VERDICT (A.2.2, BINDING): %d/%d junctions short on "
                 "commanded speeds; the" % (len(bad_j), len(junctions)))
        L.append("      LOWER SPEED binds this rung to %.3f m/s." % j_bind)
        L.append("      ⚠ OPEN DISPUTE (for Newton — this module's docstring): "
                 "run verbatim, A.2.2")
        L.append("      refuses rungs C2 records as completed on hardware. It "
                 "gates anyway: a frozen")
        L.append("      clause is not a peer result's to suspend, and with it "
                 "advisory nothing gates")
        L.append("      a junction offline. Loud on purpose.")
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
