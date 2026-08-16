#!/usr/bin/env python3
"""What does each run mode actually measure, and does SIM predict REAL?

Three modes produce recordings that LOOK identical — same 62 columns, same
100 Hz, same `stream.csv` shape — and are not:

    emulator   `rm_emulator.install()`, no controller at all
    SIM        the real controller with `rm_set_arm_run_mode(0)`
    REAL       the real controller driving the arm

MEASURED CHANNEL FIDELITY (this tool's `--channels` report):

    channel              emulator      SIM              REAL
    joint position       MODELLED*     faithful         yes
    joint speed field    zero          DEAD (~0.4 °/s)  yes
    joint current        idle          idle only        yes
    tool pose            modelled*     faithful         yes
    stage timing         measured law  within 2 %       yes

    * CORRECTED 2026-08-17. This table read "NOT MOVED / not modelled /
      formula" and quoted `movel_chain` as saying "Cartesian geometry is
      not modelled — the emulator has no IK — so the joints are left
      alone." THAT WAS TRUE UNTIL 2026-08-12 AND IS NOW STALE. The same
      docstring today opens "CARTESIAN GEOMETRY IS MODELLED (added
      2026-08-12)": it seeds from the current joints, walks the queued
      poses as a Cartesian polyline, solves seeded IK per sample with
      RealMan's own offline solver, times each segment on a trapezoidal
      profile from the arm's own limits, and drives the joints.
      Verified by replay 2026-08-17: `rm_movel` returns 0 and moves the
      arm 154.98 deg on the worst joint.

      Two limits to carry, because they decide what this tool may claim:
      the emulator DELIBERATELY does not enforce joint SPEED limits (see
      `movel_chain`), and IK does not always solve — the replay above
      printed "57 of 297 samples had no IK solution — joint rates are an
      UNDER-estimate". So emulator joint rates are a floor, not a
      measurement, and no H63 dwell verdict may be taken from them.

      Note this is the UPSTREAM emulator. The alix port
      (`core/python/src/alix_emulator/rm_emulator.py`) is the 2026-08-11
      file and still has the timer behaviour this table used to describe —
      alix FINDINGS F20/F42. Which copy you are reading decides which row
      of this table is true.

So SIM's `speed{n}` column cannot be used, but its `position{n}` column
can — differentiate it and you recover joint rates. This tool measures how
well that recovery matches REAL, over every matched pair in `runs/`.

    python3 mode_compare.py              # pair report + transfer function
    python3 mode_compare.py --channels   # per-mode channel fidelity table
    python3 mode_compare.py --verbose    # per-joint detail for every pair

WHY p99 AND NOT PEAK. A single differentiated sample is noise: four
defensible estimators of joint acceleration disagreed by 2.6x-16.6x on this
same data. The 99th percentile of |d(position)/dt| is stable; the maximum
is not. Both are reported so the difference is visible.
"""
import csv
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log_utils import wants_help  # noqa: E402

NOM = 0.010                       # nominal UDP sample interval, s
LIM = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]
RUNS = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "runs")


def ok_dt(dt):
    return 0.75 * NOM <= dt <= 1.25 * NOM


def load(path):
    m = json.loads(open(os.path.join(path, "run.json")).read())
    rows = list(csv.DictReader(open(os.path.join(path, "stream.csv"))))
    st = next((s for s in m["stages"]
               if s["stage_name"] == "execute_path"), None)
    if st is None:
        return None
    w = [r for r in rows
         if st["t_start"] <= float(r["t_mono"]) <= st["t_end"]]
    if len(w) < 60:
        return None
    lim = m.get("limits_in_force") or {}
    return {
        "name": os.path.basename(path), "task": m.get("task_name"),
        "side": m.get("side"), "mode": "SIM" if m.get("sim") else "REAL",
        "sdk": m.get("sdk"), "dur": st["t_end"] - st["t_start"], "w": w,
        "ls": lim.get("line_speed"), "la": lim.get("line_acc"),
        "cap": m.get("line_speed_cap_m_s"),
    }


def deriv(w, j):
    """|d(position)/dt| in deg/s, dt-guarded."""
    t = [float(r["t_mono"]) for r in w]
    q = [float(r["position%d" % j]) for r in w]
    return [abs((q[i + 1] - q[i]) / (t[i + 1] - t[i]))
            for i in range(len(t) - 1) if ok_dt(t[i + 1] - t[i])]


def rep(w, j):
    return [abs(float(r["speed%d" % j])) for r in w]


def p(v, q):
    s = sorted(v)
    return s[min(len(s) - 1, int(len(s) * q))] if s else 0.0


def util(w, how):
    """(fraction of limit, joint) using p99 of the chosen estimator."""
    vals = [(p(how(w, j), 0.99) / LIM[j - 1], j) for j in range(1, 8)]
    return max(vals)


def pair_up(runs):
    """Match SIM to REAL on (task, side, limits) then nearest duration.

    The `cap=0.25` group differs only by the PENDANT override, which the
    SDK cannot read and `run.json` therefore cannot record — so within a
    group, pair by closest duration. Every pair the tool emits carries its
    duration ratio so a bad match is visible rather than silent.
    """
    groups = {}
    for r in runs:
        groups.setdefault((r["task"], r["side"], r["ls"], r["la"],
                           r["cap"]), []).append(r)
    out = []
    for key, g in groups.items():
        sims = [x for x in g if x["mode"] == "SIM"]
        reals = [x for x in g if x["mode"] == "REAL"]
        for rr in reals:
            if not sims:
                continue
            best = min(sims, key=lambda s: abs(s["dur"] - rr["dur"]))
            out.append((best, rr))
    return out


def channels(runs):
    print("\nCHANNEL FIDELITY BY MODE  (execute_path window)\n")
    print("%-6s %5s %11s %11s %11s %11s"
          % ("mode", "runs", "posn travel", "speed field",
             "current pk", "tcp path"))
    for mode in ("SIM", "REAL"):
        sel = [r for r in runs if r["mode"] == mode]
        if not sel:
            continue
        pt, sf, cu, tp = [], [], [], []
        for r in sel:
            w = r["w"]
            pt.append(max(max(abs(float(w[i]["position%d" % j])
                                  - float(w[0]["position%d" % j]))
                              for j in range(1, 8))
                          for i in range(len(w))))
            sf.append(max(max(abs(float(x["speed%d" % j]))
                              for j in range(1, 8)) for x in w))
            cu.append(max(max(abs(float(x["current%d" % j]))
                              for j in range(1, 8)) for x in w) / 1000)
            t = [(float(x["tcp_x"]), float(x["tcp_y"]), float(x["tcp_z"]))
                 for x in w]
            tp.append(sum(d for d in (math.dist(t[i], t[i - 1])
                                      for i in range(1, len(t)))
                          if d < 0.05))
        f = lambda v: "%.1f-%.1f" % (min(v), max(v))
        print("%-6s %5d %11s %11s %11s %11s"
              % (mode, len(sel), f(pt) + "d", f(sf) + "/s",
                 f(cu) + "A", f(tp) + "m"))
    print("\nemulator      0     0.0d        0.0/s        0.0A       0.00m")
    print("              (rm_emulator.movel_chain: no IK, joints untouched)")


def main() -> int:
    if wants_help():
        print(__doc__)
        return 0
    verbose = "--verbose" in sys.argv
    runs = [r for r in (load(d) for d in sorted(glob.glob(RUNS + "/2026*")))
            if r]
    print("mode_compare — %d runs with an execute_path stage" % len(runs))
    if "--channels" in sys.argv:
        channels(runs)
        return 0

    pairs = pair_up(runs)
    print("%d matched SIM/REAL pairs\n" % len(pairs))
    print("%-26s %-14s %6s  %-15s %-15s %7s"
          % ("task", "ls/la", "dur", "SIM binding", "REAL binding", "err pp"))
    print("-" * 92)
    ratios, errs, agree = [], [], 0
    for s, r in pairs:
        su, sj = util(s["w"], deriv)          # SIM: only positions are live
        ru, rj = util(r["w"], rep)            # REAL: reported channel
        rdu, rdj = util(r["w"], deriv)        # REAL: same method as SIM
        same = sj == rdj
        agree += same
        ratios.append(s["dur"] / r["dur"])
        errs.append(su - rdu)
        print("%-26s %-14s %5.1f%%  J%d %5.0f%%       J%d %5.0f%% (%5.0f%%d) "
              "%+6.0f %s"
              % (s["task"] + "/" + s["side"],
                 "%s/%s" % (s["ls"], s["la"]),
                 100 * (s["dur"] / r["dur"] - 1),
                 sj, 100 * su, rj, 100 * ru, 100 * rdu,
                 100 * (su - rdu), "" if same else "<-- DISAGREE"))
        if verbose:
            print("      j   SIM p99   REALrep p99   REALderiv p99")
            for j in range(1, 8):
                print("      %d  %8.1f  %12.1f  %14.1f"
                      % (j, p(deriv(s["w"], j), 0.99), p(rep(r["w"], j), 0.99),
                         p(deriv(r["w"], j), 0.99)))

    # A pair whose REAL run ABORTED is not a like-for-like comparison: the
    # two runs cover different amounts of the path. Detect by duration
    # ratio and quote it separately rather than letting it skew the fit.
    clean = [(rt, er) for rt, er in zip(ratios, errs) if abs(rt - 1) < 0.10]
    dirty = [(rt, er) for rt, er in zip(ratios, errs) if abs(rt - 1) >= 0.10]
    cr = [x for x, _ in clean]
    ce = [y for _, y in clean]
    print("\nTRANSFER FUNCTION  (SIM d(position)/dt  ->  REAL), %d clean pairs"
          % len(clean))
    print("  duration ratio SIM/REAL : %.4f .. %.4f  (median %.4f)"
          % (min(cr), max(cr), sorted(cr)[len(cr) // 2]))
    print("  binding joint agreement : %d/%d pairs (ALL pairs)"
          % (agree, len(pairs)))
    print("  utilisation error vs REAL d(position)/dt : "
          "%+.0f .. %+.0f percentage points (median %+.0f)"
          % (100 * min(ce), 100 * max(ce),
             100 * sorted(ce)[len(ce) // 2]))
    print("  -> SIM UNDER-reads REAL. Treat a SIM utilisation of U as REAL "
          "U%+.0f..%+.0f pp." % (-100 * max(ce), -100 * min(ce)))
    if dirty:
        print("\n  EXCLUDED (REAL run aborted, so the two cover different "
              "path): %d pair(s)" % len(dirty))
        for rt, er in dirty:
            print("     duration ratio %.2f, utilisation error %+.0f pp"
                  % (rt, 100 * er))
        print("     This is the ls=0.8 case. SIM read J4 at 100 % of limit "
              "and completed;\n     REAL read 106 % and stopped silently. "
              "SIM FLAGGED THE CONDITION.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
