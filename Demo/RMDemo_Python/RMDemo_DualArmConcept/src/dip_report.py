#!/usr/bin/env python3
"""Where a recorded run's time went, and what `line_acc` can reach.

Reads runs/<id>/ and reports, using FIRST-ORDER quantities only — event
durations, joint speed, joint current. No differentiation, because the four
defensible estimators of joint acceleration disagree by 2.6x-16.6x on this
data (SPEED_INVESTIGATION.md §5).

    python3 dip_report.py ../runs/20260811T183500_toplid_left_left
    python3 dip_report.py ../runs/*_toplid_left_left        # compare a ladder

The number to watch across a `line_acc` ladder is `effective ramp accel`.
It should track line_acc. Today it sits at 1.58 against a commanded 2.4.
If it does not move, `line_acc` is inert for movel ramps and the dips are a
trajectory-connect problem instead (H35).
"""
import csv
import json
import math
import pathlib
import sys

DEAD_M_S = 0.010        # below this the arm is stopped, not ramping
JOINT_SPEED_LIMIT = [180, 180, 225, 225, 225, 225, 225]   # deg/s


def report(run):
    run = pathlib.Path(run)
    meta = json.loads((run / "run.json").read_text())
    rows = list(csv.DictReader((run / "stream.csv").open()))
    stage = next((s for s in meta["stages"]
                  if s["stage_name"] == "execute_path"), None)
    if stage is None:
        print(f"{run.name}: no execute_path"); return

    if meta.get("sim"):
        print(f"{run.name}: sim=True — joint speed reads ~0 in SIM, "
              f"nothing below is meaningful. Skipped.")
        return

    win = [r for r in rows
           if stage["t_start"] <= float(r["t_mono"]) <= stage["t_end"]]
    t = [float(r["t_mono"]) for r in win]
    tcp = [(float(r["tcp_x"]), float(r["tcp_y"]), float(r["tcp_z"]))
           for r in win]
    v = [(t[i], math.dist(tcp[i], tcp[i - 1]) / (t[i] - t[i - 1]))
         for i in range(1, len(t)) if 0.0075 <= t[i] - t[i - 1] <= 0.0125]

    lim = meta.get("limits_in_force") or {}
    cruise = meta["speed_achieved"]["typical_mm_s"] / 1000.0
    p95 = meta["speed_achieved"]["p95_mm_s"]
    half = cruise / 2

    print(f"\n{run.name}")
    print(f"  line_speed={lim.get('line_speed')}  "
          f"line_acc={lim.get('line_acc')}  "
          f"cruise={cruise*1000:.0f} mm/s  p95={p95:.0f} mm/s")

    # --- time budget -----------------------------------------------------
    T = sum(t[i] - t[i - 1] for i in range(1, len(t))
            if 0.0075 <= t[i] - t[i - 1] <= 0.0125)
    L = sum(s * (tt - t[t.index(tt) - 1]) for tt, s in v) if False else \
        sum(math.dist(tcp[i], tcp[i - 1]) for i in range(1, len(t))
            if 0.0075 <= t[i] - t[i - 1] <= 0.0125)
    for label, lo, hi in ((">90% cruise", 0.9, 9.0),
                          ("50-90%     ", 0.5, 0.9),
                          ("<50% (dips)", 0.0, 0.5)):
        tt = sum(0.010 for _, s in v if lo <= s / cruise < hi)
        ll = sum(s * 0.010 for _, s in v if lo <= s / cruise < hi)
        print(f"    {label}  {tt:5.1f} s ({100*tt/T:4.1f}% of time)   "
              f"{ll:5.3f} m ({100*ll/L:4.1f}% of path)")

    # --- dips: what line_acc can and cannot reach ------------------------
    dips, cur = [], None
    for tt, s in v:
        if s < half:
            if cur is None:
                cur = [tt, tt, []]
            cur[1], _ = tt, cur[2].append(s)
        elif cur:
            dips.append(cur); cur = None
    if cur:
        dips.append(cur)
    dips = [d for d in dips if d[1] - d[0] > 0.02]
    if dips:
        tot = sum(d[1] - d[0] for d in dips)
        dwell = sum(0.010 for d in dips for s in d[2] if s < DEAD_M_S)
        eff = 2 * half / ((tot - dwell) / len(dips))
        print(f"    {len(dips)} dips, {tot:.2f} s below half cruise")
        print(f"      dwell < {DEAD_M_S*1000:.0f} mm/s   {dwell:5.2f} s "
              f"({100*dwell/tot:3.0f}%)  command gap — line_acc cannot fix")
        print(f"      ramping             {tot-dwell:5.2f} s "
              f"({100*(tot-dwell)/tot:3.0f}%)  line_acc CAN fix")
        print(f"      >> effective ramp accel {eff:.2f} m/s^2   "
              f"(line_acc = {lim.get('line_acc')})")

    # --- first-order safety channels -------------------------------------
    peak = [max(abs(float(r[f"speed{j}"])) for r in win) for j in range(1, 8)]
    cur_a = [max(abs(float(r[f"current{j}"])) for r in win) / 1000.0
             for j in range(1, 8)]
    over = [f"J{j+1} {peak[j]:.0f}>{JOINT_SPEED_LIMIT[j]}"
            for j in range(7) if peak[j] > JOINT_SPEED_LIMIT[j]]
    print(f"    peak joint speed  {[round(x) for x in peak]} deg/s"
          f"   {'OVER LIMIT: ' + ', '.join(over) if over else 'all within limits'}")
    print(f"    peak joint current {[round(x, 1) for x in cur_a]} A"
          f"   (J4 baseline 7.3 A passing / 16.7 A the violent run)")
    print(f"    predicted J4 peak from p95: {0.368*p95:.0f} deg/s "
          f"(measured {peak[3]:.0f}, residual {peak[3]-0.368*p95:+.0f})")
    errs = sorted({(j, int(r[f"err{j}"])) for r in win for j in range(1, 8)
                   if int(r[f"err{j}"])})
    st = sorted({int(r["arm_status"]) for r in win})
    print(f"    joint error bits: {errs or 'none'}   arm_status seen: {st}"
          f"   {'*** RM_STOP_E ***' if 9 in st else ''}")


for a in sys.argv[1:]:
    report(a)
