#!/usr/bin/env python3
"""Predict a cleaning task's REAL joint utilisation from its saved plan.

Offline. Reads plans only — no arm, no controller, no emulator.

WHY THIS AND NOT THE EMULATOR. Measured 2026-08-12 over all 24 commode_c
cleaning plans:

    predictor   binding joint vs REAL     rank correlation vs plan
    plan               13/13                       —
    SIM                 9/9                    (needs the controller)
    emulator            6/24                      +0.09

The emulator resolves the RM75's redundant DOF with
`rm_algo_inverse_kinematics`, whose choice is its own: on `toplid_left` it
walks J5 through 1127 deg where the plan uses 113 (10x), while J4 — which
the redundancy cannot reach — matches the plan to 1 %. Constraining the
search to the minimum-motion arm angle only narrows 10x to 5.6x. So joints
in the null space are not predictable without the controller's own
resolution scheme, and the emulator must not be used to screen these tasks.

THE MODEL. Over 13 REAL recordings the plan named the binding joint every
time, and the ratio of measured to planned utilisation moves with the
commanded Cartesian limits:

    line_speed / line_acc      REAL d(pos)/dt / PLAN
       0.45 / 1.6                    0.92, 0.93
       0.45 / 2.4                    0.96, 0.99, 1.05
       0.45 / 3.6                    1.06, 1.11
       0.50 / 3.6                    1.18
       0.80 / 2.4                    1.42   (this run STOPPED silently)

so `k` is interpolated from those and applied to the plan's utilisation.
Ratios below 0.5 come from the pendant-override ladder (38-68 % of cap) and
are excluded from the fit — that slider is unreadable by the SDK.

    python3 predict_task.py                 # all tasks at 0.45 / 3.6
    python3 predict_task.py --ls 0.45 --la 1.6
    python3 predict_task.py --all           # include mechanism motions
"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log_utils import wants_help  # noqa: E402

LIM = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]
# THE BUNDLED PLANS, and only those — the same rule `segment_verifier.
# resolve_plan` enforces for the runner. Reading the ROS workspace directly
# is what produced the 2026-08-08 failure where two machines verified
# different plans under the same filename. A task without a bundled plan is
# not screened here; bundle it first, then it is screened and runnable by
# `stage_runner` in the same move.
PLANDIRS = [
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "plans"),
]
NOT_CLEANING = ("glove_", "lid_open", "lid_close", "seat_open", "seat_close",
                "flush_press", "dual_bin", "dual_arm_gate", "test_motion",
                "backup_", "move_to_", "_totg")

# (line_speed, line_acc) -> measured REAL/PLAN utilisation ratio, from the
# 9 REAL runs whose limits were recorded. Each entry is the mean of its
# repeats; the spread within a setting is <= 0.09.
CALIB = {
    (0.45, 1.6): 0.925,
    (0.45, 2.4): 1.000,
    (0.45, 3.6): 1.085,
    (0.50, 3.6): 1.180,
    (0.80, 2.4): 1.420,
}


def k_for(ls, la):
    """Nearest-neighbour in (line_speed, line_acc), with the distance
    reported so an extrapolation is visible rather than silent."""
    best, bd = None, None
    for (s, a), v in CALIB.items():
        d = math.hypot((s - ls) / 0.1, (a - la) / 1.0)
        if bd is None or d < bd:
            best, bd = v, d
    return best, bd


def plan_util(path):
    plan = json.load(open(path))
    sub = [s for s in plan.get("sub_trajectories", [])
           if s.get("stage_name") == "execute_path"]
    if not sub:
        return None
    W = sub[0]["waypoints"]
    if not W or "velocities" not in W[0]:
        return None
    pk = [max(abs(math.degrees(w["velocities"][j])) for w in W)
          for j in range(7)]
    u, j = max((pk[j] / LIM[j], j + 1) for j in range(7))
    return {"util": u, "joint": j, "dur": W[-1].get("time_from_start", 0.0),
            "peaks": pk}


def main() -> int:
    if wants_help():
        print(__doc__)
        return 0
    argv = sys.argv[1:]
    ls = float(argv[argv.index("--ls") + 1]) if "--ls" in argv else 0.45
    la = float(argv[argv.index("--la") + 1]) if "--la" in argv else 3.6
    k, dist = k_for(ls, la)

    seen, rows = set(), []
    for d in PLANDIRS:
        for f in sorted(glob.glob(os.path.join(d, "*.json"))):
            name = os.path.basename(f).replace("_ruckig_pro_only.json", "") \
                .replace(".json", "")
            if name in seen:
                continue
            if "--all" not in argv and any(x in name for x in NOT_CLEANING):
                continue
            r = plan_util(f)
            if r is None:
                continue
            seen.add(name)
            r["task"] = name
            rows.append(r)

    rows.sort(key=lambda r: -r["util"])
    print("Predicted REAL joint utilisation at line_speed %.2f, line_acc %.1f"
          % (ls, la))
    print("k = %.3f from the calibration table%s\n"
          % (k, "" if dist < 0.6 else
             "  [WARN] EXTRAPOLATED — nearest calibration point is far"))
    print("%-30s %7s %8s %9s   %s"
          % ("task", "plan", "predicted", "binding", "verdict"))
    print("-" * 76)
    over = tight = 0
    for r in rows:
        p = r["util"] * k
        if p >= 1.0:
            v, over = "OVER LIMIT", over + 1
        elif p >= 0.95:
            v, tight = "at limit", tight + 1
        elif p >= 0.90:
            v = "tight"
        else:
            v = ""
        print("%-30s %6.0f%% %8.0f%% %9s   %s"
              % (r["task"], 100 * r["util"], 100 * p, "J%d" % r["joint"], v))
    print("\n%d tasks: %d PREDICTED OVER the joint speed limit, %d at it "
          "(>=95 %%)." % (len(rows), over, tight))
    print("A task predicted over the limit is the condition under which "
          "20260811T222451\nstopped mid-path and reported nothing (H45). "
          "Do not run it at these settings.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
