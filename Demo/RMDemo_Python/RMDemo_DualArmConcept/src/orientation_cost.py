#!/usr/bin/env python3
"""Kinematic audit of a cleaning plan: what the joint-speed budget is spent on.

Offline. `rm_algo_*` is a pure library — no arm, no emulator, no network.

WHY. `20260811T222451` (toplid_right, line_speed 0.600) stopped dead mid-path
with joint 4 at 224.6 deg/s against a reported limit of 225, and reported
nothing on any surface we can read. Tuning `line_speed` / `line_acc` only
scales how fast that limit is reached — both saturate the same joint at the
same point on the path (H44, H46). So the question is what the joint speed is
being SPENT on, and which tasks are closest to the wall.

REPORTED PER TASK
  util      peak joint speed as a fraction of rm_get_joint_max_speed
            ([180,180,225,225,225,225,225] deg/s), and which joint, and where
            on the path. This is the number that decides whether a task can be
            run faster at all.
  ori       ORIENTATION COST. At each sample, the min-norm joint rates that
            reproduce the plan's full 6-DOF twist, divided by those that
            reproduce only its LINEAR part. How many times more joint speed
            the same tool TRANSLATION costs once the tool must also keep
            pointing the same way. Dimensionless, so it compares across tasks
            and is independent of the plan's time scaling.
  elbow     arm-angle (null-space) excursion over the stroke, unwrapped. The
            RM75 has one redundant DOF; nothing in the Cartesian limits bounds
            its rate.

USAGE
  python3 orientation_cost.py                       # the four local plans
  python3 orientation_cost.py --plans <dir>         # any directory of plans
  python3 orientation_cost.py --plans <dir> --stride 10
  python3 orientation_cost.py --plans <dir> --all   # include mechanism tasks

CAVEAT. The Jacobian's orientation rows are EULER RATES, not angular velocity,
so per-axis attribution (which of rx/ry/rz costs most) is parameterisation-
dependent and is NOT reported. The HELD-vs-FREE ratio compares "all
orientation constrained" against "none", which is well defined either way.
"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log_utils import wants_help  # noqa: E402

sys.path.insert(0, "/home/newtonjeri/realman_API/RM_API2/Python")
from Robotic_Arm.rm_robot_interface import (  # noqa: E402
    Algo, rm_robot_arm_model_e, rm_force_type_e)

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PLANS = os.path.join(HERE, "plans")
JOINT_LIMIT = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]

# Mechanism/utility motions, not surface cleaning. Excluded unless --all.
NOT_CLEANING = (
    "glove_", "lid_open", "lid_close", "seat_open", "seat_close",
    "flush_press", "dual_bin", "dual_arm_gate", "test_motion", "backup_",
    "move_to_", "_totg",
)

_algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,
             rm_force_type_e.RM_MODEL_RM_ISF_E)
_algo.handle = None


def fk(q):
    return _algo.rm_algo_forward_kinematics(list(q), flag=1)


def _wrap(d):
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def jacobian(q, eps=1e-4):
    """6x7 by central difference. Rows 0-2 m/deg, rows 3-5 rad/deg."""
    J = [[0.0] * 7 for _ in range(6)]
    for j in range(7):
        qp, qm = list(q), list(q)
        qp[j] += eps
        qm[j] -= eps
        p, m = fk(qp), fk(qm)
        for i in range(3):
            J[i][j] = (p[i] - m[i]) / (2 * eps)
        for i in range(3, 6):
            J[i][j] = _wrap(p[i] - m[i]) / (2 * eps)
    return J


def min_norm(J, target, n):
    """Least-2-norm qdot with J[:n] . qdot = target[:n]. None if singular."""
    A = [[sum(J[i][t] * J[j][t] for t in range(7)) for j in range(n)]
         for i in range(n)]
    M = [A[i][:] + [target[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-13:
            return None
        for r in range(n):
            if r != c:
                f = M[r][c] / M[c][c]
                for cc in range(c, n + 1):
                    M[r][cc] -= f * M[c][cc]
    lam = [M[i][n] / M[i][i] for i in range(n)]
    return [sum(J[i][j] * lam[i] for i in range(n)) for j in range(7)]


def unwrap(seq):
    out = [seq[0]]
    for x in seq[1:]:
        d = x - out[-1]
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        out.append(out[-1] + d)
    return out


def analyse(path, stride):
    name = os.path.basename(path).replace("_ruckig_pro_only.json", "") \
        .replace(".json", "")
    try:
        plan = json.load(open(path))
    except Exception as exc:
        return {"task": name, "error": str(exc)[:40]}
    sub = [s for s in plan.get("sub_trajectories", [])
           if s.get("stage_name") == "execute_path"]
    if not sub:
        return {"task": name, "error": "no execute_path"}
    full = sub[0]["waypoints"]
    dur = full[-1].get("time_from_start", 0.0)

    # ELBOW AT FULL RESOLUTION. Unwrapping is only valid while consecutive
    # samples differ by < 180 deg; at stride 20 some tasks step 176 deg
    # between samples, which makes the sign of the wrap a coin toss. The
    # arm-angle call is cheap (no Jacobian), so take every waypoint and
    # report the largest step so the caller can see the margin.
    AA = [_algo.rm_algo_calculate_arm_angle_from_config_rm75(
        [math.degrees(v) for v in w["positions"]])[1] for w in full]
    steps = [abs(AA[i + 1] - AA[i]) for i in range(len(AA) - 1)]
    worst_step = max((min(d, 360 - d) for d in steps), default=0.0)
    elbow = (lambda u: max(u) - min(u))(unwrap(AA))

    # JOINT UTILISATION AT FULL RESOLUTION. The plan carries `velocities`
    # directly, so this needs no Jacobian and no sampling — and sampling it
    # at stride 20 UNDER-READ the peak by up to 11 points (`hinge_area_left`
    # 88 % sampled against 95 % true, `seat_ring_bottom_left` 89 % against
    # 100 %). A peak is exactly the statistic a stride destroys.
    vpk = [max(abs(math.degrees(w["velocities"][j])) for w in full)
           for j in range(7)] if full[0].get("velocities") else [0.0] * 7
    util_full, joint_full = max((vpk[j] / JOINT_LIMIT[j], j + 1)
                                for j in range(7))

    wps = full[::stride]
    if len(wps) < 5:
        return {"task": name, "error": "too few waypoints"}
    P = [fk([math.degrees(v) for v in w["positions"]])[:3] for w in wps]
    cum, tot = [0.0], 0.0
    for i in range(1, len(P)):
        tot += math.dist(P[i], P[i - 1])
        cum.append(tot)

    rows = []
    for i, w in enumerate(wps):
        q = [math.degrees(v) for v in w["positions"]]
        qd = [math.degrees(v) for v in w.get("velocities") or [0] * 7]
        if max(abs(x) for x in qd) < 1e-6:
            continue
        J = jacobian(q)
        twist = [sum(J[r][c] * qd[c] for c in range(7)) for r in range(6)]
        held = min_norm(J, twist, 6)
        free = min_norm(J, twist[:3], 3)
        if not held or not free:
            continue
        mh, mf = max(abs(x) for x in held), max(abs(x) for x in free)
        if mf < 1e-6:
            continue
        rows.append({
            "frac": cum[i] / tot if tot else 0.0,
            "ori": mh / mf,
            "util": max(abs(qd[j]) / JOINT_LIMIT[j] for j in range(7)),
            "joint": 1 + max(range(7),
                             key=lambda j: abs(qd[j]) / JOINT_LIMIT[j]),
        })
    if not rows:
        return {"task": name, "error": "no usable samples"}

    ori = sorted(r["ori"] for r in rows)
    hot = max(rows, key=lambda r: r["util"])
    return {"task": name, "n": len(rows), "dur": dur, "path": tot,
            "elbow": elbow, "elbow_per_m": elbow / tot if tot else 0.0,
            "unwrap_step": worst_step,
            "ori_med": ori[len(ori) // 2], "ori_max": ori[-1],
            "util": util_full, "joint": joint_full, "at": hot["frac"],
            "util_sampled": hot["util"], "joint_sampled": hot["joint"],
            "ori_at_hot": hot["ori"]}


def discover(plans_dir, include_all):
    files = sorted(glob.glob(os.path.join(plans_dir, "*.json")))
    if include_all:
        return files
    return [f for f in files
            if not any(k in os.path.basename(f) for k in NOT_CLEANING)]


def main() -> int:
    if wants_help():
        print(__doc__)
        return 0
    argv = sys.argv[1:]
    plans_dir = DEFAULT_PLANS
    if "--plans" in argv:
        plans_dir = argv[argv.index("--plans") + 1]
    stride = 20
    if "--stride" in argv:
        stride = int(argv[argv.index("--stride") + 1])
    files = discover(plans_dir, "--all" in argv)
    if not files:
        print("no plans found in %s" % plans_dir)
        return 1

    print("Kinematic audit — %d plans from %s  (stride %d)"
          % (len(files), plans_dir, stride))
    print("util = peak joint speed / rm_get_joint_max_speed;  "
          "ori = orientation cost;  elbow = null-space excursion\n")
    out = [analyse(f, stride) for f in files]
    ok = [r for r in out if "error" not in r]
    ok.sort(key=lambda r: -r["util"])

    print("%-28s %7s %6s  %-12s %6s %6s %6s %7s"
          % ("task", "path m", "dur s", "peak util", "ori@pk", "ori med",
             "elbow", "elb/m"))
    print("-" * 92)
    for r in ok:
        flag = "  <-- " + ("OVER" if r["util"] >= 1.0 else
                           "at limit" if r["util"] >= 0.95 else
                           "tight") if r["util"] >= 0.90 else ""
        print("%-28s %7.2f %6.1f  %4.0f%% J%d@%2.0f%% %5.1fx %5.1fx %6.0f %6.0f%s"
              % (r["task"], r["path"], r["dur"], 100 * r["util"],
                 r["joint"], 100 * r["at"], r["ori_at_hot"], r["ori_med"],
                 r["elbow"], r["elbow_per_m"], flag))
    for r in out:
        if "error" in r:
            print("%-30s   %s" % (r["task"], r["error"]))

    if ok:
        print("\n%d tasks; %d at or above 90%% of a joint speed limit IN THE "
              "PLAN" % (len(ok), sum(1 for r in ok if r["util"] >= 0.90)))
        print("median orientation cost across tasks: %.1fx"
              % sorted(r["ori_med"] for r in ok)[len(ok) // 2])
        amb = [r["task"] for r in ok if r["unwrap_step"] >= 90]
        if amb:
            print("elbow unwrap AMBIGUOUS (>=90 deg between waypoints): %s"
                  % ", ".join(amb))
        print("binding joints: %s"
              % ", ".join("J%d x%d" % (j, sum(1 for r in ok
                                              if r["joint"] == j))
                          for j in sorted({r["joint"] for r in ok})))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
