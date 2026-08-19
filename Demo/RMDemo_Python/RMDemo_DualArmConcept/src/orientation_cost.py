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

SEGMENT MODE — screen the CARTESIAN waypoints a program dispatches, and
search for a tilt that unloads the elbow. Offline; no arm, no motion.

  python3 orientation_cost.py --segments ../paths/test_motion_001.py \
          --tool L_glove_4 --speed 0.60
  python3 orientation_cost.py --segments ../runs/<run>/run.json --speed 0.45
  ... --sweep <point label> [--axis 0|1|2]          # 0=rx 1=ry 2=rz

Reads a `paths/*.py` program (parsed with `ast`, never imported) or a
recorded `run.json`, and follows the TRAVERSAL, not the declaration order.
Reports per segment: TCP length, tool rotation, the angular rate it implies,
the speed left after the 0.60 rad/s cap time-scales it, and peak J4 demand.

  `--sweep` moves one point's tilt across a range and re-scores the WHOLE
  traversal each time, because a raster visits a point more than once and
  because tilt taken off one segment lands on its neighbour.

VALIDATED: on `test_motion_001` the sweep independently recovers ry = -0.400
as the optimum — the value derived by hand and confirmed on hardware — and
puts the superseded -0.218 at ~148 % of the J4 limit against the 146 %
computed there by a different route.

⚠ ABSOLUTE VALUES OVER-READ. Against `20260811T220617` (J4 peak 215 deg/s
measured) this mode predicts 282 — about 1.3x high, because it assumes each
segment holds `v_eff` throughout and ignores corner blending. **Use it to
RANK segments and to compare tilts, not as a deg/s prediction.**

⚠ J4 ONLY, so a pass is not a clearance. J1 binds on 11 of 24 tasks
(TASK_KINEMATICS H49) and this mode cannot see it. Screen with the
plan-based audit above, `predict_task.py`, and a SIM run before REAL motion.

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

# THE SDK, RELATIVE TO THIS FILE. A hardcoded home directory here
# broke the speed-ramp screen on the lab machine with
# "ModuleNotFoundError: No module named 'Robotic_Arm'" — and because
# `run_speed_ramp.screen()` runs this as a SUBPROCESS, the failure
# surfaced as "SCREEN: unavailable" rather than as a path problem.
# `parents[4]` is RM_API2; every other module in this tree resolves it
# the same way (dual_arm_common.py:44, rm_emulator.py:86).
import pathlib  # noqa: E402
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))
from Robotic_Arm.rm_robot_interface import (  # noqa: E402
    Algo, rm_robot_arm_model_e, rm_force_type_e)
from Robotic_Arm.rm_ctypes_wrap import (  # noqa: E402
    rm_inverse_kinematics_params_t)

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PLANS = os.path.join(HERE, "plans")
JOINT_LIMIT = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]

# Mechanism/utility motions, not surface cleaning. Excluded unless --all.
NOT_CLEANING = (
    "glove_", "lid_open", "lid_close", "seat_open", "seat_close",
    "flush_press", "dual_bin", "dual_arm_gate", "test_motion", "backup_",
    "move_to_", "_totg",
    # `deep_left` / `deep_right` reach far enough that the offline solver
    # cannot follow them (deep_right solves no segment at all), so any
    # number this tool produced for them would be noise. Excluded from the
    # screen until that is understood — Newton, 2026-08-13.
    "deep_",
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


# ── SEGMENT MODE: Cartesian waypoints, not a plan ───────────────────────
#
# The plan-based audit above needs no IK and no frames — a plan carries joint
# positions, so FK is enough. This mode works the other way, from the
# CARTESIAN poses a program actually dispatches, and that requires getting
# two transforms right. Both are VERIFIED, not assumed (see `--segments`
# self-check): FK(Arm_Tip) + tool offset, rotated by the +90 deg mount,
# reproduces `runs/20260811T220617/run.json` commanded poses to 4 decimals
# on all three axes.
#
# WHY ONLY J4 IS REPORTED. On an S-R-S 7-DOF arm the elbow angle is a
# function of the shoulder-to-wrist distance alone, which the commanded pose
# fixes — so J4 is REDUNDANCY-INVARIANT and is the one joint predictable
# from a Cartesian pose without knowing how the controller resolves the
# redundant DOF (H66; measured 0.0 % spread across the arm-angle sweep).
# Every other joint moves with the redundancy — J1/J3 by 62 % — so this mode
# is a SOUND but INCOMPLETE screen: what it flags is real, what it passes is
# not thereby safe. J1 binds on 11 of 24 tasks (TASK_KINEMATICS H49), and for
# those the plan-based audit above is the authority.

# Controller tool frames, offset from Arm_Tip in metres, zero rotation
# (FRAME_MAP.md; Arm_Tip -> ConnectorLink is a further 15.3 mm on Z).
TOOL_OFFSETS = {
    "L_glove_1": (-0.050, 0.000, 0.1553), "R_glove_1": (0.050, 0.000, 0.1603),
    "L_glove_2": (-0.020, 0.000, 0.1803), "R_glove_2": (0.0135, 0.000, 0.1803),
    "L_glove_3": (-0.075, 0.007, 0.1853), "R_glove_3": (0.075, 0.007, 0.1853),
    "L_glove_4": (-0.055, 0.007, 0.2203), "R_glove_4": (0.055, 0.007, 0.2203),
    "L_tip": (-0.015, 0.005, 0.2453), "R_tip": (0.015, 0.005, 0.2453),
}
MOUNT_RY_DEG = 90.0          # both arms; controller World = base rotated Ry(+90)
ANGULAR_CAP = 0.600          # rad/s, rm_set_arm_max_angular_speed default
MAX_STEP_DEG = 8.0           # per IK sample; larger is a solver branch flip


def _Rmat(rx, ry, rz):
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return [[cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx]]


def _Reul(R):
    sy = max(-1.0, min(1.0, -R[2][0]))
    return [math.atan2(R[2][1], R[2][2]), math.asin(sy),
            math.atan2(R[1][0], R[0][0])]


def _matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def _R_to_q(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return [0.25 * s, (R[2][1] - R[1][2]) / s,
                (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s]
    i = max(range(3), key=lambda k: R[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i][i] - R[j][j] - R[k][k]) * 2
    q = [0.0] * 4
    q[0] = (R[k][j] - R[j][k]) / s
    q[i + 1] = 0.25 * s
    q[j + 1] = (R[j][i] + R[i][j]) / s
    q[k + 1] = (R[k][i] + R[i][k]) / s
    return q


def _q_to_R(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def _slerp(qa, qb, t):
    """Shortest-arc orientation interpolation.

    EULER ANGLES MUST NOT BE INTERPOLATED COMPONENT-WISE. The right arm's
    tool sits at rz ~ 179.8 deg — on the wrap boundary — so a segment
    crossing it makes linear Euler interpolation sweep ~359 deg the long way
    round instead of ~0.2 deg the short way. Every intermediate IK target is
    then a pose the arm never visits, and the invented joint motion read as
    742 % of the J4 limit on a path that runs clean. `rm_emulator` carries
    quaternions through `_plan_cartesian` for exactly this reason.
    """
    d = sum(a * b for a, b in zip(qa, qb))
    if d < 0:
        qb, d = [-x for x in qb], -d
    if d > 0.9995:
        out = [a + (b - a) * t for a, b in zip(qa, qb)]
    else:
        th = math.acos(max(-1.0, min(1.0, d)))
        s = math.sin(th)
        wa, wb = math.sin((1 - t) * th) / s, math.sin(t * th) / s
        out = [a * wa + b * wb for a, b in zip(qa, qb)]
    n = math.sqrt(sum(x * x for x in out)) or 1.0
    return [x / n for x in out]


def world_tcp_to_base_flange(pose, tool):
    """Commanded pose (World frame, TCP) -> Arm_Tip pose in the base frame.

    Two corrections, in this order: undo the +90 deg mount, then back off
    the tool offset along the tool's own axes. Skipping the second is the
    error that reported 400-550 % of the J4 limit on segments that had
    completed on hardware — a 180 mm offset on a 200 mm segment is not a
    small correction.
    """
    Rm = _Ry(-math.radians(MOUNT_RY_DEG))
    p_b = [sum(Rm[i][k] * pose[k] for k in range(3)) for i in range(3)]
    R_b = _matmul(Rm, _Rmat(*pose[3:6]))
    t = TOOL_OFFSETS[tool]
    tip = [p_b[i] - sum(R_b[i][k] * t[k] for k in range(3)) for i in range(3)]
    return tip + _Reul(R_b)


def ik(target, seed):
    ret, q = _algo.rm_algo_inverse_kinematics(
        rm_inverse_kinematics_params_t(q_in=list(seed), q_pose=list(target),
                                       flag=1))
    return list(q) if ret == 0 else None


def rot_between(pa, pb):
    """Tool rotation across a segment, degrees."""
    d = [(pb[3 + i] - pa[3 + i] + math.pi) % (2 * math.pi) - math.pi
         for i in range(3)]
    return math.degrees(math.sqrt(sum(x * x for x in d)))


def j4_peak_per_m(wa, wb, tool, seed, step_m=0.002):
    """Peak |dJ4|/ds along a `movel` segment, per metre of TCP travel.

    `wa`/`wb` are the COMMANDED poses (World frame, TCP). Interpolation
    happens HERE, in TCP space, and each sample is then converted to a
    flange pose for IK.

    IT MUST BE THIS WAY ROUND. `rm_movel` drives the TOOL along a straight
    line; the flange does not travel a straight line whenever the segment
    also rotates, because the ~200 mm tool offset swings. Interpolating
    between the two flange endpoints instead traces a different path
    entirely and invents joint motion that never happens — it reported
    351 % of the J4 limit on a segment that runs clean on hardware.

    PEAK, not average: measured peak/average runs 1.7-2.2x, and it is the
    peak that breaks a limit. Returns (peak, end_joints) or (None, seed).
    """
    l_tcp = math.dist(wa[:3], wb[:3])
    if l_tcp < 1e-9:
        return 0.0, seed
    n = max(12, int(l_tcp / step_m))
    q = ik(world_tcp_to_base_flange(wa, tool), seed)
    if q is None:
        return None, seed
    qa, qb = _R_to_q(_Rmat(*wa[3:6])), _R_to_q(_Rmat(*wb[3:6]))
    peak, prev, ds, miss = 0.0, q, l_tcp / n, 0
    for k in range(1, n + 1):
        f = k / n
        # position linear, orientation SLERPed — see `_slerp`
        w = [wa[j] + (wb[j] - wa[j]) * f for j in range(3)] + \
            _Reul(_q_to_R(_slerp(qa, qb, f)))
        qn = ik(world_tcp_to_base_flange(w, tool), prev)
        if qn is None:
            # Carry the last good solution, as `rm_emulator._plan_cartesian`
            # does and for the same reason: the offline solver fails on
            # interpolated points the controller executes, so abandoning the
            # segment would refuse motion the arm performs. The peak is then
            # an UNDER-estimate — reported, not hidden.
            miss += 1
            continue
        if max(abs(qn[j] - prev[j]) for j in range(7)) <= MAX_STEP_DEG:
            peak = max(peak, abs(qn[3] - prev[3]) / ds)
        prev = qn
    if miss > 0.25 * n:
        return None, prev            # too sparse to mean anything
    return (peak, miss / n), prev


def segment_report(poses, tool, speed, cap=ANGULAR_CAP):
    """Per-segment geometry, throttling and J4 demand for a pose list."""
    rows, seed = [], [0, 0, 0, 90, 0, 0, 0]
    for i in range(len(poses) - 1):
        # Everything is measured on the TCP, in the commanded frame: that is
        # what `rm_movel` drives in a straight line and what
        # `rm_set_arm_max_line_speed` caps. The flange appears only inside
        # `j4_peak_per_m`, as the IK target for each interpolated sample.
        L = math.dist(poses[i][:3], poses[i + 1][:3])
        if L < 1e-6:
            continue
        rot = rot_between(poses[i], poses[i + 1])
        # The controller time-scales a segment whose tool rotation would
        # exceed the angular cap (H67). A segment UNDER the cap is not
        # slowed — which is why the smoothest segments are the dangerous
        # ones: nothing throttles them on the way to a joint limit.
        w = math.radians(rot) / (L / speed) if L > 0 else 0.0
        v_eff = speed * min(1.0, cap / w) if w > 1e-9 else speed
        res, seed = j4_peak_per_m(poses[i], poses[i + 1], tool, seed)
        pk, gap = (None, 1.0) if res is None else res
        rows.append({"i": i, "len": L, "rot": rot, "w": w, "v": v_eff,
                     "j4_per_m": pk, "gap": gap,
                     "j4": None if pk is None else pk * v_eff,
                     "throttled": w > cap})
    return rows


def sweep_tilt(poses, labels, label, tool, speed, axis=1, span=0.60, steps=25):
    """Sweep one POINT's tilt and re-score the WHOLE traversal.

    Two reasons it must be the whole traversal, not the two neighbouring
    segments. A point in a raster is visited more than once, so its tilt
    changes every segment that touches it. And moving tilt off one segment
    puts it onto another — a "fix" that overloads a segment elsewhere is
    not a fix, and only a full re-score will show it.
    """
    hits = [i for i, s in enumerate(labels) if s == label]
    if not hits:
        return [], 0
    base = poses[hits[0]][3 + axis]
    out = []
    for k in range(steps):
        val = base - span / 2 + span * k / (steps - 1)
        trial = [list(p) for p in poses]
        for i in hits:
            trial[i][3 + axis] = val
        rows = segment_report(trial, tool, speed)
        good = [r for r in rows if r["j4"] is not None]
        if not good:
            continue
        # Score only the segments that solve. Some fail for reasons that
        # have nothing to do with the swept point and fail identically at
        # every tilt, so demanding a complete solve would reject the whole
        # sweep; the unsolved count is reported so a shifting one shows up.
        worst = max(good, key=lambda r: r["j4"])
        out.append({"val": val, "worst": worst["j4"], "at": worst["i"],
                    "n": len(good), "unsolved": len(rows) - len(good)})
    return out, len(hits)


def discover(plans_dir, include_all):
    files = sorted(glob.glob(os.path.join(plans_dir, "*.json")))
    if include_all:
        return files
    return [f for f in files
            if not any(k in os.path.basename(f) for k in NOT_CLEANING)]


def load_task(task, fixture="commode_c", pole_m=0.075):
    """(poses, labels, tool frame) resolved from a TASK CONFIG.

    `CleaningPath` is the port of the runtime's own waypoint resolution and
    is exact: checked here against `20260811T220617`, all 28 waypoints agree
    with the recorded commanded program to 0.000 mm and 0.000000 rad. So a
    task with no recording can still be screened from its config alone —
    which is the point, since 20 of 24 have never been executed.

    `pole_m` shifts every pose in Z. It does not change segment geometry or
    rotation, but it does move the whole path within the arm's workspace, so
    it changes the joint configuration and therefore the elbow demand. 0.075
    is what the toplid recordings used.
    """
    from task_config import TaskConfig
    from cleaning_path import CleaningPath
    prog = CleaningPath(TaskConfig.load(task, fixture=fixture),
                        quiet=True).movel_program(pole_m)
    return ([list(p) for p in prog["poses"]], list(prog["waypoint_names"]),
            prog.get("tool_frame"))


def load_poses(src):
    """(traversal poses, per-step labels, tool frame) from a run or a program.

    THE TRAVERSAL, NOT THE DECLARATION ORDER. A raster program declares its
    points once and visits them in a `SEQUENCE` that repeats many of them —
    `test_motion_001` declares 10 and walks 37 steps. Scoring the
    declaration order scores segments the arm never makes.
    """
    if src.endswith(".json"):
        d = json.load(open(src))
        c = d.get("commanded", {})
        poses = [list(p) for p in c["poses"]]
        labels = c.get("waypoint_names") or [str(i) for i in range(len(poses))]
        return poses, list(labels)[:len(poses)], c.get("tool_frame")

    import ast                       # a program is PARSED, never imported
    tree = ast.parse(open(src).read())
    found = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        for t in node.targets:
            nm = getattr(t, "id", "")
            if nm in ("POSES_MM", "POSES", "SEQUENCE"):
                try:
                    found[nm] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    key = "POSES_MM" if "POSES_MM" in found else "POSES"
    if key not in found:
        raise SystemExit("no POSES_MM / POSES in %s" % src)
    sc = 0.001 if key == "POSES_MM" else 1.0
    raw = found[key]
    by_label = ({str(k): v for k, v in raw.items()} if isinstance(raw, dict)
                else {str(i): v for i, v in enumerate(raw)})
    by_label = {k: [v[0] * sc, v[1] * sc, v[2] * sc, v[3], v[4], v[5]]
                for k, v in by_label.items()}
    seq = [str(s) for s in found.get("SEQUENCE", list(by_label))]
    seq = [s for s in seq if s in by_label]
    return [list(by_label[s]) for s in seq], seq, None


def selfcheck(tool):
    """Prove the tool+mount transform against a recording before trusting it.

    The transform is the whole basis of this mode. An earlier version of
    this analysis omitted the tool offset and confidently reported 400-550 %
    of the J4 limit on segments that had run clean on hardware.
    """
    run = os.path.join(HERE, "runs", "20260811T220617_toplid_left_left",
                       "run.json")
    plan = os.path.join(DEFAULT_PLANS, "toplid_left_ruckig_pro_only.json")
    if not (os.path.exists(run) and os.path.exists(plan)):
        return None
    poses, _, tf = load_poses(run)
    wps = [s for s in json.load(open(plan))["sub_trajectories"]
           if s.get("stage_name") == "execute_path"][0]["waypoints"]
    tip = fk([math.degrees(v) for v in wps[0]["positions"]])
    R = _Rmat(*tip[3:6])
    t = TOOL_OFFSETS[tf or tool]
    tcp_b = [tip[i] + sum(R[i][k] * t[k] for k in range(3)) for i in range(3)]
    Rm = _Ry(math.radians(MOUNT_RY_DEG))
    tcp_w = [sum(Rm[i][k] * tcp_b[k] for k in range(3)) for i in range(3)]
    return max(abs(tcp_w[i] - poses[0][i]) for i in range(3))


def sweep_all_tasks(argv) -> int:
    """Screen EVERY task config's Cartesian program, ranked by J4 demand."""
    speed = float(argv[argv.index("--speed") + 1]) if "--speed" in argv else 0.45
    pole = float(argv[argv.index("--pole") + 1]) if "--pole" in argv else 0.075
    cap = (float(argv[argv.index("--angular-cap") + 1])
           if "--angular-cap" in argv else ANGULAR_CAP)
    fixture = argv[argv.index("--fixture") + 1] if "--fixture" in argv \
        else "commode_c"
    from task_config import config_category_dir, config_source
    cdir = config_category_dir("commode_cleaning") / fixture
    print("configs: %s" % config_source())
    tasks = sorted(p.name[:-len("_cleaning_points.yaml")]
                   for p in cdir.glob("*_cleaning_points.yaml"))
    tasks = [t for t in tasks if not any(k in t for k in NOT_CLEANING)]
    print("Segment screen of %d task configs — line_speed %.3f m/s, pole %.3f m,"
          " angular cap %.2f rad/s" % (len(tasks), speed, pole, cap))
    print("""
CALIBRATION — validated on BOTH arms against the only two speed-tested runs:
    toplid_left   projects 126 %,  hardware measured  96 %   (x1.31)
    toplid_right  projects 109 %,  hardware measured  92 %   (x1.18)
  So this over-reads ~1.2-1.3x: treat >130 % as a real overload, 90-130 % as
  worth a SIM run, below 90 % as clear on J4 alone.
  Rows with many IK gaps are interpolated across them and UNDER-read.
""")
    print("%-28s %6s %8s %9s %8s %7s  %s"
          % ("task", "segs", "worst J4", "%J4 limit", "unthrot", "IK gaps",
             "worst segment"))
    print("-" * 100)
    out = []
    for t in tasks:
        try:
            poses, labels, tf = load_task(t, pole_m=pole)
            rows = segment_report(poses, tf or "L_glove_2", speed, cap)
        except SystemExit as exc:
            print("%-28s   %s" % (t, str(exc)[:60]))
            continue
        except Exception as exc:
            print("%-28s   %s: %s" % (t, type(exc).__name__, str(exc)[:50]))
            continue
        good = [r for r in rows if r["j4"] is not None]
        if not good:
            print("%-28s %6d   no segment solved" % (t, len(rows)))
            continue
        w = max(good, key=lambda r: r["j4"])
        # UNTHROTTLED segments are the ones the angular cap does not slow —
        # the class that broke `test_motion_001` (H67).
        unthrot = sum(1 for r in good if not r["throttled"] and
                      r["j4"] > 0.9 * JOINT_LIMIT[3])
        name = ("%s->%s" % (labels[w["i"]], labels[w["i"] + 1])
                if w["i"] + 1 < len(labels) else str(w["i"]))
        out.append((t, len(rows), w["j4"], unthrot, len(rows) - len(good), name))
    out.sort(key=lambda r: -r[2])
    for t, n, j4, unthrot, gaps, name in out:
        pct = 100 * j4 / JOINT_LIMIT[3]
        flag = "  <-- " + ("OVER" if pct >= 100 else "tight") if pct >= 90 else ""
        print("%-28s %6d %8.0f %8.0f%% %8d %7d  %s%s"
              % (t, n, j4, pct, unthrot, gaps, name, flag))
    if out:
        over = [r for r in out if 100 * r[2] / JOINT_LIMIT[3] >= 100]
        print("\n%d tasks screened; %d project J4 OVER its limit; %d have at "
              "least one UNTHROTTLED segment above 90%%"
              % (len(out), len(over), sum(1 for r in out if r[3])))
        print("unthrottled = the angular cap does not slow it, so nothing "
              "protects the elbow (H67)")
    return 0


def run_segments(argv) -> int:
    src = argv[argv.index("--segments") + 1]
    speed = float(argv[argv.index("--speed") + 1]) if "--speed" in argv else 0.45
    cap = (float(argv[argv.index("--angular-cap") + 1])
           if "--angular-cap" in argv else ANGULAR_CAP)
    if src.startswith("task:"):
        pole = float(argv[argv.index("--pole") + 1]) if "--pole" in argv else 0.075
        poses, labels, tf = load_task(src[5:], pole_m=pole)
    else:
        poses, labels, tf = load_poses(src)
    tool = argv[argv.index("--tool") + 1] if "--tool" in argv else (tf or "L_glove_2")
    if tool not in TOOL_OFFSETS:
        print("unknown tool frame %r; known: %s"
              % (tool, ", ".join(sorted(TOOL_OFFSETS))))
        return 1

    err = selfcheck(tool)
    if err is None:
        print("[WARN] self-check skipped (reference run/plan not present)")
    elif err > 0.001:
        print("[FATAL] tool+mount transform self-check FAILED: %.4f m error. "
              "Every number below would be wrong; refusing." % err)
        return 1
    else:
        print("self-check OK — tool+mount transform reproduces a recorded "
              "commanded pose to %.1f um\n" % (1e6 * err))

    print("%s\n%d traversal steps (%d distinct points), tool %s, "
          "line_speed %.3f m/s, angular cap %.2f rad/s"
          % (os.path.basename(src), len(poses), len(set(labels)), tool,
             speed, cap))
    print("J4 only — redundancy-invariant, so exact; other joints need the "
          "plan (see module docstring)\n")
    rows = segment_report(poses, tool, speed, cap)
    print("%-9s %8s %7s %8s %7s %10s %8s %6s  %s"
          % ("seg", "len mm", "rot deg", "w rad/s", "v eff", "J4 deg/m",
             "J4 deg/s", "%lim", "note"))
    print("-" * 92)
    def seg_name(i):
        return "%s->%s" % (labels[i], labels[i + 1]) if i + 1 < len(labels) \
            else "%d->%d" % (i, i + 1)

    worst = None
    for r in rows:
        if r["j4"] is None:
            print("%-9s %8.1f %7.2f %8.2f %7.3f %10s %8s %6s  IK incomplete"
                  % (seg_name(r["i"]), 1000 * r["len"], r["rot"], r["w"],
                     r["v"], "-", "-", "-"))
            continue
        pct = 100 * r["j4"] / JOINT_LIMIT[3]
        note = ("UNTHROTTLED — nothing slows this segment"
                if not r["throttled"] else "throttled by angular cap")
        if pct >= 90:
            note = "<-- J4 %s. %s" % ("OVER" if pct >= 100 else "near limit",
                                      note)
        print("%-9s %8.1f %7.2f %8.2f %7.3f %10.1f %8.0f %5.0f%%  %s"
              % (seg_name(r["i"]), 1000 * r["len"], r["rot"], r["w"], r["v"],
                 r["j4_per_m"], r["j4"], pct, note))
        if worst is None or r["j4"] > worst["j4"]:
            worst = r
    if worst:
        print("\nworst: segment %s at %.0f%% of the J4 limit (rot %.2f deg, %s)"
              % (seg_name(worst["i"]), 100 * worst["j4"] / JOINT_LIMIT[3],
                 worst["rot"],
                 "throttled" if worst["throttled"] else "UNTHROTTLED"))

    if "--sweep" in argv:
        label = argv[argv.index("--sweep") + 1]
        axis = int(argv[argv.index("--axis") + 1]) if "--axis" in argv else 1
        if label not in labels:
            print("\nno point %r in the traversal; points: %s"
                  % (label, ", ".join(dict.fromkeys(labels))))
            return 1
        res, nvisits = sweep_tilt(poses, labels, label, tool, speed, axis=axis)
        print("\nsweeping point %r on r%s — visited %d time(s), so the whole "
              "%d-step traversal is re-scored each time\n"
              % (label, "xyz"[axis], nvisits, len(poses)))
        if not res:
            print("  no tilt in the swept range solves every segment")
            return 1
        cur = poses[labels.index(label)][3 + axis]
        print("%10s %12s %9s   %s"
              % ("r%s rad" % "xyz"[axis], "worst J4", "%limit",
                 "worst segment"))
        for r in res:
            mark = "  <-- current" if abs(r["val"] - cur) < 5e-4 else ""
            print("%10.3f %12.0f %8.0f%%   %s%s"
                  % (r["val"], r["worst"], 100 * r["worst"] / JOINT_LIMIT[3],
                     seg_name(r["at"]), mark))
        best = min(res, key=lambda r: r["worst"])
        now = min((r for r in res if abs(r["val"] - cur) < 5e-4),
                  key=lambda r: abs(r["val"] - cur), default=None)
        print("\nBEST  r%s = %+.3f rad -> worst J4 %.0f deg/s (%.0f%% of limit)"
              % ("xyz"[axis], best["val"], best["worst"],
                 100 * best["worst"] / JOINT_LIMIT[3]))
        if now:
            print("NOW   r%s = %+.3f rad -> worst J4 %.0f deg/s (%.0f%%)"
                  % ("xyz"[axis], now["val"], now["worst"],
                     100 * now["worst"] / JOINT_LIMIT[3]))
        print("\n  J4 only — a pass here is NOT a clearance, reachability or "
              "J1 check.\n  Verify with predict_task.py and a SIM run before "
              "REAL motion.")
    return 0


def main() -> int:
    if wants_help():
        print(__doc__)
        return 0
    argv = sys.argv[1:]
    if "--screen-all" in argv:
        return sweep_all_tasks(argv)
    if "--segments" in argv:
        return run_segments(argv)
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
