#!/usr/bin/env python3
"""Execute a cleaning stroke on the EMULATOR and report its joint kinematics.

No hardware, no network. `rm_emulator` installs itself as the Robotic_Arm
SDK, so this exercises the same `rm_movel` chain `stage_runner` dispatches —
tool frame, queued `trajectory_connect=1` segments, closing `connect=0` — and
the emulator solves the Cartesian path with RealMan's own offline IK.

WHAT THIS IS FOR. 20 of the 24 commode_c cleaning tasks have never been
executed, and 7 of them PLAN at 99.8-99.9 % of a joint speed limit
(TASK_KINEMATICS.md, H48). A REAL run at those settings is the condition
under which the arm stopped silently and reported nothing (H45). This gives
a joint-rate estimate for every task with no arm involved.

WHAT IT IS NOT. The emulator is not the controller. Its redundancy
resolution is `rm_algo_inverse_kinematics` seeded from the previous sample;
the controller's is unpublished and demonstrably different (measured: on
`toplid_right` the plan peaks J4 at 82 % and the REAL run at 92-97 %). Read
the numbers as a SCREEN — ordering and binding joint — not as a prediction of
the exact percentage. `--validate` quantifies that gap against the four
tasks for which SIM and REAL recordings exist.

    python3 emu_stroke.py                     # all cleaning tasks
    python3 emu_stroke.py toplid_left         # one task
    python3 emu_stroke.py --validate          # emulator vs SIM vs REAL
    python3 emu_stroke.py --ls 0.45 --la 3.6  # at chosen limits
"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from log_utils import wants_help  # noqa: E402

import rm_emulator  # noqa: E402

rm_emulator.install()                       # BEFORE any SDK-importing module

from task_config import TaskConfig          # noqa: E402
from cleaning_path import CleaningPath      # noqa: E402
import frame_alignment_offline as fa        # noqa: E402

LIM = [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0]
POLE_M = 0.075
# Bundled plans only — the rule `segment_verifier.resolve_plan` enforces.
LOCAL_PLANS = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "plans")

# Cleaning tasks only — mechanism motions (lid_open, glove_*, flush_press …)
# are a different motion class and are excluded from every table here.
NOT_CLEANING = ("glove_", "lid_open", "lid_close", "seat_open", "seat_close",
                "flush_press", "dual_bin", "dual_arm_gate", "test_motion",
                "backup_", "move_to_", "_totg")


def cleaning_tasks():
    """Every cleaning task with a BUNDLED plan.

    The bundled plan is what makes a task runnable — `stage_runner` resolves
    it there and nowhere else — so it is also the right definition of "a task
    this tooling can speak about". Listing the config tree instead would name
    tasks that cannot be run.
    """
    out = []
    for f in sorted(glob.glob(os.path.join(LOCAL_PLANS,
                                           "*_ruckig_pro_only.json"))):
        t = os.path.basename(f).replace("_ruckig_pro_only.json", "")
        if not any(k in t for k in NOT_CLEANING):
            out.append(t)
    return out


def plan_start_joints(task):
    """First joint configuration of the plan's execute_path, degrees."""
    for d in (LOCAL_PLANS,):
        for f in glob.glob(os.path.join(d, task + "*.json")):
            try:
                plan = json.load(open(f))
            except Exception:
                continue
            sub = [s for s in plan.get("sub_trajectories", [])
                   if s.get("stage_name") == "execute_path"]
            if sub:
                return [math.degrees(v)
                        for v in sub[0]["waypoints"][0]["positions"]], f
    return None, None


def ensure_tool_frame(robot, side, name, ctrl):
    """Create the glove frame on the emulated controller, as hardware has.

    THE OFFSET GOES IN `frame.pose.position`, IN METRES. `rm_frame_t.x/y/z`
    is the payload CENTROID, not the tool offset — putting the offset there
    leaves the tool at the flange, and then every Cartesian target sits
    ~180 mm beyond reach. Measured 2026-08-12: that mistake produced
    "3444 of 3531 samples had no IK solution (97.5 %)". Same family as the
    1000x centroid of F31 — adjacent fields, silent when confused.
    """
    from Robotic_Arm.rm_ctypes_wrap import rm_frame_t
    for _link, ctrl_name, _conn, tip, rpy in fa.frame_map(side):
        if ctrl_name != name:
            continue
        f = rm_frame_t()
        f.frame_name = name.encode() if isinstance(name, str) else name
        f.pose.position.x, f.pose.position.y, f.pose.position.z = (
            float(tip[0]), float(tip[1]), float(tip[2]))          # metres
        f.pose.euler.rx, f.pose.euler.ry, f.pose.euler.rz = (
            float(rpy[0]), float(rpy[1]), float(rpy[2]))
        f.payload = 0.0
        f.x = f.y = f.z = 0.0                                      # centroid
        # CREATE-only, exactly as the controller behaves (F17): a second
        # call for a name that already exists answers ret=1. Running many
        # tasks in ONE process hits that on every repeat of a frame, which
        # is not an error — the frame is there and correct.
        if name in ctrl.tool_frames:
            return 0
        return robot.rm_set_manual_tool_frame(f)
    return 1


def run_task(task, ls, la, verbose=False):
    try:
        return _run_task(task, ls, la)
    except BaseException as exc:
        # SystemExit too: `cleaning_path` raises SystemExit on an unchained
        # cleaning_sequence, and SystemExit is not an Exception.
        # One unrunnable task must not end the sweep: `front_left` raises on
        # an unchained cleaning_sequence, which is a real config problem but
        # a property of that task, not of the emulator.
        return {"task": task, "error": "%s: %s" % (type(exc).__name__,
                                                   str(exc).split("\n")[0][:60])}


def _run_task(task, ls, la):
    cfg = TaskConfig.load(task, fixture="commode_c")
    side = "left" if "_left" in task else "right"
    prog = CleaningPath(cfg).movel_program(POLE_M)
    q0, planfile = plan_start_joints(task)
    if q0 is None:
        return {"task": task, "error": "no plan (no start configuration)"}

    ip = rm_emulator.EMU_LEFT_IP if side == "left" else rm_emulator.EMU_RIGHT_IP
    ctrl = rm_emulator.emu_controller(ip)
    ctrl.joints_deg = list(q0)
    ctrl.limits["line_speed"] = ls
    ctrl.limits["line_acc"] = la

    from Robotic_Arm.rm_robot_interface import RoboticArm
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    h = robot.rm_create_robot_arm(ip, 8080, 3)
    if h is None or h.id <= 0:
        return {"task": task, "error": "emulated connect failed"}
    try:
        if ensure_tool_frame(robot, side, prog["tool_frame"], ctrl) != 0:
            return {"task": task, "error": "tool frame %s rejected"
                    % prog["tool_frame"]}
        if robot.rm_change_tool_frame(prog["tool_frame"]) != 0:
            return {"task": task, "error": "tool frame select failed"}

        # Take the emulator's OWN Cartesian plan directly rather than
        # sampling a threaded motion: the joint waypoints and duration are
        # what it would drive, without adding sampling noise on top. The
        # dispatch path itself is exercised by --validate.
        tool = ctrl._active_tool_pose()
        path, dur = ctrl._plan_cartesian(
            list(q0), tool, prog["poses"][1:], cfg.cleaning_speed_pct, ls, la)
        if path is None:
            return {"task": task, "error": "IK/limit failure in the stroke"}

        # Joint rates: the path is walked at uniform time by _PathMotion,
        # so dt is constant and d(position)/dt needs no guarding.
        n = len(path)
        dt = dur / max(1, n - 1)
        rate = [[abs(path[i + 1][j] - path[i][j]) / dt
                 for i in range(n - 1)] for j in range(7)]

        def p99(v):
            s = sorted(v)
            return s[min(len(s) - 1, int(len(s) * 0.99))] if s else 0.0

        util = [(p99(rate[j]) / LIM[j], j + 1) for j in range(7)]
        u, jbind = max(util)
        pk = [(max(rate[j]) / LIM[j], j + 1) for j in range(7)]
        upk, jpk = max(pk)
        cart = sum(math.dist(a[:3], b[:3])
                   for a, b in zip([tool] + [list(x[:6]) for x in
                                             prog["poses"][1:]],
                                   [list(x[:6]) for x in prog["poses"][1:]]))
        uns, tot = getattr(ctrl, "last_unsolved", (0, 1))
        return {"task": task, "side": side, "seg": prog["segments"],
                "ikfail": uns / max(1, tot),
                "samples": n, "dur": dur, "cart": cart,
                "util": u, "joint": jbind, "util_pk": upk, "joint_pk": jpk,
                "tool": prog["tool_frame"], "plan": os.path.basename(planfile)}
    finally:
        try:
            robot.rm_delete_robot_arm()
        except Exception:
            pass


def validate(ls, la):
    """Emulator vs the SIM and REAL recordings, on the four known tasks."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import mode_compare as mc
    runs = [r for r in (mc.load(d) for d in
                        sorted(glob.glob(mc.RUNS + "/2026*"))) if r]
    print("\nVALIDATION — emulator against recorded SIM and REAL\n")
    print("%-22s %-9s %-14s %-14s %-14s"
          % ("task", "ls/la", "EMULATOR", "SIM", "REAL"))
    print("-" * 78)
    for task in ("toplid_left", "toplid_right", "hinge_area_left",
                 "hinge_area_right"):
        e = run_task(task, ls, la)
        if "error" in e:
            print("%-22s %-9s %s" % (task, "%s/%s" % (ls, la), e["error"]))
            continue
        sel = [r for r in runs if r["task"] == task
               and (r["ls"] is None or abs((r["ls"] or 0) - ls) < 1e-6)]
        sim = [r for r in sel if r["mode"] == "SIM"]
        real = [r for r in sel if r["mode"] == "REAL"]

        def best(rs, how):
            if not rs:
                return None
            vals = [mc.util(r["w"], how) for r in rs]
            return max(vals)
        s = best(sim, mc.deriv)
        r = best(real, mc.rep)
        f = lambda x: ("J%d %3.0f%%" % (x[1], 100 * x[0])) if x else "   —"
        print("%-22s %-9s %-14s %-14s %-14s"
              % (task, "%s/%s" % (ls, la),
                 "J%d %3.0f%%" % (e["joint"], 100 * e["util"]),
                 f(s), f(r)))
    return 0


def main() -> int:
    if wants_help():
        print(__doc__)
        return 0
    argv = sys.argv[1:]
    ls = float(argv[argv.index("--ls") + 1]) if "--ls" in argv else 0.45
    la = float(argv[argv.index("--la") + 1]) if "--la" in argv else 3.6
    if "--validate" in argv:
        return validate(ls, la)

    named = [a for a in argv if not a.startswith("--")
             and a not in (str(ls), str(la))]
    tasks = named or cleaning_tasks()
    print("Emulator strokes — %d task(s) at line_speed %.2f, line_acc %.1f"
          % (len(tasks), ls, la))
    print("Redundancy resolved by rm_algo_inverse_kinematics, seeded per "
          "sample.\nRead as a SCREEN (ordering, binding joint), not an exact "
          "prediction.\n")
    print("%-28s %4s %6s %7s  %-11s %-11s"
          % ("task", "seg", "cart m", "dur s", "p99 util", "peak util"))
    print("-" * 76)
    out = []
    for t in tasks:
        r = run_task(t, ls, la)
        out.append(r)
        if "error" in r:
            print("%-28s   %s" % (t, r["error"]))
            continue
        flag = "  <--" if r["util"] >= 0.95 else ""
        print("%-28s %4d %6.2f %7.2f  J%d %6.0f%%  J%d %6.0f%%%s"
              % (t, r["seg"], r["cart"], r["dur"], r["joint"],
                 100 * r["util"], r["joint_pk"], 100 * r["util_pk"], flag))
    ok = [r for r in out if "error" not in r]
    if ok:
        print("\n%d ran, %d failed. %d at or above 95%% (p99) on the emulator."
              % (len(ok), len(out) - len(ok),
                 sum(1 for r in ok if r["util"] >= 0.95)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
