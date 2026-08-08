"""Phase 2 — execute a cleaning task's stage sequence, one device at a time.

The task config already serializes the devices: every entry in
`stage_sequence` names ONE `planning_group` — `pole_*`, `inspire_hand_*`,
`rm75_arm_*` — and the stages are strictly ordered. That is exactly the
regime F9 requires (concurrent pole+arm motion in the planned domain is a
vendor-confirmed Gen-3 defect; RealMan's own workaround is serialization),
so this runner does not have to impose serialization — it has to not break
it, and to FAIL LOUDLY if a future task stops satisfying it.

    RM_SERIALIZE=1   (default) refuse to run a task whose stages are not
                     strictly one-device-at-a-time, and assert at dispatch
                     that no other device is in flight
    RM_SERIALIZE=0   report the same findings but do not block

Per stage:

    pole   rm_set_lift_height at the task's pole scaling, blocking on the
           device-2 arrival event. The pole MOVES THE ARM BASE, so it must
           complete before any pose is resolved — the path is a function
           of pole height (215 mm between SRDF home and the tasks' 0.075).
    hand   rm_set_hand_angle, duration-based completion (F4: device-2
           events never fire for the hand)
    arm    named poses via rm_movej on the SAVED PLAN's joint values —
           unambiguous, and it pins the redundant 7-DOF configuration that
           MoveIt validated. `execute_path` via the CHAINED movel program
           built from the task config (cleaning_path.py).

Nothing here plans. Targets come from the task config (Cartesian strokes)
and the saved plan (joint values for named poses).

Usage:
    python3 stage_runner.py --task hinge_area_right [--mode REAL] [--dry]
    RM_SERIALIZE=0 python3 stage_runner.py --task toplid_left --dry
"""

import os
import sys
import time

from cleaning_path import CleaningPath
from task_config import TaskConfig
from dual_arm_common import (
    handle_cli, parse_mode_arg, countdown, preflight_error_gate,
    apply_run_mode, mode_label, report_run_modes, restore_run_modes,
    teardown, ArrivalMonitor, ConceptArm,
    ARM_TIMEOUT_S, DEV_JOINT, DEV_LIFT, LEFT_IP, LIFT_TIMEOUT_S,
    RIGHT_IP, ROBOT_PORT, hand_dwell_s, lift_hw_mm,
)
import speed_limits
from segment_verifier import (
    arm_stages, load_plan, resolve_plan, stage_maps)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

SERIALIZE = os.environ.get("RM_SERIALIZE", "1") != "0"
POLE_QUARTER_M = 0.075          # what every commode task commands

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}


def result(tag, name, detail=""):
    _results[tag] += 1
    print(f"  [{tag}] {name}" + (f"  ({detail})" if detail else ""))


def _arg(flag, default=None):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


class DeviceBusy(RuntimeError):
    """A dispatch was attempted while another device was in flight."""


class StageRunner:
    """Walks a task's active stages, one device at a time."""

    def __init__(self, cfg, arm, monitor, plan, dry=False,
                 serialize=SERIALIZE, sim=False):
        self.cfg = cfg
        self.arm = arm
        self.monitor = monitor
        self.plan = plan
        self.dry = dry
        self.serialize = serialize
        # SIM mode executes the ARM only — the controller has no model of
        # the pole or the hand (F3), so those stages would sit in their
        # arrival waits and time out. Assume them successful instead, so a
        # whole task can be rehearsed on the real controller and catch the
        # things that only the real SDK catches (a wrong pose frame, a bad
        # call signature) WITHOUT the arm ever moving for real.
        self.sim = sim
        self.in_flight = None          # the invariant, as state
        self.log = []
        self.pole_m = None

    # ── the invariant ──
    def _claim(self, device, what):
        """Take the single device slot, or refuse.

        Held for the whole blocking dispatch, so a second device can only
        be claimed once the first has completed. This is the runtime
        assertion for F9 — cheap, and it turns a silent truncation (arm
        faulted, or pole stopped 25 mm in) into an exception naming both
        parties.
        """
        if self.in_flight is not None:
            msg = (f"{device} dispatch for {what!r} while {self.in_flight} "
                   "is still in flight — concurrent devices in the planned "
                   "domain truncate each other (F9)")
            if self.serialize:
                raise DeviceBusy(msg)
            print(f"    [WARN] {msg}")
        self.in_flight = device

    def _release(self):
        self.in_flight = None

    # ── plan lookups (named poses) ──
    def _plan_stage(self, name):
        for st in self.plan["sub_trajectories"]:
            if st["stage_name"] == name:
                return st
        return None

    def _arm_target_deg(self, stage_name):
        """MoveIt's own joint solution for a named-pose stage (degrees)."""
        import math
        st = self._plan_stage(stage_name)
        if st is None:
            return None
        pref = "R_" if self.cfg.side == "right" else "L_"
        joints = [f"{pref}joint{i}" for i in range(1, 8)]
        last = stage_maps(st)[-1]
        if not all(j in last for j in joints):
            return None
        return [math.degrees(last[j]) for j in joints]

    # ── devices ──
    def run_pole(self, stage):
        target_m = POLE_QUARTER_M
        hw = lift_hw_mm(self.cfg.side, target_m)
        speed = self.cfg.pole_speed_pct
        self._claim("pole", stage.get("stage"))
        try:
            if self.dry:
                self.pole_m = target_m
                return True, f"DRY set_lift_height({speed}%, {hw})"
            if self.sim:
                # The pole still has to be RECORDED — the cleaning path is
                # resolved at this height — it just is not commanded.
                self.pole_m = target_m
                return True, (f"SIM assumed: {hw} hw-mm at {speed}% "
                              "(controller does not simulate the pole)")
            self.monitor.expect(self.arm.handle_id, DEV_LIFT)
            ret = self.arm.robot.rm_set_lift_height(speed, hw, 0)
            if ret != 0:
                return False, f"rejected ret={ret}"
            arrived, ok = self.monitor.wait(self.arm.handle_id, DEV_LIFT,
                                            LIFT_TIMEOUT_S)
            self.pole_m = target_m if arrived else None
            return bool(arrived and ok), f"{hw} hw-mm at {speed}%"
        finally:
            self._release()

    def run_hand(self, stage):
        """Hand target from the PLAN, converted — never from a name guess.

        The commode tasks use `open_tenth` (1.17 rad) and `quarter_grasp`
        (0.336), neither of which is a concept HAND_STATES_HW entry. An
        earlier version substituted the nearest-named state, which sent
        `open_tenth` to 993 counts instead of ~130 — the hand flying open
        mid-task while gripping a cloth against the fixture. The plan
        carries the actual joint angles for every hand stage, so use them.
        """
        from dual_arm_common import HAND_STATES_HW, hand_rad_to_hw
        pose = stage.get("pose_name", "release")
        st = self._plan_stage(stage.get("stage"))
        target, src = None, ""
        if st and st.get("waypoints"):
            angles = dict(zip(st["joint_names"],
                              st["waypoints"][-1]["positions"]))
            try:
                target, src = hand_rad_to_hw(angles), "plan"
            except KeyError as exc:
                print(f"    [WARN] {exc}")
        if target is None and pose in HAND_STATES_HW:
            target, src = HAND_STATES_HW[pose], "concept state"
        if target is None:
            return None, (f"no joint angles in the plan for {pose!r} and no "
                          "concept state — SKIPPED rather than guessed")
        self._claim("hand", pose)
        try:
            if self.dry:
                return True, (f"DRY set_hand_angle({pose}) {target} from {src}")
            if self.sim:
                return True, (f"SIM assumed: {pose} {target} from {src} "
                              "(controller does not simulate the hand)")
            # (angles, block, timeout) — timeout is POSITIONAL in the
            # real SDK; the same call shape dual_arm_common uses.
            ret = self.arm.robot.rm_set_hand_angle(target, False, 2)
            if ret != 0:
                return False, f"rejected ret={ret}"
            time.sleep(hand_dwell_s(373.0))     # F4: no arrival event
            return True, f"{pose} {target} from {src} (duration-based)"
        finally:
            self._release()

    def run_arm_pose(self, stage):
        name = stage.get("stage")
        target = self._arm_target_deg(name)
        if target is None:
            return None, f"no joint solution for {name!r} in the plan"
        speed = (self.cfg.approach_speed_pct
                 if "start" in name else self.cfg.arm_speed_pct)
        self._claim("arm", name)
        try:
            if self.dry:
                return True, (f"DRY movej({[round(v, 1) for v in target]}, "
                              f"v={speed}%)")
            self.monitor.expect(self.arm.handle_id, DEV_JOINT)
            ret = self.arm.robot.rm_movej(target, speed, 0, 0, 0)
            if ret != 0:
                return False, f"rejected ret={ret}"
            arrived, ok = self.monitor.wait(self.arm.handle_id, DEV_JOINT,
                                            ARM_TIMEOUT_S)
            return bool(arrived and ok), f"v={speed}%"
        finally:
            self._release()

    def run_cleaning_path(self, stage):
        """The chained movel program, at the CURRENT pole height."""
        if self.pole_m is None:
            return False, ("pole height unknown — the path is a function of "
                           "it; run the pole stage first")
        prog = CleaningPath(self.cfg).movel_program(self.pole_m)
        n = prog["segments"]
        self._claim("arm", "execute_path")
        try:
            if self.dry:
                return True, (f"DRY tool={prog['tool_frame']} "
                              f"{n} chained movel, r={prog['blend_pct']}%, "
                              f"v={self.cfg.cleaning_speed_pct}%")
            ret = self.arm.robot.rm_change_tool_frame(prog["tool_frame"])
            if ret != 0:
                return False, (
                    f"cannot select tool frame {prog['tool_frame']!r} "
                    f"(ret={ret}) — the task's ik_frame "
                    f"{self.cfg.ik_frame} must exist on the controller. "
                    f"Write it with: RM_ARM={self.cfg.side} python3 "
                    "test_frame_alignment.py --mode REAL --create-frames")
            # the stroke runs at the task's full scaling; transits
            # are halved (TRANSIT_DERATE) — Newton's policy
            speed = self.cfg.cleaning_speed_pct
            blend = prog["blend_pct"]
            rejects = []
            self.monitor.expect(self.arm.handle_id, DEV_JOINT)
            for i, q in enumerate(prog["poses"][1:], start=1):
                last = i == n
                # rm_movel takes a 6-element LIST [x,y,z,rx,ry,rz] and
                # builds rm_pose_t internally — handing it a struct fails
                # with "'rm_pose_t' object is not subscriptable".
                # connect=1 queues; the closing connect=0 executes the
                # whole chain and yields ONE arrival event.
                r = self.arm.robot.rm_movel(
                    [float(x) for x in q[:6]], speed,
                    blend if not last else 0, 0 if last else 1, 0)
                if r != 0:
                    rejects.append((i, r))
            if rejects:
                return False, (f"{len(rejects)} segment(s) rejected: "
                               f"{rejects[:5]} — check queue depth (C10)")
            arrived, ok = self.monitor.wait(self.arm.handle_id, DEV_JOINT,
                                            max(ARM_TIMEOUT_S, 120.0))
            return bool(arrived and ok), f"{n} segments, tool " \
                f"{prog['tool_frame']}"
        finally:
            self._release()

    # ── the walk ──
    def run(self):
        ok_all, order, problems = self.cfg.enforce_serialization(
            self.serialize)
        if problems:
            for p in problems:
                print(f"  [SERIALIZE] {p}")
        if not ok_all:
            result("FAIL", "task is serialized (one device per stage)",
                   f"{len(problems)} violation(s)")
            return False
        result("PASS", "task is serialized (one device per stage)",
               f"{len(order)} device stages")

        for stage in self.cfg.active_stages():
            name = stage.get("stage")
            dev = self.cfg.device_of(stage)
            if dev == "scene":
                self.log.append((name, "scene", None, "not a motion stage"))
                continue
            t0 = time.perf_counter()
            try:
                if dev == "pole":
                    ok, detail = self.run_pole(stage)
                elif dev == "hand":
                    ok, detail = self.run_hand(stage)
                elif name == "execute_path":
                    ok, detail = self.run_cleaning_path(stage)
                else:
                    ok, detail = self.run_arm_pose(stage)
            except DeviceBusy as exc:
                ok, detail = False, str(exc)
            dt = time.perf_counter() - t0
            self.log.append((name, dev, ok, detail))
            tag = {True: "ok  ", False: "FAIL", None: "skip"}[ok]
            print(f"    {tag} {name:28s} [{dev:4s}] {dt:6.2f}s  {detail}")
            if ok is False:
                result("FAIL", "stage sequence completed",
                       f"stopped at {name!r}")
                return False
        result("PASS", "stage sequence completed",
               f"{sum(1 for _n, _d, o, _x in self.log if o)} stages ran")
        return True


def main() -> int:
    for k in _results:
        _results[k] = 0
    handle_cli(__doc__, extra_flags=("--dry",),
               value_flags=("--task", "--fixture"))
    task = _arg("--task", "hinge_area_right")
    fixture = _arg("--fixture", "commode_c")
    dry = "--dry" in sys.argv
    forced = parse_mode_arg()

    cfg = TaskConfig.load(task, fixture=fixture)
    ip = RIGHT_IP if cfg.side == "right" else LEFT_IP
    print("=" * 72)
    print(f"Phase 2 stage runner — {task}  [{cfg.side}]")
    print(f"    arm={ip}   ik={cfg.ik_frame}")
    print(f"    speeds: cleaning={cfg.cleaning_speed_pct}%  "
          f"transit={cfg.arm_speed_pct}%  approach={cfg.approach_speed_pct}%"
          f"  pole={cfg.pole_speed_pct}%   (task asks "
          f"{cfg.arm_speed_intended_pct}% before the transit/derate policy)")
    print(f"    serialization: {'ENFORCED' if SERIALIZE else 'report-only'}"
          "   (RM_SERIALIZE=0 to relax)")
    print(f"    mode: {mode_label(forced)}"
          + ("   DRY RUN — no commands are sent" if dry else
             "   THE ARM, ITS POLE AND ITS HAND WILL MOVE"))
    print("=" * 72)

    plan = load_plan(resolve_plan(f"{task}_ruckig_pro_only.json"))
    pref = "R_" if cfg.side == "right" else "L_"
    print(f"  plan arm stages: "
          f"{[s['stage_name'] for s in arm_stages(plan, prefix=f'{pref}joint')]}")

    if dry:
        runner = StageRunner(cfg, None, None, plan, dry=True)
        ok = runner.run()
        print(f"\n  Summary: {_results['PASS']} PASS, {_results['FAIL']} "
              f"FAIL, {_results['SKIP']} SKIP")
        return 0 if ok else 1

    arm = None
    originals = {}
    try:
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] hardware not reachable at {ip}")
            _results["SKIP"] += 2
            return 0
        arm = ConceptArm(cfg.side, robot, handle)
        monitor = ArrivalMonitor()
        monitor.register(robot)
        originals = apply_run_mode(forced, arm)
        if originals is None:
            result("FAIL", "run-mode selection", "did not engage")
            return 1
        report_run_modes(arm)
        # What mode are we ACTUALLY in? --mode wins; otherwise ask.
        try:
            _r, mode_now = robot.rm_get_arm_run_mode()
            mode_now = mode_now if _r == 0 else None
        except Exception:
            mode_now = None
        # A percentage is meaningless without its ceiling — print what the
        # commanded v% actually is in m/s and deg/s on THIS controller.
        lim = speed_limits.read(robot)
        print("  [INFO] limits: " + speed_limits.describe(
            lim, cfg.cleaning_speed_pct, cfg.arm_speed_pct))
        sim_mode = (forced == 0) or (forced is None and mode_now == 0)
        ok_err, detail = preflight_error_gate(arm)
        if not ok_err:
            result("FAIL", "no latched controller errors", detail)
            return 1
        result("PASS", "no latched controller errors", detail)
        countdown()
        if sim_mode:
            print("  [INFO] SIMULATION: the arm executes; pole and hand "
                  "stages are ASSUMED successful (the controller models "
                  "neither — F3)")
        ok = StageRunner(cfg, arm, monitor, plan, sim=sim_mode).run()
        return 0 if ok else 1
    finally:
        restore_run_modes(originals)
        teardown(arm)
        print(f"\n  Summary: {_results['PASS']} PASS, {_results['FAIL']} "
              f"FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
