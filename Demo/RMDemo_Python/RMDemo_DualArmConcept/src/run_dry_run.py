"""Offline verification of the dual-arm concept logic. NO HARDWARE, NO MOTION.

Exercises the real conversion helpers, ArrivalMonitor demux, and all three
mode runners (locked / chained / free) against mock robots whose arrival
events fire asynchronously through the real monitor — verifying the
concurrency logic, ordering invariants, and failure handling before any
hardware run. Run this first.
"""

import math
import sys
import threading
import time

from log_utils import setup_log as _setup_log
_log_path = _setup_log(__file__)

import os as _os
import pathlib as _pathlib
_os.environ.setdefault("RM_HAND_DWELL_S", "0.05")
# the canfd stream runs in real wall time, unlike the mocks
_os.environ.setdefault("RM_CANFD_ARM_S", "0.05")
import dual_arm_common as dac

_checks = {"run": 0, "fail": 0}


def check(name: str, cond: bool, detail: str = ""):
    _checks["run"] += 1
    if cond:
        print(f"  [PASS] {name}" + (f"  ({detail})" if detail else ""))
    else:
        _checks["fail"] += 1
        print(f"  [FAIL] {name}" + (f"  ({detail})" if detail else ""))


class _FakeEventData:
    def __init__(self, handle_id, device, ok=True, connect=0, event_type=1):
        self.handle_id = handle_id
        self.event_type = event_type
        self.trajectory_state = ok
        self.device = device
        self.trajectory_connect = connect
        self.program_id = 0


class MockRobot:
    """Fires an arrival event through the monitor after a short delay."""

    def __init__(self, handle_id, monitor, arm_s=0.05, lift_s=0.03,
                 fail_at=None):
        self.handle_id = handle_id
        self.monitor = monitor
        self.arm_s = arm_s
        self.lift_s = lift_s
        self.fail_at = fail_at            # step counter that returns ret != 0
        self.dispatched = 0
        self.stopped = False
        self.joints = [0.0] * 7           # tracked so verify_device works

    def _arrive(self, device, delay):
        threading.Timer(
            delay, self.monitor._on_event,
            args=(_FakeEventData(self.handle_id, device),)).start()

    def rm_movej(self, joint, v, r, connect, block):
        self.dispatched += 1
        if self.fail_at is not None and self.dispatched == self.fail_at:
            return 1
        self.joints = list(joint)
        self._arrive(dac.DEV_JOINT, self.arm_s)
        return 0

    def rm_set_lift_height(self, speed, hw_mm, block):
        self.dispatched += 1
        if self.fail_at is not None and self.dispatched == self.fail_at:
            return 1
        self._arrive(dac.DEV_LIFT, self.lift_s)
        return 0

    def rm_get_current_arm_state(self):
        return 0, {"joint": list(self.joints)}

    def rm_movej_canfd(self, joint, follow, expand=0, trajectory_mode=0,
                       radio=0):
        # Passthrough: the setpoint becomes the state, and NO arrival event
        # is emitted (that is the point — completion is the stream ending).
        self.joints = list(joint)
        return 0

    def rm_set_hand_angle(self, hand_angle, block=True, timeout=10):
        self.dispatched += 1
        if self.fail_at is not None and self.dispatched == self.fail_at:
            return 1
        if block:
            time.sleep(self.arm_s)      # blocking path: SDK-internal wait
            return 0
        self._arrive(dac.DEV_HAND, self.arm_s)
        return 0

    def rm_get_lift_state(self):
        return 0, {"pos": 0, "err_flag": 0}

    def rm_set_arm_stop(self):
        self.stopped = True
        return 0

    def rm_set_lift_speed(self, speed):
        return 0

    def rm_get_self_collision_enable(self):
        return 0, True                 # already on: no noise in the log

    def rm_set_self_collision_enable(self, enable):
        return 0

    def rm_get_arm_software_info(self):
        return 0, {"ctrl_info": {"version": "V1.7.4"}}


class _Handle:
    def __init__(self, hid):
        self.id = hid


def make_pair(monitor, **kw_left):
    left = dac.ConceptArm("left", MockRobot(1, monitor, **kw_left), _Handle(1))
    right = dac.ConceptArm("right", MockRobot(2, monitor), _Handle(2))
    return left, right


def main() -> int:
    dac.handle_cli(__doc__)
    print("=" * 68)
    print("Dual-arm concept — offline dry run (no hardware, no motion)")
    print("=" * 68)

    # ── A. Conversions ──────────────────────────────────────────────────
    print("\nA. Unit conversions")
    lr = dac.state_deg("left", "ready")
    check("left ready J2 rad->deg", abs(lr[1] - (-108.0028)) < 0.01,
          f"{lr[1]:.4f}")
    check("left ready J7 rad->deg", abs(lr[6] - 180.0004) < 0.01,
          f"{lr[6]:.4f}")
    rr = dac.state_deg("right", "rest")
    check("right rest J3 rad->deg", abs(rr[2] - (-26.0009)) < 0.01,
          f"{rr[2]:.4f}")
    check("right zero all 0", all(v == 0.0 for v in dac.state_deg("right", "zero")))
    # Both poles: 1:1 true mm, controller travel 315 mm (2026-08-07)
    right_hw = {"minimum": 5, "quarter": 75, "half": 150, "full": 300}
    left_hw = {"minimum": 5, "quarter": 75, "half": 150, "full": 300}
    for name, hw in right_hw.items():
        check(f"lift {name} -> {hw} (right, 1to1)",
              dac.lift_hw_mm("right", dac.LIFT_M[name]) == hw)
    for name, hw in left_hw.items():
        check(f"lift {name} -> {hw} (left, 1to1)",
              dac.lift_hw_mm("left", dac.LIFT_M[name]) == hw)
    check("full 300 leaves headroom under the 315 mm ceiling",
          left_hw["full"] < dac.LIFT_GEAR["left"]["hw_max"] == 315
          and right_hw["full"] < dac.LIFT_GEAR["right"]["hw_max"])
    for bad in (0.0, 0.31, -0.1):
        try:
            dac.lift_hw_mm("left", bad)
            check(f"lift range guard rejects {bad}", False)
        except AssertionError:
            check(f"lift range guard rejects {bad}", True)

    # ── B. Sequence integrity ───────────────────────────────────────────
    print("\nB. Concept sequence")
    seq = dac.CONCEPT_SEQUENCE
    check("7 steps", len(seq) == 7)
    check("first step homes to ready", seq[0] == ("arm", "ready"))

    def target_ok(kind, name):
        if kind == "arm":
            return name in dac.STATES_RAD["left"] and name in dac.STATES_RAD["right"]
        if kind == "lift":
            return name in dac.LIFT_M
        if kind == "hand":
            return name in dac.HAND_STATES_HW
        if kind == "sync":
            return name[0] in dac.STATES_RAD["left"] and name[1] in dac.LIFT_M
        return False
    check("all targets defined for both sides",
          all(target_ok(k, n) for k, n in seq))
    check("sequence includes sync steps",
          sum(1 for k, _ in seq if k == "sync") == 2)
    check("sequence includes hand steps",
          sum(1 for k, _ in seq if k == "hand") == 3)

    # Hand values: cross-checked against bench §3.8 ANGLE_SET echo
    check("hand grasp = 33/33/33/33/133/944 (bench echo)",
          dac.HAND_STATES_HW["grasp"] == [33, 33, 33, 33, 133, 944])
    check("hand values in range",
          all(0 <= v <= 1000 for vals in dac.HAND_STATES_HW.values()
              for v in vals))

    # Sync duration matching — polarity INVERTED 2026-08-07: the pole must
    # OUTLAST the arm move (a pole completing mid-trajectory faults the
    # moving joints), so quantization rounds DOWN and targets
    # arm_duration * SYNC_POLE_OUTLAST.
    check("matched lift speed: 105 phys mm over 4 s arm move -> 19%",
          dac.matched_lift_speed_pct(4.0, 105.0) == 19)
    _pct = dac.matched_lift_speed_pct(4.0, 105.0)
    _pole_s = dac.lift_travel_time_s(105.0, _pct)
    check("matched pole is predicted to finish AFTER the arm",
          _pole_s > 4.0, f"pole {_pole_s:.2f} s vs arm 4.00 s")
    check("matched speed is the SLOWEST that still fits the target",
          dac.lift_travel_time_s(105.0, _pct) <= 4.0 * dac.SYNC_POLE_OUTLAST
          < dac.lift_travel_time_s(105.0, _pct - 1))

    # Trapezoidal model vs the 2026-08-07 arm-idle hardware measurements
    for dist, pct, asc, meas, tol in ((140.0, 50, True, 3.47, 0.15),
                                      (10.0, 50, False, 0.75, 0.25)):
        pred = dac.lift_travel_time_s(dist, pct, asc)
        check(f"pole model: {dist:.0f} mm @ {pct}% ~ {meas} s measured",
              abs(pred - meas) / meas <= tol, f"predicts {pred:.2f} s")
    # Acceleration-limited regime: once the stroke is too short to reach
    # the commanded speed, commanding faster changes nothing (bench §3.5).
    check("short strokes are acceleration-limited (commanding faster is a no-op)",
          abs(dac.lift_travel_time_s(20.0, 80)
              - dac.lift_travel_time_s(20.0, 100)) < 1e-9)
    check("long strokes DO respond to speed",
          dac.lift_travel_time_s(280.0, 100) < dac.lift_travel_time_s(280.0, 40))
    check("matched lift speed floors at 4%",
          dac.matched_lift_speed_pct(30.0, 1.0) == 4)
    check("matched lift speed caps at 100%",
          dac.matched_lift_speed_pct(0.5, 300.0) == 100)

    # ── C. ArrivalMonitor demux ─────────────────────────────────────────
    print("\nC. ArrivalMonitor demux")
    mon = dac.ArrivalMonitor.__new__(dac.ArrivalMonitor)
    mon._lock = threading.Lock()
    mon._waiters = {}
    mon._registered = False
    mon.expect(1, dac.DEV_JOINT)
    mon._on_event(_FakeEventData(2, dac.DEV_JOINT))          # wrong handle
    arrived, _ = mon.wait(1, dac.DEV_JOINT, 0.01)
    check("wrong handle ignored", not arrived)
    mon._on_event(_FakeEventData(1, dac.DEV_JOINT, event_type=0))
    arrived, _ = mon.wait(1, dac.DEV_JOINT, 0.01)
    check("event_type 0 ignored", not arrived)
    mon._on_event(_FakeEventData(1, dac.DEV_JOINT, connect=1))
    arrived, _ = mon.wait(1, dac.DEV_JOINT, 0.01)
    check("trajectory_connect=1 does not complete", not arrived)
    mon._on_event(_FakeEventData(1, dac.DEV_JOINT))
    arrived, success = mon.wait(1, dac.DEV_JOINT, 0.5)
    check("matching event completes with success", arrived and success)
    mon.expect(1, dac.DEV_LIFT)
    mon._on_event(_FakeEventData(1, dac.DEV_LIFT, ok=False))
    arrived, success = mon.wait(1, dac.DEV_LIFT, 0.5)
    check("failed trajectory reported", arrived and not success)

    def fresh_monitor():
        m = dac.ArrivalMonitor.__new__(dac.ArrivalMonitor)
        m._lock = threading.Lock()
        m._waiters = {}
        m._registered = False
        return m

    # ── D. Locked mode ──────────────────────────────────────────────────
    print("\nD. Locked mode (mock)")
    mon = fresh_monitor()
    left, right = make_pair(mon)
    t0 = time.perf_counter()
    rep = dac.run_locked(left, right, mon)
    dur = time.perf_counter() - t0
    check("locked completes ok", rep["ok"])
    check("locked ran all steps", len(rep["steps"]) == len(seq))
    check("locked arrivals confirmed (events / acked hand)",
          all(d["event"] or d.get("acked")
              for e in rep["steps"] for rec in (e["left"], e["right"])
              for d in rec["devices"].values()))
    max_skew = max(e["skew_ms"] for e in rep["steps"])
    check("locked dispatch skew < 20 ms (mock)", max_skew < 20.0,
          f"max {max_skew:.2f} ms")
    barrier_ok = all(
        rep["steps"][k + 1]["left"]["t_dispatch"]
        >= max(rep["steps"][k]["left"]["t_done"], rep["steps"][k]["right"]["t_done"])
        for k in range(len(rep["steps"]) - 1))
    check("locked barrier: step k+1 after both k done", barrier_ok)
    check("locked wall time plausible", dur < 5.0, f"{dur:.2f} s")

    mon = fresh_monitor()
    left, right = make_pair(mon, fail_at=2)     # left's 2nd dispatch rejected
    rep = dac.run_locked(left, right, mon)
    check("locked stops on dispatch failure",
          not rep["ok"] and len(rep["steps"]) == 2)
    check("locked partner halted on failure",
          left.robot.stopped and right.robot.stopped)

    # ── E. Chained mode ─────────────────────────────────────────────────
    print("\nE. Chained mode (mock)")
    mon = fresh_monitor()
    left, right = make_pair(mon, arm_s=0.02, lift_s=0.02)
    rep = dac.run_chained(left, right, mon)
    check("chained completes ok", rep["ok"])
    order_ok = all(
        rep["follower"][k]["t_dispatch"] >= rep["leader"][k]["t_done"]
        for k in range(len(seq)))
    check("chained ordering: follower k after leader k done", order_ok)
    pipelined = any(
        rep["leader"][k + 1]["t_dispatch"] < rep["follower"][k]["t_done"]
        for k in range(len(seq) - 1))
    check("chained pipelines (leader k+1 overlaps follower k)", pipelined)

    # ── F. Free mode ────────────────────────────────────────────────────
    print("\nF. Free mode (mock)")
    mon = fresh_monitor()
    left, right = make_pair(mon)
    rep = dac.run_free(left, right, mon)
    check("free completes ok", rep["ok"])
    check("free ran all steps both arms",
          len(rep["left"]) == len(seq) and len(rep["right"]) == len(seq))
    mon = fresh_monitor()
    left, right = make_pair(mon, fail_at=3)
    rep = dac.run_free(left, right, mon)
    check("free stops partner on failure", not rep["ok"] and right.robot.stopped)

    # ── F1a. Combo step (concurrent arm + hand on one arm) ──────────────
    print("\nF1a. Combo step kind")
    mon = fresh_monitor()
    left, right = make_pair(mon)
    rec = dac.run_step(left, mon, ("combo", (("arm", "ready"),
                                             ("hand", "release"))))
    check("combo arm+hand: joint event + hand acked, step ok",
          rec["ok"] and rec["devices"][dac.DEV_JOINT]["event"]
          and rec["devices"][dac.DEV_HAND]["acked"]
          and set(rec["devices"]) == {dac.DEV_JOINT, dac.DEV_HAND})
    check("--no-hands strips hand parts",
          dac.strip_hands(dac.CONCEPT_SEQUENCE) == [
              ("arm", "ready"), ("sync", ("zero", "half")),
              ("sync", ("ready", "full")), ("arm", "rest")]
          and dac.strip_hands([("combo", (("arm", "a"), ("hand", "h")))])
          == [("arm", "a")])
    check("--no-pole strips lift parts, sync keeps its arm half",
          dac.strip_poles(dac.CONCEPT_SEQUENCE) == [
              ("arm", "ready"), ("hand", "release"), ("arm", "zero"),
              ("hand", "grasp"), ("arm", "ready"), ("hand", "half_grasp"),
              ("arm", "rest")]
          and dac.strip_poles([("lift", "full"),
                               ("combo", (("arm", "a"), ("lift", "l")))])
          == [("arm", "a")])
    check("--no-hands + --no-pole leaves arm-only choreography",
          dac.strip_poles(dac.strip_hands(dac.CONCEPT_SEQUENCE)) == [
              ("arm", "ready"), ("arm", "zero"),
              ("arm", "ready"), ("arm", "rest")])

    # ── F1d. Sync dispatch ORDER (the 2026-08-07 arm-freeze fix) ────────
    print("\nF1d. Sync dispatch order")
    mon = fresh_monitor()
    left, _ = make_pair(mon)
    saved_order = dac.SYNC_ORDER
    try:
        dac.SYNC_ORDER = "lift_first"
        check("default order dispatches the LIFT first",
              [d for d, _ in left.parts_for(("sync", ("zero", "half")))]
              == [dac.DEV_LIFT, dac.DEV_JOINT])
        dac.SYNC_ORDER = "arm_first"
        check("arm_first restores the old order for A/B tests",
              [d for d, _ in left.parts_for(("sync", ("zero", "half")))]
              == [dac.DEV_JOINT, dac.DEV_LIFT])
    finally:
        dac.SYNC_ORDER = saved_order
    check("shipped default is lift_first", saved_order == "lift_first")
    check("non-sync steps are unaffected by the order",
          left.parts_for(("arm", "ready")) == [(dac.DEV_JOINT, "ready")]
          and left.parts_for(("lift", "full")) == [(dac.DEV_LIFT, "full")])

    # ── F1f. Firmware-detected lift gearing (right-arm upgrade safety) ──
    print("\nF1f. Lift gearing auto-detection")

    class _VerRobot:
        def __init__(self, version):
            self.version = version

        def rm_get_arm_software_info(self):
            if self.version is None:
                return 1, {}
            return 0, {"ctrl_info": {"version": self.version}}

    check("version parser handles V-prefix and suffixes",
          dac._version_tuple("V1.7.4-emu") == (1, 7, 4)
          and dac._version_tuple("1.7.1") == (1, 7, 1)
          and dac._version_tuple("") == ())
    check("V1.7.4 -> true-mm 1to1",
          dac.detect_lift_gear(_VerRobot("V1.7.4")) == "1to1")
    check("V1.7.1 -> geared 2to3",
          dac.detect_lift_gear(_VerRobot("1.7.1")) == "2to3")
    check("unreadable version -> no verdict",
          dac.detect_lift_gear(_VerRobot(None)) == "")

    _saved = dict(dac.LIFT_GEAR)
    try:
        # The right-arm upgrade: detection must flip it without any edit.
        dac.LIFT_GEAR["right"] = dict(dac._GEARS["2to3"], name="2to3")
        dac.apply_detected_lift_gear("right", _VerRobot("V1.7.4"))
        check("upgraded right arm auto-switches to 1to1 (full -> 300)",
              dac.LIFT_GEAR["right"]["name"] == "1to1"
              and dac.lift_hw_mm("right", dac.LIFT_M["full"]) == 300)
        # An unreadable version must NOT silently change the assumption.
        dac.LIFT_GEAR["right"] = dict(dac._GEARS["2to3"], name="2to3")
        dac.apply_detected_lift_gear("right", _VerRobot(None))
        check("unreadable version keeps the assumed gearing",
              dac.LIFT_GEAR["right"]["name"] == "2to3")
        # An explicit env pin outranks detection.
        _os.environ["RM_RIGHT_LIFT_GEAR"] = "2to3"
        try:
            dac.apply_detected_lift_gear("right", _VerRobot("V1.7.4"))
            check("env pin outranks detection",
                  dac.LIFT_GEAR["right"]["name"] == "2to3")
        finally:
            _os.environ.pop("RM_RIGHT_LIFT_GEAR", None)
    finally:
        dac.LIFT_GEAR.update(_saved)

    # ── F1g. Self-collision detection is turned ON at connect ───────────
    print("\nF1g. Self-collision default")

    class _ScRobot:
        def __init__(self, start=False, readable=True):
            self.enabled, self.readable, self.sets = start, readable, 0

        def rm_get_self_collision_enable(self):
            return (0, self.enabled) if self.readable else (-1, False)

        def rm_set_self_collision_enable(self, enable):
            self.enabled, self.sets = bool(enable), self.sets + 1
            return 0

    # minimal arm-like objects: ConceptArm.__init__ would re-enter this
    off = _ScRobot(start=False)

    class _A:
        side, robot = "left", off
    dac.ensure_self_collision_enabled(_A)
    check("self-collision is ENABLED when found off",
          off.enabled and off.sets == 1)
    on = _ScRobot(start=True)

    class _B:
        side, robot = "left", on
    dac.ensure_self_collision_enabled(_B)
    check("already-enabled arm is left alone (no redundant write)",
          on.enabled and on.sets == 0)
    _os.environ["RM_SELF_COLLISION"] = "0"
    try:
        opt = _ScRobot(start=False)

        class _C:
            side, robot = "left", opt
        dac.ensure_self_collision_enabled(_C)
        check("RM_SELF_COLLISION=0 leaves the arm as-found",
              not opt.enabled and opt.sets == 0)
    finally:
        _os.environ.pop("RM_SELF_COLLISION", None)

    # ── F1e. Latched-error gate (a trajectory abort faults the joints) ──
    print("\nF1e. Latched-error gate")

    class _ErrRobot:
        """Minimal robot exposing the three error surfaces."""

        def __init__(self, sys_codes=(), joints=(0,) * 7, lift_err=0):
            self.sys_codes, self.joints = list(sys_codes), list(joints)
            self.lift_err, self.cleared = lift_err, 0

        def rm_get_current_arm_state(self):
            return 0, {"joint": [0.0] * 7,
                       "err": {"err_len": max(1, len(self.sys_codes)),
                               "err": self.sys_codes or ["0"]}}

        def rm_get_joint_err_flag(self):
            return {"return_code": 0, "err_flag": self.joints,
                    "brake_state": [0] * 7}

        def rm_get_lift_state(self):
            return 0, {"pos": 0, "err_flag": self.lift_err, "mode": 0}

        def rm_clear_system_err(self):
            # faithful to hardware: does NOT clear per-joint flags (R8)
            self.cleared += 1
            self.sys_codes, self.lift_err = [], 0
            return 0

        def rm_set_joint_clear_err(self, joint_num):
            self.joint_clears = getattr(self, "joint_clears", 0) + 1
            self.joints[int(joint_num) - 1] = 0
            return 0

    clean_arm = dac.ConceptArm("left", _ErrRobot(), _Handle(1))
    check("clean controller reads clean",
          dac.error_state_clean(dac.error_state(clean_arm))
          and dac.describe_error_state(dac.error_state(clean_arm)) == "clean")
    check("clean controller passes the gate",
          dac.preflight_error_gate(clean_arm)[0])
    faulted = _ErrRobot(sys_codes=["5001"], joints=[16] * 7)
    bad_arm = dac.ConceptArm("left", faulted, _Handle(1))
    ok_gate, detail = dac.preflight_error_gate(bad_arm)
    check("latched joint errors BLOCK the run before motion",
          not ok_gate and "J1=16" in detail and "--clear-errors" in detail)
    ok_gate, _ = dac.preflight_error_gate(bad_arm, clear=True)
    check("--clear-errors clears and then passes",
          ok_gate and faulted.cleared == 1)
    check("R8: flagged joints cleared PER JOINT (system clear is not enough)",
          getattr(faulted, "joint_clears", 0) == 7
          and not any(faulted.joints))
    check("lift driver error alone also blocks",
          not dac.preflight_error_gate(
              dac.ConceptArm("left", _ErrRobot(lift_err=1), _Handle(1)))[0])

    # ── F1b. Pole pre-positioning helper ────────────────────────────────
    print("\nF1b. Pole homing to full length")
    mon = fresh_monitor()
    left, right = make_pair(mon)
    check("pole homing completes on both arms",
          dac.home_poles_full(mon, left, right))
    mon = fresh_monitor()
    left, right = make_pair(mon, fail_at=1)   # left's homing dispatch rejected
    check("pole homing failure halts the arms",
          not dac.home_poles_full(mon, left, right) and left.robot.stopped)

    # ── F1c. CLI guard (-h/--help and unknown-arg rejection) ────────────
    print("\nF1c. CLI guard")
    for flags, code, name in ((["-h"], 0, "-h exits 0 with docs"),
                              (["--help"], 0, "--help exits 0 with docs"),
                              (["--typo"], 2, "unknown flag exits 2"),
                              (["SIM"], 2, "stray positional exits 2")):
        try:
            dac.handle_cli("doc", flags)
            check(name, False)
        except SystemExit as e:
            check(name, e.code == code, f"exit {e.code}")
    try:
        dac.handle_cli("doc", ["--mode", "SIM", "--no-hands", "--no-pole"])
        check("valid flags pass through", True)
    except SystemExit:
        check("valid flags pass through", False)
    try:
        dac.handle_cli("doc", ["--diagnose-only", "--clear-errors"],
                       extra_flags=("--diagnose-only", "--clear-errors"))
        check("registered extra flags pass through (C8)", True)
    except SystemExit:
        check("registered extra flags pass through (C8)", False)
    try:
        dac.handle_cli("doc", ["--diagnose-only"])
        check("unregistered extra flag still exits 2", False)
    except SystemExit as e:
        check("unregistered extra flag still exits 2", e.code == 2)

    # ── F2. --mode argument parser ──────────────────────────────────────
    print("\nF2. --mode SIM|REAL parser")
    check("--mode SIM -> 0", dac.parse_mode_arg(["--mode", "SIM"]) == 0)
    check("--mode=real -> 1", dac.parse_mode_arg(["--mode=real"]) == 1)
    check("absent -> None", dac.parse_mode_arg([]) is None)
    try:
        dac.parse_mode_arg(["--mode", "banana"])
        check("invalid value exits with usage", False)
    except SystemExit:
        check("invalid value exits with usage", True)

    # ── G. Endpoint configuration plumbing ──────────────────────────────
    print("\nG. Endpoint configuration (env overrides)")
    import json
    import os
    import pathlib
    import subprocess

    probe = (
        "import json, dual_arm_common as dac, rm_emulator, "
        "test_sim_motion_visibility as c5; "
        "print(json.dumps({'left': dac.LEFT_IP, 'right': dac.RIGHT_IP, "
        "'port': dac.ROBOT_PORT, 'host': dac.HOST_IP, 'udp': dac.UDP_PORT, "
        "'c5host': c5.HOST_IP, 'c5udp': c5.UDP_PORT, "
        "'route': dac.host_ip_for(dac.LEFT_IP), "
        "'emul': rm_emulator.EMU_LEFT_IP, 'emur': rm_emulator.EMU_RIGHT_IP, "
        "'emuhosts': sorted(rm_emulator.EMU_HOST_IPS), 'r_full': dac.lift_hw_mm('right', 0.30), 'l_full': dac.lift_hw_mm('left', 0.30)}))"
    )

    def run_probe(extra_env):
        # Fresh interpreter: env is read at import time, so overrides must
        # be probed in a subprocess with a controlled environment.
        env = {k: v for k, v in os.environ.items()
               if not k.startswith("RM_")}
        env.update(extra_env)
        out = subprocess.run(
            [sys.executable, "-c", probe], env=env, capture_output=True,
            text=True, cwd=str(pathlib.Path(__file__).resolve().parent))
        return json.loads(out.stdout.strip().splitlines()[-1])

    try:
        cfg = run_probe({})
        check("defaults apply when env unset",
              cfg["left"] == "192.168.1.10" and cfg["right"] == "192.168.1.103"
              and cfg["port"] == 8080 and cfg["host"] == "192.168.1.239"
              and cfg["udp"] == 8095)
        # C5's push target is RESOLVED from the route to the arm, not
        # copied from the HOST_IP constant — so it must track host_ip_for.
        check("C5 resolves its push target from the route",
              cfg["c5host"] == cfg["route"] and cfg["c5udp"] == cfg["udp"])
        check("emulator defaults match the tests",
              cfg["emul"] == cfg["left"] and cfg["emur"] == cfg["right"]
              and cfg["host"] in cfg["emuhosts"]
              and cfg["route"] in cfg["emuhosts"])

        ov = run_probe({"RM_LEFT_IP": "10.9.9.1", "RM_RIGHT_IP": "10.9.9.2",
                        "RM_ROBOT_PORT": "9080", "RM_HOST_IP": "10.9.9.100",
                        "RM_UDP_PORT": "9002"})
        check("env overrides reach dual_arm_common",
              ov["left"] == "10.9.9.1" and ov["right"] == "10.9.9.2"
              and ov["port"] == 9080 and ov["host"] == "10.9.9.100"
              and ov["udp"] == 9002)
        check("env overrides reach C5 (RM_HOST_IP still pins it)",
              ov["c5host"] == "10.9.9.100" and ov["c5udp"] == 9002)
        check("env overrides reach the emulator",
              ov["emul"] == "10.9.9.1" and ov["emur"] == "10.9.9.2"
              # superset: the emulator also accepts whatever address
              # actually routes to the arms, matching host_ip_for
              and "10.9.9.100" in ov["emuhosts"])
        check("default gearing: both 1to1, full -> 300",
              cfg["l_full"] == 300 and cfg["r_full"] == 300)
        gv = run_probe({"RM_RIGHT_LIFT_GEAR": "2to3"})
        check("RM_RIGHT_LIFT_GEAR=2to3 pins a pre-upgrade controller",
              gv["r_full"] == 200)
        import subprocess as _sp
        env_l = dict(_os.environ, RM_SYNC_BACKEND="planned")
        env_l.pop("RM_UNLOCK_PLANNED_SYNC", None)
        r = _sp.run([sys.executable, "-c", "import dual_arm_common"],
                    env=env_l, capture_output=True, text=True)
        check("RM_SYNC_BACKEND=planned is LOCKED (vendor defect)",
              r.returncode != 0 and "LOCKED" in (r.stderr + r.stdout))
        env_u = dict(env_l, RM_UNLOCK_PLANNED_SYNC="1")
        r = _sp.run([sys.executable, "-c", "import dual_arm_common"],
                    env=env_u, capture_output=True, text=True)
        check("RM_UNLOCK_PLANNED_SYNC=1 re-opens it for post-fix re-tests",
              r.returncode == 0)
    except Exception as exc:
        check("configuration probe subprocess", False, repr(exc))

    # ── F1h. C11 residual math (rehearsal validator, pure geometry) ─────
    print("\nF1h. C11 rehearsal residual")
    import test_rehearsal_validate as trv

    st = [{"name": "A", "targets": [[0.0] * 7, [10.0] + [0.0] * 6]},
          {"name": "B", "targets": [[10.0] + [0.0] * 6,
                                    [10.0, 10.0] + [0.0] * 5]}]
    path, owner = trv.predicted_path(st, per_segment=25)
    check("predicted_path labels every point with its stage",
          len(path) == len(owner) == 50 and set(owner) == {0, 1})

    # A point ON a stage-0 segment attributes to stage 0 at zero distance.
    d0, _, _, s0 = trv.attribute([[5.0] + [0.0] * 6], path, owner)[0]
    check("on-path sample: zero deviation, correct stage",
          d0 < 1e-9 and s0 == 0)
    # A known perpendicular offset is measured exactly.
    d1, _, _, s1 = trv.attribute([[5.0, 3.0] + [0.0] * 5], path, owner)[0]
    check("off-path sample: deviation measured exactly",
          abs(d1 - 3.0) < 1e-9 and s1 == 0)

    # The bug this locks in: a capture dominated by ONE stage (the cleaning
    # stroke is 2002 of 2033 waypoints, and the slowest in wall time) must
    # still yield samples for the short stages — attribution happens over
    # every frame, and balancing is per stage, not global.
    cap = [[1.0] + [0.0] * 6] + [[10.0, y * 0.05] + [0.0] * 5
                                 for y in range(200)]
    dev = trv.attribute(cap, path, owner)
    per = {k: sum(1 for d in dev if d[3] == k) for k in (0, 1)}
    check("lopsided capture still attributes to every stage",
          per[0] >= 1 and per[1] >= 1)

    # ── F1k. Controller tool-frame names must track the URDF ────────────
    print("\nF1k. IK frame naming (controller <-> URDF)")
    import frame_alignment_offline as fao

    for side, pref in (("right", "R_"), ("left", "L_")):
        links = list(fao.IK_FRAMES[side])
        check(f"{side}: all 6 ik frames present, correctly prefixed",
              len(links) == 6 and all(ln.startswith(pref) for ln in links))
        names = [fao.controller_frame_name(ln) for ln in links]
        # c_char_Array_12 = 11 usable chars; a longer name would be
        # truncated by the SDK and two frames could silently collide.
        check(f"{side}: every controller name fits in 11 chars",
              all(len(n) <= fao.FRAME_NAME_MAX for n in names),
              f"longest {max(names, key=len)!r}")
        check(f"{side}: names are unique after mapping",
              len(set(names)) == len(names))
        check(f"{side}: mapping is the documented rule",
              all(n == ln.replace("_frame", "")
                  for ln, n in zip(links, names)))
    _raised = False
    try:
        fao.controller_frame_name("R_a_very_long_frame_name_here")
    except ValueError:
        _raised = True
    check("an over-long name RAISES rather than truncating", _raised)

    # The map is the contract between MoveIt's ik_frame and the controller
    # tool frame. It must be complete, consistent, and single-sourced.
    for side in ("right", "left"):
        rows = fao.frame_map(side)
        check(f"{side}: frame map covers every ik frame",
              len(rows) == len(fao.IK_FRAMES[side]))
        check(f"{side}: Arm_Tip offset = ConnectorLink offset + 15.3 mm Z",
              all(abs(tip[2] - conn[2] - fao.ARM_TIP_TO_CONNECTOR_M) < 1e-9
                  and tip[0] == conn[0] and tip[1] == conn[1]
                  for _l, _n, conn, tip, _r in rows))
    # The hinge task's ik_frame must be IN the map, or the whole exercise
    # misses its target: cleaning points name R_glove_frame_4 explicitly.
    check("the hinge task's ik_frame R_glove_frame_4 is mapped",
          "R_glove_frame_4" in fao.IK_FRAMES["right"]
          and fao.controller_frame_name("R_glove_frame_4") == "R_glove_4")
    # One residual constant, imported — never re-declared in the writer.
    _src = (_pathlib.Path(__file__).resolve().parent
            / "test_frame_alignment.py").read_text()
    check("the frame writer imports the residual, not a copy of it",
          "ARM_TIP_TO_CONNECTOR_M" in _src and "0.0153" not in _src)

    # ── F1j. The C11 capture runs where there is NO ROS workspace ───────
    print("\nF1j. Plan resolution (lab laptop has no workspace)")
    import segment_verifier as sv

    bundled = sv.BUNDLED_PLANS / "hinge_area_right_ruckig_pro_only.json"
    check("plan is bundled in the repo", bundled.exists(),
          f"{bundled.stat().st_size // 1024} KB" if bundled.exists()
          else "MISSING — the lab machine cannot capture")
    ws_plan = (sv.WS / "Resource" / "plans" / "commode_c" / "hardware"
               / "hinge_area_right_ruckig_pro_only.json")
    got = sv.resolve_plan("hinge_area_right_ruckig_pro_only.json")
    check("workspace copy wins when present" if ws_plan.exists()
          else "falls back to the bundled copy",
          got == (ws_plan if ws_plan.exists() else bundled))
    # The capture half must build its targets with NO workspace at all.
    _saved_ws = sv.WS
    try:
        sv.WS = _pathlib.Path("/nonexistent/no/workspace")
        check("resolver falls back when the workspace is absent",
              sv.resolve_plan("hinge_area_right_ruckig_pro_only.json")
              == bundled)
    finally:
        sv.WS = _saved_ws
    plan = sv.load_plan(bundled)
    stages = sv.arm_stages(plan, prefix="R_joint")
    check("bundled plan yields the 4 arm stages",
          [s["stage_name"] for s in stages]
          == ["move_to_pre_start", "move_to_start", "execute_path",
              "move_to_rest"])
    check("bundled plan waypoints are radians, 7 joints",
          len(stages[0]["joint_names"]) == 7
          and all(abs(v) < 7 for v in stages[0]["waypoints"][0]["positions"]))

    # ── F1i. UDP push target is resolved, not guessed ───────────────────
    print("\nF1i. UDP push target resolution")
    _os.environ.pop("RM_HOST_IP", None)
    auto = dac.host_ip_for(dac.LEFT_IP)
    # A wrong push target is accepted by the controller with ret=0 and
    # silently delivers nothing, so this must never quietly yield loopback.
    check("route lookup yields a real local address",
          isinstance(auto, str) and auto.count(".") == 3
          and not auto.startswith("127."))
    _os.environ["RM_HOST_IP"] = "10.9.9.99"
    try:
        check("RM_HOST_IP pins the push target",
              dac.host_ip_for(dac.LEFT_IP) == "10.9.9.99")
    finally:
        _os.environ.pop("RM_HOST_IP", None)
    check("unroutable arm still yields an address (falls back)",
          bool(dac.host_ip_for("203.0.113.7")))

    n, f = _checks["run"], _checks["fail"]
    print("\n" + "=" * 68)
    print(f"Dry run: {n - f}/{n} passed")
    print("=" * 68)
    return 0 if f == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
