"""Offline verification of the dual-arm concept logic. NO HARDWARE, NO MOTION.

Exercises the real conversion helpers, ArrivalMonitor demux, and all three
mode runners (locked / chained / free) against mock robots whose arrival
events fire asynchronously through the real monitor — verifying the
concurrency logic, ordering invariants, and failure handling before any
hardware run. Run this first.
"""

import math
import sys
import tempfile
import threading
import time

import log_utils

from log_utils import setup_log as _setup_log
_log_path = _setup_log(__file__)

import os as _os
import pathlib as _pathlib
import json as _json
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

    # ── F1b2. Payload mirror law + consensus ────────────────────────────
    print("\nF1b2. Payload centroid: consensus and the mirror law")
    import payload_audit as pa
    # The real calibrated table, 2026-08-10. Four frames agree, two do not.
    LEFT = {
        "L_glove_1": {"payload": 0.627, "com": (-9.0, 32.0, 188.0)},
        "L_glove_2": {"payload": 0.568, "com": (-24.0, 42.0, 222.0)},
        "L_glove_3": {"payload": 0.565, "com": (-25.0, 41.0, 224.0)},
        "L_glove_4": {"payload": 0.646, "com": (-11.0, 27.0, 177.0)},
        "L_tip": {"payload": 0.569, "com": (-25.0, 40.0, 224.0)},
        "L_index_tip": {"payload": 0.566, "com": (-27.0, 42.0, 226.0)},
    }
    cons, off, _ = pa.consensus(LEFT)
    com, kg, n_in, n_all = cons
    check("consensus uses only the agreeing frames", n_in == 4 and n_all == 6,
          f"{n_in} of {n_all}")
    check("consensus CoM is the cluster, not the mean",
          abs(com[0] + 25.2) < 0.3 and abs(com[1] - 41.2) < 0.3
          and abs(com[2] - 224.0) < 0.3,
          f"({com[0]:.1f}, {com[1]:.1f}, {com[2]:.1f})")
    check("consensus mass", abs(kg - 0.567) < 0.002, f"{kg:.3f} kg")
    check("both outliers are named",
          {n for n, _d, _k in off} == {"L_glove_1", "L_glove_4"},
          str(sorted(n for n, _d, _k in off)))
    # mirror() is a correct geometric operation and stays as a comparison,
    # but it is NOT how the right arm's payload is obtained — the measured
    # right calibration refutes it by 78.4 mm and 0.144 kg (2026-08-10).
    check("mirror negates X, keeps Y and Z",
          pa.mirror((-25.2, 41.2, 224.0)) == (25.2, 41.2, 224.0))
    check("mirror is an involution",
          pa.mirror(pa.mirror((-25.2, 41.2, 224.0))) == (-25.2, 41.2, 224.0))
    RIGHT = {
        "R_glove_1": {"payload": 0.785, "com": (-21.0, 8.0, 126.0)},
        "R_glove_2": {"payload": 0.717, "com": (-22.0, 25.0, 163.0)},
        "R_glove_3": {"payload": 0.708, "com": (-24.0, 26.0, 166.0)},
        "R_glove_4": {"payload": 0.711, "com": (-24.0, 25.0, 165.0)},
        "R_tip": {"payload": 0.712, "com": (-23.0, 25.0, 164.0)},
        "R_index_tip": {"payload": 0.707, "com": (-25.0, 26.0, 166.0)},
    }
    rc, roff, _ = pa.consensus(RIGHT)
    rcom, rkg, rn, rall = rc
    check("right consensus uses 5 of 6 frames", rn == 5 and rall == 6,
          f"{rn} of {rall}")
    check("right outlier is R_glove_1",
          {n for n, _d, _k in roff} == {"R_glove_1"},
          str(sorted(n for n, _d, _k in roff)))
    # The refutation itself, pinned: if someone reinstates the mirror as a
    # source of values, this fails.
    pred = pa.mirror(com)
    check("mirror does NOT predict the measured right arm",
          pa.dist(rcom, pred) > 50.0,
          f"{pa.dist(rcom, pred):.1f} mm off")
    check("CX agrees between arms — it does not mirror",
          abs(rcom[0] - com[0]) < 3.0,
          f"left {com[0]:.1f} vs right {rcom[0]:.1f}")
    # Flange-relative, confirmed independently on the right arm.
    check("right frames agree despite 65 mm of origin spread",
          max(pa.dist(RIGHT[n]["com"], rcom) for n in RIGHT
              if n != "R_glove_1") < 4.0)
    # A side with NOTHING declared agrees with itself perfectly; it must
    # never be chosen as the source to mirror FROM.
    ZERO = {f"R_{i}": {"payload": 0.0, "com": (0.0, 0.0, 0.0)}
            for i in range(6)}
    zc, _zoff, _ = pa.consensus(ZERO)
    check("an undeclared side reads as zero payload",
          zc is not None and zc[1] == 0.0 and max(abs(v) for v in zc[0]) == 0.0)
    # 128 metres: every value beyond the bound leaves NO usable record.
    HUGE = {f"L_{i}": {"payload": 0.706, "com": (-12000.0, 44000.0, 128000.0)}
            for i in range(6)}
    hc, _hoff, hgood = pa.consensus(HUGE)
    check("an impossible centroid yields no consensus",
          hc is None and not hgood)

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

    # allow_common=False: an OFFLINE analysis script must reject --mode
    # rather than ignore it, so `--mode REAL` can never read as "that
    # check ran against the real arm".
    for flags, allow, code, name in (
            (["--mode", "REAL"], True, None, "--mode accepted when common on"),
            (["--mode", "REAL"], False, 2, "--mode REJECTED when common off"),
            (["--no-hands"], False, 2, "--no-hands rejected when common off")):
        try:
            dac.handle_cli("doc", flags, allow_common=allow)
            check(name, code is None)
        except SystemExit as e:
            check(name, e.code == code, f"exit {e.code}")

    # A per-script usage block replaces the shared motion USAGE.
    import io
    import contextlib
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            dac.handle_cli("doc", ["--help"], usage="OFFLINE USAGE LINE")
    except SystemExit:
        pass
    check("usage= overrides the shared motion USAGE",
          "OFFLINE USAGE LINE" in buf.getvalue()
          and "--no-hands" not in buf.getvalue())

    # --help must not open the run log. Every script calls setup_log()
    # from __main__ BEFORE main() reaches handle_cli(), so an unguarded
    # setup_log appends a "Started:" banner to <script>.log and a run
    # that never happened shows up in the evidence trail.
    check("wants_help sees -h", log_utils.wants_help(["-h"]))
    check("wants_help sees --help", log_utils.wants_help(["--help"]))
    check("wants_help ignores other flags",
          not log_utils.wants_help(["--mode", "REAL"]))
    # _pathlib, not pathlib: main() has a local `import pathlib` further
    # down, which makes the bare name local to the whole function.
    probe = _pathlib.Path(tempfile.mkdtemp()) / "cli_probe.py"
    probe.write_text("x = 1\n")
    logfile = probe.parent / "cli_probe.log"
    saved_argv, saved_out, saved_err = sys.argv, sys.stdout, sys.stderr
    try:
        sys.argv = ["cli_probe.py", "--help"]
        log_utils.setup_log(str(probe))
        check("--help does NOT create the run log", not logfile.exists())
    finally:
        sys.argv, sys.stdout, sys.stderr = saved_argv, saved_out, saved_err
    try:
        sys.argv = ["cli_probe.py"]
        log_utils.setup_log(str(probe))
        sys.stdout, sys.stderr = saved_out, saved_err
        check("a real run DOES create the run log", logfile.exists())
    finally:
        sys.argv, sys.stdout, sys.stderr = saved_argv, saved_out, saved_err

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
    # The repo copy is THE plan on every machine. Preferring a workspace
    # copy is what made the lab rehearse a different plan (stroke stage
    # named execute_cleaning_path, 2012 wp, extra retreat) than the one
    # C12 had verified — both runs looking perfectly healthy.
    got = sv.resolve_plan("hinge_area_right_ruckig_pro_only.json")
    check("the repo's plans/ copy is the resolved plan", got == bundled)
    _saved_ws = sv.WS
    try:
        sv.WS = _pathlib.Path("/some/other/workspace")
        check("a workspace elsewhere does NOT change the plan",
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

    # ── F1m. C11 sampling and stage-name independence ───────────────────
    print("\nF1m. C11 capture reduction")

    # Stage names differ per task (execute_path / execute_cleaning_path /
    # square1_motion / test_motion ... — see cleaning_tasks/config), so the
    # path stage MUST be found by size. Matching on a name silently
    # collapsed a 2012-waypoint stroke to 2 targets on 2026-08-08.
    fake = {"sub_trajectories": [
        {"stage_name": "whatever_it_is_called", "num_waypoints": 100,
         "joint_names": [f"R_joint{i}" for i in range(1, 8)],
         "waypoints": [{"positions": [i * 0.001] * 7} for i in range(100)]},
        {"stage_name": "short_hop", "num_waypoints": 3,
         "joint_names": [f"R_joint{i}" for i in range(1, 8)],
         "waypoints": [{"positions": [0.2] * 7} for _ in range(3)]},
    ]}
    _fp = _pathlib.Path(_os.environ.get(
        "TMPDIR", "/tmp")) / "dryrun_fake_plan.json"
    _fp.write_text(_json.dumps(fake))
    _os.environ["RM_ARM"] = "right"
    import test_rehearsal_validate as trv2
    st = trv2.build_targets(str(_fp), 20)
    check("path stage found by SIZE, not by stage name",
          len(st[0]["targets"]) == 20 and st[0]["name"]
          == "whatever_it_is_called")
    check("a short stage stays at its two endpoints",
          len(st[1]["targets"]) == 2)

    # Arc-length resampling: the push streams while the arm is idle, so
    # index-based sampling lands in the idle clusters and misses the real
    # geometry. Here 90% of the "capture" is a single stationary pose.
    idle = [[0.0] * 7] * 90 + [[float(i)] + [0.0] * 6 for i in range(1, 11)]

    def _distinct(pts):
        return len({round(p[0], 6) for p in pts})
    # Endpoint spread is NOT the metric — subsample() always keeps first
    # and last, so it looks fine while spending 10 of 11 picks inside the
    # dwell. What matters is INTERIOR coverage: how much of the traverse
    # actually gets a sample, since that is where a close approach hides.
    check("arc-length resampling covers the traverse",
          _distinct(trv2.resample_by_arclength(idle, 11)) >= 10,
          f"{_distinct(trv2.resample_by_arclength(idle, 11))}/11 distinct")
    check("index subsampling clusters in the dwell (the bug this replaces)",
          _distinct(sv.subsample(idle, 11)) <= 4,
          f"only {_distinct(sv.subsample(idle, 11))}/11 distinct")
    check("a capture that never moved degrades safely",
          len(trv2.resample_by_arclength([[1.0] * 7] * 50, 10)) == 1)

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

    # ── P1. Hand angle -> hardware counts (calibrated, not guessed) ─────
    print("\nP1. Hand angle conversion")
    # The three states we hold in BOTH forms must round-trip exactly; the
    # fit is only trustworthy if it reproduces its own calibration points.
    for name, ang in (("release", 0.01), ("half_grasp", 0.65),
                      ("grasp", 1.3)):
        thumb = {"release": (0.01, 0.01)}.get(name, (0.0698, 0.454))
        got = dac.hand_rad_to_hw({
            "little_1": ang, "ring_1": ang, "middle_1": ang, "index_1": ang,
            "thumb_1": thumb[0], "thumb_2": thumb[1]})
        want = dac.HAND_STATES_HW[name]
        check(f"{name} rad->counts reproduces the bench values",
              all(abs(a - b) <= 1 for a, b in zip(got, want)),
              f"{got} vs {want}")
    # The bug this replaces: `open_tenth` is 1.17 rad — nearly CLOSED —
    # and an earlier version aliased it to `release`, i.e. 993 counts.
    ot = dac.hand_rad_to_hw({"little_1": 1.17, "ring_1": 1.17,
                             "middle_1": 1.17, "index_1": 1.17,
                             "thumb_1": 0.0698, "thumb_2": 0.454})
    check("open_tenth is nearly closed, NOT open",
          120 <= ot[0] <= 140, f"{ot[0]} counts (alias would give 993)")
    qg = dac.hand_rad_to_hw({"little_1": 0.336, "ring_1": 0.336,
                             "middle_1": 0.336, "index_1": 0.336,
                             "thumb_1": 0.0, "thumb_2": 0.454})
    check("quarter_grasp is mostly open, NOT a full grasp",
          740 <= qg[0] <= 760, f"{qg[0]} counts (alias would give 33)")
    check("counts are clamped into 0..1000",
          all(0 <= c <= 1000 for c in qg + ot))
    _raised = False
    try:
        dac.hand_rad_to_hw({"little_1": 0.1})       # missing joints
    except KeyError:
        _raised = True
    check("a missing finger RAISES rather than defaulting", _raised)

    # ── P2. Task config: four tasks, two arms, serialization ────────────
    print("\nP2. Task config + serialization")
    from task_config import TaskConfig
    TASKS = ("hinge_area_right", "hinge_area_left", "toplid_right",
             "toplid_left")
    cfgs = {}
    for t in TASKS:
        cfgs[t] = TaskConfig.load(t)
    check("all four tasks load", len(cfgs) == 4)
    check("side is read per task",
          [cfgs[t].side for t in TASKS] == ["right", "left", "right",
                                            "left"])
    check("ik_frame comes from the task, not the arm default",
          cfgs["hinge_area_right"].ik_frame == "R_glove_frame_4"
          and cfgs["toplid_right"].ik_frame == "R_glove_frame_2")
    check("groups resolve from arm_defaults by suffix",
          cfgs["toplid_left"].arm_group == "rm75_arm_left"
          and cfgs["toplid_left"].pole_group == "pole_left")
    # Policy (Newton, 2026-08-08): cleaning at the task's full scaling,
    # everything else at half. MoveIt's joint_limits.yaml maxima ARE the
    # controller's, so scaling 1.0 == v=100% — an earlier version scaled
    # by 20 and ran everything 5x slow.
    import task_config as _tc
    check("cleaning runs at the task's full scaling",
          all(cfgs[t].cleaning_speed_pct == 100 for t in TASKS))
    check("everything else runs at half",
          all(cfgs[t].arm_speed_pct == 50
              and cfgs[t].approach_speed_pct == 50
              and cfgs[t].pole_speed_pct == 50 for t in TASKS))
    check("the pole saturates against the DRIVE, not MoveIt's limit",
          _tc.POLE_MAX_MM_S_MOVEIT > _tc.POLE_MAX_MM_S_DRIVE
          and cfgs["hinge_area_right"].pole_speed_pct == 50)
    check("the intended (pre-policy) speed stays visible",
          all(cfgs[t].arm_speed_intended_pct == 100 for t in TASKS))
    for t in TASKS:
        ok, order, problems = cfgs[t].enforce_serialization()
        check(f"{t}: no stage drives two devices", ok and not problems)
        check(f"{t}: every motion stage maps to one device",
              all(d in ("arm", "hand", "pole") for _n, d in order))
    # toplid sets use_prestart_stage: false but still LISTS the stage
    check("conditional stages are resolved, not blindly run",
          any(s.get("stage") == "move_to_pre_start"
              for s in cfgs["toplid_right"].stage_sequence)
          and not any(s.get("stage") == "move_to_pre_start"
                      for s in cfgs["toplid_right"].active_stages()))
    check("hinge KEEPS pre_start (use_prestart_stage: true)",
          any(s.get("stage") == "move_to_pre_start"
              for s in cfgs["hinge_area_right"].active_stages()))
    check("contact contract is loaded per ik_frame",
          len(cfgs["hinge_area_right"].contact_links()) > 20)

    # ── P3. Cleaning path: resolution, chaining, anchor ─────────────────
    print("\nP3. Cleaning path resolution")
    from cleaning_path import CleaningPath
    import numpy as _np
    for t, n_wp in (("hinge_area_right", 44), ("hinge_area_left", 44),
                    ("toplid_right", 28), ("toplid_left", 28)):
        p = CleaningPath(cfgs[t])
        wps = p.waypoints_ref()
        check(f"{t}: {n_wp} waypoints from the chained sequence",
              len(wps) == n_wp, f"got {len(wps)}")
    # Deltas are expressed in the REFERENCE frame: a pure translation delta
    # must move the waypoint by exactly that vector, whatever the anchor's
    # orientation is. (task_base.cpp adds in reference_frame axes and
    # LEFT-multiplies the rotation; it does not rotate into the tool frame.)
    p = CleaningPath(cfgs["hinge_area_right"])
    T_anchor = p.anchor()
    named = dict(p.waypoints_ref())
    pt2 = cfgs["hinge_area_right"].cleaning_points["point2"]
    want = T_anchor[:3, 3] + _np.asarray(pt2["translation"], dtype=float)
    check("translation delta is applied in the reference frame",
          _np.allclose(named["point2"][:3, 3], want, atol=1e-9))
    check("zero rotation delta leaves the anchor orientation untouched",
          _np.allclose(named["point2"][:3, :3], T_anchor[:3, :3],
                       atol=1e-9))
    # A broken chain must RAISE — the runtime emits only each segment's
    # second point, so a break would silently drop a stroke.
    import copy as _copy
    broken = _copy.deepcopy(cfgs["hinge_area_right"])
    broken.cleaning_sequence = [["point1", "point2"], ["point3", "point1"]]
    _raised = False
    try:
        CleaningPath(broken).waypoint_names()
    except SystemExit:
        _raised = True
    check("a non-chained cleaning_sequence RAISES", _raised)
    # No start_pose and no override must fail loudly, not resolve to origin
    anchorless = _copy.deepcopy(cfgs["toplid_right"])
    anchorless.cartesian_poses = {}
    _raised = False
    try:
        CleaningPath(anchorless).anchor()
    except SystemExit:
        _raised = True
    check("a missing anchor RAISES (lookback must be supplied)", _raised)
    prog = CleaningPath(cfgs["hinge_area_right"]).movel_program(0.075)
    check("movel program queues every segment",
          prog["segments"] == 43 and prog["queue_depth"] == 43)
    check("movel program selects the task's own tool frame",
          prog["tool_frame"] == "R_glove_4")
    # The path is a function of POLE HEIGHT — resolving at the wrong one
    # displaces every pose by the difference (the 215 mm bug).
    hi = CleaningPath(cfgs["hinge_area_right"]).movel_program(0.29)
    # NOT along z: the arm is mounted rotated 90 deg about Y, so world-up
    # is +X in the arm base frame (the same rotation that makes the
    # controller report x where the offline solver reports z). Compare the
    # 3-D magnitude and let the axis fall where it may.
    shift = _np.linalg.norm(_np.asarray(hi["poses"][0][:3])
                            - _np.asarray(prog["poses"][0][:3]))
    check("pole height shifts the whole path by 215 mm",
          abs(shift - 0.215) < 1e-3, f"{shift * 1000:.1f} mm")
    # In the CONTROLLER's frame (mount applied) the shift is along -Z:
    # raising the arm base moves a fixed world target down relative to it.
    # Before the mounting rotation was applied this read +X, which is the
    # URDF base_link convention — the two differ by Ry(+90).
    check("raising the pole shifts commanded targets along -Z",
          abs(hi["poses"][0][2] - prog["poses"][0][2] + 0.215) < 1e-3
          and abs(hi["poses"][0][0] - prog["poses"][0][0]) < 1e-6)
    check("orientation is unchanged by pole height",
          _np.allclose(hi["poses"][0][3:], prog["poses"][0][3:], atol=1e-9))

    # ── P4. Stage runner: the serialization invariant ───────────────────
    print("\nP4. Stage runner invariant")
    from stage_runner import StageRunner, DeviceBusy
    from segment_verifier import load_plan as _lp, resolve_plan as _rp
    plan = _lp(_rp("hinge_area_right_ruckig_pro_only.json"))
    r = StageRunner(cfgs["hinge_area_right"], None, None, plan, dry=True)
    check("dry run walks the whole sequence", r.run())
    check("every stage reported a result", len(r.log) >= 8)
    check("the pole is recorded so the path can be resolved",
          r.pole_m == 0.075)
    # Claiming a second device while one is in flight must raise
    r2 = StageRunner(cfgs["hinge_area_right"], None, None, plan, dry=True)
    r2._claim("pole", "x")
    _raised = False
    try:
        r2._claim("arm", "y")
    except DeviceBusy:
        _raised = True
    check("a concurrent device dispatch RAISES (F9 invariant)", _raised)
    r3 = StageRunner(cfgs["hinge_area_right"], None, None, plan, dry=True,
                     serialize=False)
    r3._claim("pole", "x")
    try:
        r3._claim("arm", "y")
        _ok = True
    except DeviceBusy:
        _ok = False
    check("RM_SERIALIZE=0 reports but does not block", _ok)
    # execute_path must refuse to resolve before the pole has moved
    r4 = StageRunner(cfgs["hinge_area_right"], None, None, plan, dry=True)
    ok4, why = r4.run_cleaning_path({"stage": "execute_path"})
    check("the cleaning path refuses to run before the pole",
          ok4 is False and "pole" in why)

    # ── P5. Contact contract + fixture articulation ─────────────────────
    print("\nP5. Contact contract")
    import run_hinge_verify as rhv
    for t, surf, stages in (("hinge_area_right", "all",
                             {"move_to_start", "execute_path"}),
                            ("toplid_left", "seat_and_lid",
                             {"move_to_start", "execute_path"})):
        s_got, st_got = cfgs[t].contact_window()
        check(f"{t}: contact window read from allow/forbid_collision",
              s_got == surf and st_got == stages, f"{s_got} over {st_got}")
    check("stages outside the bracket are NOT contact stages",
          "move_to_rest" not in cfgs["hinge_area_right"].contact_window()[1]
          and "move_to_pre_start" not in
          cfgs["hinge_area_right"].contact_window()[1])
    check("declared objects map to scene meshes",
          rhv.allowed_mesh_keys(cfgs["toplid_left"], "seat_and_lid")
          == {"lid_closed", "lid_open", "seat_closed", "seat_open"})
    check("an unmapped contact object is NOT silently allowed",
          rhv.allowed_mesh_keys(cfgs["toplid_left"], "bin_link") == set())
    # The commode is ARTICULATED — lid/seat have closed (0 deg) and open
    # (-110 deg) variants and only one exists at a time. Checking against
    # both put phantom contacts in stages that must be clear.
    check("every commode task declares a fixture articulation",
          all(cfgs[t].params.get("commode_fixture_type") == "closed"
              for t in TASKS))

    # ── P6. Speed limits: v% is a percentage OF THESE ───────────────────
    print("\nP6. Controller speed limits")
    import speed_limits as _sl

    class _LimRobot:
        def __init__(self):
            self.set = {}

        def rm_get_arm_max_line_speed(self):
            return 0, 0.250

        def rm_get_arm_max_line_acc(self):
            return 0, 1.600

        def rm_get_arm_max_angular_speed(self):
            return 0, 0.600

        def rm_get_arm_max_angular_acc(self):
            return 0, 4.000

        def rm_get_joint_max_speed(self):
            return 0, [180, 180, 225, 225, 225, 225, 225]

        def rm_get_joint_max_acc(self):
            return 0, [600] * 7

        def rm_set_arm_max_line_speed(self, v):
            self.set["line_speed"] = v
            return 0
    r = _LimRobot()
    lim = _sl.read(r)
    check("all six limit families read back",
          all(lim.get(k) is not None for k in
              ("line_speed", "line_acc", "angular_speed", "angular_acc",
               "joint_speed", "joint_acc")))
    txt = _sl.describe(lim, 100, 50)
    check("a percentage is reported in PHYSICAL units",
          "0.250 m/s" in txt and "0.125 m/s" in txt and "112 deg/s" in txt,
          txt[:60])
    # The limits are global controller state and the machine's safety
    # envelope — raising one must be deliberate, never incidental.
    _raised = False
    try:
        _sl.apply(r, line_speed=0.9)
    except ValueError:
        _raised = True
    check("raising a limit past the F10 envelope RAISES", _raised)
    before = _sl.apply(r, line_speed=0.1)
    check("lowering is allowed and returns the previous value",
          r.set.get("line_speed") == 0.1 and before["line_speed"] == 0.250)
    _sl.restore(r, before)
    check("restore puts the envelope back",
          r.set.get("line_speed") == 0.250)

    # ── P7. Every SDK call matches the REAL signature ───────────────────
    print("\nP7. SDK call-site audit")
    # Two bugs of the same shape reached the arms on 2026-08-08:
    # rm_set_hand_angle called without its positional `timeout`, and
    # rm_movel handed an rm_pose_t when it indexes `pose[:3]` and builds
    # the struct itself. Both passed under emulation because the emulator
    # was more permissive than the API. The emulator now mirrors those
    # signatures exactly; this catches the arity half statically, across
    # every call site, so the next one never gets as far as hardware.
    import ast as _ast
    import inspect as _inspect
    from Robotic_Arm.rm_robot_interface import RoboticArm as _RA, Algo as _Al
    _real = {}
    for _cls in (_RA, _Al):
        for _n, _fn in _inspect.getmembers(_cls, _inspect.isfunction):
            if _n.startswith("rm_"):
                _real[_n] = _inspect.signature(_fn)
    _src = _pathlib.Path(__file__).resolve().parent
    _bad, _seen = [], 0
    for _f in sorted(_src.glob("*.py")):
        if _f.name in ("rm_emulator.py", _pathlib.Path(__file__).name):
            continue
        for _node in _ast.walk(_ast.parse(_f.read_text())):
            if not isinstance(_node, _ast.Call):
                continue
            _fn = _node.func
            _nm = _fn.attr if isinstance(_fn, _ast.Attribute) else None
            if not _nm or _nm not in _real:
                continue
            # *args / **kwargs cannot be counted statically
            if any(isinstance(a, _ast.Starred) for a in _node.args) or \
                    any(k.arg is None for k in _node.keywords):
                continue
            _seen += 1
            _params = [p for p in _real[_nm].parameters.values()
                       if p.name != "self"]
            _req = [p for p in _params
                    if p.default is _inspect.Parameter.empty]
            _n_args = len(_node.args) + len(_node.keywords)
            if not (len(_req) <= _n_args <= len(_params)):
                _bad.append(f"{_f.name}:{_node.lineno} {_nm}() got "
                            f"{_n_args}, wants {len(_req)}..{len(_params)}")
    check("every SDK call site matches the real signature arity",
          not _bad, f"{_seen} call sites checked"
          if not _bad else "; ".join(_bad))

    # ── P8. Controller frame + SIM assumptions ──────────────────────────
    print("\nP8. Controller base frame + SIM")
    # The controller knows nothing of the UGV, the pole, the hand or
    # butterfli_ref_frame, AND it is mounted rotated. Poses must land in
    # its own base frame: controller = Ry(+90) . urdf_base_link, measured
    # against both controllers (0.0 mm, 0.07 deg).
    from cleaning_path import CleaningPath as _CP
    _prog = _CP(cfgs["hinge_area_right"]).movel_program(0.075)
    _p0 = _prog["poses"][0]
    check("the mounting rotation is applied to commanded poses",
          abs(_CP.MOUNT_RY_DEG - 90.0) < 1e-9)
    # world-up became +X earlier; with the mount applied it is +Z again,
    # which is what the controller reports.
    check("commanded poses are plausible for this arm (|p| < 1.2 m)",
          0.1 < (_p0[0] ** 2 + _p0[1] ** 2 + _p0[2] ** 2) ** 0.5 < 1.2,
          f"|p|={(_p0[0]**2+_p0[1]**2+_p0[2]**2)**0.5:.3f} m")
    # SIM: the controller executes the ARM only (F3), so pole and hand
    # stages are assumed successful — that is what lets a whole task be
    # rehearsed on the real SDK without the arm moving for real.
    _sr = StageRunner(cfgs["hinge_area_right"], None, None, plan,
                      dry=False, sim=True)
    _ok, _why = _sr.run_pole({"stage": "move_pole_to_quarter_height"})
    check("SIM assumes the pole and still RECORDS its height",
          _ok and _sr.pole_m == 0.075 and "SIM assumed" in _why)
    _ok2, _why2 = _sr.run_hand({"stage": "hand_release",
                                "pose_name": "release"})
    check("SIM assumes the hand", _ok2 and "SIM assumed" in _why2)

    # ── P9. Disabled joints fail the gate (the J6/J7 incident) ─────────
    print("\nP9. Disabled-joint gate")

    class _EnRobot(MockRobot):
        def __init__(self, *a, disabled=(), **kw):
            super().__init__(*a, **kw)
            self._en = [0 if (i + 1) in disabled else 1 for i in range(7)]

        def rm_get_joint_en_state(self):
            return 0, list(self._en)

        def rm_get_joint_err_flag(self):
            return {"return_code": 0, "err_flag": [0] * 7}

    _m = dac.ArrivalMonitor()
    _dead = dac.ConceptArm("left", _EnRobot(31, _m, disabled=(6, 7)),
                           _Handle(31))
    _st = dac.error_state(_dead)
    check("de-enabled joints are seen as an error surface",
          _st.get("disabled") == [6, 7])
    check("the gate REFUSES with disabled joints",
          not dac.error_state_clean(_st))
    check("the description warns about teach mode",
          "teach mode" in dac.describe_error_state(_st))
    _ok = dac.ConceptArm("left", _EnRobot(32, _m), _Handle(32))
    check("all-enabled arm is clean",
          dac.error_state_clean(dac.error_state(_ok)))
    # An SDK without the getter (older fw) must not break the gate.
    _plain = dac.ConceptArm("left", MockRobot(33, _m), _Handle(33))
    check("a missing en-state getter degrades to the old gate",
          dac.error_state_clean(dac.error_state(_plain)))

    # ── P10. Hover A/B + power-probe math ───────────────────────────────
    print("\nP10. Hover and power telemetry")
    import numpy as _np2

    # The hover direction is the TASK'S OWN retreat vector — the yaml
    # header claims pre_start = start retreated along local +Z, but the
    # data contradicts it (hinge alignment 0.198 — hand-edited values —
    # and toplid -0.991, the opposite sign). These checks pin the DATA
    # facts the hover implementation rests on instead of the comment.
    from cleaning_path import quat_to_R as _q2R
    for _t in ("hinge_area_right", "toplid_right", "toplid_left",
               "hinge_area_left"):
        _cp2 = cfgs[_t].cartesian_poses
        check(f"{_t}: declares start AND pre_start (hover needs both)",
              bool(_cp2.get("start_pose")) and
              bool(_cp2.get("pre_start_pose")))
    _sp = cfgs["toplid_right"].cartesian_poses["start_pose"]
    _pp = cfgs["toplid_right"].cartesian_poses["pre_start_pose"]
    _dp = _np2.asarray(_pp[:3], float) - _np2.asarray(_sp[:3], float)
    _zloc = _q2R(_sp[3:7])[:, 2]
    check("toplid's local +Z points INTO the surface (sign proof against "
          "the local-axis convention)",
          float(_dp @ _zloc / _np2.linalg.norm(_dp)) < -0.9)

    _base = _CP(cfgs["hinge_area_right"]).movel_program(0.075)
    _hov = _CP(cfgs["hinge_area_right"]).movel_program(0.075, hover_mm=10)
    _deltas = [_np2.asarray(h[:3]) - _np2.asarray(b[:3])
               for b, h in zip(_base["poses"], _hov["poses"])]
    check("hover shifts every waypoint by exactly 10 mm",
          all(abs(_np2.linalg.norm(d) - 0.010) < 1e-6 for d in _deltas))
    check("the shift is one CONSTANT vector (the declared retreat)",
          all(_np2.allclose(d, _deltas[0], atol=1e-9) for d in _deltas))
    import copy as _cp3
    _noretreat = _cp3.deepcopy(cfgs["hinge_area_right"])
    _noretreat.cartesian_poses = {"start_pose":
                                  _noretreat.cartesian_poses["start_pose"]}
    _raised = False
    try:
        _CP(_noretreat).movel_program(0.075, hover_mm=10)
    except SystemExit:
        _raised = True
    check("hover without a declared retreat REFUSES (never guesses)",
          _raised)
    check("orientations are untouched by hover",
          all(b[3:] == h[3:]
              for b, h in zip(_base["poses"], _hov["poses"])))
    check("the program records its hover", _hov["hover_mm"] == 10.0)

    # power probe summary math on synthetic frames: J6 sags, J1 clean
    import power_probe as _pp2
    _recs = []
    for k in range(100):
        v = [48.0] * 7
        i = [400.0] * 7
        en = [1] * 7
        if 40 <= k < 50:
            v[5] = 44.5           # 3.5 V sag on J6
            i[5] = 6200.0
        if k == 49:
            en[5] = 0             # and the drive drops out once
        _recs.append((k * 0.01, v, i, [35.0] * 7, en))
    _rep = _pp2.summarize(_recs)
    _j6, _j1 = _rep["joints"][5], _rep["joints"][0]
    check("probe: the sagging joint is flagged with dip count and depth",
          _j6["dips"] == 10 and abs(_j6["worst_dip_v"] - 3.5) < 1e-6
          and _j6["en_dropped"])
    check("probe: a clean joint stays clean",
          _j1["dips"] == 0 and not _j1["en_dropped"])

    # ── P11. Capabilities, mount-from-controller, diagnostics ───────────
    print("\nP11. Controller capabilities + mount source")
    import controller_caps as _cc

    class _CapRobot:
        def __init__(self):
            self.v = {"avoid_singularity": 0, "self_collision": False,
                      "endeff_collision": False, "collision_stage": 4,
                      "static_collision": 0, "collision_remove": False}
            self.install = {"return_code": 0, "x": 0.0, "y": 90.0, "z": 0.0}

        def rm_get_avoid_singularity_mode(self):
            return 0, self.v["avoid_singularity"]

        def rm_set_avoid_singularity_mode(self, m):
            self.v["avoid_singularity"] = int(m); return 0

        def rm_get_self_collision_enable(self):
            return 0, self.v["self_collision"]

        def rm_set_self_collision_enable(self, e):
            self.v["self_collision"] = bool(e); return 0

        def rm_get_self_endeffector_collision_enable(self):
            return 0, self.v["endeff_collision"]

        def rm_set_self_endeffector_collision_enable(self, e):
            self.v["endeff_collision"] = bool(e); return 0

        def rm_get_collision_stage(self):
            return 0, self.v["collision_stage"]

        def rm_set_collision_state(self, x):
            self.v["collision_stage"] = int(x); return 0

        def rm_get_collision_detection(self):
            return 0, self.v["static_collision"]

        def rm_set_collision_detection(self, m):
            self.v["static_collision"] = int(m); return 0

        def rm_get_collision_remove_enable(self):
            return 0, self.v["collision_remove"]

        def rm_set_collision_remove_enable(self, e):
            self.v["collision_remove"] = bool(e); return 0

        def rm_get_install_pose(self):
            return self.install
    r = _CapRobot()
    caps = _cc.read(r)
    check("every capability reads back",
          all(not isinstance(caps[k], str) for k in
              ("avoid_singularity", "self_collision", "endeff_collision",
               "collision_stage", "static_collision", "collision_remove")))
    check("the report flags what is OFF but wanted",
          "Phase 2 wants it ON" in _cc.describe(caps))
    # Singularity avoidance is the capability this whole exercise turns on:
    # supported since V1.7.3 for 7-axis, arms on V1.7.4, shipped OFF.
    prev = _cc.prepare(r, "(test)")
    check("prepare() switches singularity avoidance ON",
          r.v["avoid_singularity"] == 1 and r.v["self_collision"] is True)
    _cc.restore(r, prev)
    check("restore() puts the controller back as found",
          r.v["avoid_singularity"] == 0 and r.v["self_collision"] is False)

    # The mounting angle must come from the ARM, not the constant.
    _p = _CP(cfgs["hinge_area_right"])
    got = _p.set_mount_from_controller(r)
    check("mount angle is read from rm_get_install_pose", got == (0.0, 90.0, 0.0)
          and abs(_p.MOUNT_RY_DEG - 90.0) < 1e-9)
    r.install = {"return_code": 0, "x": 0.0, "y": 75.0, "z": 0.0}
    _p2 = _CP(cfgs["hinge_area_right"])
    _p2.set_mount_from_controller(r)
    check("a DIFFERENT install pose wins over the constant",
          abs(_p2.MOUNT_RY_DEG - 75.0) < 1e-9
          and abs(_CP.MOUNT_RY_DEG - 90.0) < 1e-9)
    r.install = {"return_code": 0, "x": 5.0, "y": 90.0, "z": 0.0}
    _raised = False
    try:
        _CP(cfgs["hinge_area_right"]).set_mount_from_controller(r)
    except SystemExit:
        _raised = True
    check("a non-Ry mounting REFUSES rather than mis-modelling", _raised)
    r.install = {"return_code": 1}
    _p3 = _CP(cfgs["hinge_area_right"])
    check("an unreadable install pose falls back to the measured constant",
          _p3.set_mount_from_controller(r) is None
          and abs(_p3.MOUNT_RY_DEG - 90.0) < 1e-9)

    # ── P12. Run recorder: format, alignment, provenance ────────────────
    print("\nP12. Run recorder")
    import run_recorder as _rr

    # The saved MoveIt plans keep scalar arrays on ONE line and floats at
    # %.6f; a run must be diffable against them side by side.
    _j = _rr.dumps({"task_name": "t", "v": 1.5, "flag": True, "none": None,
                    "names": ["a", "b"], "poses": [[1.0, 2.0], [3.0, 4.0]],
                    "nested": {"k": 2}})
    check("scalar arrays stay inline (plan style)",
          '"names": ["a", "b"]' in _j)
    check("floats render at 6 decimals", '"v": 1.500000' in _j)
    check("bools/None render as JSON literals",
          '"flag": true' in _j and '"none": null' in _j)
    check("nested arrays-of-arrays expand one per line",
          "[1.000000, 2.000000],\n" in _j)
    import json as _js
    check("output is valid JSON", isinstance(_js.loads(_j), dict))

    class _RecRobot:
        def rm_realtime_arm_state_call_back(self, cb):
            self.cb = cb

        def rm_set_realtime_push(self, cfg):
            return 0
    _rec = _rr.RunRecorder(_RecRobot(), "t", "left", "1.2.3.4", 8095,
                           out_dir=str(_pathlib.Path(
                               _os.environ.get("TMPDIR", "/tmp")) / "rec_t"))
    check("header width matches a row's width",
          len(_rec.header()) == 1 + 5 * 7 + 7 + 7 + 5 + 6 + 1)
    # hand is NOT requested: measured ALL-ZERO on the arm's UDP line
    _src = (_pathlib.Path(__file__).resolve().parent
            / "run_recorder.py").read_text()
    check("the recorder does NOT request hand_state (bench: ALL-ZERO)",
          "hand_state=1" not in _src and "lift_state=1" in _src)
    check("100 Hz default (cycle=2, 5 ms units)",
          _rr.DEFAULT_CYCLE == 2)
    # runs/ holds recorded HARDWARE data and is committed as such; an
    # emulated run must never land there (the emulated suite wrote four
    # such directories on 2026-08-10 before this guard existed).
    check("an EMULATED recorder writes outside runs/",
          "rm_emulator" not in sys.modules or
          str(_rr.RUNS_DIR) not in str(
              _rr.RunRecorder(_RecRobot(), "t", "left", "1.2.3.4", 8095).dir))
    _under_runs = str(_rr.RunRecorder(
        _RecRobot(), "t", "left", "1.2.3.4", 8095).dir)
    check("a HARDWARE recorder writes INTO runs/",
          ("rm_emulator" in sys.modules) or
          str(_rr.RUNS_DIR) in _under_runs, _under_runs[-48:])

    # a recorder that silently records nothing is worse than one that errors
    _rec._t0 = 0.0
    _rec._on_state(object())          # no joint_status -> must be counted
    check("an unparseable frame is COUNTED, not swallowed",
          _rec._dropped == 1 and _rec._first_error)

    n, f = _checks["run"], _checks["fail"]
    print("\n" + "=" * 68)
    print(f"Dry run: {n - f}/{n} passed")
    print("=" * 68)
    return 0 if f == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
