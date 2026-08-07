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
_os.environ.setdefault("RM_HAND_DWELL_S", "0.05")
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

    def _arrive(self, device, delay):
        threading.Timer(
            delay, self.monitor._on_event,
            args=(_FakeEventData(self.handle_id, device),)).start()

    def rm_movej(self, joint, v, r, connect, block):
        self.dispatched += 1
        if self.fail_at is not None and self.dispatched == self.fail_at:
            return 1
        self._arrive(dac.DEV_JOINT, self.arm_s)
        return 0

    def rm_set_lift_height(self, speed, hw_mm, block):
        self.dispatched += 1
        if self.fail_at is not None and self.dispatched == self.fail_at:
            return 1
        self._arrive(dac.DEV_LIFT, self.lift_s)
        return 0

    def rm_get_current_arm_state(self):
        return 0, {"joint": [0.0] * 7}

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
    right_hw = {"minimum": 7, "quarter": 50, "half": 100, "full": 193}
    left_hw = {"minimum": 10, "quarter": 75, "half": 150, "full": 290}
    for name, hw in right_hw.items():
        check(f"lift {name} -> {hw} (right, 2to3)",
              dac.lift_hw_mm("right", dac.LIFT_M[name]) == hw)
    for name, hw in left_hw.items():
        check(f"lift {name} -> {hw} (left, 1to1)",
              dac.lift_hw_mm("left", dac.LIFT_M[name]) == hw)
    check("left full 290 within controller ceiling 330",
          left_hw["full"] <= dac.LIFT_GEAR["left"]["hw_max"])
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
    check("matched lift speed: 105 phys mm over 4 s arm move -> 10%",
          dac.matched_lift_speed_pct(4.0, 105.0) == 10)
    _pct = dac.matched_lift_speed_pct(4.0, 105.0)
    _pole_s = dac.LIFT_START_LATENCY_S + 105.0 / (_pct * dac.LIFT_MM_S_PER_PCT)
    check("matched pole is predicted to finish AFTER the arm",
          _pole_s > 4.0, f"pole {_pole_s:.2f} s vs arm 4.00 s")
    check("a bigger outlast factor makes the pole slower, never faster",
          dac.matched_lift_speed_pct(4.0, 105.0)
          <= math.ceil(105.0 / (4.0 - dac.LIFT_START_LATENCY_S)
                       / dac.LIFT_MM_S_PER_PCT))
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
        check("upgraded right arm auto-switches to 1to1 (full -> 290)",
              dac.LIFT_GEAR["right"]["name"] == "1to1"
              and dac.lift_hw_mm("right", dac.LIFT_M["full"]) == 290)
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
            self.cleared += 1
            self.sys_codes, self.joints, self.lift_err = [], [0] * 7, 0
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
        "'emul': rm_emulator.EMU_LEFT_IP, 'emur': rm_emulator.EMU_RIGHT_IP, "
        "'emuhosts': sorted(rm_emulator.EMU_HOST_IPS), 'r_full': dac.lift_hw_mm('right', 0.29), 'l_full': dac.lift_hw_mm('left', 0.29)}))"
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
        check("C5 shares the common config",
              cfg["c5host"] == cfg["host"] and cfg["c5udp"] == cfg["udp"])
        check("emulator defaults match the tests",
              cfg["emul"] == cfg["left"] and cfg["emur"] == cfg["right"]
              and cfg["emuhosts"] == [cfg["host"]])

        ov = run_probe({"RM_LEFT_IP": "10.9.9.1", "RM_RIGHT_IP": "10.9.9.2",
                        "RM_ROBOT_PORT": "9080", "RM_HOST_IP": "10.9.9.100",
                        "RM_UDP_PORT": "9002"})
        check("env overrides reach dual_arm_common",
              ov["left"] == "10.9.9.1" and ov["right"] == "10.9.9.2"
              and ov["port"] == 9080 and ov["host"] == "10.9.9.100"
              and ov["udp"] == 9002)
        check("env overrides reach C5",
              ov["c5host"] == "10.9.9.100" and ov["c5udp"] == 9002)
        check("env overrides reach the emulator",
              ov["emul"] == "10.9.9.1" and ov["emur"] == "10.9.9.2"
              and ov["emuhosts"] == ["10.9.9.100"])
        check("default gearing: left 1to1 (290), right 2to3 (193)",
              cfg["l_full"] == 290 and cfg["r_full"] == 193)
        gv = run_probe({"RM_RIGHT_LIFT_GEAR": "1to1"})
        check("RM_RIGHT_LIFT_GEAR=1to1 flips right to 290 (post-upgrade)",
              gv["r_full"] == 290)
    except Exception as exc:
        check("configuration probe subprocess", False, repr(exc))

    n, f = _checks["run"], _checks["fail"]
    print("\n" + "=" * 68)
    print(f"Dry run: {n - f}/{n} passed")
    print("=" * 68)
    return 0 if f == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
