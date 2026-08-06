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
    expected_hw = {"minimum": 7, "quarter": 50, "half": 100, "full": 193}
    for name, hw in expected_hw.items():
        check(f"lift {name} -> {hw} hw-mm",
              dac.lift_hw_mm(dac.LIFT_M[name]) == hw)
    for bad in (0.0, 0.31, -0.1):
        try:
            dac.lift_hw_mm(bad)
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

    # Sync duration matching (butterfli_hw contract, round-UP quantization)
    check("matched lift speed: 105 phys mm over 4 s arm move -> 16%",
          dac.matched_lift_speed_pct(4.0, 105.0) == 16)
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
    check("locked arrivals via event",
          all(e["left"]["event"] and e["right"]["event"] for e in rep["steps"]))
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
        "'emuhosts': sorted(rm_emulator.EMU_HOST_IPS)}))"
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
              and cfg["port"] == 8080 and cfg["host"] == "192.168.1.235"
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
    except Exception as exc:
        check("configuration probe subprocess", False, repr(exc))

    n, f = _checks["run"], _checks["fail"]
    print("\n" + "=" * 68)
    print(f"Dry run: {n - f}/{n} passed")
    print("=" * 68)
    return 0 if f == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
