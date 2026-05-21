#!/usr/bin/env python3
"""
test_lift_preemption.py
-----------------------
Q3: Pole command preemption -- what happens when MoveIt sends a new trajectory
mid-execution of the current one?

Builds on test_lift_trajectory_rate.py: starts replaying the TRAJECTORY loaded
from lift_trajectory.json (75mm -> 295mm), then at three injection points the
current trajectory is abandoned and a dynamically-built return trajectory
(back to 75 mm, same waypoint format) is issued.

This mirrors the MoveIt action-server preemption flow exactly:
  1. Action server receives TRAJECTORY_A, starts execution.
  2. New goal arrives mid-travel -> cancel A, execute TRAJECTORY_B.
  3. Measure: does the motor redirect immediately (REDIRECT) or not?

Injection points
----------------
  T1: 0.3 s  (early  - motor ~20 mm into stroke)
  T2: 1.2 s  (mid    - motor ~80 mm into stroke)
  T3: 2.2 s  (late   - motor ~147 mm into stroke)

Trajectory source
-----------------
  lift_trajectory.json -- same file loaded by test_lift_trajectory_rate.

Run:
    python3 test_lift_preemption.py

Exit codes:
    0  -- assertions passed (or hardware absent)
    1  -- one or more FAIL results
"""

import sys
import json
import time
import threading
from collections import deque
import pathlib

_SRC = str(pathlib.Path(__file__).resolve().parent)
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))

# -- Trajectory loaded from JSON (same file as test_lift_trajectory_rate) -----
_TRAJ_JSON = pathlib.Path(__file__).resolve().parent / "lift_trajectory.json"
with _TRAJ_JSON.open() as _f:
    _traj_data = json.load(_f)
    TRAJECTORY = _traj_data["waypoints"] if isinstance(_traj_data, dict) else _traj_data

# -- Conversion helpers shared with test_lift_trajectory_rate -----------------
from test_lift_trajectory_rate import (
    parse_trajectory,
    ROBOT_IP, ROBOT_PORT,
    UDP_TARGET_IP, UDP_PORT,
    POLE_MIN_MM, POLE_MAX_MM,
    PHYS_TO_HW, HW_TO_PHYS,
    hw_height, phys_height,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import (
    rm_thread_mode_e,
    rm_realtime_push_config_t,
    rm_udp_custom_config_t,
    rm_realtime_arm_state_callback_ptr,
    rm_realtime_arm_state_call_back,
    rm_realtime_arm_joint_state_t,
)
import hw_baseline

# -- Injection points: (delay_into_TRAJECTORY_s, label) ----------------------
# At 100 mm/s (plateau): 0.3s ~20mm, 1.2s ~80mm, 2.2s ~147mm into stroke
INJECT_POINTS = [
    (0.3, "T1 -- early  (~20mm in)"),
    (1.2, "T2 -- mid    (~80mm in)"),
    (2.2, "T3 -- late   (~147mm in)"),
]

REVERSAL_THRESHOLD_MM = 5
REVERSAL_TIMEOUT_S    = 3.0

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


# -- UDP callback -------------------------------------------------------------
_udp_buf  = deque(maxlen=8000)
_udp_lock = threading.Lock()


@rm_realtime_arm_state_callback_ptr
def _udp_cb(state: rm_realtime_arm_joint_state_t):
    ts = time.perf_counter_ns()
    try:
        h = phys_height(state.liftState.height)
    except Exception:
        h = -1.0
    with _udp_lock:
        _udp_buf.append((ts, h))


def _configure_udp(robot: RoboticArm, cycle: int):
    return robot.rm_set_realtime_push(rm_realtime_push_config_t(
        cycle=cycle, enable=True, port=UDP_PORT, ip=UDP_TARGET_IP,
        custom_config=rm_udp_custom_config_t(lift_state=1),
    ))


def _drain():
    with _udp_lock:
        out = list(_udp_buf); _udp_buf.clear()
    return out


def _latest_pos() -> float:
    with _udp_lock:
        return _udp_buf[-1][1] if _udp_buf else 0.0


# -- Return trajectory builder ------------------------------------------------
def build_return_trajectory(current_mm: int, target_mm: int) -> list:
    """
    Build a MoveIt-format trajectory from current_mm back to target_mm at
    100 mm/s in 10 mm / 100 ms steps -- same density as TRAJECTORY.
    """
    direction = 1 if target_mm > current_mm else -1
    distance  = abs(current_mm - target_mm)
    n_steps   = max(1, round(distance / 10))
    wps = []
    for i in range(n_steps + 1):
        pos_mm = current_mm + direction * i * 10
        if direction == -1 and pos_mm < target_mm: pos_mm = target_mm
        if direction ==  1 and pos_mm > target_mm: pos_mm = target_mm
        vel = 0.1 if i < n_steps else 0.0  # 100 mm/s = 0.1 m/s
        wps.append({
            "time_from_start": i * 0.1,
            "positions":     [pos_mm / 1000.0],
            "velocities":    [vel],
            "accelerations": [0.0],
        })
        if pos_mm == target_mm:
            break
    return wps


# -- Preemption sub-test ------------------------------------------------------
def run_preemption(robot: RoboticArm,
                   inject_delay_s: float, label: str) -> dict:
    print(f"\n  {'-'*60}")
    print(f"  {label}   inject_delay={inject_delay_s:.1f} s")
    print(f"  {'-'*60}")

    parsed_fwd = parse_trajectory(TRAJECTORY)
    home_mm    = parsed_fwd[0][1]   # trajectory start (75 mm)

    print(f"  [INFO] Homing to {home_mm} mm (blocking) ...")
    robot.rm_set_lift_height(100, hw_height(home_mm), True)
    time.sleep(0.5)
    _drain()

    # Execute TRAJECTORY forward; break at inject point
    print(f"  [INFO] Replaying TRAJECTORY ({home_mm}->{parsed_fwd[-1][1]} mm) ...")
    errors_fwd = 0
    t0 = time.perf_counter()
    for t_sched, h_mm, spct in parsed_fwd:
        if time.perf_counter() - t0 >= inject_delay_s:
            break
        sleep = t0 + t_sched - time.perf_counter()
        if sleep > 0: time.sleep(sleep)
        if robot.rm_set_lift_height(spct, hw_height(h_mm), False) != 0:
            errors_fwd += 1

    pos_at_inject = _latest_pos()
    print(f"  [INFO] Position at inject: ~{pos_at_inject:.1f} mm  "
          f"(fwd errors={errors_fwd})")
    _drain()

    # Issue return trajectory (MoveIt-style preemption / goal cancel + new goal)
    return_wps = build_return_trajectory(int(round(pos_at_inject)), home_mm)
    parsed_ret = parse_trajectory(return_wps)
    print(f"  [INFO] Issuing return trajectory: "
          f"~{pos_at_inject:.0f}->{home_mm} mm  "
          f"({len(parsed_ret)} waypoints)")

    t_inject_ns = time.perf_counter_ns()
    t_ret_start = time.perf_counter()
    for t_sched, h_mm, spct in parsed_ret:
        sleep = t_ret_start + t_sched - time.perf_counter()
        if sleep > 0: time.sleep(sleep)
        robot.rm_set_lift_height(spct, hw_height(h_mm), False)

    # Collect post-inject UDP for REVERSAL_TIMEOUT_S
    post: list = []
    deadline = time.perf_counter() + REVERSAL_TIMEOUT_S
    while time.perf_counter() < deadline:
        time.sleep(0.005)
        with _udp_lock:
            post.extend(list(_udp_buf)); _udp_buf.clear()

    # Print position trace
    print(f"  [INFO] Post-inject: {len(post)} samples")
    if post:
        step = max(1, len(post) // 30)
        print(f"  {'dt_from_inject(ms)':>22}  {'pos(mm)':>9}")
        print(f"  {'-'*22}  {'-'*9}")
        for ts_ns, pos in post[::step]:
            print(f"  {(ts_ns-t_inject_ns)/1e6:>22.1f}  {pos:>9.1f}")

    # Classify
    if not post:
        print("  [WARN] No UDP samples after inject")
        return {"label": label, "verdict": "NO_DATA",
                "pos_at_inject_mm": pos_at_inject}

    peak_pos = max(s[1] for s in post)
    peak_ts  = max((s for s in post), key=lambda s: s[1])[0]

    t_rev_ns = next(
        (s[0] for s in post
         if s[0] > peak_ts and s[1] < peak_pos - REVERSAL_THRESHOLD_MM),
        None)

    if t_rev_ns:
        lat_ms  = (t_rev_ns - t_inject_ns) / 1e6
        verdict = "REDIRECT"
        print(f"\n  Peak after inject : {peak_pos:.1f} mm")
        print(f"  Reversal detected : dropped >{REVERSAL_THRESHOLD_MM} mm below peak")
        print(f"  Reversal latency  : {lat_ms:.0f} ms from inject")
    else:
        lat_ms  = None
        verdict = "NO_REDIRECT"
        print(f"\n  Peak: {peak_pos:.1f} mm  final: {post[-1][1]:.1f} mm")
        print(f"  No reversal in {REVERSAL_TIMEOUT_S:.1f}s")

    print(f"  Verdict: {verdict}")
    return {
        "label": label, "inject_delay_s": inject_delay_s,
        "pos_at_inject_mm": pos_at_inject, "peak_pos_mm": peak_pos,
        "verdict": verdict, "reversal_latency_ms": lat_ms,
    }


# -- Main ---------------------------------------------------------------------
def main():
    print("=" * 72)
    print("Pole hardware: travel=300 mm, max_speed=100 mm/s, "
          "safe range=[10, 290] mm")
    print("=" * 72)
    print("test_lift_preemption.py -- Q3: MoveIt trajectory preemption")
    print(f"Trajectory: {_TRAJ_JSON.name}")
    print("=" * 72)

    robot = handle = None
    try:
        robot  = RoboticArm(rm_thread_mode_e(2))
        handle = robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ROBOT_IP}:{ROBOT_PORT}")
            _results["SKIP"] += len(INJECT_POINTS)
            return 0

        print(f"  [INFO] Connected (handle.id={handle.id})")
        rm_realtime_arm_state_call_back(_udp_cb)
        _configure_udp(robot, 1)    # cycle=1 -> 200 Hz
        time.sleep(0.3)

        subs = [run_preemption(robot, d, lbl) for d, lbl in INJECT_POINTS]

        # Summary table
        print("\n\n  Summary")
        print("  " + "-" * 68)
        print(f"  {'Label':35s}  {'@inject(mm)':>11}  {'Verdict':12}  {'Latency(ms)':>11}")
        print("  " + "-" * 68)
        for r in subs:
            if not r: continue
            lat = f"{r['reversal_latency_ms']:.0f}" \
                  if r.get("reversal_latency_ms") is not None else "-"
            print(f"  {r['label']:35s}  "
                  f"{r.get('pos_at_inject_mm', 0):>11.1f}  "
                  f"{r.get('verdict','?'):12}  {lat:>11}")
        print("  " + "-" * 68)

        redirects = [r for r in subs if r.get("verdict") == "REDIRECT"]
        print()
        if len(redirects) == len(INJECT_POINTS):
            print("  [INFO] All points -> REDIRECT: "
                  "streaming/preemption feasible")
        else:
            print("  [INFO] Some points -> NO_REDIRECT: "
                  "motor queues or ignores mid-travel commands")

        print()
        for r in subs:
            if not r or r.get("verdict") == "NO_DATA":
                result("SKIP", f"{r.get('label','?')}: no UDP data"); continue
            if r["verdict"] == "REDIRECT":
                result("PASS",
                       f"{r['label']}: motor redirected",
                       f"latency={r['reversal_latency_ms']:.0f} ms")
            else:
                result("FAIL",
                       f"{r['label']}: no reversal in {REVERSAL_TIMEOUT_S:.0f}s")

    finally:
        if robot and handle:
            try:
                robot.rm_delete_robot_arm(); RoboticArm.rm_destroy()
            except Exception:
                pass

    print("-" * 72)
    print(f"Results:  PASS={_results['PASS']}  "
          f"FAIL={_results['FAIL']}  SKIP={_results['SKIP']}")
    print("-" * 72)
    return 0 if _results["FAIL"] == 0 else 1


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
