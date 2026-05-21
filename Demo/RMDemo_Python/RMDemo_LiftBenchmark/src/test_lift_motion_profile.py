#!/usr/bin/env python3
"""
test_lift_motion_profile.py
---------------------------
Q2: Pole internal motion profile — commanded vs actual.

Loads the same trajectory that MoveIt produces from lift_trajectory.json,
replays it with the timed-waypoint scheduler used by test_lift_trajectory_rate,
but captures UDP at cycle=1 (200 Hz) instead of cycle=2.

The UDP position stream is cross-referenced against the waypoint schedule to
answer: "Does the motor actually follow the commanded velocity profile?"

Analysis
--------
  1. Load TRAJECTORY from lift_trajectory.json (same file as
     test_lift_trajectory_rate uses).
  2. Execute with the same timed-waypoint scheduler (non-blocking calls timed
     to time_from_start).
  3. Capture UDP at 200 Hz throughout.
  4. Compute rolling velocity: center-difference over 100 ms window
     (~10 mm/s resolution).
  5. For every waypoint print:
       t_sched | cmd_mm | cmd_% | udp_mm | actual_vel (mm/s) | err (mm)
  6. Report: mean/max tracking error, peak velocity, profile-shape heuristic.

Run:
    python3 test_lift_motion_profile.py

Exit codes:
    0  -- assertions passed (or hardware absent)
    1  -- one or more FAIL results
"""

import sys
import json
import time
import statistics
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
    POLE_MAX_SPEED_MM_S,
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

# -- Velocity window ----------------------------------------------------------
# At cycle=1 (5 ms): VEL_HALF=10 -> 100 ms window -> ~10 mm/s resolution
VEL_HALF = 10

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


# -- UDP callback -------------------------------------------------------------
_udp_buf  = deque(maxlen=8000)   # (perf_counter_ns, phys_mm)
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


def _drain() -> list:
    with _udp_lock:
        out = list(_udp_buf); _udp_buf.clear()
    return out


# -- Rolling velocity ---------------------------------------------------------
def rolling_velocity(samples: list, half: int = VEL_HALF) -> list:
    """Center-difference velocity. Returns (t_s, pos_mm, vel_mm_s) list."""
    if len(samples) < 2 * half + 1:
        return []
    t0 = samples[0][0]
    out = []
    for i in range(half, len(samples) - half):
        dt_s = (samples[i + half][0] - samples[i - half][0]) / 1e9
        dp   = samples[i + half][1] - samples[i - half][1]
        vel  = dp / dt_s if dt_s > 0 else 0.0
        out.append(((samples[i][0] - t0) / 1e9, samples[i][1], vel))
    return out


def nearest(vel_series: list, t_target: float) -> tuple:
    """Return entry closest to t_target."""
    return min(vel_series, key=lambda x: abs(x[0] - t_target)) \
           if vel_series else (t_target, 0.0, 0.0)


# -- Profile shape heuristic --------------------------------------------------
def classify_shape(vels: list, t_first: float, t_95: float) -> str:
    span = t_95 - t_first
    if span <= 0:
        return "unknown (acc phase too short)"

    def vel_near(t):
        return min(vels, key=lambda x: abs(x[0] - t))[2]

    v1 = vel_near(t_first + span * 0.25)
    v2 = vel_near(t_first + span * 0.50)
    v3 = vel_near(t_first + span * 0.75)
    dev = v2 - (v1 + v3) / 2.0
    if abs(dev) <= 5.0:
        return f"trapezoidal (linear ramp)  mid-dev={dev:+.1f} mm/s"
    elif dev > 5.0:
        return f"S-curve (concave-down)  mid-dev={dev:+.1f} mm/s"
    else:
        return f"concave-up  mid-dev={dev:+.1f} mm/s"


# -- Main ---------------------------------------------------------------------
def main():
    print("=" * 72)
    print("Pole hardware: travel=300 mm, max_speed=100 mm/s, "
          "safe range=[10, 290] mm")
    print("=" * 72)
    print("test_lift_motion_profile.py -- Q2: commanded vs actual velocity")
    print(f"Trajectory: {_TRAJ_JSON.name}")
    print("=" * 72)

    parsed = parse_trajectory(TRAJECTORY)
    print(f"\n  [INFO] {len(parsed)} waypoints  "
          f"t=0..{parsed[-1][0]:.1f}s  "
          f"{parsed[0][1]}mm -> {parsed[-1][1]}mm")

    robot = handle = None
    try:
        robot  = RoboticArm(rm_thread_mode_e(2))
        handle = robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ROBOT_IP}:{ROBOT_PORT}")
            _results["SKIP"] += 4
            return 0

        print(f"  [INFO] Connected (handle.id={handle.id})")
        rm_realtime_arm_state_call_back(_udp_cb)
        _configure_udp(robot, 1)    # cycle=1 -> 200 Hz
        time.sleep(0.3)

        # Home to trajectory start (blocking)
        print(f"\n  [INFO] Homing to {parsed[0][1]} mm (blocking) ...")
        if robot.rm_set_lift_height(50, hw_height(parsed[0][1]), True) != 0:
            result("FAIL", "Homing failed"); return 1
        time.sleep(0.3)
        _drain()

        # Execute TRAJECTORY -- same scheduler as test_lift_trajectory_rate
        print(f"  [INFO] Replaying {len(parsed)} waypoints ...")
        errors = 0
        t0 = time.perf_counter()
        for t_sched, h_mm, spct in parsed:
            sleep = t0 + t_sched - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            if robot.rm_set_lift_height(spct, hw_height(h_mm), False) != 0:
                errors += 1

        print(f"  [INFO] Dispatch done  errors={errors}")
        print("  [INFO] Waiting 4 s for final motion ...")
        time.sleep(4.0)

        with _udp_lock:
            all_samples = list(_udp_buf); _udp_buf.clear()
        print(f"  [INFO] {len(all_samples)} UDP samples captured")

        vels = rolling_velocity(all_samples)
        if not vels:
            result("FAIL", "Insufficient UDP samples for velocity analysis")
            return 1

        # Side-by-side: commanded vs actual
        win_ms = VEL_HALF * 2 * 5
        print(f"\n  Velocity window: {win_ms} ms  |  resolution ~10 mm/s")
        print(f"\n  {'#':>3}  {'t_sched':>8}  {'cmd_mm':>7}  {'cmd_%':>6}  "
              f"{'udp_mm':>7}  {'vel(mm/s)':>10}  {'err(mm)':>8}")
        print("  " + "-" * 60)
        errs = []
        for i, (ts, cmd_mm, cmd_pct) in enumerate(parsed):
            _, udp_mm, av = nearest(vels, ts)
            err = udp_mm - cmd_mm; errs.append(abs(err))
            print(f"  {i:>3}  {ts:>8.2f}  {cmd_mm:>7}  {cmd_pct:>6}  "
                  f"{udp_mm:>7.1f}  {av:>10.1f}  {err:>+8.1f}")

        v_peak  = max(v for _, _, v in vels)
        t_first = next((t for t, _, v in vels if v > 5.0), None)
        t_95    = next((t for t, _, v in vels if v >= 0.95 * v_peak), None)
        acc_ms  = (t_95 - t_first) * 1000 if t_first and t_95 else None
        shape   = classify_shape(vels, t_first, t_95) \
                  if t_first and t_95 and acc_ms else "n/a"
        mean_e  = statistics.mean(errs) if errs else 0.0
        max_e   = max(errs)             if errs else 0.0

        print()
        print("  -- Profile summary " + "-" * 52)
        if t_first:
            print(f"  {'Motor-start latency':35s}: {t_first*1000:.0f} ms")
        print(f"  {'Peak velocity (UDP)':35s}: {v_peak:.1f} mm/s")
        if acc_ms:
            print(f"  {'Acceleration phase':35s}: {acc_ms:.0f} ms")
        print(f"  {'Profile shape':35s}: {shape}")
        print(f"  {'Mean / max tracking error':35s}: {mean_e:.1f} / {max_e:.1f} mm")
        print(f"  {'Trajectory file':35s}: {_TRAJ_JSON.name}")
        print(f"  {'API errors':35s}: {errors}")
        print()

        # Assertions
        result("PASS" if errors == 0 else "FAIL",
               f"All {len(parsed)} waypoints dispatched ret=0" if errors == 0
               else f"{errors}/{len(parsed)} waypoints errored")

        result("PASS" if v_peak > 5 else "FAIL",
               f"Motor moved during trajectory  (peak={v_peak:.1f} mm/s)")

        if t_first is None:
            result("FAIL", "No movement detected in UDP feedback")
        elif t_first <= 0.5:
            result("PASS", "Motor-start latency <= 500ms",
                   f"{t_first*1000:.0f} ms")
        else:
            result("FAIL",
                   f"Motor-start latency {t_first*1000:.0f} ms > 500ms")

        result("PASS" if max_e <= 30 else "FAIL",
               f"Max tracking error <= 30 mm  ({max_e:.1f} mm)")

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
