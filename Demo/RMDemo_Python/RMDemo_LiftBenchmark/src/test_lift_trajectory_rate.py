#!/usr/bin/env python3
"""
test_lift_trajectory_rate.py
----------------------------
Variant of test_lift_nonblocking_rate.py that accepts trajectory waypoints
expressed as (time_from_start, position_m, velocity_m_s) — the same format
produced by ROS2 JointTrajectory messages — and converts them to RM_API2
``rm_set_lift_height(speed_pct, height_mm, block=False)`` calls.

This lets you replay a pre-planned motion profile (e.g. from a MoveIt / nav
planner) directly over the RM_API2 Python bindings, bypassing the ROS2
rm_driver / rm_control stack entirely.

Conversion rules
~~~~~~~~~~~~~~~~
    position_mm  = int(round(position_m  * 1000))          # metres  → mm
    velocity_mm_s = velocity_m_s * 1000                     # m/s     → mm/s
    speed_pct    = max(1, min(100, int(round(velocity_mm_s))))
        # Because the pole's rated max is 100 mm/s = 100%:
        #   0.1 m/s  → 100 mm/s → 100%
        #   0.05 m/s →  50 mm/s →  50%
        #   0.0005 m/s → 0.5 mm/s → rounds to 1% minimum

Trajectory execution
~~~~~~~~~~~~~~~~~~~~
Waypoints are executed by issuing non-blocking position commands timed to
the ``time_from_start`` schedule:

    for each waypoint[i]:
        dt = waypoint[i].time_from_start - waypoint[i-1].time_from_start
        issue rm_set_lift_height(speed_pct, height_mm, block=False)
        sleep(dt)   # pace the next command

Performance metrics reported
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  • Per-command call latency (µs)
  • Command timing jitter (deviation from scheduled dt)
  • API error counts
  • Motion profile (actual heights, if UDP feedback is wired up — else
    a note that the data came from the command stream only)

Run:
    python3 test_lift_trajectory_rate.py

Exit codes:
    0  – assertions passed (or hardware absent)
    1  – one or more FAIL results
"""

import sys
import time
import statistics
from collections import deque

sys.path.insert(0, '/home/newtonjeri/realman_API/RM_API2/Python')
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import (
    rm_thread_mode_e,
    rm_realtime_push_config_t,
    rm_udp_custom_config_t,
    rm_realtime_arm_state_callback_ptr,
    rm_realtime_arm_state_call_back,
    rm_realtime_arm_joint_state_t,
)

# ─── Hardware / safety constants ────────────────────────────────────────────
ROBOT_IP            = "192.168.1.10"
ROBOT_PORT          = 8080
UDP_TARGET_IP       = "192.168.1.11"
UDP_PORT            = 8089
POLE_MIN_MM         = 10
POLE_MAX_MM         = 290
POLE_MAX_SPEED_MM_S = 100.0

# ─── Scale factor: hardware reports 200 mm for 300 mm physical travel ────────
PHYS_TO_HW: float = 2.0 / 3.0
HW_TO_PHYS: float = 3.0 / 2.0


def safe_height(h: int) -> int:
    """Validate that a physical height is within safe operating limits."""
    assert POLE_MIN_MM <= h <= POLE_MAX_MM, (
        f"Height {h} mm violates pole limits [{POLE_MIN_MM}, {POLE_MAX_MM}]"
    )
    return h


def hw_height(phys_mm: int) -> int:
    """Validate physical height then convert to hardware command units (×2/3)."""
    safe_height(phys_mm)
    return int(round(phys_mm * PHYS_TO_HW))


def phys_height(hw_mm) -> float:
    """Convert hardware-reported height to physical mm (×3/2)."""
    return hw_mm * HW_TO_PHYS


# ─── Unit-conversion helpers ─────────────────────────────────────────────────

def pos_m_to_mm(pos_m: float) -> int:
    """Convert a joint position in metres to millimetres (integer)."""
    return int(round(pos_m * 1000.0))


def vel_m_s_to_pct(vel_m_s: float) -> int:
    """Convert a joint velocity in m/s to an RM_API2 speed percentage.

    Mapping (pole rated max = 100 mm/s = 100%):
        0.10  m/s  →  100 mm/s  →  100%
        0.05  m/s  →   50 mm/s  →   50%
        0.005 m/s  →    5 mm/s  →    5%
        ≤ 0.001 m/s → clamped to 1% minimum
    """
    mm_s = vel_m_s * 1000.0
    pct  = int(round(mm_s))
    return max(1, min(100, pct))


# ─── Result tracker ──────────────────────────────────────────────────────────
_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


# ─── Trajectory data ─────────────────────────────────────────────────────────
# Mirrors the JSON trajectory supplied by the user.
# Units: time_from_start in seconds, positions in metres, velocities in m/s.
# Accelerations are included for completeness but ignored by this API
# (the RM_API2 rm_set_lift_height interface does not accept an acceleration
# parameter; the controller uses its own internal profile).
TRAJECTORY = [
    {"time_from_start": 0.0,   "positions": [0.075000], "velocities": [0.000500], "accelerations": [ 0.500000]},
    {"time_from_start": 0.1,   "positions": [0.077500], "velocities": [0.050000], "accelerations": [ 0.500000]},
    {"time_from_start": 0.2,   "positions": [0.085000], "velocities": [0.100000], "accelerations": [ 0.500000]},
    {"time_from_start": 0.3,   "positions": [0.095000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 0.4,   "positions": [0.105000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 0.5,   "positions": [0.115000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 0.6,   "positions": [0.125000], "velocities": [0.100000], "accelerations": [ 0.000000]},
    {"time_from_start": 0.7,   "positions": [0.135000], "velocities": [0.100000], "accelerations": [ 0.000000]},
    {"time_from_start": 0.8,   "positions": [0.145000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 0.9,   "positions": [0.155000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.0,   "positions": [0.165000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.1,   "positions": [0.175000], "velocities": [0.100000], "accelerations": [ 0.000000]},
    {"time_from_start": 1.2,   "positions": [0.185000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.3,   "positions": [0.195000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.4,   "positions": [0.205000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.5,   "positions": [0.215000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.6,   "positions": [0.225000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.7,   "positions": [0.235000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.8,   "positions": [0.245000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 1.9,   "positions": [0.255000], "velocities": [0.100000], "accelerations": [-0.000000]},
    {"time_from_start": 2.0,   "positions": [0.265000], "velocities": [0.100000], "accelerations": [ 0.000000]},
    {"time_from_start": 2.1,   "positions": [0.275000], "velocities": [0.100000], "accelerations": [ 0.000000]},
    {"time_from_start": 2.2,   "positions": [0.285000], "velocities": [0.100000], "accelerations": [ 0.000000]},
    {"time_from_start": 2.3,   "positions": [0.292500], "velocities": [0.050000], "accelerations": [-0.500000]},
    {"time_from_start": 2.4,   "positions": [0.295000], "velocities": [0.000000], "accelerations": [-0.500000]},
    # duplicate final waypoint (as in original JSON)
    {"time_from_start": 2.4,   "positions": [0.295000], "velocities": [0.000000], "accelerations": [-0.500000]},
]


def parse_trajectory(traj):
    """Convert raw trajectory waypoints to (t_s, height_mm, speed_pct) tuples.

    Waypoints with velocity == 0 are mapped to speed_pct = 1 so the controller
    still accepts the position target (the arm will use its minimum speed to
    creep to the setpoint and stop there).  Duplicate timestamps are removed
    (keep first occurrence).

    Returns a list of (time_from_start_s, height_mm, speed_pct).
    """
    seen_t = set()
    parsed = []
    for wp in traj:
        t   = wp["time_from_start"]
        pos = wp["positions"][0]
        vel = wp["velocities"][0]

        # Skip duplicate timestamps
        if t in seen_t:
            continue
        seen_t.add(t)

        height_mm = pos_m_to_mm(pos)
        speed_pct = vel_m_s_to_pct(vel)   # 0 velocity → 1% minimum

        # Clamp to safe limits
        height_mm = max(POLE_MIN_MM, min(POLE_MAX_MM, height_mm))

        parsed.append((t, height_mm, speed_pct))

    return parsed


def print_conversion_table(parsed):
    """Pretty-print the converted trajectory."""
    print()
    print(f"  {'#':>3}  {'t (s)':>6}  {'pos (m)':>8}  {'vel (m/s)':>10}  "
          f"{'→ mm':>6}  {'→ %':>5}  {'clamped?':>8}")
    print(f"  {'-'*3}  {'-'*6}  {'-'*8}  {'-'*10}  {'-'*6}  {'-'*5}  {'-'*8}")

    for i, (wp, (t, h_mm, spct)) in enumerate(
            zip(TRAJECTORY, parse_trajectory(TRAJECTORY))):
        raw_mm  = pos_m_to_mm(wp["positions"][0])
        clamped = "YES" if raw_mm != h_mm else ""
        print(f"  {i:>3}  {t:>6.1f}  {wp['positions'][0]:>8.6f}  "
              f"{wp['velocities'][0]:>10.6f}  {h_mm:>6}  {spct:>5}  {clamped:>8}")
    print()


def main():
    print("=" * 66)
    print("Pole hardware: travel=300 mm, max_speed=100 mm/s, "
          "safe range=[10, 290] mm")
    print("=" * 66)
    print("test_lift_trajectory_rate.py — trajectory waypoint replay "
          "(pos + vel inputs)")
    print("=" * 66)

    # ── Parse & display trajectory ─────────────────────────────────────────
    parsed = parse_trajectory(TRAJECTORY)

    print(f"\n  [INFO] Trajectory: {len(parsed)} unique waypoints  "
          f"(total duration {parsed[-1][0]:.1f} s)")
    print(f"  [INFO] First waypoint : t={parsed[0][0]:.1f}s  "
          f"pos={parsed[0][1]} mm  speed={parsed[0][2]}%")
    print(f"  [INFO] Last waypoint  : t={parsed[-1][0]:.1f}s  "
          f"pos={parsed[-1][1]} mm  speed={parsed[-1][2]}%")

    print_conversion_table(parsed)

    print("  [INFO] Physical note: at 100 mm/s max, a 20 mm stroke takes ≥ 0.2 s.")
    print("  [INFO] Commanding faster than 5 Hz will outpace the mechanism.")
    print("  [INFO] API rate (how fast commands are accepted) is measured "
          "below.")
    print()

    robot  = None
    handle = None

    # ── UDP feedback buffer ────────────────────────────────────────────────
    udp_samples = deque(maxlen=2000)   # (timestamp_ns, height_mm)

    @rm_realtime_arm_state_callback_ptr
    def on_udp_state(state: rm_realtime_arm_joint_state_t):
        ts  = time.perf_counter_ns()
        try:
            h = int(phys_height(state.liftState.height))
        except Exception:
            h = -1
        udp_samples.append((ts, h))

    try:
        robot  = RoboticArm(rm_thread_mode_e(2))
        handle = robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)

        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ROBOT_IP}:{ROBOT_PORT}")
            _results["SKIP"] += 4
            return 0

        print(f"  [INFO] Connected (handle.id={handle.id})")

        # Register UDP callback (best-effort; may show no data if UDP not
        # configured or firewall blocks packets)
        rm_realtime_arm_state_call_back(on_udp_state)

        # Configure UDP push (lift_state=1 enables liftState field)
        cfg = rm_realtime_push_config_t(
            cycle=2,
            enable=True,
            port=UDP_PORT,
            ip=UDP_TARGET_IP,
            custom_config=rm_udp_custom_config_t(lift_state=1),
        )
        robot.rm_set_realtime_push(cfg)
        time.sleep(0.3)   # let push config take effect

        # ── Home to trajectory start position (blocking) ───────────────────
        start_mm  = parsed[0][1]
        print(f"  [INFO] Homing to trajectory start: {start_mm} mm "
              f"at speed=50% (blocking, expect ≤ 3 s) …")
        t_home = time.perf_counter()
        ret = robot.rm_set_lift_height(50, hw_height(start_mm), True)
        if ret != 0:
            result("FAIL", f"Homing to start position (ret={ret})")
            return 1
        print(f"  [INFO] Homed in {time.perf_counter()-t_home:.2f} s")
        time.sleep(0.3)   # settle

        # ── Execute trajectory ─────────────────────────────────────────────
        print(f"\n  [INFO] Executing {len(parsed)} trajectory waypoints …")
        print(f"  {'#':>3}  {'t_sched (s)':>11}  {'t_actual (s)':>12}  "
              f"{'jitter (ms)':>11}  {'mm':>5}  {'%':>4}  {'ret':>4}  "
              f"{'lat (µs)':>10}")
        print(f"  {'-'*3}  {'-'*11}  {'-'*12}  {'-'*11}  {'-'*5}  "
              f"{'-'*4}  {'-'*4}  {'-'*10}")

        latencies_us  = []
        jitters_ms    = []
        error_count   = 0
        cmd_log       = []   # (t_actual, height_mm, speed_pct, ret)

        udp_samples.clear()
        t_traj_start = time.perf_counter()

        for i, (t_sched, height_mm, speed_pct) in enumerate(parsed):
            # Wait until scheduled time relative to trajectory start
            t_target = t_traj_start + t_sched
            now = time.perf_counter()
            if t_target > now:
                time.sleep(t_target - now)

            t_before = time.perf_counter()
            ret = robot.rm_set_lift_height(speed_pct,
                                           hw_height(height_mm), False)
            lat_us = (time.perf_counter() - t_before) * 1e6
            t_actual  = t_before - t_traj_start
            jitter_ms = (t_actual - t_sched) * 1000.0

            latencies_us.append(lat_us)
            jitters_ms.append(abs(jitter_ms))
            if ret != 0:
                error_count += 1
            cmd_log.append((t_actual, height_mm, speed_pct, ret))

            print(f"  {i:>3}  {t_sched:>11.3f}  {t_actual:>12.3f}  "
                  f"{jitter_ms:>+11.1f}  {height_mm:>5}  {speed_pct:>4}  "
                  f"{ret:>4}  {lat_us:>10.1f}")

        total_traj_s = time.perf_counter() - t_traj_start

        # Wait for final motion to complete (final vel=0, allow extra time)
        print(f"\n  [INFO] Trajectory dispatch done in {total_traj_s:.3f} s "
              f"(scheduled {parsed[-1][0]:.3f} s)")
        print("  [INFO] Waiting up to 4 s for final waypoint motion …")
        time.sleep(4.0)

        # ── Statistics ────────────────────────────────────────────────────
        mean_lat   = statistics.mean(latencies_us)
        max_lat    = max(latencies_us)
        min_lat    = min(latencies_us)
        mean_jit   = statistics.mean(jitters_ms)
        max_jit    = max(jitters_ms)

        n_cmds     = len(parsed)
        pass_rate  = (n_cmds - error_count) / n_cmds

        print()
        print("  ── Trajectory execution statistics ──────────────────────")
        print(f"  {'Waypoints dispatched':35s}: {n_cmds}")
        print(f"  {'Total dispatch time':35s}: {total_traj_s*1000:.1f} ms")
        print(f"  {'Mean per-call latency':35s}: {mean_lat:.1f} µs")
        print(f"  {'Min / Max per-call latency':35s}: "
              f"{min_lat:.1f} / {max_lat:.1f} µs")
        print(f"  {'Mean timing jitter':35s}: {mean_jit:.1f} ms")
        print(f"  {'Max timing jitter':35s}: {max_jit:.1f} ms")
        print(f"  {'API errors (non-zero ret)':35s}: {error_count}")

        # ── UDP feedback summary ──────────────────────────────────────────
        if len(udp_samples) > 1:
            udp_list = list(udp_samples)
            intervals_ms = [
                (udp_list[k+1][0] - udp_list[k][0]) / 1e6
                for k in range(len(udp_list) - 1)
            ]
            print(f"\n  ── UDP feedback (observed during trajectory) ────────")
            print(f"  {'UDP packets received':35s}: {len(udp_list)}")
            print(f"  {'Mean packet interval':35s}: "
                  f"{statistics.mean(intervals_ms):.1f} ms")
            print(f"  {'Std dev interval':35s}: "
                  f"{statistics.stdev(intervals_ms):.2f} ms")
            print()
            print(f"  {'Sample':>6}  {'t (s)':>8}  {'height (mm)':>12}")
            print(f"  {'-'*6}  {'-'*8}  {'-'*12}")
            for idx in range(0, len(udp_list), max(1, len(udp_list)//15)):
                ts_s = (udp_list[idx][0] - udp_list[0][0]) / 1e9
                print(f"  {idx:>6}  {ts_s:>8.3f}  {udp_list[idx][1]:>12}")
        else:
            print("\n  [INFO] No UDP feedback packets received "
                  "(check UDP config / firewall)")

        # ── Assertions ───────────────────────────────────────────────────
        print()
        if mean_lat <= 5000.0:
            result("PASS",
                   f"Mean per-call latency ≤ 5 ms  ({mean_lat:.1f} µs)")
        else:
            result("FAIL",
                   f"Mean per-call latency > 5 ms  ({mean_lat:.1f} µs)")

        if pass_rate >= 0.80:
            result("PASS",
                   f"≥ 80% calls returned 0  ({pass_rate*100:.1f}%)")
        else:
            result("FAIL",
                   f"< 80% calls returned 0  ({pass_rate*100:.1f}%)",
                   f"errors={error_count}")

        if max_jit <= 50.0:
            result("PASS",
                   f"Max timing jitter ≤ 50 ms  ({max_jit:.1f} ms)")
        else:
            result("FAIL",
                   f"Max timing jitter > 50 ms  ({max_jit:.1f} ms)",
                   "system under load or scheduler latency")

        # Verify final position was reached (within 10 mm)
        ret_s, final_state = robot.rm_get_lift_state()
        if ret_s == 0:
            final_pos = phys_height(final_state.get("pos", 0))
            target    = parsed[-1][1]
            if abs(final_pos - target) <= 10:
                result("PASS",
                       f"Final position within 10 mm of target "
                       f"(pos={final_pos} mm, target={target} mm)")
            else:
                result("FAIL",
                       f"Final position {final_pos} mm, target {target} mm "
                       f"(deviation={abs(final_pos-target)} mm > 10 mm)")
        else:
            result("FAIL", f"Could not read final lift state (ret={ret_s})")

    finally:
        if robot is not None and handle is not None:
            try:
                robot.rm_delete_robot_arm()
                RoboticArm.rm_destroy()
            except Exception:
                pass

    # ── Summary ──────────────────────────────────────────────────────────────
    print()
    print("─" * 66)
    print(f"Results:  PASS={_results['PASS']}  "
          f"FAIL={_results['FAIL']}  SKIP={_results['SKIP']}")
    print("─" * 66)
    return 0 if _results["FAIL"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
