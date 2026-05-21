#!/usr/bin/env python3
"""
test_lift_nonblocking_rate.py
-----------------------------
Measure the maximum rate at which rm_set_lift_height(speed, height, block=False)
can be called without queueing or error.  Also runs a controlled rate sweep to
show which rates the API accepts cleanly.

Run:
    python3 test_lift_nonblocking_rate.py

Exit codes:
    0  – assertions passed (or hardware absent)
    1  – one or more FAIL results
"""

import sys
import time
import statistics
import pathlib

# Resolve SDK location relative to this file: src/ → … → RM_API2/Python
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

# ─── Hardware / safety constants ────────────────────────────────────────────
ROBOT_IP              = "192.168.1.10"
ROBOT_PORT            = 8080
POLE_MIN_MM           = 10
POLE_MAX_MM           = 290
POLE_MAX_SPEED_MM_S   = 100.0

# ─── Scale factor: hardware reports 200 mm for 300 mm physical travel ────────
PHYS_TO_HW: float = 2.0 / 3.0
HW_TO_PHYS: float = 3.0 / 2.0

# Test parameters
MID_HEIGHT_MM   = 150   # safe mid-point for homing
ALT_HEIGHT_A_MM = 140   # oscillation endpoint A
ALT_HEIGHT_B_MM = 160   # oscillation endpoint B
N_BURST         = 200   # commands in burst test
HOME_SPEED_PCT  = 50    # speed% for homing move (~50 mm/s)


def safe_height(h: int) -> int:
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


_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def skip_all(n: int, reason: str):
    print(f"  [SKIP] {reason}")
    _results["SKIP"] += n


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("Pole hardware: travel=300 mm, max_speed=100 mm/s, "
          "safe range=[10, 290] mm")
    print("=" * 60)
    print("test_lift_nonblocking_rate.py — non-blocking command rate")
    print("=" * 60)

    robot  = None
    handle = None

    try:
        robot  = RoboticArm(rm_thread_mode_e(2))
        handle = robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)

        if handle is None or handle.id <= 0:
            skip_all(10, f"Hardware not reachable at {ROBOT_IP}:{ROBOT_PORT}")
            return 0

        print(f"  [INFO] Connected (handle.id={handle.id})")

        # ── Step 1: home to mid-height (blocking) ────────────────────────────
        print(f"\n  [INFO] Homing to {MID_HEIGHT_MM} mm at speed={HOME_SPEED_PCT}% "
              f"(blocking, expect ≤ 3 s from any position) …")
        t0 = time.perf_counter()
        ret = robot.rm_set_lift_height(HOME_SPEED_PCT,
                                       hw_height(MID_HEIGHT_MM), True)
        home_time = time.perf_counter() - t0
        if ret != 0:
            result("FAIL", f"Homing move (ret={ret})", "cannot continue")
            return 1
        print(f"  [INFO] Homed in {home_time:.2f} s (ret={ret})")

        # ── Step 2: burst test — N non-blocking commands ─────────────────────
        print(f"\n  [INFO] Burst: sending {N_BURST} non-blocking commands "
              f"(alternating {ALT_HEIGHT_A_MM}/{ALT_HEIGHT_B_MM} mm) …")

        heights    = [ALT_HEIGHT_A_MM if i % 2 == 0 else ALT_HEIGHT_B_MM
                      for i in range(N_BURST)]
        latencies  = []   # per-call wall-time in µs
        error_count = 0

        t_burst_start = time.perf_counter()
        for h in heights:
            t_call = time.perf_counter()
            ret = robot.rm_set_lift_height(20, hw_height(h), False)
            latency_us = (time.perf_counter() - t_call) * 1e6
            latencies.append(latency_us)
            if ret != 0:
                error_count += 1

        total_burst_s = time.perf_counter() - t_burst_start

        mean_lat   = statistics.mean(latencies)
        min_lat    = min(latencies)
        max_lat    = max(latencies)
        cmd_rate   = N_BURST / total_burst_s
        pass_rate  = (N_BURST - error_count) / N_BURST

        print()
        print(f"  {'Burst results':30s}")
        print(f"  {'N commands':30s}: {N_BURST}")
        print(f"  {'Total wall time':30s}: {total_burst_s*1000:.1f} ms")
        print(f"  {'Mean per-call latency':30s}: {mean_lat:.1f} µs")
        print(f"  {'Min per-call latency':30s}: {min_lat:.1f} µs")
        print(f"  {'Max per-call latency':30s}: {max_lat:.1f} µs")
        print(f"  {'Achieved command rate':30s}: {cmd_rate:.1f} Hz")
        print(f"  {'Non-zero returns (errors)':30s}: {error_count}")

        # Assertions
        if mean_lat <= 5000.0:
            result("PASS",
                   f"Mean per-call latency ≤ 5 ms  ({mean_lat:.1f} µs)")
        else:
            result("FAIL",
                   f"Mean per-call latency > 5 ms  ({mean_lat:.1f} µs)",
                   "non-blocking call too slow")

        if pass_rate >= 0.80:
            result("PASS",
                   f"≥ 80% calls returned 0  ({(pass_rate*100):.1f}%)")
        else:
            result("FAIL",
                   f"< 80% calls returned 0  ({(pass_rate*100):.1f}%)",
                   f"error count={error_count}")

        # ── Step 3: rate sweep ────────────────────────────────────────────────
        print()
        print("  [INFO] Physical note: at 100 mm/s max, a 20 mm stroke takes ≥ 0.2 s.")
        print("  [INFO] Commanding faster than 5 Hz will outpace the mechanism.")
        print("  [INFO] API rate (how fast commands are accepted) is measured "
              "separately above.")
        print()

        sweep_rates = [10, 25, 50]   # Hz — do NOT test above 50 Hz
        N_SWEEP     = 50

        print(f"  {'Rate (Hz)':>10}  {'Errors':>8}  {'Actual Hz':>10}  Note")
        print(f"  {'-'*10}  {'-'*8}  {'-'*10}  {'-'*30}")

        for rate in sweep_rates:
            period_s     = 1.0 / rate
            sweep_errors = 0
            t_sweep_start = time.perf_counter()

            for i in range(N_SWEEP):
                t_cmd = time.perf_counter()
                h = ALT_HEIGHT_A_MM if i % 2 == 0 else ALT_HEIGHT_B_MM
                ret = robot.rm_set_lift_height(20, hw_height(h), False)
                if ret != 0:
                    sweep_errors += 1
                # sleep remainder of period
                elapsed = time.perf_counter() - t_cmd
                remaining = period_s - elapsed
                if remaining > 0:
                    time.sleep(remaining)

            actual_hz  = N_SWEEP / (time.perf_counter() - t_sweep_start)
            note = ("above 5 Hz physical limit"
                    if rate > 5 else "within physical limit")
            print(f"  {rate:>10}  {sweep_errors:>8}  {actual_hz:>10.1f}  {note}")

        print()

    finally:
        if robot is not None and handle is not None:
            try:
                robot.rm_delete_robot_arm()
                RoboticArm.rm_destroy()
            except Exception:
                pass

    # ── Summary ──────────────────────────────────────────────────────────────
    print("─" * 60)
    print(f"Results:  PASS={_results['PASS']}  "
          f"FAIL={_results['FAIL']}  SKIP={_results['SKIP']}")
    print("─" * 60)
    return 0 if _results["FAIL"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
