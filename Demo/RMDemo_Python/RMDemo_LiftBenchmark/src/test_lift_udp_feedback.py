#!/usr/bin/env python3
"""
test_lift_udp_feedback.py
-------------------------
Measure UDP state push latency — the time between issuing a non-blocking
command and seeing the height change in the UDP realtime-state callback.

Also characterises observed packet intervals at three different push cycle
settings (cycle=1 / cycle=2 / cycle=4).

Run:
    python3 test_lift_udp_feedback.py

Exit codes:
    0  – assertions passed (or hardware absent)
    1  – one or more FAIL results
"""

import sys
import time
import statistics
import threading
from collections import deque
import pathlib

# Resolve SDK location relative to this file: src/ → … → RM_API2/Python
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))
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


_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


# ─── Shared UDP sample buffer (thread-safe) ──────────────────────────────────
_udp_buf   = deque(maxlen=4000)   # (timestamp_ns, height_mm)
_udp_lock  = threading.Lock()


@rm_realtime_arm_state_callback_ptr
def _udp_cb(state: rm_realtime_arm_joint_state_t):
    ts = time.perf_counter_ns()
    try:
        h = int(phys_height(state.liftState.height))
    except Exception:
        h = -1
    with _udp_lock:
        _udp_buf.append((ts, h))


def _configure_udp(robot: RoboticArm, cycle: int):
    cfg = rm_realtime_push_config_t(
        cycle=cycle,
        enable=True,
        port=UDP_PORT,
        ip=UDP_TARGET_IP,
        custom_config=rm_udp_custom_config_t(lift_state=1),
    )
    return robot.rm_set_realtime_push(cfg)


def _collect_udp(duration_s: float) -> list:
    """Snapshot the buffer, sleep, then return packets received in that window."""
    with _udp_lock:
        _udp_buf.clear()
    time.sleep(duration_s)
    with _udp_lock:
        return list(_udp_buf)


def _print_interval_stats(label: str, packets: list):
    if len(packets) < 2:
        print(f"  [INFO] {label}: < 2 packets — cannot compute intervals")
        return None, None
    intervals = [(packets[k+1][0] - packets[k][0]) / 1e6
                 for k in range(len(packets) - 1)]
    mean_ms = statistics.mean(intervals)
    std_ms  = statistics.stdev(intervals) if len(intervals) > 1 else 0.0
    print(f"  [INFO] {label}: n={len(packets)}  "
          f"mean={mean_ms:.2f} ms  std={std_ms:.2f} ms")
    return mean_ms, std_ms


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    print("=" * 64)
    print("Pole hardware: travel=300 mm, max_speed=100 mm/s, "
          "safe range=[10, 290] mm")
    print("=" * 64)
    print("test_lift_udp_feedback.py — UDP state push latency")
    print("=" * 64)

    robot  = None
    handle = None

    try:
        robot  = RoboticArm(rm_thread_mode_e(2))
        handle = robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)

        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ROBOT_IP}:{ROBOT_PORT}")
            _results["SKIP"] += 6
            return 0

        print(f"  [INFO] Connected (handle.id={handle.id})")

        # ── Register UDP callback ──────────────────────────────────────────
        rm_realtime_arm_state_call_back(_udp_cb)

        # ── Cycle sweep (characterise cycle parameter) ─────────────────────
        print("\n  ── Cycle parameter sweep ─────────────────────────────────")
        cycle_table = {1: "5 ms / 200 Hz", 2: "10 ms / 100 Hz",
                       4: "20 ms / 50 Hz"}

        for cyc, label in cycle_table.items():
            _configure_udp(robot, cyc)
            time.sleep(0.2)   # let config settle
            samples = _collect_udp(1.0)
            mean_ms, _ = _print_interval_stats(
                f"cycle={cyc} ({label})", samples)

        # ── Main latency test — cycle=2 (10 ms / 100 Hz) ──────────────────
        print("\n  ── Main latency test (cycle=2, 100 Hz) ───────────────────")
        _configure_udp(robot, 2)
        time.sleep(0.3)

        # Home to 50 mm (blocking, speed=50)
        print("  [INFO] Moving to 50 mm at speed=50% (blocking, ≤ 5 s) …")
        ret = robot.rm_set_lift_height(50, hw_height(50), True)
        if ret != 0:
            result("FAIL", f"Homing to 50 mm (ret={ret})")
            return 1
        print("  [INFO] At 50 mm.  Stabilising 0.5 s …")
        time.sleep(0.5)

        # Clear buffer and capture pre-command state
        with _udp_lock:
            _udp_buf.clear()

        # Issue timed non-blocking move: 50 mm → 250 mm, speed=50%
        # Expected travel: 200 mm / 50 mm/s = 4 s
        MOVE_TARGET   = 250
        MOVE_SPEED    = 50
        HEIGHT_THRESH = 230   # declare feedback received when height ≥ this

        t_cmd_ns = time.perf_counter_ns()
        ret = robot.rm_set_lift_height(MOVE_SPEED,
                                       hw_height(MOVE_TARGET), False)
        if ret != 0:
            result("FAIL", f"Non-blocking move command (ret={ret})")
            return 1

        print(f"  [INFO] Non-blocking move issued: 50→{MOVE_TARGET} mm "
              f"at speed={MOVE_SPEED}%")
        print(f"  [INFO] Expected travel: ~4 s  |  "
              f"Waiting up to 10 s for height ≥ {HEIGHT_THRESH} mm …")

        # Wait for UDP feedback to cross the threshold
        t_feedback_ns = None
        deadline_ns   = t_cmd_ns + int(10e9)  # 10 s

        # Collect samples every 20 ms
        all_samples = []
        while time.perf_counter_ns() < deadline_ns:
            time.sleep(0.02)
            with _udp_lock:
                new = list(_udp_buf)
            all_samples.extend(new)
            with _udp_lock:
                _udp_buf.clear()

            for ts, h in new:
                if h >= HEIGHT_THRESH and t_feedback_ns is None:
                    t_feedback_ns = ts

            if t_feedback_ns is not None:
                break

        # Collect any remaining packets
        time.sleep(0.5)
        with _udp_lock:
            final = list(_udp_buf)
        all_samples.extend(final)

        # ── Report ────────────────────────────────────────────────────────
        if t_feedback_ns is not None:
            latency_ms = (t_feedback_ns - t_cmd_ns) / 1e6
            print(f"\n  [INFO] First UDP ≥ {HEIGHT_THRESH} mm at "
                  f"latency = {latency_ms:.1f} ms from command issue")
        else:
            latency_ms = None
            print(f"\n  [INFO] Height never reached ≥ {HEIGHT_THRESH} mm "
                  "within 10 s")

        if len(all_samples) > 1:
            intervals = [(all_samples[k+1][0] - all_samples[k][0]) / 1e6
                         for k in range(len(all_samples)-1)]
            iv_mean   = statistics.mean(intervals)
            iv_std    = statistics.stdev(intervals) if len(intervals) > 1 else 0.0

            print(f"  [INFO] UDP packets received during move: {len(all_samples)}")
            print(f"  [INFO] Packet interval: mean={iv_mean:.2f} ms  "
                  f"std={iv_std:.2f} ms  (expected ~10 ms)")

            # Print every 10th sample to show motion profile
            print()
            print(f"  {'Sample':>6}  {'t from cmd (s)':>14}  {'height (mm)':>12}")
            print(f"  {'-'*6}  {'-'*14}  {'-'*12}")
            for idx in range(0, len(all_samples), max(1, len(all_samples)//40)):
                ts_s = (all_samples[idx][0] - t_cmd_ns) / 1e9
                print(f"  {idx:>6}  {ts_s:>14.3f}  {all_samples[idx][1]:>12}")

            # Assertions
            print()
            udp_lo, udp_hi = hw_baseline.udp_interval_window_ms(3.0)
            print(f"  [BASELINE] UDP interval window: [{udp_lo:.2f}, {udp_hi:.2f}] ms"
                  f"  [= mean ± 3σ from test 3]")
            if udp_lo <= iv_mean <= udp_hi:
                result("PASS",
                       f"UDP interval mean ∈ [{udp_lo:.1f}, {udp_hi:.1f}] ms"
                       f"  ({iv_mean:.2f} ms)")
            else:
                result("FAIL",
                       f"UDP interval mean {iv_mean:.2f} ms outside"
                       f" [{udp_lo:.1f}, {udp_hi:.1f}] ms")

            # Expected move ~4 s → ~400 packets at 100 Hz; assert ≥ 300
            if len(all_samples) >= 300:
                result("PASS",
                       f"UDP packet count ≥ 300  ({len(all_samples)} packets)")
            else:
                result("FAIL",
                       f"UDP packet count < 300  ({len(all_samples)} packets)")
        else:
            print("  [INFO] Insufficient UDP packets — check UDP config / firewall")
            result("FAIL", "UDP packet count < 2 (no feedback received)")
            result("FAIL", "UDP interval mean (no data)")

        # Expected travel time 4 s + 1 s buffer = 5 s
        if t_feedback_ns is not None:
            if latency_ms <= 5000.0:
                result("PASS",
                       f"Height ≥ {HEIGHT_THRESH} mm within 5 s  "
                       f"(latency={latency_ms:.0f} ms)")
            else:
                result("FAIL",
                       f"Height ≥ {HEIGHT_THRESH} mm only after {latency_ms:.0f} ms "
                       f"(limit 5000 ms)")
        else:
            result("FAIL", f"Height never reached ≥ {HEIGHT_THRESH} mm within 10 s")

    finally:
        if robot is not None and handle is not None:
            try:
                robot.rm_delete_robot_arm()
                RoboticArm.rm_destroy()
            except Exception:
                pass

    print()
    print("─" * 64)
    print(f"Results:  PASS={_results['PASS']}  "
          f"FAIL={_results['FAIL']}  SKIP={_results['SKIP']}")
    print("─" * 64)
    return 0 if _results["FAIL"] == 0 else 1


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
