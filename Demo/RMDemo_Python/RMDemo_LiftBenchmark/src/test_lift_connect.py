#!/usr/bin/env python3
"""
test_lift_connect.py
--------------------
Verify the RM_API2 Python bindings can initialise, connect to the RM-75,
and read basic lift state.  No motion is commanded.

Run:
    python3 test_lift_connect.py

Exit codes:
    0  – all tests passed (or skipped due to absent hardware)
    1  – one or more FAIL results
"""

import sys
import os
import time
import pathlib

# Resolve SDK location relative to this file: src/ → … → RM_API2/Python
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
import hw_baseline

# ─── Hardware / safety constants ────────────────────────────────────────────
ROBOT_IP         = "192.168.1.10"
ROBOT_PORT       = 8080
POLE_MIN_MM      = 10
POLE_MAX_MM      = 290
POLE_TRAVEL_MM   = 300
POLE_MAX_SPEED   = 100.0

# ─── Scale factor: hardware reports 200 mm for 300 mm physical travel ────────
# Physical mm × 2/3  → hardware command units (0–200 range)
# Hardware reported mm × 3/2  → physical mm (0–300 range)
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


# ─── Minimal result tracker ──────────────────────────────────────────────────
_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


# ─── Tests ───────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("Pole hardware: travel=300 mm, max_speed=100 mm/s, "
          "safe range=[10, 290] mm")
    print("=" * 60)
    print("test_lift_connect.py — connection & basic state (no motion)")
    print("=" * 60)

    robot  = None
    handle = None

    try:
        # ── Test 1: initialise SDK in triple-thread mode ─────────────────────
        try:
            robot = RoboticArm(rm_thread_mode_e(2))   # RM_TRIPLE_MODE_E = 2
            result("PASS", "T1: SDK init (TRIPLE mode)")
        except Exception as exc:
            result("FAIL", "T1: SDK init (TRIPLE mode)", str(exc))
            print("\nFATAL: cannot initialise SDK – aborting.\n")
            return 1

        # ── Test 2: connect to robot arm ────────────────────────────────────
        try:
            t_conn = time.perf_counter()
            handle = robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)
            connect_ms = (time.perf_counter() - t_conn) * 1000.0
            if handle is None or handle.id <= 0:
                print(f"  [SKIP] T2–T7: hardware not reachable at "
                      f"{ROBOT_IP}:{ROBOT_PORT}  (handle.id="
                      f"{getattr(handle,'id',None)})")
                _results["SKIP"] += 6
                return 0
            result("PASS", f"T2: rm_create_robot_arm (handle.id={handle.id}, "
                           f"connect={connect_ms:.0f} ms)")
            hw_baseline.update({"connect_time_ms": connect_ms})
        except Exception as exc:
            print(f"  [SKIP] T2–T7: hardware not reachable – {exc}")
            _results["SKIP"] += 6
            return 0

        # ── Test 3: get robot info, confirm RM-75 (arm_dof = 7) ─────────────
        try:
            ret, info = robot.rm_get_robot_info()
            if ret == 0:
                dof = info.get("arm_dof", -1)
                if dof == 7:
                    result("PASS", f"T3: rm_get_robot_info (arm_dof={dof} → RM-75)")
                else:
                    result("FAIL", f"T3: rm_get_robot_info (arm_dof={dof}, expected 7)")
            else:
                result("FAIL", f"T3: rm_get_robot_info (ret={ret})")
        except Exception as exc:
            result("FAIL", "T3: rm_get_robot_info", str(exc))

        # ── Test 4: rm_get_lift_state returns ret=0, pos in [0, 300] ────────
        try:
            ret, state = robot.rm_get_lift_state()
            if ret != 0:
                result("FAIL", f"T4: rm_get_lift_state (ret={ret})")
            else:
                pos = phys_height(state.get("pos", 0))
                if 0 <= pos <= POLE_TRAVEL_MM:
                    result("PASS",
                           f"T4: rm_get_lift_state (pos={pos:.1f} mm physical, "
                           f"within [0, {POLE_TRAVEL_MM}])")
                else:
                    result("FAIL",
                           f"T4: rm_get_lift_state (pos={pos:.1f} mm physical out of "
                           f"range [0, {POLE_TRAVEL_MM}])")
        except Exception as exc:
            result("FAIL", "T4: rm_get_lift_state", str(exc))

        # ── Test 5: err_flag == 0 ────────────────────────────────────────────
        try:
            ret, state = robot.rm_get_lift_state()
            if ret == 0:
                err = state.get("err_flag", -1)
                if err == 0:
                    result("PASS", "T5: err_flag == 0 (no errors)")
                else:
                    result("FAIL", f"T5: err_flag = {err} (expected 0)")
            else:
                result("FAIL", f"T5: rm_get_lift_state failed (ret={ret})")
        except Exception as exc:
            result("FAIL", "T5: err_flag check", str(exc))

        # ── Test 6: en_flag — note: rm_expand_state_t has no en_flag field;
        #    en_flag is only available via the UDP realtime state callback.
        #    Skipping gracefully here.
        print("  [SKIP] T6: en_flag (not present in rm_expand_state_t; "
              "use UDP callback for en_flag)")
        _results["SKIP"] += 1

        # ── Test 7: clean teardown ───────────────────────────────────────────
        del_ret  = robot.rm_delete_robot_arm()
        dest_ret = RoboticArm.rm_destroy()
        handle   = None       # prevent double-delete in finally

        if del_ret == 0 and dest_ret == 0:
            result("PASS",
                   f"T7: teardown (rm_delete_robot_arm={del_ret}, "
                   f"rm_destroy={dest_ret})")
        else:
            result("FAIL",
                   f"T7: teardown (rm_delete_robot_arm={del_ret}, "
                   f"rm_destroy={dest_ret})")

    finally:
        if robot is not None and handle is not None:
            try:
                robot.rm_delete_robot_arm()
                RoboticArm.rm_destroy()
            except Exception:
                pass

    # ── Summary ──────────────────────────────────────────────────────────────
    print()
    print("─" * 60)
    print(f"Results:  PASS={_results['PASS']}  "
          f"FAIL={_results['FAIL']}  SKIP={_results['SKIP']}")
    print("─" * 60)
    return 0 if _results["FAIL"] == 0 else 1


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
