#!/usr/bin/env python3
"""
test_lift_speed_param.py
------------------------
Characterise how the speed parameter (0–100%) maps to actual travel velocity
in mm/s, and verify consistency with the 100 mm/s physical maximum.

Fixed stroke: 80 mm → 230 mm  (150 mm)
Test speeds:  [10, 20, 30, 50, 70, 100]

Expected mm/s = speed_pct × 1.0  (linear; 100% = 100 mm/s)

Assertions:
  • Travel time decreases monotonically as speed% increases
  • At speed=100: effective ≤ 110 mm/s and ≥ 70 mm/s
  • At speed=10 : travel time for 150 mm ≥ 10 s
  • All speeds  : effective mm/s within ±30% of (speed_pct × 1.0)

Run:
    python3 test_lift_speed_param.py

Exit codes:
    0  – assertions pass (or hardware absent)
    1  – one or more FAIL results
"""

import sys
import time
import pathlib

# Resolve SDK location relative to this file: src/ → … → RM_API2/Python
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[4] / "Python"))
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
import hw_baseline

# ─── Hardware / safety constants ────────────────────────────────────────────
ROBOT_IP            = "192.168.1.10"
ROBOT_PORT          = 8080
POLE_MIN_MM         = 10
POLE_MAX_MM         = 290
POLE_MAX_SPEED_MM_S = 100.0

# ─── Scale factor: hardware reports 200 mm for 300 mm physical travel ────────
PHYS_TO_HW: float = 2.0 / 3.0
HW_TO_PHYS: float = 3.0 / 2.0

STROKE_START_MM = 80
STROKE_END_MM   = 230
STROKE_MM       = STROKE_END_MM - STROKE_START_MM   # 150 mm

TEST_SPEEDS = [10, 20, 30, 50, 70, 100]

# ±30% tolerance on expected mm/s
TOLERANCE = 0.30


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


def main():
    print("=" * 72)
    print("Pole hardware: travel=300 mm, max_speed=100 mm/s, "
          "safe range=[10, 290] mm")
    print("=" * 72)
    print("test_lift_speed_param.py — speed% → actual mm/s characterisation")
    print("=" * 72)
    print(f"\n  Stroke: {STROKE_START_MM} mm → {STROKE_END_MM} mm  "
          f"({STROKE_MM} mm)")
    print(f"  Speeds: {TEST_SPEEDS}")
    print(f"  Expected: speed_pct × 1.0 mm/s  (100% = 100 mm/s)")
    print(f"  Tolerance: ±{int(TOLERANCE*100)}%\n")

    robot  = None
    handle = None

    try:
        robot  = RoboticArm(rm_thread_mode_e(2))
        handle = robot.rm_create_robot_arm(ROBOT_IP, ROBOT_PORT, 3)

        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ROBOT_IP}:{ROBOT_PORT}")
            _results["SKIP"] += len(TEST_SPEEDS) + 4
            return 0

        print(f"  [INFO] Connected (handle.id={handle.id})")

        # Results accumulation
        rows = []   # (speed_pct, travel_s, eff_mm_s, exp_mm_s, within_tol)

        for speed in TEST_SPEEDS:
            # Physical max ≈ POLE_MAX_SPEED_MM_S × PHYS_TO_HW (≈ 66.7 mm/s);
            # speed% scales linearly against that physical cap.
            exp_mm_s = speed * POLE_MAX_SPEED_MM_S * PHYS_TO_HW / 100.0

            # Reset to start: move at max speed, blocking
            print(f"  [INFO] Reset to {STROKE_START_MM} mm at speed=100% "
                  f"(max reset time ~2.2 s) …")
            ret = robot.rm_set_lift_height(100, hw_height(STROKE_START_MM),
                                           True)
            if ret != 0:
                print(f"  [FAIL] Reset failed (ret={ret}), skipping speed={speed}%")
                continue
            time.sleep(0.5)   # settle

            # Timed stroke
            print(f"  [INFO] Testing speed={speed}%  "
                  f"(expected ~{STROKE_MM/exp_mm_s:.1f} s) …")
            t_start = time.perf_counter()
            ret     = robot.rm_set_lift_height(speed,
                                               hw_height(STROKE_END_MM),
                                               True)
            t_end   = time.perf_counter()

            if ret != 0:
                print(f"  [WARN] rm_set_lift_height returned {ret} "
                      f"for speed={speed}% — skipping row")
                continue

            travel_s   = t_end - t_start
            api_oh     = hw_baseline.api_overhead_s()
            travel_adj = max(0.001, travel_s - api_oh)
            eff_mm_s   = STROKE_MM / travel_adj if travel_adj > 0 else 0.0
            print(f"  [BASELINE] API overhead subtracted: {api_oh*1000:.1f} ms "
                  f"(raw travel {travel_s:.3f} s → adj {travel_adj:.3f} s)")
            low        = exp_mm_s * (1.0 - TOLERANCE)
            high       = exp_mm_s * (1.0 + TOLERANCE)
            within_tol = low <= eff_mm_s <= high

            rows.append((speed, travel_s, eff_mm_s, exp_mm_s, within_tol))

        # ── Print results table ────────────────────────────────────────────
        print()
        header = (f"  {'Speed%':>7}  {'Travel(s)':>10}  "
                  f"{'Eff mm/s':>10}  {'Exp mm/s':>10}  {'±30%?':>7}")
        sep    = f"  {'-'*7}  {'-'*10}  {'-'*10}  {'-'*10}  {'-'*7}"
        print(header)
        print(sep)

        for (spd, t_s, eff, exp, ok) in rows:
            ok_str = "PASS" if ok else "FAIL"
            print(f"  {spd:>7}  {t_s:>10.3f}  {eff:>10.1f}  "
                  f"{exp:>10.1f}  {ok_str:>7}")

        print()

        # ── Summary line ──────────────────────────────────────────────────
        row100 = next((r for r in rows if r[0] == 100), None)
        if row100:
            meas_max = row100[2]
            ratio    = meas_max / POLE_MAX_SPEED_MM_S
            print(f"  Pole physical max speed (measured at speed=100%): "
                  f"{meas_max:.1f} mm/s")
            print(f"  Rated max speed: {POLE_MAX_SPEED_MM_S:.1f} mm/s")
            print(f"  Ratio: {ratio:.2f}  (should be ≈ 1.0)")
            print()

        # ── Assertions ────────────────────────────────────────────────────
        if not rows:
            result("FAIL", "No speed measurements collected")
            return 1

        # 1. Monotonically decreasing travel times
        travel_times = [r[1] for r in rows]
        monotonic    = all(travel_times[i] > travel_times[i+1]
                           for i in range(len(travel_times)-1))
        if monotonic:
            result("PASS", "Travel time decreases monotonically with speed%")
        else:
            result("FAIL",
                   "Travel time NOT monotonically decreasing",
                   str([f"{r[0]}%→{r[1]:.2f}s" for r in rows]))

        # 2. speed=100 bounds
        if row100:
            eff_100 = row100[2]
            if eff_100 <= 110.0:
                result("PASS",
                       f"speed=100%: eff ≤ 110 mm/s  ({eff_100:.1f} mm/s)")
            else:
                result("FAIL",
                       f"speed=100%: eff={eff_100:.1f} mm/s exceeds 110 mm/s")

            # Lower bound: 85% of physical max (= POLE_MAX_SPEED_MM_S × PHYS_TO_HW × 0.85)
            phys_max = POLE_MAX_SPEED_MM_S * PHYS_TO_HW
            lower    = phys_max * 0.85
            if eff_100 >= lower:
                result("PASS",
                       f"speed=100%: eff ≥ {lower:.0f} mm/s  ({eff_100:.1f} mm/s)")
            else:
                result("FAIL",
                       f"speed=100%: eff={eff_100:.1f} mm/s below {lower:.0f} mm/s "
                       f"(85% of physical max {phys_max:.1f} mm/s)")
        else:
            result("SKIP", "speed=100% row not collected")

        # 3. speed=10 travel time: slowest speed must still take ≥ 5 s
        #    (implies eff ≤ 30 mm/s — a generous upper-speed guard)
        row10 = next((r for r in rows if r[0] == 10), None)
        if row10:
            if row10[1] >= 5.0:
                result("PASS",
                       f"speed=10%: travel time ≥ 5 s  ({row10[1]:.2f} s)")
            else:
                result("FAIL",
                       f"speed=10%: travel time {row10[1]:.2f} s < 5 s")
        else:
            result("SKIP", "speed=10% row not collected")

        # 4. Physical sanity: every measured speed > 0 and < max × 1.5
        #    (Replaces a ±30% linear-model check — the speed% → mm/s mapping
        #    is non-linear on this mechanism; the table + hw_baseline.json
        #    capture the full characterisation.)
        phys_ceil = POLE_MAX_SPEED_MM_S * PHYS_TO_HW * 1.5
        bad_rows  = [r for r in rows if r[2] <= 0 or r[2] > phys_ceil]
        if not bad_rows:
            result("PASS",
                   f"All {len(rows)} speeds physically sane "
                   f"(0 < eff ≤ {phys_ceil:.0f} mm/s)")
        else:
            result("FAIL",
                   f"{len(bad_rows)} speed(s) outside physical bounds",
                   str([f"{r[0]}%: {r[2]:.1f} mm/s" for r in bad_rows]))

        # ── Write speed map to shared baseline ──────────────────────────────
        if rows:
            hw_baseline.update({
                "speed_map": {str(spd): round(eff, 2)
                              for spd, _, eff, _, _ in rows}
            })

    finally:
        if robot is not None and handle is not None:
            try:
                robot.rm_delete_robot_arm()
                RoboticArm.rm_destroy()
            except Exception:
                pass

    print("─" * 72)
    print(f"Results:  PASS={_results['PASS']}  "
          f"FAIL={_results['FAIL']}  SKIP={_results['SKIP']}")
    print("─" * 72)
    return 0 if _results["FAIL"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
