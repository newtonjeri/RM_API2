"""C15 — Pole speed calibration. ONLY THE POLE MOVES (unless --with-arm).

Separates the two variables that are currently confounded in every sync
failure: the COMMANDED SPEED and whether the ARM IS MOVING.

What we know going in (2026-08-07):
    pole alone @ 50%   300 mm in 6.48 s  -> 48.4 mm/s   (model: 52.1)  OK
    during arm @ 28%   140 mm in 19.5 s  ->  7.2 mm/s   (model: 29.2)  4x slow
    during arm @ 37%   150 mm NEVER      -> <2.5 mm/s   (model: 38.5)  hung
Every accurate sample was pole-alone and every slow one was during an arm
move, so "slow at low speed%" and "slow while the arm moves" cannot be
told apart from the existing logs. This test measures them separately.

  Phase A  pole ALONE, one leg per speed% (down then up).
           -> is there a low-speed threshold below which the pole crawls
              or never completes?
  Phase B  (--with-arm) the SAME legs with a ready<->zero arm move
           dispatched lift-first alongside.
           -> does arm motion slow the pole, independently of speed%?

Comparing the two tables answers it: if Phase A is clean at every speed,
the cause is arm coupling; if Phase A already collapses below ~40%, it is
a drive/speed floor and the duration-matching model must clamp above it.

Safety: strokes stay inside the configured travel and every leg is bounded
by RM_LIFT_TIMEOUT_S; a leg that times out is reported and the sweep
continues at the next speed.

Flags: --speeds 10,20,...  --stroke MM  --with-arm
Arm selection: RM_ARM=left (default) or RM_ARM=right.
"""

import os
import sys
import time

from dual_arm_common import (
    handle_cli,
    preflight_error_gate, describe_error_state, error_state,
    ARM_SPEED_PCT, ARM_TIMEOUT_S, DEV_JOINT, DEV_LIFT, LEFT_IP, LIFT_M,
    LIFT_MM_S_PER_PCT, LIFT_TIMEOUT_S, RIGHT_IP, ROBOT_PORT,
    ArrivalMonitor, ConceptArm, apply_run_mode, countdown,
    home_poles_full, lift_hw_mm, lift_travel_time_s, mode_label,
    parse_clear_errors_arg, parse_mode_arg, report_run_modes,
    restore_run_modes, state_deg, teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
DEFAULT_SPEEDS = [10, 20, 30, 37, 40, 50, 70, 100]
DEFAULT_STROKE_MM = 100

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 4


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _arg_value(flag: str, default):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def _lift_leg(robot, monitor, handle_id, target_hw, pct):
    """One timed lift leg. Returns (seconds, arrived)."""
    monitor.expect(handle_id, DEV_LIFT)
    t0 = time.perf_counter()
    ret = robot.rm_set_lift_height(int(pct), int(target_hw), 0)
    if ret != 0:
        return None, False
    arrived, ok = monitor.wait(handle_id, DEV_LIFT, LIFT_TIMEOUT_S)
    return time.perf_counter() - t0, bool(arrived and ok)


def _row(pct, direction, dist_mm, secs, arrived):
    """Print one measurement row and return the measured mm/s (or None)."""
    model_s = lift_travel_time_s(dist_mm, pct,
                                 ascending=(direction == "up"))
    if not arrived or not secs:
        print(f"    {pct:3d}%  {direction:4s}  "
              f"{'DID NOT ARRIVE':>10s}  (model {model_s:5.2f} s)"
              f"   <-- incomplete")
        return None
    v = dist_mm / secs
    model_v = dist_mm / model_s
    print(f"    {pct:3d}%  {direction:4s}  {secs:8.2f} s  "
          f"(model {model_s:5.2f} s)   {v:6.1f} mm/s vs model {model_v:6.1f}"
          f"   ratio {model_s / secs:4.2f}x")
    return v


def main() -> int:
    for k in _results:
        _results[k] = 0
    handle_cli(__doc__, extra_flags=("--with-arm",),
               value_flags=("--speeds", "--stroke"))
    forced = parse_mode_arg()
    clear_errs = parse_clear_errors_arg()
    with_arm = "--with-arm" in sys.argv
    speeds = [int(s) for s in
              str(_arg_value("--speeds",
                             ",".join(map(str, DEFAULT_SPEEDS)))).split(",")
              if s.strip()]
    stroke = int(_arg_value("--stroke", DEFAULT_STROKE_MM))
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    top_hw = lift_hw_mm(ARM_SIDE, LIFT_M["full"])
    bottom_hw = top_hw - stroke

    print("=" * 68)
    print("C15  Pole speed calibration")
    print(f"    pole on arm={ARM_SIDE} @ {ip}   stroke {stroke} mm "
          f"({top_hw} <-> {bottom_hw} hw-mm)")
    print(f"    speeds: {speeds}")
    print("    Phase A: pole ALONE" + ("  +  Phase B: WITH a concurrent "
                                       "arm move (--with-arm)" if with_arm
                                       else "   (add --with-arm for Phase B)"))
    print(f"    mode: {mode_label(forced)}"
          + ("" if forced is not None else "  (select with --mode SIM|REAL)")
          + "   [the lift does NOT execute in SIM]")
    print(f"    THE {ARM_SIDE.upper()} POLE WILL MOVE"
          + (" — AND THE ARM in phase B" if with_arm else
             " — the arm stays still"))
    print("=" * 68)

    arm = None
    originals = {}
    try:
        robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
        if handle is None or handle.id <= 0:
            print(f"  [SKIP] Hardware not reachable at {ip}")
            _results["SKIP"] += N_CHECKS
            return 0
        arm = ConceptArm(ARM_SIDE, robot, handle)
        monitor = ArrivalMonitor()
        monitor.register(robot)
        originals = apply_run_mode(forced, arm)
        if originals is None:
            result("FAIL", "run-mode selection", "did not engage")
            return 1
        report_run_modes(arm)
        ok_err, err_detail = preflight_error_gate(arm, clear=clear_errs)
        if ok_err:
            result("PASS", "no latched controller errors", err_detail)
        else:
            result("FAIL", "no latched controller errors", err_detail)
            return 1
        countdown()

        if not home_poles_full(monitor, arm):
            result("FAIL", "pole homed to the top before the sweep")
            return 1
        result("PASS", "pole homed to the top before the sweep")

        # ── Phase A: pole alone ──
        print(f"\n  Phase A — pole ALONE ({stroke} mm legs)")
        print("  " + "─" * 66)
        alone = {}
        for pct in speeds:
            secs, arrived = _lift_leg(robot, monitor, arm.handle_id,
                                      bottom_hw, pct)
            alone[(pct, "down")] = _row(pct, "down", stroke, secs, arrived)
            secs, arrived = _lift_leg(robot, monitor, arm.handle_id,
                                      top_hw, pct)
            alone[(pct, "up")] = _row(pct, "up", stroke, secs, arrived)
            if alone[(pct, "up")] is None:
                # pole is not at the top any more — re-home before the next
                print("    (re-homing after an incomplete leg)")
                home_poles_full(monitor, arm)

        done = [v for v in alone.values() if v]
        if len(done) >= len(speeds):          # at least one leg per speed
            result("PASS", "phase A completed every speed",
                   f"{len(done)}/{len(alone)} legs arrived")
        else:
            stalled = sorted({p for (p, _), v in alone.items() if v is None})
            result("FAIL", "phase A completed every speed",
                   f"{len(done)}/{len(alone)} legs arrived; stalled at "
                   f"{stalled}% — a DRIVE/SPEED FLOOR, not arm coupling")

        # ── Phase B: same legs, arm moving ──
        if not with_arm:
            result("SKIP", "phase B (arm coupling)", "--with-arm not given")
            print("\n  Rerun with --with-arm to measure the coupling.")
        else:
            print(f"\n  Phase B — SAME legs with a concurrent arm move "
                  f"(lift dispatched first)")
            print("  " + "─" * 66)
            for pct in speeds:
                for direction, target_hw, pose in (
                        ("down", bottom_hw, "zero"), ("up", top_hw, "ready")):
                    monitor.expect(arm.handle_id, DEV_LIFT)
                    monitor.expect(arm.handle_id, DEV_JOINT)
                    t0 = time.perf_counter()
                    lret = robot.rm_set_lift_height(int(pct),
                                                    int(target_hw), 0)
                    aret = robot.rm_movej(state_deg(ARM_SIDE, pose),
                                          ARM_SPEED_PCT, 0, 0, 0)
                    if lret != 0 or aret != 0:
                        print(f"    {pct:3d}%  {direction:4s}  dispatch "
                              f"rejected (lift={lret} arm={aret})")
                        continue
                    l_ok = monitor.wait(arm.handle_id, DEV_LIFT,
                                        LIFT_TIMEOUT_S)
                    l_secs = time.perf_counter() - t0
                    a_ok = monitor.wait(arm.handle_id, DEV_JOINT,
                                        ARM_TIMEOUT_S)
                    with_v = _row(pct, direction, stroke, l_secs,
                                  l_ok[0] and l_ok[1])
                    base = alone.get((pct, direction))
                    if with_v and base:
                        print(f"           coupling: {base / with_v:4.2f}x "
                              f"slower than pole-alone")
                    if not (a_ok[0] and a_ok[1]):
                        print("           [WARN] the ARM did not arrive — "
                              "check joint errors")
                    st = error_state(arm)
                    if not (not st["sys"] and not st["joints"]
                            and not st["lift_err"]):
                        print(f"           [DIAG] {describe_error_state(st)}")
                        break
                else:
                    continue
                break
            result("PASS", "phase B measured", "see the coupling column")

        # ── fit ──
        clean = {p: v for (p, d), v in alone.items() if v and d == "down"}
        if clean:
            print("\n  Implied k (mm/s per %) from phase A, down legs:")
            for pct, v in sorted(clean.items()):
                print(f"    {pct:3d}%  ->  k = {v / pct:5.3f}"
                      f"   (model uses {LIFT_MM_S_PER_PCT:.3f})")
            result("PASS", "speed law characterized",
                   f"{len(clean)} clean points")
        else:
            result("FAIL", "speed law characterized", "no clean legs")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        if arm is not None:
            arm.halt()
        restore_run_modes(originals)
        teardown(arm)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
