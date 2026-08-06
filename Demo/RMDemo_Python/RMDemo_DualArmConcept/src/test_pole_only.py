"""C8 — Pole-only diagnostic. ONLY THE SELECTED POLE MOVES (no arm, no hand).

Built for the 2026-08-06 20:38 incident: BOTH controllers suddenly rejected
every lift command (rm_set_lift_height / rm_set_lift_speed -> ret=1,
"[...] set_state: false") that had physically worked minutes earlier with
identical parameters — i.e. an ARM-STATE error, not a script problem.

Reads first, moves second:

  D1  connect
  D2  arm power state          (OFF -> e-stop pressed / arm powered down)
  D3  controller + joint errors (latched system error blocks motion)
  D4  raw lift state            (pos / current / mode / driver err_flag)
  D5  optional --clear-errors   (rm_clear_system_err — clears latched
      system errors; does NOT move anything)
  D6  ACCEPTANCE probe: command the pole to its CURRENT position
      (zero-distance, no physical motion) and await the arrival event —
      the pure "will the controller take lift commands?" test
  D7  small stroke: 10 hw-mm away and back, arrival-event verified
  D8  home to full length (the pre-run state every motion test needs)

On a healthy arm all eight PASS.  In the rejection state D6 FAILs and the
state dump names the blocking condition.  Recovery ladder: release/reset
the physical e-stop -> --clear-errors -> Web GUI lift panel -> power cycle.

Flags: --diagnose-only (skip D6-D8, purely read-only), --clear-errors,
--mode SIM|REAL (note: the lift does NOT execute in SIM — run REAL).
Arm selection: RM_ARM=left (default) or RM_ARM=right.
"""

import os
import sys
import time

from dual_arm_common import (
    handle_cli,
    DEV_LIFT, LEFT_IP, LIFT_GEAR, LIFT_M, LIFT_SPEED_PCT, RIGHT_IP,
    ROBOT_PORT, ArrivalMonitor, ConceptArm, apply_run_mode, countdown,
    diagnose_lift_rejection, home_poles_full, lift_hw_mm, mode_label,
    parse_mode_arg, report_run_modes, restore_run_modes, teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
PROBE_TIMEOUT_S = 10.0        # zero-distance arrival is ~0.02 s when healthy
STROKE_HW_MM = 10             # D7 stroke, well inside every rail

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 8


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _lift_to(robot, monitor, handle_id, target_hw: int, label: str) -> bool:
    """One event-verified rm_set_lift_height dispatch."""
    monitor.expect(handle_id, DEV_LIFT)
    t0 = time.perf_counter()
    ret = robot.rm_set_lift_height(LIFT_SPEED_PCT, int(target_hw), 0)
    if ret != 0:
        print(f"    {label}: REJECTED ret={ret}")
        return False
    arrived, success = monitor.wait(handle_id, DEV_LIFT, PROBE_TIMEOUT_S)
    dur = time.perf_counter() - t0
    print(f"    {label}: ret=0 arrived={arrived} success={success} "
          f"({dur:.2f} s)")
    return arrived and success


def main() -> int:
    for k in _results:                 # reset: the emulated suite calls
        _results[k] = 0                # main() more than once per process
    handle_cli(__doc__, extra_flags=("--diagnose-only", "--clear-errors"))
    # --no-pole on the pole test = read-only, same as --diagnose-only.
    diag_only = "--diagnose-only" in sys.argv or "--no-pole" in sys.argv
    clear_errs = "--clear-errors" in sys.argv
    forced = parse_mode_arg()
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    hw_max = LIFT_GEAR[ARM_SIDE]["hw_max"]
    print("=" * 68)
    print("C8  Pole-only diagnostic (lift acceptance + state dump)")
    print(f"    pole on arm={ARM_SIDE} @ {ip}   gearing="
          f"{LIFT_GEAR[ARM_SIDE]['name']} (hw max {hw_max})")
    print("    ONLY THE POLE MOVES — arm and hand stay still"
          if not diag_only else
          "    READ-ONLY (--diagnose-only/--no-pole): nothing moves at all")
    if clear_errs:
        print("    --clear-errors: rm_clear_system_err will run before D6")
    print(f"    mode: {mode_label(forced)}"
          + ("" if forced is not None else "  (select with --mode SIM|REAL)")
          + "  [lift does NOT execute in SIM]")
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
            result("FAIL", "run-mode selection",
                   "requested mode did not engage — aborting")
            return 1
        report_run_modes(arm)
        result("PASS", "connected", f"handle id={handle.id}")

        # ── D2: power ──
        try:
            ret, power = robot.rm_get_arm_power_state()
            if ret == 0 and power == 1:
                result("PASS", "arm power", "ON")
            elif ret == 0:
                result("FAIL", "arm power",
                       "OFF — release/reset the e-stop chain, power the "
                       "arm on (Web GUI or rm_set_arm_power(1))")
            else:
                result("PASS", "arm power", f"unreadable ret={ret} — "
                       "informational only")
        except Exception as exc:
            result("PASS", "arm power", f"getter unavailable {exc!r}")

        # ── D3: controller + joint errors ──
        err_detail = []
        try:
            ret, st = robot.rm_get_current_arm_state()
            if ret == 0:
                err = (st or {}).get("err")
                if isinstance(err, dict):
                    err_detail = [c for c in err.get("err", [])
                                  if str(c) not in ("0", "")]
        except Exception as exc:
            print(f"    arm-state getter unavailable: {exc!r}")
        joint_bad = []
        try:
            jd = robot.rm_get_joint_err_flag()
            if jd.get("return_code") == 0:
                joint_bad = [(i + 1, f) for i, f
                             in enumerate(jd.get("err_flag", [])) if f]
                print(f"    joint err_flag={jd.get('err_flag')}  "
                      f"brake_state={jd.get('brake_state')}")
        except Exception as exc:
            print(f"    joint-err getter unavailable: {exc!r}")
        if not err_detail and not joint_bad:
            result("PASS", "controller/joint errors", "clean")
        elif clear_errs:
            # Finding latched errors is the EXPECTED precondition of a
            # --clear-errors run; the probes below give the verdict.
            result("PASS", "controller/joint errors",
                   f"latched: sys={err_detail} joints={joint_bad} — "
                   "clearing next")
        else:
            result("FAIL", "controller/joint errors",
                   f"sys={err_detail} joints={joint_bad} — latched errors "
                   "block motion; rerun with --clear-errors")

        # ── D4: raw lift state ──
        lift_pos = None
        try:
            ret, lst = robot.rm_get_lift_state()
        except Exception as exc:
            ret, lst = -1, {"exc": repr(exc)}
        if ret == 0:
            lift_pos = int(lst.get("pos", 0))
            ef = lst.get("err_flag", 0)
            detail = (f"pos={lift_pos} hw-mm  current={lst.get('current')} "
                      f"mA  mode={lst.get('mode')}  err_flag={ef}")
            if ef and not clear_errs:
                result("FAIL", "lift state",
                       detail + " — LIFT DRIVER ERROR latched "
                       "(stall/overcurrent?); try --clear-errors, then "
                       "power cycle")
            elif ef:
                result("PASS", "lift state",
                       detail + " — driver error latched, clearing next")
            else:
                result("PASS", "lift state", detail)
        else:
            result("FAIL", "lift state", f"unreadable ret={ret} {lst}")

        # ── D5: optional error clear ──
        if clear_errs:
            try:
                ret = robot.rm_clear_system_err()
                if ret == 0:
                    result("PASS", "rm_clear_system_err", "ret=0")
                else:
                    result("FAIL", "rm_clear_system_err", f"ret={ret}")
            except Exception as exc:
                result("FAIL", "rm_clear_system_err", repr(exc))
        else:
            result("SKIP", "rm_clear_system_err", "not requested "
                   "(--clear-errors)")

        if diag_only:
            result("SKIP", "lift acceptance probe", "--diagnose-only")
            result("SKIP", "small stroke probe", "--diagnose-only")
            result("SKIP", "home to full length", "--diagnose-only")
            return 0 if _results["FAIL"] == 0 else 1
        if lift_pos is None:
            result("FAIL", "lift acceptance probe", "no lift position")
            result("SKIP", "small stroke probe", "no lift position")
            result("SKIP", "home to full length", "no lift position")
            return 1

        countdown()

        # ── D6: acceptance probe — zero-distance, event-verified ──
        print(f"  D6 acceptance probe: command CURRENT position "
              f"({lift_pos} hw-mm) — no physical motion")
        if _lift_to(robot, monitor, arm.handle_id, lift_pos, "hold-position"):
            result("PASS", "lift acceptance probe",
                   "controller accepts lift commands")
            accepted = True
        else:
            result("FAIL", "lift acceptance probe",
                   "the 20:38 signature — controller rejects/ignores "
                   "lift commands")
            diagnose_lift_rejection(arm)
            accepted = False

        # ── D7: small stroke, there and back ──
        if not accepted:
            result("SKIP", "small stroke probe", "acceptance probe failed")
        else:
            step = -STROKE_HW_MM if lift_pos >= STROKE_HW_MM else STROKE_HW_MM
            away = max(0, min(hw_max, lift_pos + step))
            ok = _lift_to(robot, monitor, arm.handle_id, away,
                          f"stroke to {away}")
            ok = _lift_to(robot, monitor, arm.handle_id, lift_pos,
                          f"back to {lift_pos}") and ok
            if ok:
                result("PASS", "small stroke probe",
                       f"{STROKE_HW_MM} hw-mm there and back, events OK")
            else:
                result("FAIL", "small stroke probe",
                       "dispatch accepted but motion/event failed — "
                       "if mode is SIM this is expected (lift does not "
                       "simulate)")
                accepted = False

        # ── D8: the standard pre-run state ──
        if not accepted:
            result("SKIP", "home to full length", "earlier probe failed")
        elif home_poles_full(monitor, arm):
            result("PASS", "home to full length",
                   f"{LIFT_M['full']} m -> "
                   f"{lift_hw_mm(ARM_SIDE, LIFT_M['full'])} hw-mm")
        else:
            result("FAIL", "home to full length")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        restore_run_modes(originals)
        teardown(arm)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
