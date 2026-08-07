"""C9 — The DUAL-LOCKED sequence on ONE arm (freeze-isolation probe).
THE SELECTED ARM, ITS POLE, AND ITS HAND WILL MOVE.

Purpose: reproduce the 2026-08-06 21:25 C2 freeze condition with the
second arm removed.  This runs the EXACT dual-locked concept sequence

    arm->ready -> hand->release -> SYNC(arm->zero + pole->half)
    -> hand->grasp -> SYNC(arm->ready + pole->full)
    -> hand->half_grasp -> arm->rest

through the EXACT per-arm code path run_locked uses (ConceptArm.begin /
finish: movej dispatched first, then the duration-matched
rm_set_lift_height, back-to-back non-blocking, arrival events awaited)
— only the partner arm and its barrier are absent.  If the freeze is the
planned-movej + lift conflict, it reproduces here at the FIRST SYNC STEP:
the pole will travel, the arm will stop short of zero, and the step will
sit in its 40 s joint-event wait before failing over to position verify.

Unlike C2, every step's result prints IMMEDIATELY, so a frozen step is
visible as it happens instead of after a silent hang.

Comparisons this enables (one variable at a time):
    C6  (passes)       movej + hand           on one arm
    C9  (this)         movej + pole [+ hand]  on one arm, locked sequence
    C9 --no-hands      movej + pole           on one arm
    C2                 the same with both arms

Arm selection: RM_ARM=left (default) or RM_ARM=right.
Note: the lift does NOT execute in SIM (2026-08-06 logs) — sync steps can
only complete on REAL hardware; use --no-pole for a SIM rehearsal.
"""

import os
import sys

from dual_arm_common import (
    handle_cli,
    ARM_SPEED_PCT, CONCEPT_SEQUENCE, DEV_HAND, DEV_JOINT, DEV_LIFT,
    LEFT_IP, LIFT_SPEED_PCT, RIGHT_IP, ROBOT_PORT, ArrivalMonitor,
    ConceptArm, apply_run_mode, countdown, home_poles_full, mode_label,
    parse_mode_arg, parse_no_hands_arg, parse_no_pole_arg,
    report_run_modes, restore_run_modes, strip_hands, strip_poles,
    teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
SYNC_DISPATCH_SKEW_LIMIT_MS = 50.0   # movej -> lift dispatch gap in a sync step

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 6

DEV_NAME = {DEV_JOINT: "arm", DEV_LIFT: "lift", DEV_HAND: "hand"}


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _step_line(rec: dict):
    """One live line per step: per-device durations + sync skews."""
    parts = []
    for device, d in rec["devices"].items():
        if d["t_done"]:
            parts.append(f"{DEV_NAME[device]} "
                         f"{d['t_done'] - d['t_dispatch']:5.2f} s")
        else:
            parts.append(f"{DEV_NAME[device]} INCOMPLETE ret={d['ret']}")
    if "sync_start_skew_s" in rec:
        parts.append(f"start-skew {rec['sync_start_skew_s']*1000:5.1f} ms")
    if rec.get("sync_finish_skew_s") is not None:
        parts.append(f"finish-skew {rec['sync_finish_skew_s']*1000:+6.0f} ms")
    print(f"  {str(rec['step']):26s} ok={rec['ok']}  " + "   ".join(parts))


def main() -> int:
    for k in _results:                 # reset: the emulated suite calls
        _results[k] = 0                # main() more than once per process
    handle_cli(__doc__)
    forced = parse_mode_arg()
    no_hands = parse_no_hands_arg()
    no_pole = parse_no_pole_arg()
    seq = CONCEPT_SEQUENCE
    if no_hands:
        seq = strip_hands(seq)
    if no_pole:
        seq = strip_poles(seq)
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    print("=" * 68)
    print("C9  Single-arm LOCKED sequence (the C2 recipe on one arm)")
    print(f"    arm={ARM_SIDE} @ {ip}   arm v={ARM_SPEED_PCT}%   "
          f"lift v={LIFT_SPEED_PCT}% (duration-matched on sync)")
    print(f"    sequence: {seq}"
          + ("   [--no-hands: hand steps stripped]" if no_hands else "")
          + ("   [--no-pole: pole/sync-lift steps stripped]"
             if no_pole else ""))
    print(f"    mode: {mode_label(forced)}"
          + ("" if forced is not None else "  (select with --mode SIM|REAL)"))
    print("    pole pre-positioning SKIPPED (--no-pole)" if no_pole else
          "    pole pre-positioned to full length (0.29 m) first")
    moving = [f"THE {ARM_SIDE.upper()} ARM"]
    if not no_pole:
        moving.append("ITS POLE")
    if not no_hands:
        moving.append("ITS HAND")
    print(f"    {', '.join(moving[:-1])}{' AND ' if len(moving) > 1 else ''}"
          f"{moving[-1]} WILL MOVE"
          + (" (VIRTUALLY — SIM forced; lift/hand do NOT simulate)"
             if forced == 0 else ""))
    if not no_pole:
        print("    A frozen sync step waits up to 40 s for the arm event,"
              " then position-verifies, halts, and FAILS — let it finish.")
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
                   "requested mode did not engage — aborting before motion")
            return 1
        report_run_modes(arm)
        countdown()

        if no_pole:
            result("PASS", "pole pre-positioned to full length",
                   "SKIPPED — pole disabled (--no-pole)")
        elif home_poles_full(monitor, arm):
            result("PASS", "pole pre-positioned to full length")
        else:
            result("FAIL", "pole pre-positioned to full length")
            return 1

        # ── the run_locked per-arm loop, verbatim minus the partner ──
        print("\n  step results (live):")
        print("  " + "─" * 60)
        recs = []
        for step in seq:
            beg = arm.begin(monitor, step)
            rets = [d["ret"] for d in beg["devices"].values()]
            rec = arm.finish(monitor, beg)
            recs.append(rec)
            _step_line(rec)
            if any(r != 0 for r in rets) or not rec["ok"]:
                arm.halt()             # run_locked's stop_all, one arm
                break

        bad_ret = sum(1 for rec in recs
                      for d in rec["devices"].values() if d["ret"] != 0)
        if bad_ret == 0:
            result("PASS", "all dispatches accepted")
        else:
            result("FAIL", "all dispatches accepted", f"{bad_ret} rejected")

        if len(recs) == len(seq) and all(rec["ok"] for rec in recs):
            result("PASS", "sequence completed")
        else:
            frozen = next((rec["step"] for rec in recs if not rec["ok"]),
                          None)
            result("FAIL", "sequence completed",
                   f"{len(recs)}/{len(seq)} steps; first failure at "
                   f"{frozen}")

        no_event = sum(1 for rec in recs
                       for d in rec["devices"].values()
                       if d["ret"] == 0 and not d["event"]
                       and not d.get("acked"))
        fallbacks = sum(rec["verified"] for rec in recs)
        if no_event == 0:
            result("PASS", "arrival via event for every device")
        else:
            print(f"  [WARN] {no_event} dispatched devices never delivered "
                  f"an arrival event ({fallbacks} recovered by position "
                  "fallback; hand has no fallback) — the C2 freeze "
                  "signature if the device is the ARM on a sync step")
            result("FAIL", "arrival via event for every device",
                   f"{no_event} missing")

        sync_recs = [rec for rec in recs if rec["step"][0] == "sync"]
        if no_pole:
            result("PASS", "sync dispatch back-to-back",
                   "SKIPPED — poles disabled (--no-pole)")
            result("PASS", "arm-pole sync finish",
                   "SKIPPED — poles disabled (--no-pole)")
        else:
            for rec in sync_recs:
                print(f"  [INFO] sync {str(rec['step'][1]):20s} "
                      f"arm-dur-est {rec.get('arm_dur_est_s') or 0:.2f} s, "
                      f"matched lift {rec.get('lift_speed_pct')}%, "
                      f"start skew {rec['sync_start_skew_s']*1000:6.1f} ms")
            worst_disp = max((rec["sync_start_skew_s"] for rec in sync_recs),
                            default=None)
            if worst_disp is None:
                result("FAIL", "sync dispatch back-to-back",
                       "no sync steps ran")
            elif worst_disp * 1000 < SYNC_DISPATCH_SKEW_LIMIT_MS:
                result("PASS", "sync dispatch back-to-back",
                       f"max movej->lift gap {worst_disp*1000:.1f} ms")
            else:
                result("FAIL", "sync dispatch back-to-back",
                       f"{worst_disp*1000:.1f} ms >= "
                       f"{SYNC_DISPATCH_SKEW_LIMIT_MS} ms")
            # Only steps that genuinely completed have a meaningful finish
            # skew — a frozen arm's timeout timestamp would otherwise make
            # the pole look benignly "early" and pass the check.
            finished = [rec["sync_finish_skew_s"] for rec in sync_recs
                        if rec.get("sync_finish_skew_s") is not None
                        and rec["ok"]]
            if finished:
                worst_late = max(finished)
                if worst_late <= 0.5:
                    result("PASS", "arm-pole sync finish",
                           f"worst pole lateness {worst_late*1000:+.0f} ms")
                else:
                    result("FAIL", "arm-pole sync finish",
                           f"pole late by {worst_late:.2f} s")
            else:
                result("FAIL", "arm-pole sync finish",
                       "no sync step produced both completions "
                       "(the freeze outcome)")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        if arm is not None and _results["FAIL"] > 0:
            arm.halt()
        restore_run_modes(originals)
        teardown(arm)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
