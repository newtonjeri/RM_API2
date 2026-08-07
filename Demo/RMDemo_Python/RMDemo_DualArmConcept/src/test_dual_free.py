"""C4 — Free execution mode. BOTH ARMS AND BOTH POLES WILL MOVE.

Both arms run the concept sequence independently, each advancing on its own
arrivals with no cross-arm gates. The concept sequence (rest/ready/zero +
pole strokes) is collision-free by construction; collision gating for
arbitrary free-running tasks is future work, not this test.
"""

import sys

from dual_arm_common import (
    handle_cli,
    parse_clear_errors_arg, preflight_error_gate,
    ARM_SPEED_PCT, CONCEPT_SEQUENCE, LEFT_IP, LIFT_SPEED_PCT, RIGHT_IP,
    ArrivalMonitor, apply_run_mode, connect_both, countdown, mode_label,
    home_poles_full, parse_mode_arg, parse_no_hands_arg,
    parse_no_pole_arg, report_run_modes,
    restore_run_modes, strip_hands, strip_poles, run_free, teardown,
)

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 5


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def main() -> int:
    for k in _results:                 # reset: the emulated suite calls
        _results[k] = 0                # main() more than once per process
    handle_cli(__doc__)
    forced = parse_mode_arg()
    no_hands = parse_no_hands_arg()
    no_pole = parse_no_pole_arg()
    clear_errs = parse_clear_errors_arg()
    seq = CONCEPT_SEQUENCE
    if no_hands:
        seq = strip_hands(seq)
    if no_pole:
        seq = strip_poles(seq)
    print("=" * 68)
    print("C4  Free-running dual-arm execution (no cross-arm gates)")
    print(f"    left={LEFT_IP}  right={RIGHT_IP}  "
          f"arm v={ARM_SPEED_PCT}%  lift v={LIFT_SPEED_PCT}%")
    print(f"    sequence: {seq}"
          + ("   [--no-hands: hand steps stripped]" if no_hands else "")
          + ("   [--no-pole: pole/sync-lift steps stripped]"
             if no_pole else ""))
    print(f"    mode: {mode_label(forced)}"
          + ("" if forced is not None else "  (select with --mode SIM|REAL)"))
    print("    pole pre-positioning SKIPPED (--no-pole)" if no_pole else
          "    poles pre-positioned to full length (0.29 m) before the sequence")
    print(("    BOTH ARMS WILL MOVE (poles stay put)" if no_pole else
           "    BOTH ARMS AND BOTH POLES WILL MOVE")
          + (" (VIRTUALLY — SIM forced)" if forced == 0 else ""))
    print("=" * 68)

    left = right = None
    originals = {}
    try:
        left, right = connect_both()
        if left is None:
            print(f"  [SKIP] Hardware not reachable at {LEFT_IP} / {RIGHT_IP}")
            _results["SKIP"] += N_CHECKS
            return 0

        monitor = ArrivalMonitor()
        monitor.register(left.robot)
        originals = apply_run_mode(forced, left, right)
        if originals is None:
            result("FAIL", "run-mode selection",
                   "requested mode did not engage — aborting before motion")
            return 1
        report_run_modes(left, right)
        ok_err, err_detail = preflight_error_gate(
            left, right, clear=clear_errs)
        if ok_err:
            result("PASS", "no latched controller errors", err_detail)
        else:
            result("FAIL", "no latched controller errors", err_detail)
            return 1
        countdown()

        if no_pole:
            result("PASS", "poles pre-positioned to full length",
                   "SKIPPED — poles disabled (--no-pole)")
        elif home_poles_full(monitor, left, right):
            result("PASS", "poles pre-positioned to full length")
        else:
            result("FAIL", "poles pre-positioned to full length")
            return 1

        report = run_free(left, right, monitor, sequence=seq)

        n = len(seq)
        bad_ret = fallbacks = 0
        print("\n  per-arm timelines:")
        print("  " + "─" * 60)
        windows = {}
        for key in ("left", "right"):
            recs = report[key]
            bad_ret += sum(r["ret"] != 0 for r in recs)
            fallbacks += sum(r["verified"] for r in recs)
            if recs and recs[-1]["t_done"]:
                windows[key] = (recs[0]["t_dispatch"], recs[-1]["t_done"])
                total = windows[key][1] - windows[key][0]
                print(f"  {key:5s}: {len(recs)}/{n} steps in {total:.2f} s")
            else:
                print(f"  {key:5s}: {len(recs)}/{n} steps, incomplete")

        if bad_ret == 0:
            result("PASS", "all dispatches accepted")
        else:
            result("FAIL", "all dispatches accepted", f"{bad_ret} rejected")

        if report["ok"]:
            result("PASS", "both arms completed independently")
        else:
            result("FAIL", "both arms completed independently")

        if len(windows) == 2:
            # Real concurrency evidence: the two execution windows must
            # overlap for most of the shorter run.
            (l0, l1), (r0, r1) = windows["left"], windows["right"]
            shared = max(0.0, min(l1, r1) - max(l0, r0))
            shorter = max(min(l1 - l0, r1 - r0), 1e-9)
            frac = shared / shorter
            if frac >= 0.5:
                result("PASS", "concurrent free run",
                       f"window overlap {frac*100:.0f}% of shorter run")
            else:
                result("FAIL", "concurrent free run",
                       f"window overlap only {frac*100:.0f}%")
        else:
            result("FAIL", "concurrent free run", "incomplete timelines")

        if fallbacks:
            print(f"  [WARN] {fallbacks} arrivals confirmed by position "
                  "fallback, not event")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        restore_run_modes(originals)
        teardown(left, right)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
