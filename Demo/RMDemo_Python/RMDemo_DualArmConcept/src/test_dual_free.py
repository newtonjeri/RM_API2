"""C4 — Free execution mode. BOTH ARMS AND BOTH POLES WILL MOVE.

Both arms run the concept sequence independently, each advancing on its own
arrivals with no cross-arm gates. The concept sequence (rest/ready/zero +
pole strokes) is collision-free by construction; collision gating for
arbitrary free-running tasks is future work, not this test.
"""

import sys

from dual_arm_common import (
    handle_cli,
    ARM_SPEED_PCT, CONCEPT_SEQUENCE, LEFT_IP, LIFT_SPEED_PCT, RIGHT_IP,
    ArrivalMonitor, apply_run_mode, connect_both, countdown, mode_label,
    home_poles_full, parse_mode_arg, parse_no_hands_arg, report_run_modes,
    restore_run_modes, strip_hands, run_free, teardown,
)

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 4


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def main() -> int:
    handle_cli(__doc__)
    forced = parse_mode_arg()
    no_hands = parse_no_hands_arg()
    seq = strip_hands(CONCEPT_SEQUENCE) if no_hands else CONCEPT_SEQUENCE
    print("=" * 68)
    print("C4  Free-running dual-arm execution (no cross-arm gates)")
    print(f"    left={LEFT_IP}  right={RIGHT_IP}  "
          f"arm v={ARM_SPEED_PCT}%  lift v={LIFT_SPEED_PCT}%")
    print(f"    sequence: {seq}"
          + ("   [--no-hands: hand steps stripped]" if no_hands else ""))
    print(f"    mode: {mode_label(forced)}"
          + ("" if forced is not None else "  (select with --mode SIM|REAL)"))
    print("    poles pre-positioned to full length (0.29 m) before the sequence")
    print("    BOTH ARMS AND BOTH POLES WILL MOVE"
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
        countdown(5)

        if home_poles_full(monitor, left, right):
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
