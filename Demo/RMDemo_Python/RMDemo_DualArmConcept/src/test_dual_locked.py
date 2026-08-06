"""C2 — Parallel locked mode. BOTH ARMS AND BOTH POLES WILL MOVE.

Every step of the concept sequence is dispatched to both arms back-to-back
(non-blocking), then a barrier waits for BOTH arrivals before the next step.
Reports dispatch skew and per-arm completion deltas at every step boundary.
"""

import sys

from dual_arm_common import (
    handle_cli,
    ARM_SPEED_PCT, CONCEPT_SEQUENCE, LEFT_IP, LIFT_SPEED_PCT, RIGHT_IP,
    ArrivalMonitor, apply_run_mode, connect_both, countdown, mode_label,
    home_poles_full, parse_mode_arg, parse_no_hands_arg, report_run_modes,
    restore_run_modes, strip_hands, run_locked, teardown,
)

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 6
SKEW_LIMIT_MS = 50.0


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
    print("C2  Parallel locked dual-arm execution")
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

        report = run_locked(left, right, monitor, sequence=seq)

        print("\n  step results:")
        print("  " + "─" * 60)
        skews, deltas, fallbacks, bad_ret = [], [], 0, 0
        no_event = 0
        for e in report["steps"]:
            l, r = e["left"], e["right"]
            skews.append(e["skew_ms"])
            bad_ret += (l["ret"] != 0) + (r["ret"] != 0)
            fallbacks += l["verified"] + r["verified"]
            no_event += sum(1 for rec in (l, r)
                            for d in rec.get("devices", {}).values()
                            if d["ret"] == 0 and not d["event"]
                            and not d.get("acked"))
            if l["t_done"] and r["t_done"]:
                dl = l["t_done"] - l["t_dispatch"]
                dr = r["t_done"] - r["t_dispatch"]
                deltas.append(abs(dl - dr))
                print(f"  {str(e['step']):24s} skew {e['skew_ms']:6.2f} ms   "
                      f"L {dl:6.2f} s   R {dr:6.2f} s   |d| {abs(dl-dr):5.2f} s"
                      f"   ok={l['ok'] and r['ok']}")
            else:
                print(f"  {str(e['step']):24s} skew {e['skew_ms']:6.2f} ms   "
                      f"INCOMPLETE (retL={l['ret']} retR={r['ret']})")

        if bad_ret == 0:
            result("PASS", "all dispatches accepted")
        else:
            result("FAIL", "all dispatches accepted", f"{bad_ret} rejected")

        if report["ok"] and len(report["steps"]) == len(seq):
            result("PASS", "sequence completed with barrier per step")
        else:
            result("FAIL", "sequence completed",
                   f"{len(report['steps'])}/{len(seq)} steps, "
                   f"ok={report['ok']}")

        max_skew = max(skews) if skews else float("inf")
        if max_skew < SKEW_LIMIT_MS:
            result("PASS", "dispatch skew", f"max {max_skew:.2f} ms")
        else:
            result("FAIL", "dispatch skew",
                   f"max {max_skew:.2f} ms >= {SKEW_LIMIT_MS} ms")

        if no_event == 0:
            result("PASS", "arrival via event for every device")
        else:
            print(f"  [WARN] {no_event} dispatched devices never delivered "
                  f"an arrival event ({fallbacks} recovered by position "
                  "fallback; hand has no fallback)")
            result("FAIL", "arrival via event for every device",
                   f"{no_event} missing")

        if deltas:
            print(f"  [INFO] completion delta between arms: "
                  f"mean {sum(deltas)/len(deltas):.2f} s, max {max(deltas):.2f} s")

        # PL5 — arm-pole sync steps: pole must not finish late vs the arm
        # by more than 0.5 s (early is benign: the device waits; bench_sync
        # targets |finish skew| <= ~200-300 ms at T=4 s).
        sync_recs = [(e["step"], rec)
                     for e in report["steps"] for rec in (e["left"], e["right"])
                     if e["step"][0] == "sync" and "sync_finish_skew_s" in rec]
        if sync_recs:
            worst_late = max(rec["sync_finish_skew_s"] for _, rec in sync_recs)
            for step, rec in sync_recs:
                print(f"  [INFO] sync {rec['side']:5s} {str(step[1]):20s} "
                      f"arm-dur-est {rec.get('arm_dur_est_s', 0):.2f} s, "
                      f"matched lift {rec.get('lift_speed_pct')}%, "
                      f"start skew {rec['sync_start_skew_s']*1000:6.1f} ms, "
                      f"finish skew {rec['sync_finish_skew_s']*1000:+7.1f} ms")
            if worst_late <= 0.5:
                result("PASS", "arm-pole sync finish",
                       f"worst pole lateness {worst_late*1000:+.0f} ms")
            else:
                result("FAIL", "arm-pole sync finish",
                       f"pole late by {worst_late:.2f} s")
        else:
            result("FAIL", "arm-pole sync finish", "no sync steps measured")
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
