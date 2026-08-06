"""C2 — Parallel locked mode. BOTH ARMS AND BOTH POLES WILL MOVE.

Every step of the concept sequence is dispatched to both arms back-to-back
(non-blocking), then a barrier waits for BOTH arrivals before the next step.
Reports dispatch skew and per-arm completion deltas at every step boundary.
"""

import sys

from dual_arm_common import (
    ARM_SPEED_PCT, CONCEPT_SEQUENCE, LEFT_IP, LIFT_SPEED_PCT, RIGHT_IP,
    ArrivalMonitor, connect_both, countdown, run_locked, teardown,
)

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 5
SKEW_LIMIT_MS = 50.0


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def main() -> int:
    print("=" * 68)
    print("C2  Parallel locked dual-arm execution")
    print(f"    left={LEFT_IP}  right={RIGHT_IP}  "
          f"arm v={ARM_SPEED_PCT}%  lift v={LIFT_SPEED_PCT}%")
    print(f"    sequence: {CONCEPT_SEQUENCE}")
    print("    BOTH ARMS AND BOTH POLES WILL MOVE")
    print("=" * 68)

    left = right = None
    try:
        left, right = connect_both()
        if left is None:
            print(f"  [SKIP] Hardware not reachable at {LEFT_IP} / {RIGHT_IP}")
            _results["SKIP"] += N_CHECKS
            return 0

        monitor = ArrivalMonitor()
        monitor.register(left.robot)
        countdown(5)

        report = run_locked(left, right, monitor)

        print("\n  step results:")
        print("  " + "─" * 60)
        skews, deltas, fallbacks, bad_ret = [], [], 0, 0
        for e in report["steps"]:
            l, r = e["left"], e["right"]
            skews.append(e["skew_ms"])
            bad_ret += (l["ret"] != 0) + (r["ret"] != 0)
            fallbacks += l["verified"] + r["verified"]
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

        if report["ok"] and len(report["steps"]) == len(CONCEPT_SEQUENCE):
            result("PASS", "sequence completed with barrier per step")
        else:
            result("FAIL", "sequence completed",
                   f"{len(report['steps'])}/{len(CONCEPT_SEQUENCE)} steps, "
                   f"ok={report['ok']}")

        max_skew = max(skews) if skews else float("inf")
        if max_skew < SKEW_LIMIT_MS:
            result("PASS", "dispatch skew", f"max {max_skew:.2f} ms")
        else:
            result("FAIL", "dispatch skew",
                   f"max {max_skew:.2f} ms >= {SKEW_LIMIT_MS} ms")

        if fallbacks == 0:
            result("PASS", "arrival via event for every step")
        else:
            print(f"  [WARN] {fallbacks} arrivals confirmed by position "
                  "fallback, not event")
            result("PASS", "arrivals confirmed", f"{fallbacks} via fallback")

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
        teardown(left, right)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
