"""C3 — Chained mode. BOTH ARMS AND BOTH POLES WILL MOVE.

The left arm leads; the right arm dispatches step k only after the left has
completed step k. The leader advances freely, so follower step k overlaps
leader step k+1 (pipelined) — the follower performs the *following* task,
it is not a synchronized copy.
"""

import sys

from dual_arm_common import (
    ARM_SPEED_PCT, CONCEPT_SEQUENCE, LEFT_IP, LIFT_SPEED_PCT, RIGHT_IP,
    ArrivalMonitor, connect_both, countdown, run_chained, teardown,
)

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 4
GATE_LATENCY_LIMIT_S = 1.0


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def main() -> int:
    print("=" * 68)
    print("C3  Chained dual-arm execution (left leads, right follows)")
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

        report = run_chained(left, right, monitor)

        n = len(CONCEPT_SEQUENCE)
        print("\n  chain results:")
        print("  " + "─" * 60)
        ordering_ok = True
        gate_lat, bad_ret, fallbacks = [], 0, 0
        for k, step in enumerate(CONCEPT_SEQUENCE):
            lead, foll = report["leader"][k], report["follower"][k]
            if lead is None or foll is None:
                print(f"  step {k} {str(step):22s} INCOMPLETE")
                ordering_ok = False
                continue
            bad_ret += (lead["ret"] != 0) + (foll["ret"] != 0)
            fallbacks += lead["verified"] + foll["verified"]
            in_order = (lead["t_done"] is not None
                        and foll["t_dispatch"] >= lead["t_done"])
            ordering_ok &= in_order
            lat = foll["t_dispatch"] - report["gate_open_t"][k] \
                if report["gate_open_t"][k] else float("inf")
            gate_lat.append(lat)
            print(f"  step {k} {str(step):22s} follower start "
                  f"{'after' if in_order else 'BEFORE'} leader done, "
                  f"gate latency {lat*1000:7.1f} ms, "
                  f"ok={lead['ok'] and foll['ok']}")

        if bad_ret == 0:
            result("PASS", "all dispatches accepted")
        else:
            result("FAIL", "all dispatches accepted", f"{bad_ret} rejected")

        if report["ok"]:
            result("PASS", "both chains completed")
        else:
            result("FAIL", "both chains completed")

        if ordering_ok:
            result("PASS", "ordering invariant",
                   "follower step k always after leader step k done")
        else:
            result("FAIL", "ordering invariant violated")

        max_lat = max(gate_lat) if gate_lat else float("inf")
        if max_lat < GATE_LATENCY_LIMIT_S:
            result("PASS", "follower gate latency", f"max {max_lat*1000:.1f} ms")
        else:
            result("FAIL", "follower gate latency",
                   f"max {max_lat:.2f} s >= {GATE_LATENCY_LIMIT_S} s")

        if fallbacks:
            print(f"  [WARN] {fallbacks} arrivals confirmed by position "
                  "fallback, not event")
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        teardown(left, right)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
