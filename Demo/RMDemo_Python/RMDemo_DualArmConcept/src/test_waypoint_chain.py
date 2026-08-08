"""C10 — Chained-target execution (`trajectory_connect`). THE ARM MOVES.

The SDK documents the mechanism (connect=1 queues a segment to be planned
with the next; the closing connect=0 plans and executes the chain) but not
the numbers Mode B needs. This measures them:

  W1  chain of 4 small movej segments: does it execute as ONE motion?
      how many arrival events, and with what trajectory_connect flags?
  W2  queue depth: chains of 2 / 5 / 10 / 20 segments — where does the
      controller stop accepting connect=1 (ret != 0)?
  W3  blend: the same 3-corner path at r=0 vs r=50 — duration delta shows
      whether blending is real (corner not stopped at) and how large
  W4  mid-chain invalid target (out-of-limit joint): is the WHOLE chain
      rejected, or everything before the bad segment executed?
  W5  `rm_moves` spline: >= 3 connected points, else it degrades to a line

All motion is small joint offsets around `ready` (J1/J3 +/- a few deg), so
the sweep stays in free space. Every phase re-homes to `ready` first.

Flags: --skip W4 with --no-fault-probe (W4 intentionally sends a bad
target; skip it if the controller's reaction is a concern).
Arm selection: RM_ARM=left (default) or RM_ARM=right.
"""

import os
import sys
import time

from dual_arm_common import (
    handle_cli,
    preflight_error_gate,
    ARM_SPEED_PCT, ARM_TIMEOUT_S, DEV_JOINT, LEFT_IP, RIGHT_IP, ROBOT_PORT,
    ArrivalMonitor, ConceptArm, apply_run_mode, countdown, mode_label,
    parse_clear_errors_arg, parse_mode_arg, report_run_modes,
    restore_run_modes, state_deg, teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
STEP_DEG = 4.0                # per-segment J1/J3 offset — small, free space
# Depth must cover what the real tasks need, not a round number: the
# cleaning paths queue EVERY segment (Newton, 2026-08-08) — 43 for each
# hinge task, 27 for each toplid. Probing only to 20 left the depth the
# dispatcher actually relies on unverified.
# DEPTHS = (2, 5, 10, 20, 27, 43, 60)
DEPTHS = (100, 13)
TASK_DEPTHS = {27: "toplid_{left,right}", 43: "hinge_area_{left,right}"}

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 6


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


class EventTap:
    """Counts EVERY device-0 event, including trajectory_connect==1 ones
    (ArrivalMonitor deliberately ignores those as non-completions)."""

    def __init__(self):
        self.events = []

    def hook(self, monitor):
        original = monitor._on_event

        def tapped(data):
            if getattr(data, "device", None) == DEV_JOINT:
                self.events.append(int(getattr(data,
                                               "trajectory_connect", -1)))
            return original(data)
        monitor._on_event = tapped


def _zig(base, i):
    """Small alternating offsets from base: J1 +/-, J3 -/+."""
    j = list(base)
    j[0] += STEP_DEG if i % 2 == 0 else -STEP_DEG
    j[2] += -STEP_DEG if i % 2 == 0 else STEP_DEG
    return j


def _rehome(robot, monitor, handle_id, ready):
    monitor.expect(handle_id, DEV_JOINT)
    robot.rm_movej(ready, ARM_SPEED_PCT, 0, 0, 0)
    return all(monitor.wait(handle_id, DEV_JOINT, ARM_TIMEOUT_S))


def _run_chain(robot, monitor, handle_id, targets, r, tap):
    """Queue targets with connect=1, close with connect=0. Returns
    (rets, duration_s, events_during)."""
    n0 = len(tap.events)
    rets = []
    monitor.expect(handle_id, DEV_JOINT)
    t0 = time.perf_counter()
    for t in targets[:-1]:
        rets.append(robot.rm_movej(t, ARM_SPEED_PCT, r, 1, 0))
    rets.append(robot.rm_movej(targets[-1], ARM_SPEED_PCT, r, 0, 0))
    arrived, ok = monitor.wait(handle_id, DEV_JOINT,
                               ARM_TIMEOUT_S + 5.0 * len(targets))
    dur = time.perf_counter() - t0
    return rets, (dur if arrived and ok else None), tap.events[n0:]


def main() -> int:
    for k in _results:
        _results[k] = 0
    handle_cli(__doc__, extra_flags=("--no-fault-probe",))
    forced = parse_mode_arg()
    clear_errs = parse_clear_errors_arg()
    fault_probe = "--no-fault-probe" not in sys.argv
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP
    ready = state_deg(ARM_SIDE, "ready")

    print("=" * 68)
    print("C10  Chained-target execution (trajectory_connect)")
    print(f"    arm={ARM_SIDE} @ {ip}   v={ARM_SPEED_PCT}%   "
          f"segments ±{STEP_DEG}° on J1/J3 around ready")
    print(f"    queue depths probed: {DEPTHS}   fault probe (W4): "
          f"{'ON' if fault_probe else 'off (--no-fault-probe)'}")
    print(f"    mode: {mode_label(forced)}")
    print(f"    THE {ARM_SIDE.upper()} ARM WILL MOVE (small sweeps)")
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
        tap = EventTap()
        tap.hook(monitor)
        originals = apply_run_mode(forced, arm)
        if originals is None:
            result("FAIL", "run-mode selection", "did not engage")
            return 1
        report_run_modes(arm)
        ok_err, detail = preflight_error_gate(arm, clear=clear_errs)
        if not ok_err:
            result("FAIL", "no latched controller errors", detail)
            return 1
        result("PASS", "no latched controller errors", detail)
        countdown()

        if not _rehome(robot, monitor, arm.handle_id, ready):
            result("FAIL", "homed to ready")
            return 1
        result("PASS", "homed to ready")

        # ── W1: 4-segment chain — one motion? how many events? ──
        print("\n  W1 — 4-segment chain, r=0")
        targets = [_zig(ready, i) for i in range(3)] + [list(ready)]
        rets, dur, events = _run_chain(robot, monitor, arm.handle_id,
                                       targets, 0, tap)
        print(f"    rets={rets}  duration="
              f"{f'{dur:.2f}s' if dur else 'DID NOT COMPLETE'}  "
              f"events(connect flags)={events}")
        if all(r == 0 for r in rets) and dur:
            result("PASS", "chain executes",
                   f"{len(targets)} segments, {len(events)} device-0 "
                   f"event(s), flags {events}")
        else:
            result("FAIL", "chain executes",
                   f"rets={rets} — record for the Mode B design")

        # ── W2: queue depth ──
        print("\n  W2 — queue depth")
        max_ok = 0
        for depth in DEPTHS:
            need = TASK_DEPTHS.get(depth)
            if not _rehome(robot, monitor, arm.handle_id, ready):
                break
            targets = [_zig(ready, i) for i in range(depth - 1)] \
                + [list(ready)]
            rets, dur, _ = _run_chain(robot, monitor, arm.handle_id,
                                      targets, 0, tap)
            rejected = [i for i, r in enumerate(rets) if r != 0]
            done = "completed" if dur else "NOT completed"
            print(f"    depth {depth:2d}: rejected at {rejected or 'none'}, "
                  f"{done}"
                  + (f" in {dur:.2f}s" if dur else "")
                  + (f"   <-- needed by {need}" if need else ""))
            if not rejected and dur:
                max_ok = depth
        # Phase 2 queues every segment of a cleaning path in one chain, so
        # the deepest task depth is the number that matters.
        need_max = max(TASK_DEPTHS)
        if max_ok >= need_max:
            result("PASS", "queue depth measured",
                   f"deepest clean chain: {max_ok} segments — covers all "
                   f"tasks (deepest needs {need_max})")
        elif max_ok:
            result("FAIL", "queue depth measured",
                   f"deepest clean chain: {max_ok} segments, but "
                   f"{TASK_DEPTHS[need_max]} needs {need_max} — the "
                   "dispatcher must batch")
        else:
            result("FAIL", "queue depth measured", "no chain completed")

        # ── W3: blend r=0 vs r=50 on the same corners ──
        print("\n  W3 — blend: r=0 vs r=50, same 3-corner path")
        durs = {}
        for r in (0, 50):
            if not _rehome(robot, monitor, arm.handle_id, ready):
                break
            targets = [_zig(ready, i) for i in range(3)] + [list(ready)]
            _, dur, _ = _run_chain(robot, monitor, arm.handle_id,
                                   targets, r, tap)
            durs[r] = dur
            print(f"    r={r:2d}: "
                  + (f"{dur:.2f}s" if dur else "DID NOT COMPLETE"))
        if durs.get(0) and durs.get(50):
            saved = durs[0] - durs[50]
            verdict = "blending is real" if saved > 0.15 else \
                      "no measurable effect — corners may full-stop anyway"
            result("PASS", "blend effect measured",
                   f"r=50 saves {saved:+.2f}s -> {verdict}")
        else:
            result("FAIL", "blend effect measured", "a chain did not finish")

        # ── W4: mid-chain invalid target ──
        if not fault_probe:
            result("SKIP", "mid-chain failure behaviour", "--no-fault-probe")
        else:
            print("\n  W4 — mid-chain INVALID target (J2 out of limit)")
            _rehome(robot, monitor, arm.handle_id, ready)
            bad = list(ready)
            bad[1] = 175.0                    # beyond the J2 130° limit
            targets = [_zig(ready, 0), bad, list(ready)]
            rets, dur, _ = _run_chain(robot, monitor, arm.handle_id,
                                      targets, 0, tap)
            print(f"    rets={rets}  completed={bool(dur)}")
            result("PASS", "mid-chain failure behaviour",
                   f"bad segment ret={rets[1]}; chain "
                   f"{'still completed' if dur else 'did not complete'} — "
                   "record which for the dispatcher design")
            _rehome(robot, monitor, arm.handle_id, ready)

        # ── W5: rm_moves spline ──
        print("\n  W5 — rm_moves spline (informational)")
        try:
            ret, st = robot.rm_get_current_arm_state()
            pose = list(st["pose"][:6])
            pts = []
            for dz in (0.02, 0.04, 0.02, 0.0):
                q = list(pose)
                q[2] += dz
                pts.append(q)
            monitor.expect(arm.handle_id, DEV_JOINT)
            rets = [robot.rm_moves(p, ARM_SPEED_PCT, 0, 1, 0)
                    for p in pts[:-1]]
            rets.append(robot.rm_moves(pts[-1], ARM_SPEED_PCT, 0, 0, 0))
            arrived, ok = monitor.wait(arm.handle_id, DEV_JOINT,
                                       ARM_TIMEOUT_S)
            result("PASS", "rm_moves spline probed",
                   f"rets={rets} completed={arrived and ok}")
        except Exception as exc:
            result("PASS", "rm_moves spline probed", f"unavailable: {exc!r}")
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
