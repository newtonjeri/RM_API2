#!/usr/bin/env python3
"""Read, and optionally reset, an arm's Cartesian speed/acceleration limits.

NO MOTION. Reads `rm_get_arm_max_line_*` / `rm_get_arm_max_angular_*`, and
with --apply writes the F10 envelope back.

WHY THIS EXISTS. These limits are GLOBAL controller state and they RATCHET:

  * `stage_runner` restores the value it read at the START of a run. If a run
    aborts before the restore, the raised value survives, and the NEXT run
    reads it as its baseline and "restores" it forever.
  * `stage_runner.py:626` nests `speed_limits.restore()` inside
    `if arm is not None and caps_before:` — when `controller_caps.prepare()`
    returns {} (nothing to change, or RM_SET_CAPS=0), a raised limit is
    NEVER put back.

Measured consequence, census 2026-08-11 20:09: the LEFT arm was left at
`line_speed 0.800 / line_acc 2.400` — the exact settings under which it
executed a violent 4-joint reversal at 16.7 A and reported no error
(SPEED_INVESTIGATION.md §1-2). A bare `stage_runner.py --mode REAL` with no
RM_LINE_SPEED in the environment reproduces that run, because
`speed_limits.prepare()` returns {} when nothing is asked for and leaves the
controller at whatever it was.

    python3 reset_limits.py --side left               # report only
    python3 reset_limits.py --side left --apply       # write F10 back
    python3 reset_limits.py --side both

Run it BEFORE and AFTER every hardware session.
"""
import sys

sys.path.insert(0, __file__.rsplit("/", 1)[0])

from dual_arm_common import handle_cli, LEFT_IP, RIGHT_IP, ROBOT_PORT  # noqa
import speed_limits  # noqa

SAFE = dict(speed_limits.CONFIGURED)     # the F10 envelope: 0.250 / 1.600
KEYS = ("line_speed", "line_acc", "angular_speed", "angular_acc")


def connect(ip):
    from Robotic_Arm.rm_robot_interface import RoboticArm
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    h = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
    return robot if (h is not None and h.id > 0) else None


def one(side, apply_it):
    ip = LEFT_IP if side == "left" else RIGHT_IP
    robot = connect(ip)
    if robot is None:
        print(f"  [FAIL] {side}: no connection to {ip}")
        return 1
    try:
        now = speed_limits.read(robot)
        print(f"  {side} ({ip})")
        bad = []
        for k in KEYS:
            v, want = now.get(k), SAFE[k]
            if not isinstance(v, float):
                print(f"    {k:<16} unreadable ({v!r})")
                continue
            over = v > want + 1e-6
            if over:
                bad.append(k)
            print(f"    {k:<16} {v:.3f}   envelope {want:.3f}"
                  f"{'   <-- ABOVE ENVELOPE' if over else ''}")
        if not bad:
            print(f"    within the F10 envelope; nothing to do")
            return 0
        if not apply_it:
            print(f"    [WARN] {side} is above the envelope on "
                  f"{', '.join(bad)} — re-run with --apply")
            return 1
        # Write acceleration first: the controller enforces acc/speed >= 3,
        # so lowering speed before acc is always legal, and raising acc
        # before speed never trips it either. Order the pair defensively.
        speed_limits.apply(robot, allow_raise=False,
                           **{k: SAFE[k] for k in KEYS})
        after = speed_limits.read(robot)
        ok = all(isinstance(after.get(k), float)
                 and abs(after[k] - SAFE[k]) < 1e-3 for k in KEYS)
        for k in KEYS:
            print(f"    {k:<16} -> {after.get(k)}")
        print(f"    {'[PASS]' if ok else '[FAIL]'} readback "
              f"{'matches' if ok else 'DOES NOT MATCH'} the envelope")
        return 0 if ok else 1
    finally:
        try:
            robot.rm_delete_robot_arm()
        except Exception:
            pass


def main() -> int:
    handle_cli(__doc__, extra_flags=("--apply",), value_flags=("--side",))
    args = sys.argv[1:]
    side = "both"
    if "--side" in args:
        side = args[args.index("--side") + 1]
    apply_it = "--apply" in args
    sides = ["left", "right"] if side == "both" else [side]
    rc = 0
    for s in sides:
        rc |= one(s, apply_it)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
