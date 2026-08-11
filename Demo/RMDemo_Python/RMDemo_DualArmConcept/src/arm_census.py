"""Read EVERYTHING the controller will tell us, and check it against what
our code assumes. READ-ONLY — no setter is ever called.

WHY THIS EXISTS

Twice now a constant in our code has disagreed with the robot, and both
times the failure surfaced far from the cause:

    the payload centroid, written 1000x too large, came back two days
    later as `0x100D arm collision detected`

    joint_max_speed was assumed [180]*6 + [225]; the arm reports
    [180, 180, 225, 225, 225, 225, 225], so four joints were scored
    against a limit 25 % below the real one

    joint_max_acc was called "already at its documented ceiling" from
    reading the DOCS' default (500 RPM/s). The arm reports 600 deg/s^2
    = 100 RPM/s — **20 % of the ceiling**, so it is a 5x lever, not a
    dead end

The SDK exposes **93 `rm_get_*` methods, 79 of which take no argument.**
We were reading about six. This calls all of them, by introspection rather
than by a hand-maintained list, so a getter added by a future SDK is
picked up without anyone remembering to add it here.

    python3 arm_census.py                 # both arms, compare, report
    python3 arm_census.py --side left     # one arm
    python3 arm_census.py --save          # write census/<side>.json
    python3 arm_census.py --diff FILE     # what changed since that census

The JSON is the point as much as the report: it is the baseline a
commissioning reconciler diffs against (H32 / alix_commissioning).
"""

import datetime
import inspect
import json
import pathlib
import sys

from dual_arm_common import (
    handle_cli, LEFT_IP, RIGHT_IP, ROBOT_PORT,
)

OUT = pathlib.Path(__file__).resolve().parent.parent / "census"

USAGE = (
    "Usage: python3 arm_census.py [--side left|right|both] [--save]\n"
    "       [--diff FILE] [--all] [-h|--help]\n"
    "  --side S     which arm (default: both)\n"
    "  --save       write census/<side>.json\n"
    "  --diff FILE  compare a saved census against the arm now\n"
    "  --all        print every field, not just the interesting ones\n"
    "  -h, --help   show this documentation and exit")

# Getters that need an argument, or that would do something other than
# read. Everything else is discovered by introspection.
SKIP = {
    "rm_get_arm_event_call_back",       # takes a callback, registers it
    "rm_get_realtime_push",             # returns config, harmless, but noisy
}

# What our code believes, and where it believes it. The census FAILS if the
# arm disagrees — that is the whole point of the file.
def assumptions():
    import speed_limits
    import controller_caps
    from dual_arm_common import LIFT_GEAR
    from segment_verifier import FORCE_MODEL_NAME
    return [
        ("joint_max_speed", "rm_get_joint_max_speed",
         [180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0],
         "rm_emulator.joint_max_speed"),
        ("joint_max_acc", "rm_get_joint_max_acc", [600.0] * 7,
         "rm_emulator.joint_max_acc"),
        ("line_speed", "rm_get_arm_max_line_speed",
         speed_limits.CONFIGURED["line_speed"], "speed_limits.CONFIGURED"),
        ("line_acc", "rm_get_arm_max_line_acc",
         speed_limits.CONFIGURED["line_acc"], "speed_limits.CONFIGURED"),
        ("angular_speed", "rm_get_arm_max_angular_speed",
         speed_limits.CONFIGURED["angular_speed"], "speed_limits.CONFIGURED"),
        ("angular_acc", "rm_get_arm_max_angular_acc",
         speed_limits.CONFIGURED["angular_acc"], "speed_limits.CONFIGURED"),
        ("install_pose", "rm_get_install_pose", [0.0, 90.0, 0.0],
         "cleaning_path.MOUNT_RY_DEG — the arms are mounted Ry(+90)"),
        ("arm model", "rm_get_robot_info", "RM_75",
         f"segment_verifier.FORCE_MODEL_NAME = {FORCE_MODEL_NAME}"),
    ]


def _plain(v):
    """ctypes structs and arrays -> something json can hold."""
    if hasattr(v, "to_dictionary"):
        try:
            return _plain(v.to_dictionary())
        except Exception:
            pass
    if isinstance(v, dict):
        return {str(k): _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    if isinstance(v, (int, float, str, bool)) or v is None:
        return v
    if isinstance(v, bytes):
        return v.decode(errors="replace")
    if hasattr(v, "_fields_"):
        return {f[0]: _plain(getattr(v, f[0])) for f in v._fields_}
    return repr(v)


def read_all(robot):
    """Call every no-argument rm_get_*. Never raises; records failures."""
    out, failed = {}, {}
    for name in sorted(dir(robot)):
        if not name.startswith("rm_get") or name in SKIP:
            continue
        fn = getattr(robot, name)
        try:
            sig = inspect.signature(fn)
        except (TypeError, ValueError):
            continue
        if any(p.default is inspect.Parameter.empty
               for p in sig.parameters.values()):
            continue                      # needs an argument; handled below
        try:
            out[name] = _plain(fn())
        except Exception as exc:
            failed[name] = repr(exc)[:120]
    return out, failed


def read_frames(robot, census):
    """The per-name getters we care about: tool frames and their payloads."""
    frames = {}
    tot = census.get("rm_get_total_tool_frame") or {}
    names = tot.get("tool_names") if isinstance(tot, dict) else None
    for n in (names or []):
        try:
            ret, got = robot.rm_get_given_tool_frame(n)
            if ret == 0:
                frames[n] = _plain(got)
        except Exception as exc:
            frames[n] = {"error": repr(exc)[:100]}
    return frames


def check(census, side):
    """Compare the arm against what the code believes. This is the payoff."""
    rows = []
    for label, getter, expect, where in assumptions():
        got = census.get(getter)
        if isinstance(got, list) and len(got) == 2 and got[0] in (0, 1):
            got = got[1]                  # (ret, value) pairs
        actual = got
        if label == "arm model" and isinstance(got, dict):
            actual = str(got.get("arm_model"))
        # Some getters answer with a dict keyed x/y/z plus a return_code
        # rather than a (ret, list) pair — rm_get_install_pose is one.
        # Comparing the raw dict against a list is a false WRONG, which is
        # exactly the kind of noise that gets a check ignored.
        if isinstance(actual, dict) and {"x", "y", "z"} <= set(actual):
            actual = [float(actual["x"]), float(actual["y"]),
                      float(actual["z"])]
        ok = None
        if actual is None:
            ok = None
        elif isinstance(expect, list) and isinstance(actual, list):
            ok = (len(expect) == len(actual)
                  and all(abs(float(a) - float(b)) < 1e-6
                          for a, b in zip(expect, actual)))
        elif isinstance(expect, float) and isinstance(actual, (int, float)):
            ok = abs(float(actual) - expect) < 1e-6
        else:
            ok = str(actual) == str(expect)
        rows.append((label, expect, actual, ok, where))
    return rows


def report(side, census, failed, frames, show_all=False):
    print(f"\n{'=' * 74}")
    print(f"  {side.upper()} arm — {len(census)} getters answered, "
          f"{len(failed)} unavailable")
    print(f"{'=' * 74}")

    print("\n  IDENTITY")
    for k in ("rm_get_sn", "rm_get_robot_info", "rm_get_arm_software_info",
              "rm_get_joint_software_version", "rm_get_tool_software_version",
              "rm_get_system_runtime"):
        if k in census:
            print(f"    {k[7:]:<26} {json.dumps(census[k])[:150]}")

    print("\n  LIMITS — the class of value that has bitten us twice")
    for k in ("rm_get_joint_max_speed", "rm_get_joint_drive_max_speed",
              "rm_get_joint_max_acc", "rm_get_joint_drive_max_acc",
              "rm_get_joint_min_pos", "rm_get_joint_max_pos",
              "rm_get_arm_max_line_speed", "rm_get_arm_max_line_acc",
              "rm_get_arm_max_angular_speed", "rm_get_arm_max_angular_acc"):
        if k in census:
            print(f"    {k[7:]:<28} {json.dumps(census[k])}")

    print("\n  SAFETY / CAPABILITY")
    for k in ("rm_get_collision_stage", "rm_get_collision_detection",
              "rm_get_self_collision_enable",
              "rm_get_self_endeffector_collision_enable",
              "rm_get_collision_remove_enable",
              "rm_get_avoid_singularity_mode", "rm_get_arm_run_mode",
              "rm_get_arm_power_state", "rm_get_joint_en_state",
              "rm_get_joint_err_flag", "rm_get_electronic_fence_enable",
              "rm_get_virtual_wall_enable", "rm_get_install_pose"):
        if k in census:
            print(f"    {k[7:]:<40} {json.dumps(census[k])[:110]}")

    print("\n  PERIPHERALS")
    for k in ("rm_get_lift_state", "rm_get_expand_state",
              "rm_get_gripper_state", "rm_get_tool_voltage",
              "rm_get_tool_rs485_mode", "rm_get_controller_rs485_mode",
              "rm_get_rm_plus_base_info", "rm_get_rm_plus_state_info",
              "rm_get_force_data", "rm_get_fz"):
        if k in census:
            print(f"    {k[7:]:<30} {json.dumps(census[k])[:120]}")

    if frames:
        print(f"\n  TOOL FRAMES ({len(frames)})")
        print(f"    {'name':<14} {'payload':>8}  {'centroid mm':<24} pose")
        for n, f in frames.items():
            if "error" in f:
                print(f"    {n:<14} {f['error']}")
                continue
            com = (f.get('x', 0), f.get('y', 0), f.get('z', 0))
            pose = [round(v, 4) for v in (f.get("pose") or [])[:3]]
            print(f"    {n:<14} {f.get('payload', 0):8.3f}  "
                  f"{str(tuple(round(c, 1) for c in com)):<24} {pose}")

    rows = check(census, side)
    print("\n  DOES THE ARM AGREE WITH OUR CODE?")
    bad = 0
    for label, expect, actual, ok, where in rows:
        mark = {True: "  ok  ", False: " WRONG", None: "  ?   "}[ok]
        print(f"   {mark} {label:<16} code={json.dumps(expect)[:34]:<36} "
              f"arm={json.dumps(actual)[:34]}")
        if ok is False:
            bad += 1
            print(f"          -> our value lives in {where}")
    print(f"\n    {len(rows) - bad}/{len(rows)} agree"
          + ("" if not bad else f"   {bad} DISAGREE — fix the code, not the arm"))

    if failed:
        print(f"\n  UNAVAILABLE ({len(failed)}) — firmware or model does not "
              "support these")
        for k, v in sorted(failed.items()):
            print(f"    {k[7:]:<40} {v[:60]}")

    if show_all:
        print("\n  EVERYTHING ELSE")
        shown = set()
        for grp in (census,):
            for k in sorted(grp):
                if k not in shown:
                    print(f"    {k[7:]:<42} {json.dumps(grp[k])[:110]}")
    return bad


def connect(ip):
    from Robotic_Arm.rm_robot_interface import RoboticArm
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    h = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
    return robot if (h is not None and h.id > 0) else None


def main() -> int:
    handle_cli(__doc__, extra_flags=("--save", "--all"),
               value_flags=("--side", "--diff"), usage=USAGE,
               allow_common=False)
    argv = sys.argv[1:]

    def arg(flag, default=None):
        for i, a in enumerate(argv):
            if a == flag and i + 1 < len(argv):
                return argv[i + 1]
        return default

    which = arg("--side", "both")
    sides = ("left", "right") if which == "both" else (which,)
    bad_total = 0
    for side in sides:
        robot = connect(LEFT_IP if side == "left" else RIGHT_IP)
        if robot is None:
            print(f"\n  [SKIP] {side} arm not reachable")
            continue
        census, failed = read_all(robot)
        frames = read_frames(robot, census)
        bad_total += report(side, census, failed, frames, "--all" in argv)
        if "--save" in argv:
            OUT.mkdir(parents=True, exist_ok=True)
            doc = {
                "side": side,
                "taken": datetime.datetime.now().isoformat(timespec="seconds"),
                "getters": census,
                "unavailable": failed,
                "tool_frames": frames,
            }
            p = OUT / f"{side}.json"
            p.write_text(json.dumps(doc, indent=2, ensure_ascii=False,
                                    sort_keys=True) + "\n")
            print(f"\n  wrote {p}")
        old = arg("--diff")
        if old:
            prev = json.loads(pathlib.Path(old).read_text())
            a, b = prev.get("getters", {}), census
            changed = [k for k in sorted(set(a) | set(b))
                       if json.dumps(a.get(k)) != json.dumps(b.get(k))]
            # these move on their own; they are not configuration
            live = {"rm_get_current_arm_state", "rm_get_joint_degree",
                    "rm_get_current_joint_current", "rm_get_system_runtime",
                    "rm_get_current_joint_temperature", "rm_get_force_data",
                    "rm_get_current_joint_voltage", "rm_get_torque_data",
                    "rm_get_arm_all_state", "rm_get_fz", "rm_get_joint_odom"}
            cfg = [k for k in changed if k not in live]
            print(f"\n  DIFF vs {old} — {len(cfg)} CONFIGURATION changes "
                  f"({len(changed) - len(cfg)} live readings ignored)")
            for k in cfg:
                print(f"    {k[7:]:<40}\n        was {json.dumps(a.get(k))[:90]}"
                      f"\n        now {json.dumps(b.get(k))[:90]}")
        try:
            robot.rm_delete_robot_arm()
        except Exception:
            pass
    return 1 if bad_total else 0


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
