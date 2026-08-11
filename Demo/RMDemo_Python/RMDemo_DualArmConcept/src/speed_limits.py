"""Controller speed limits — read, report, and (optionally) set them.

`v` in `rm_movej` / `rm_movel` is a PERCENTAGE OF THESE, so "cleaning at
100 %" has no physical meaning until you know what the maxima are. All
three levels are settable, with getters for each:

    joint    rm_set_joint_max_speed(joint_num, deg/s)      per joint 1..7
             rm_set_joint_max_acc(joint_num, deg/s^2)
    linear   rm_set_arm_max_line_speed(m/s)                TCP
             rm_set_arm_max_line_acc(m/s^2)
    angular  rm_set_arm_max_angular_speed(rad/s)           TCP
             rm_set_arm_max_angular_acc(rad/s^2)

WHAT THE MACHINE IS CONFIGURED FOR (F10, read from both arms):

    joints   180 deg/s (J1-J6), 225 (J7), 600 deg/s^2
    linear   0.250 m/s, 1.600 m/s^2
    angular  0.600 rad/s, 4.000 rad/s^2

WHAT MOVEIT PLANS (measured from the four saved plans, at the ik_frame):

    stage              peak TCP     mean TCP    peak joint
    move_to_pre_start  0.476 m/s    0.319       112 deg/s
    execute_path       0.62-0.95    0.24-0.35   180-197
    move_to_rest       0.40-0.63    0.23-0.36   112

Two things follow, and they pull in opposite directions:

  * The JOINT limits already agree with MoveIt's joint_limits.yaml, and
    MoveIt's planned joint speeds stay inside them. Nothing to change.
  * The LINEAR limit does NOT. MoveIt time-parameterizes against joint
    limits only — there is no TCP speed limit in joint_limits.yaml — so
    its Cartesian peaks reach 0.95 m/s, nearly 4x the controller's
    configured 0.250 m/s. Its MEAN (0.24-0.35 m/s) is about the cap.

So `rm_movel` at v=100 % runs the stroke at 0.250 m/s: close to MoveIt's
average pace, well under its peaks. For a contact wipe that is arguably
the better behaviour — but it means "100 %" is 0.25 m/s, not "whatever
MoveIt drew". Raising the cap to match MoveIt's peaks would mean wiping
porcelain at nearly a metre per second.

RECOMMENDATION: leave the limits alone. They are the machine's safety
envelope, they are global controller state shared with the Web GUI and
every other program, and the joint level already matches MoveIt. Use this
module to REPORT what a percentage means in physical units. `apply()`
exists for when a limit genuinely must change — it returns the previous
values so the caller can restore them, and it refuses to raise a limit
beyond the configured envelope unless explicitly told to.
"""

import os

# F10 — read from both arms 2026-08-07. Treated as the envelope: apply()
# will not exceed these without allow_raise=True.
CONFIGURED = {
    "line_speed": 0.250,        # m/s
    "line_acc": 1.600,          # m/s^2
    "angular_speed": 0.600,     # rad/s
    "angular_acc": 4.000,       # rad/s^2
}

# From RealMan's JSON-protocol documentation, transcribed 2026-08-10:
#   develop.realman-robotics.com/en/robot/json/armConfig/
#   develop.realman-robotics.com/en/robot/json/jointParameter/
#
# CARTESIAN — set_arm_max_line_speed (m/s) / set_arm_max_line_acc (m/s^2),
#   both 0.001 resolution, and a HARD CONSTRAINT the controller enforces:
#       max linear ACCELERATION / max linear SPEED  >=  3
#   The same ratio applies to the angular pair. As shipped we are at
#   1.600 / 0.250 = 6.4, so there is headroom to raise the speed to
#   1.600 / 3 = 0.533 m/s without touching acceleration.
#
# JOINT — set_joint_max_speed is in RPM (default 30 RPM = 180 deg/s, which
#   matches what both arms report) and set_joint_max_acc is in RPM/s,
#   **default 500 and documented "no more than 500"** — so joint
#   acceleration is already AT its ceiling and is not a lever. Its own
#   constraint is acc/speed >= 1.5 (500/30 = 16.7, satisfied).
CARTESIAN_ACC_SPEED_RATIO = 3.0
JOINT_ACC_SPEED_RATIO = 1.5
JOINT_MAX_ACC_RPM_S = 500.0     # documented ceiling, and the default
RPM_TO_DEG_S = 6.0              # 1 RPM = 360/60 deg/s

_GETTERS = {
    "line_speed": "rm_get_arm_max_line_speed",
    "line_acc": "rm_get_arm_max_line_acc",
    "angular_speed": "rm_get_arm_max_angular_speed",
    "angular_acc": "rm_get_arm_max_angular_acc",
}
_SETTERS = {
    "line_speed": "rm_set_arm_max_line_speed",
    "line_acc": "rm_set_arm_max_line_acc",
    "angular_speed": "rm_set_arm_max_angular_speed",
    "angular_acc": "rm_set_arm_max_angular_acc",
}


def read(robot) -> dict:
    """Everything the controller will scale a `v%` against."""
    out = {}
    for key, getter in _GETTERS.items():
        try:
            ret, val = getattr(robot, getter)()
            out[key] = float(val) if ret == 0 else None
        except Exception as exc:
            out[key] = f"unavailable: {exc!r}"
    try:
        ret, speeds = robot.rm_get_joint_max_speed()
        out["joint_speed"] = list(speeds) if ret == 0 else None
    except Exception as exc:
        out["joint_speed"] = f"unavailable: {exc!r}"
    try:
        ret, accs = robot.rm_get_joint_max_acc()
        out["joint_acc"] = list(accs) if ret == 0 else None
    except Exception as exc:
        out["joint_acc"] = f"unavailable: {exc!r}"
    return out


def describe(limits: dict, cleaning_pct: int, transit_pct: int) -> str:
    """What the commanded percentages mean in physical units."""
    ls = limits.get("line_speed")
    js = limits.get("joint_speed")
    parts = []
    if isinstance(ls, float):
        parts.append(f"movel: cleaning {cleaning_pct}% = "
                     f"{ls * cleaning_pct / 100:.3f} m/s, transit "
                     f"{transit_pct}% = {ls * transit_pct / 100:.3f} m/s "
                     f"(cap {ls:.3f})")
    if isinstance(js, list) and js:
        top = max(js)
        parts.append(f"movej: transit {transit_pct}% = "
                     f"{top * transit_pct / 100:.0f} deg/s on the fastest "
                     f"joint (cap {top:.0f})")
    return "; ".join(parts) if parts else "limits unreadable"


def apply(robot, allow_raise: bool = False, **values) -> dict:
    """Set limits, returning the PREVIOUS values for restoration.

    These are GLOBAL controller state — shared with the Web GUI and every
    other program on the arm — so a caller that changes them owns putting
    them back. Raising past the F10 envelope needs allow_raise=True: it
    is the machine's safety configuration, not a per-task knob.
    """
    before = read(robot)
    # The controller REJECTS a pair that violates acc/speed >= 3 (>= 1.5
    # for joints). Catching it here names the constraint; letting the
    # controller catch it returns a bare ret=1.
    merged = {k: (values.get(k, before.get(k))) for k in
              ("line_speed", "line_acc", "angular_speed", "angular_acc")}
    for spd, acc, ratio, what in (
            ("line_speed", "line_acc", CARTESIAN_ACC_SPEED_RATIO, "linear"),
            ("angular_speed", "angular_acc", CARTESIAN_ACC_SPEED_RATIO,
             "angular")):
        v_s, v_a = merged.get(spd), merged.get(acc)
        if isinstance(v_s, float) and isinstance(v_a, float) and v_s > 0:
            if v_a / v_s < ratio:
                raise ValueError(
                    f"{what}: acceleration/speed = {v_a:.3f}/{v_s:.3f} = "
                    f"{v_a / v_s:.2f}, below the documented minimum of "
                    f"{ratio:.0f}. Raise {acc} to at least "
                    f"{ratio * v_s:.3f} or lower {spd} to at most "
                    f"{v_a / ratio:.3f}.")
    for key, want in values.items():
        if key not in _SETTERS:
            raise KeyError(f"not a settable limit: {key!r}")
        cap = CONFIGURED.get(key)
        if cap is not None and want > cap and not allow_raise:
            raise ValueError(
                f"{key}={want} exceeds the configured envelope {cap} — "
                "pass allow_raise=True if that is genuinely intended")
        ret = getattr(robot, _SETTERS[key])(float(want))
        if ret != 0:
            raise RuntimeError(f"{_SETTERS[key]}({want}) -> ret={ret}")
    return before


def restore(robot, before: dict) -> None:
    """Put back what apply() returned; best-effort, never raises."""
    for key, val in (before or {}).items():
        if key in _SETTERS and isinstance(val, float):
            try:
                getattr(robot, _SETTERS[key])(val)
            except Exception:
                pass


ENABLED = os.environ.get("RM_SET_LIMITS") == "1"
