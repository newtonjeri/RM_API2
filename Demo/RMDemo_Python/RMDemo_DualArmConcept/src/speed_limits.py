"""Controller speed limits — read, report, and (optionally) set them.

WHAT `v` SCALES (RealMan, 2026-08-12, consistent across both replies):

    movej:  effective = v_interface x realtime_adjust x JOINT speed limit
    movel:  effective = v_interface x realtime_adjust x TCP speed constraint

The BASELINE DIFFERS BY COMMAND: `movej` scales against the joint row below,
`movel` against the linear row. `realtime_adjust` is the pendant's global
override — it multiplies everything, it is NOT readable through the SDK, and
it must be verified at 100 % by eye before any timing measurement (H58). A
useful after-the-fact check: at 100 % override the achieved p95 lands AT or
slightly ABOVE the commanded constraint (H59, measured 105-110 %).

⚠ REALMAN ADVISES AGAINST CHANGING THE TCP VALUES (H62): "不建议修改TCP的值...
点击停止能够立刻停下来" — the defaults are algorithm-designed and preserve the
ability to STOP IMMEDIATELY on command. Raising `line_acc` is our single most
effective lever (-15 %/-17 % of stroke time, H57) but its cost in stopping
distance has never been measured. Treat that as an open safety item.

All three levels are settable, with getters for each:

    joint    rm_set_joint_max_speed(joint_num, deg/s)      per joint 1..7
             rm_set_joint_max_acc(joint_num, deg/s^2)
    linear   rm_set_arm_max_line_speed(m/s)                TCP
             rm_set_arm_max_line_acc(m/s^2)
    angular  rm_set_arm_max_angular_speed(rad/s)           TCP
             rm_set_arm_max_angular_acc(rad/s^2)

THE FACTORY DEFAULTS (H64 — CONFIRMED 2026-08-12, Newton). Established on a
NEVER-USED arm of the same model, values read back after pressing the
pendant's "Default" button, so they are uncontaminated by our own programs:

    joints   180 deg/s (J1-J6), 225 (J7), 600 deg/s^2
    linear   0.250 m/s, 1.600 m/s^2
    angular  0.600 rad/s, 4.000 rad/s^2

These are EXACTLY what the F10 read of both our arms returned on 2026-08-07,
so both arms were at their shipped state at that point, and 1.600 m/s^2 IS
the default linear acceleration. Two consequences: `reset_limits.py --apply`
restores correctly, and the `line_acc` ladder's 1.6 rung is the default
configuration -- so 2.4 and 3.6 are genuinely ABOVE default, which is
precisely what RealMan caution against (H62).

Pendant restore path: Security Config -> TCP Speed Limit -> "Default"
(and "Default Limit" for the joint block). The API equivalent is still
unidentified; `rm_set_arm_tcp_init` remains the candidate, untested.

The pendant also documents the JOINT setter ranges: speed 1-180 deg/s,
acceleration 1-600 deg/s^2. So the joint values above are simultaneously the
defaults AND the maxima -- there is no headroom to raise them into.

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

So `rm_movel` at v=100 % TARGETS 0.250 m/s — close to MoveIt's average
pace, well under its peaks. Raising the cap to match MoveIt's peaks would
mean wiping porcelain at nearly a metre per second.

But a target is not an achieved speed, and the stroke does NOT hold it.
MEASURED 2026-08-12 across `runs/*/run.json`: at line_speed 0.45 with
v=100 fixed, the achieved MEDIAN TCP is 62.6 % of cap at line_acc 1.6,
77.0 % at 2.4, 87.6 % at 3.6 (left arm; right replicates 62.8/78.4/88.1)
while p95 stays pinned AT the cap (474->492 mm/s against 450). The tool
reaches the commanded speed; it cannot hold it between corners. The median
is ACCELERATION-bound (H57) — 42 % of the stroke time covers 11 % of the
path and 88 % of that is ramping (H43). Raising `line_speed` without
raising `line_acc` buys little: ramp distance is v^2/2a, so at line_acc
2.4 a 0.45 m/s cap needs ~84 mm of accel+decel against a ~230 mm mean
segment, while a 0.80 m/s cap needs ~267 mm — more than the segment, so
cruise is never reached at all.

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

# Vendor-stated ceilings and the ratio rule (RealMan, 2026-08-12).
MAX_LINE_SPEED = 1.800          # m/s — the only maximum they have given us
ACC_SPEED_RATIO = 3.0           # acceleration >= 3 x speed, enforced (ret=1)
# NOTE: no maximum for line_acc has ever been stated. At MAX_LINE_SPEED the
# ratio alone would demand >= 5.4 m/s^2, which we have never run — the
# highest tested is 3.6. It is an open question in QUESTIONS_FOR_REALMAN.


def scale_for(line_speed, ratio=ACC_SPEED_RATIO, acc=None):
    """A legal (line_speed, line_acc) pair for a target speed.

    The controller rejects a pair whose acceleration is under `ratio` times
    the speed with a bare ret=1, so a test that raises only the speed fails
    to set anything and then runs at whatever was already there — reading as
    "the speed made no difference". Scale BOTH, together, or not at all.

    Returns (line_speed, line_acc, notes). Raises SystemExit above the
    vendor ceiling rather than letting the controller refuse it later.
    """
    notes = []
    if line_speed <= 0:
        raise SystemExit("line_speed must be positive")
    if line_speed > MAX_LINE_SPEED:
        raise SystemExit(
            "line_speed %.3f exceeds the vendor maximum %.3f m/s"
            % (line_speed, MAX_LINE_SPEED))
    need = ratio * line_speed
    if acc is None:
        # Keep the shipped 6.4x headroom where it still satisfies the rule,
        # so a modest speed rise does not silently raise acceleration too —
        # RealMan advise against changing these at all (H62).
        acc = max(CONFIGURED["line_acc"], need)
        if acc > CONFIGURED["line_acc"]:
            notes.append("line_acc raised %.3f -> %.3f to satisfy the %.0fx "
                         "ratio" % (CONFIGURED["line_acc"], acc, ratio))
    elif acc < need - 1e-9:
        raise SystemExit(
            "line_acc %.3f is below %.0f x line_speed %.3f = %.3f; the "
            "controller would reject the pair with ret=1"
            % (acc, ratio, line_speed, need))
    if acc > 3.6:
        notes.append("line_acc %.3f is above the highest we have ever tested "
                     "(3.6) and no vendor maximum is documented" % acc)
    if line_speed > CONFIGURED["line_speed"]:
        notes.append("above the factory default %.3f m/s — RealMan advise "
                     "against changing the TCP values (H62); these limits "
                     "also RATCHET, so run reset_limits.py afterwards"
                     % CONFIGURED["line_speed"])
    return line_speed, acc, notes

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
# JOINT — UNITS SETTLED 2026-08-11 from three independent sources, after
#   an earlier note here quoted an RPM ceiling that came from the wrong
#   interface entirely:
#
#     Python API reference (apipython/classes/jointsConfigQuery)
#         rm_get_joint_max_speed -> "Maximum joint speed, in °/s."
#         rm_get_joint_max_acc   -> "Maximum joint acceleration, in °/s."
#                                    (their typo; an acceleration is °/s^2)
#     C header rm_interface.h
#         rm_set_joint_max_acc   -> 关节最大加速度，单位：°/s²
#     and the cross-check: 180 °/s = 30 RPM, exactly the JSON default.
#
#   **The C/Python API is in DEGREES. The JSON protocol page is a
#   DIFFERENT interface, in RPM with 0.001 scaling.** Its "no more than
#   500 RPM/s" is a limit on the JSON `set_joint_max_acc`, not on anything
#   we call. Quoting it as though it were read off the arm was a category
#   error and it produced a "5x headroom" recommendation that measurement
#   then refuted.
#
#   MEASURED off the left arm:
#     rm_get_joint_max_speed = [180, 180, 225, 225, 225, 225, 225]  °/s
#     rm_get_joint_max_acc   = [600] * 7                            °/s^2
#     the *_drive_* variants return exactly the same values
#     joint POSITION limits DIFFER between arms: J4 is ±169° left,
#       ±140° right — every other joint matches
#
#   RETRACTED 2026-08-11 — a claim stood here that `joint_max_acc` does
#   not bound Cartesian motion, with a "line_acc 1.6 -> 900-1250 °/s^2,
#   line_acc 2.4 -> 2700-4500" table. Re-audited against runs/ and it does
#   not survive. Four separate defects, each on its own sufficient:
#
#     * NO RUN EVER RECORDED line_acc = 1.6. That number is CONFIGURED
#       above — our own envelope constant — used as if it were a per-run
#       setting. Every run that records `limits_in_force` shows
#       line_acc = 2.4, unchanged. The variable was never varied.
#     * the runs labelled "1.6" (20260810T221245/221548/221832) are the
#       PENDANT-OVERRIDE ladder at a constant line_speed 0.250: they
#       achieved 38 / 68 / 94 % of that cap. The rise in joint rates
#       across them is the pendant slider, not any controller limit.
#     * 18 of the 24 runs are sim=True, and SIM reports joint speed ~0
#       while still moving the TCP. One cited run (173010) also has dt
#       spanning 0.04 ms .. 178 ms, so an unguarded derivative divided by
#       ~zero.
#     * the magnitude is method-dependent, not measured. Four defensible
#       estimators over the same window disagree by 2.6x-16.6x; on
#       20260810T221832 they give 1385 / 11410 / 2310 / 1065 °/s^2. And
#       `speed{n}` is not a clean derivative of `position{n}` — they
#       differ by a median 12 °/s and up to 123 °/s on J4 — so
#       differentiating it again compounds an unknown filter.
#
#   WHAT IS ACTUALLY MEASURED, first-order and cross-checked on both the
#   `speed{n}` channel and dt-guarded d(position)/dt (they agree in
#   ordering, dpos/dt running ~20 % higher):
#
#     line_speed   peak joint speed °/s (reported | dpos/dt)   outcome
#       0.250        125 J3  |  164 J6                          ok
#       0.450        195 J4  |  248 J4                          ok
#       0.800        238 J4, 186 J1, 225 J6  |  287 J3          FAILED
#
#   At 0.800 the run exceeded rm_get_joint_max_speed on J1 (186 > 180)
#   and J4 (238 > 225) and that is the run that failed. HYPOTHESIS ONLY:
#   n = 1 pass against n = 1 fail, the excess covers 0.5 % of samples,
#   and it begins 1.1 s before the stage ends rather than at it. The
#   failure carried no joint error bits and no collision code.
#
#   Ratio constraints still hold: joint acc/speed >= 1.5 (600/180 = 3.3).
#   Nothing here supports asking RealMan about a `line_acc` ceiling yet;
#   what it supports is a run that holds line_speed fixed and varies
#   line_acc, REAL only, with the pendant slider verified at 100 %.
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
    """What the commanded percentages mean in physical units.

    The BASELINE DIFFERS BY COMMAND (RealMan, 2026-08-12): `movel`
    scales v% against the TCP speed constraint, `movej` against the joint
    speed limit. Both figures are therefore reported, each against its own
    baseline — reporting only one of them is what made the earlier version
    of this function misleading.

    Both are TARGETS, not achieved speeds. Two things sit between the
    number printed here and the tool: the pendant's real-time override,
    which multiplies and is unreadable (H58), and ramping, which holds the
    achieved median well below the target (H57).
    """
    ls = limits.get("line_speed")
    js = limits.get("joint_speed")
    parts = []
    if isinstance(ls, float):
        parts.append(f"movel target (v% x TCP constraint): cleaning "
                     f"{cleaning_pct}% = {ls * cleaning_pct / 100:.3f} m/s, "
                     f"transit {transit_pct}% = "
                     f"{ls * transit_pct / 100:.3f} m/s (constraint "
                     f"{ls:.3f})")
    if isinstance(js, list) and js:
        top = max(js)
        parts.append(f"movej target (v% x joint limit): transit "
                     f"{transit_pct}% = {top * transit_pct / 100:.0f} deg/s "
                     f"on the fastest joint (limit {top:.0f})")
    if parts:
        parts.append("TARGETS only — the pendant override multiplies both "
                     "and is unreadable; achieved median runs below target "
                     "because the stroke is acceleration-bound")
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
    """Put back what apply()/prepare() returned; never raises."""
    if before and "_joints" in before:
        restore_joints(robot, before.pop("_joints"))
    for key, val in (before or {}).items():
        if key in _SETTERS and isinstance(val, float):
            try:
                getattr(robot, _SETTERS[key])(val)
            except Exception:
                pass


ENABLED = os.environ.get("RM_SET_LIMITS") == "1"


# What a run may raise, by env, and why you would.
#
# Measured 2026-08-10 on `toplid_right` (27 segments, 6202 mm):
#   * at the shipped 0.250 m/s the joints run at **23 % of their limits**
#     at cruise — there is a lot of headroom.
#   * `movel` is bounded by the linear AND the angular Cartesian cap at the
#     same time — two independent ceilings, neither scaled by `v` — so
#     whichever saturates first throttles the whole segment. At 0.250 m/s
#     only 7 of 27 segments are angular-limited and it costs +2 % of cruise
#     time. Raise the linear cap alone and that flips: 11/27 at 0.350,
#     13/27 at 0.400, 24/27 and +36 % at 0.533.
#     (These counts still stand after the 2026-08-12 v-semantics correction:
#     they were measured at v=100, where the old `cap * v/100` model and the
#     correct unscaled-cap model give the identical cap. Only the sentence
#     explaining WHY was wrong — it is the two Cartesian caps racing each
#     other, not one `v%` feeding both.)
#
# So the two move TOGETHER or the angular cap eats the gain. With
# angular_speed at 1.200 rad/s, a 0.350 m/s linear cap leaves 1 of 27
# angular-limited instead of 11.
#
#   RM_LINE_SPEED=0.350        m/s     (needs line_acc/line_speed >= 3)
#   RM_ANGULAR_SPEED=1.200     rad/s   (needs angular_acc/angular_speed >= 3)
#   RM_LINE_ACC / RM_ANGULAR_ACC       if the ratio needs the other side
#
# These are GLOBAL controller state, shared with the Web GUI and every other
# program on the arm. `prepare()` returns the previous values and the caller
# MUST restore them — the same contract as controller_caps.
_ENV = {
    "line_speed": "RM_LINE_SPEED",
    "line_acc": "RM_LINE_ACC",
    "angular_speed": "RM_ANGULAR_SPEED",
    "angular_acc": "RM_ANGULAR_ACC",
}

# Per-joint limits. Set on EVERY joint at once — a per-joint ladder is not
# something this project needs and a partial application is a trap.
#
#   RM_JOINT_ACC=1200      deg/s^2 on all 7; the arm ships at 600.
#     ⚠ THE "5x LEVER" CLAIM THAT STOOD HERE IS RETRACTED. It read the
#     500 RPM/s ceiling off the JSON-protocol page — a DIFFERENT interface
#     from the C/Python API we call. See the JOINT block above. The
#     3000 deg/s^2 constant below is kept only as a conservative guard; it
#     is NOT a vendor figure for this API and must not be quoted as one.
#   RM_JOINT_SPEED=200     deg/s on all 7. And note H41: J1 and J4 exceeded
#     rm_get_joint_max_speed during movel with no clamp and no fault, so
#     this may be a planning parameter that protects nothing at execution.
#     Enforce joint-speed bounds before dispatch, not by setting this.
#
# NOTE: RM_SET_LIMITS is NOT a gate. `ENABLED` below is unused; setting any
# RM_LINE_* / RM_ANGULAR_* / RM_JOINT_* var applies it on its own.
_JOINT_ENV = {
    "joint_acc": ("RM_JOINT_ACC", "rm_set_joint_max_acc",
                  "rm_get_joint_max_acc"),
    "joint_speed": ("RM_JOINT_SPEED", "rm_set_joint_max_speed",
                    "rm_get_joint_max_speed"),
}
JOINT_ACC_CEILING_DEG_S2 = 3000.0    # 500 RPM/s, RealMan's documented max


def _joint_wanted():
    out = {}
    for key, (var, _s, _g) in _JOINT_ENV.items():
        v = os.environ.get(var)
        if v is not None:
            out[key] = float(v)
    return out


def apply_joints(robot, **values) -> dict:
    """Set a per-joint limit on all 7 joints; return the previous arrays.

    Enforces the documented ceiling and the acc/speed >= 1.5 ratio here,
    because the controller answers a violation with a bare ret=1.
    """
    before = {}
    for key, want in values.items():
        var, setter, getter = _JOINT_ENV[key]
        ret, cur = getattr(robot, getter)()
        before[key] = list(cur) if ret == 0 else None
        if key == "joint_acc" and want > JOINT_ACC_CEILING_DEG_S2:
            raise ValueError(
                f"joint_acc {want:.0f} deg/s^2 exceeds RealMan's documented "
                f"maximum of {JOINT_ACC_CEILING_DEG_S2:.0f} deg/s^2 "
                "(500 RPM/s)")
        # ratio check against whatever the other quantity will be
        spd = values.get("joint_speed")
        acc = values.get("joint_acc")
        if spd is None and before.get("joint_speed") is None:
            ret2, cs = robot.rm_get_joint_max_speed()
            spd = max(cs) if ret2 == 0 else None
        if acc is not None and spd:
            if acc / spd < JOINT_ACC_SPEED_RATIO:
                raise ValueError(
                    f"joint acceleration/speed = {acc:.0f}/{spd:.0f} = "
                    f"{acc / spd:.2f}, below the documented minimum of "
                    f"{JOINT_ACC_SPEED_RATIO}")
        for j in range(1, 8):
            r = getattr(robot, setter)(j, float(want))
            if r != 0:
                print(f"  [WARN] {setter}(J{j}, {want}) -> ret={r}")
    return before


def restore_joints(robot, before: dict) -> None:
    for key, arr in (before or {}).items():
        if not arr:
            continue
        _var, setter, _g = _JOINT_ENV[key]
        for j, v in enumerate(arr, start=1):
            try:
                getattr(robot, setter)(j, float(v))
            except Exception:
                pass


def wanted_from_env() -> dict:
    out = {}
    for key, var in _ENV.items():
        v = os.environ.get(var)
        if v is not None:
            out[key] = float(v)
    return out


def prepare(robot, label: str = "") -> dict:
    """Apply any env-requested limits; return the previous values.

    Returns {} when nothing was asked for, so the caller's restore is a
    no-op. Raising past the F10 envelope is deliberate here — that is what
    the env var means — but the acc/speed ratio is still enforced, because
    the controller enforces it too and a bare ret=1 is a poor error.
    """
    want = wanted_from_env()
    jwant = _joint_wanted()
    if not want and not jwant:
        return {}
    before = read(robot)
    shown = ", ".join(f"{k}={v}" for k, v in sorted(want.items()))
    print(f"  [INFO] raising limits {label}: {shown}")
    for k, v in sorted(want.items()):
        was = before.get(k)
        if isinstance(was, float):
            print(f"           {k:<14} {was:.3f} -> {v:.3f}")
    out = {}
    try:
        if jwant:
            for k, v in sorted(jwant.items()):
                print(f"           {k:<14} all joints -> {v:.0f}")
            out["_joints"] = apply_joints(robot, **jwant)
        if want:
            out.update(apply(robot, allow_raise=True, **want))
    except ValueError as exc:
        print(f"  [FAIL] {exc}")
        raise
    return out
