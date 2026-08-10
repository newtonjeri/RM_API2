"""Controller safety/assist capabilities — read, report, set, restore.

The arms run V1.7.4 and expose several protections that were simply never
switched on. The one that matters most for Phase 2:

    SINGULARITY AVOIDANCE (rm_set_avoid_singularity_mode)
        supported, OFF. 7-axis support landed in V1.7.3; both arms are on
        V1.7.4. This is the controller's own answer to the redundancy
        problem a Cartesian path poses on a 7-DOF arm — exactly the regime
        `execute_path` runs in, where the controller re-solves IK per
        target and nothing otherwise keeps the wrist on one branch.

Everything here is GLOBAL controller state shared with the Web GUI and
every other program on the arm, so `apply()` returns the previous values
and the caller restores them. Read-only reporting is the default.

Capability            getter / setter                          note
--------------------  ---------------------------------------  -----------
avoid_singularity     rm_{get,set}_avoid_singularity_mode       0/1
self_collision        rm_{get,set}_self_collision_enable        bool
endeff_collision      rm_{get,set}_self_endeffector_collision_enable
collision_stage       rm_get_collision_stage /                  0..8
                      rm_set_collision_state                    (8 = most
                                                                sensitive)
static_collision      rm_{get,set}_collision_detection          0/1, Gen-3
collision_remove      rm_{get,set}_collision_remove_enable      V1.7.4

Usage:
    caps = controller_caps.read(robot)
    print(controller_caps.describe(caps))
    prev = controller_caps.apply(robot, avoid_singularity=1)
    ...
    controller_caps.restore(robot, prev)
"""

import os

# (getter, setter, coercion for the setter argument)
_CAPS = {
    "avoid_singularity": ("rm_get_avoid_singularity_mode",
                          "rm_set_avoid_singularity_mode", int),
    "self_collision": ("rm_get_self_collision_enable",
                       "rm_set_self_collision_enable", bool),
    "endeff_collision": ("rm_get_self_endeffector_collision_enable",
                         "rm_set_self_endeffector_collision_enable", bool),
    "collision_stage": ("rm_get_collision_stage",
                        "rm_set_collision_state", int),
    "static_collision": ("rm_get_collision_detection",
                         "rm_set_collision_detection", int),
    "collision_remove": ("rm_get_collision_remove_enable",
                         "rm_set_collision_remove_enable", bool),
}

# What Phase 2 wants switched on before a Cartesian cleaning path runs.
# Deliberately small: only the ones that bear on this failure mode.
# RM_AVOID_SINGULARITY=0 opts out; RM_SET_CAPS=0 disables all changes.
#
# The collision knobs are settable BOTH ways, which matters as of
# 2026-08-10: `execute_path` aborts with system 0x100D = 机械臂发生碰撞
# ("arm collision detected"), in SIMULATION mode, where no physical
# contact is possible — so the verdict is the controller's own MODEL, and
# the only way to attribute it is to turn each check off in isolation and
# see which one owns the abort. Everything is restored on exit by
# restore(), so a diagnostic run does not leave the arm reconfigured for
# the next program.
#
#   RM_SELF_COLLISION=0     turn the arm's self-collision model OFF
#   RM_ENDEFF_COLLISION=0   turn END-EFFECTOR self-collision OFF (this one
#                           depends on the ACTIVE TOOL FRAME, and ours is
#                           L_glove_4, far off the flange)
#   RM_COLLISION_STAGE=N    0..8 detection sensitivity; 0 disables it
def _env_bool(name, default):
    v = os.environ.get(name)
    return default if v is None else (v not in ("0", "false", "False"))


WANTED = {
    "avoid_singularity": int(os.environ.get("RM_AVOID_SINGULARITY", "1")),
    "self_collision": _env_bool("RM_SELF_COLLISION", True),
}
if os.environ.get("RM_ENDEFF_COLLISION") is not None:
    WANTED["endeff_collision"] = _env_bool("RM_ENDEFF_COLLISION", True)
if os.environ.get("RM_COLLISION_STAGE") is not None:
    WANTED["collision_stage"] = int(os.environ["RM_COLLISION_STAGE"])
ENABLED = os.environ.get("RM_SET_CAPS", "1") != "0"


def read(robot) -> dict:
    """Every capability the controller will report. Never raises."""
    out = {}
    for key, (getter, _setter, _cast) in _CAPS.items():
        try:
            res = getattr(robot, getter)()
        except Exception as exc:
            out[key] = f"unavailable: {exc!r}"
            continue
        # most are (ret, value); a few return a bare dict
        if isinstance(res, tuple) and len(res) == 2:
            ret, val = res
            out[key] = val if ret == 0 else f"ret={ret}"
        elif isinstance(res, dict):
            out[key] = (res if res.get("return_code") == 0
                        else f"ret={res.get('return_code')}")
        else:
            out[key] = res
    return out


def describe(caps: dict) -> str:
    """One line per capability, flagging what is OFF but wanted."""
    bits = []
    for key in _CAPS:
        val = caps.get(key)
        flag = ""
        if key in WANTED and isinstance(val, (int, bool)) and \
                not isinstance(val, str):
            if int(val) != int(WANTED[key]):
                flag = "  <-- OFF, Phase 2 wants it ON"
        bits.append(f"    {key:20s} {str(val):>10s}{flag}")
    return "\n".join(bits)


def apply(robot, **wanted) -> dict:
    """Set capabilities; returns the PREVIOUS values for restoration.

    Global controller state — the caller owns putting it back. A setter
    that fails is reported, not raised: losing an assist is not worth
    aborting a run that is otherwise safe, but it must be visible.
    """
    before = read(robot)
    for key, val in wanted.items():
        if key not in _CAPS:
            raise KeyError(f"not a known capability: {key!r}")
        _get, setter, cast = _CAPS[key]
        try:
            ret = getattr(robot, setter)(cast(val))
            if ret != 0:
                print(f"    [WARN] {setter}({val}) -> ret={ret} "
                      "(capability not applied)")
        except Exception as exc:
            print(f"    [WARN] {setter}({val}) raised {exc!r}")
    return before


def restore(robot, before: dict) -> None:
    """Best-effort restoration of whatever apply() captured."""
    for key, val in (before or {}).items():
        if key not in _CAPS or isinstance(val, str):
            continue
        _get, setter, cast = _CAPS[key]
        try:
            getattr(robot, setter)(cast(val))
        except Exception:
            pass


def prepare(robot, label: str = "") -> dict:
    """Report the capability state, then switch on what Phase 2 wants.

    Returns the previous values (pass to restore()). With RM_SET_CAPS=0 it
    reports and changes nothing — useful for establishing what the arm was
    configured with before anyone touched it.
    """
    caps = read(robot)
    print(f"  [INFO] controller capabilities{' ' + label if label else ''}:")
    print(describe(caps))
    if not ENABLED:
        print("    (RM_SET_CAPS=0 — reporting only, nothing changed)")
        return {}
    todo = {k: v for k, v in WANTED.items()
            if not isinstance(caps.get(k), str) and
            int(caps.get(k, -1)) != int(v)}
    if not todo:
        print("    (already as Phase 2 wants — nothing to change)")
        return {}
    print(f"    enabling: {', '.join(f'{k}={v}' for k, v in todo.items())}")
    return apply(robot, **todo)
