"""Query both arms (READ-ONLY) and refresh docs/arm_version_info.md.

Regenerates the live sections of the versions doc — the "Current versions"
table and the capability probe — between AUTO-VERSIONS markers, and stamps
the query date. The research sections (release notes, upgrade path) are
left untouched. No motion commands are issued; getters only.

Usage:
    python3 docs/update_arm_version_info.py               # query + update
    python3 docs/update_arm_version_info.py --print-only  # show block only

Config (env, same variables as RMDemo_DualArmConcept):
    RM_LEFT_IP  (default 192.168.1.10)    RM_RIGHT_IP (default 192.168.1.103)
    RM_ROBOT_PORT (default 8080)          RM_VERSION_DOC (target .md override)

Both arms must be reachable: a partial refresh would present half-stale
data as current, so the doc is left untouched and the script exits 1.
"""

import datetime
import os
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "Python"))

from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

LEFT_IP = os.environ.get("RM_LEFT_IP", "192.168.1.10")
RIGHT_IP = os.environ.get("RM_RIGHT_IP", "192.168.1.103")
ROBOT_PORT = int(os.environ.get("RM_ROBOT_PORT", "8080"))
DOC_PATH = pathlib.Path(os.environ.get(
    "RM_VERSION_DOC",
    pathlib.Path(__file__).resolve().parent / "arm_version_info.md"))

BEGIN = "<!-- BEGIN AUTO-VERSIONS -->"
END = "<!-- END AUTO-VERSIONS -->"


def _call(fn, *args):
    """Run a getter defensively: returns (ret, payload) or (None, exc)."""
    try:
        out = fn(*args)
    except Exception as exc:
        return None, exc
    if isinstance(out, tuple):
        return out[0], (out[1] if len(out) == 2 else out[1:])
    return out, None


def _fmt_joint_versions(vals):
    parts, i = [], 0
    while i < len(vals):
        j = i
        while j < len(vals) and vals[j] == vals[i]:
            j += 1
        n = j - i
        parts.append(f"{vals[i]} × {n}" if n > 1 else f"{vals[i]}")
        i = j
    return ", ".join(parts) if parts else "?"


def query_arm(robot) -> dict:
    d = {}
    ret, info = _call(robot.rm_get_robot_info)
    d["model"] = (f"{info.get('arm_model')}, {info.get('arm_dof')}-DOF"
                  if ret == 0 else f"query failed ({ret})")
    d["gen"] = (f"Gen-{info.get('robot_controller_version')}"
                if ret == 0 else "?")
    d["force"] = info.get("force_type", "?") if ret == 0 else "?"

    ret, sw = _call(robot.rm_get_arm_software_info)
    if ret == 0:
        ctrl = sw.get("ctrl_info", {})
        plan = sw.get("plan_info", {})
        d["product"] = sw.get("product_version", "?")
        d["ctrl"] = (f"**{ctrl.get('version', '?')}** "
                     f"(build {ctrl.get('build_time', '?')})")
        d["plan"] = (f"**{plan.get('version', '?')}** "
                     f"(build {plan.get('build_time', '?')})")
        d["algo"] = sw.get("algorithm_info", {}).get("version", "?")
        d["dyn"] = sw.get("dynamic_info", {}).get("model_version", "?")
    else:
        d.update(product="query failed", ctrl="?", plan="?", algo="?", dyn="?")

    ret, jv = _call(robot.rm_get_joint_software_version)
    d["joints"] = (_fmt_joint_versions(list(jv.get("version", [])))
                   if ret == 0 and isinstance(jv, dict) else "query failed")

    ret, sn = _call(robot.rm_get_sn)
    d["sn"] = sn if ret == 0 and sn else "not supported on this firmware"

    ret, mode = _call(robot.rm_get_arm_run_mode)
    d["mode"] = ({0: "SIMULATION", 1: "REAL"}.get(mode, f"? ({mode})")
                 if ret == 0 else "?")
    return d


def _onoff(val, on="on", off="off"):
    return on if val else off


def probe_arm(robot) -> dict:
    """Capability probe, read-only. ret -2 == controller never answered."""
    def cap(fn, fmt):
        ret, payload = _call(fn)
        if ret is None:
            return f"query failed ({type(payload).__name__})"
        if ret == -2:
            return "**no response**"
        if ret != 0:
            return f"ret {ret}"
        try:
            return fmt(payload)
        except Exception:
            return f"unparsed: {payload!r}"

    return {
        "collision_stage": cap(
            robot.rm_get_collision_stage,
            lambda v: f"**level {v}**" + (" (OFF)" if v == 0 else "")),
        "static_collision": cap(
            robot.rm_get_collision_detection,
            lambda v: f"supported, {_onoff(v)}"),
        "singularity": cap(
            robot.rm_get_avoid_singularity_mode,
            lambda v: f"supported, {_onoff(v)}"),
        "self_collision": cap(
            robot.rm_get_self_collision_enable,
            lambda v: f"supported, {_onoff(v)}"),
        "payload_collision": cap(
            robot.rm_get_self_endeffector_collision_enable,
            lambda v: f"supported, {_onoff(v)}"),
        "fence": cap(
            robot.rm_get_electronic_fence_enable,
            lambda v: "supported, "
                      + _onoff(v.get("enable_state") if isinstance(v, dict)
                               else v)),
        "collision_release": cap(robot.rm_get_collision_remove_enable,
                                 lambda v: f"supported, {_onoff(v)}"),
        "torque": cap(robot.rm_get_torque_data, lambda v: f"{v}"),
        "sn": cap(robot.rm_get_sn, lambda v: v or "empty"),
    }


def host_side(robot) -> str:
    try:
        from Robotic_Arm.rm_ctypes_wrap import rm_api_version
        api = rm_api_version()
        if isinstance(api, bytes):
            api = api.decode()
    except Exception:
        api = "?"
    try:
        algo = robot.rm_algo_version()
        if isinstance(algo, bytes):
            algo = algo.decode()
    except Exception:
        algo = "?"
    return (f"**Host-side (this repo):** RM_API2 C API **{api}**, "
            f"offline algorithm library **{algo}**.")


def render(now, left, right, lp, rp, host_line) -> str:
    lh = f"Left — `{LEFT_IP}`"
    rh = f"Right — `{RIGHT_IP}`"

    rows = [
        ("Arm model", left["model"], right["model"]),
        ("Product version", left["product"], right["product"]),
        ("Controller generation", left["gen"], right["gen"]),
        ("Controller software (`ctrl`)", left["ctrl"], right["ctrl"]),
        ("Planning layer (`plan`)", left["plan"], right["plan"]),
        ("Controller algorithm library", left["algo"], right["algo"]),
        ("Dynamics model version", left["dyn"], right["dyn"]),
        ("Joint firmware", left["joints"], right["joints"]),
        ("Serial number via API", left["sn"], right["sn"]),
        ("Run mode at query time", left["mode"], right["mode"]),
    ]
    # Notes that depend on the readings must be computed, not hardcoded:
    # a fixed "levels differ" string outlives the mismatch it describes.
    stage_note = ("levels match"
                  if lp["collision_stage"] == rp["collision_stage"]
                  else "**levels differ ⇒ align them**")

    caps = [
        ("Dynamics collision detection (`collision_stage`)",
         "collision_stage", stage_note),
        ("Static-state collision switch", "static_collision",
         "added in V1.7.1"),
        ("Singularity-avoidance switch", "singularity",
         "7-axis support since V1.7.3"),
        ("Arm self-collision detection", "self_collision",
         "simulation-mode only"),
        ("Payload/end-effector self-collision", "payload_collision",
         "simulation-mode only"),
        ("Electronic fence", "fence", "simulation-mode only"),
        ("Manual collision-release (`collision_remove_enable`)",
         "collision_release", "V1.7.4 feature"),
        ("Joint torque data (`rm_get_torque_data`)", "torque",
         "N/A on RM75-6FB (wrist force sensor)"),
        ("SN read (`rm_get_sn`)", "sn",
         "no release ≤V1.7.5 documents Gen-3 SN — treat as unsupported"),
    ]

    out = [BEGIN,
           f"## 1. Current versions",
           "",
           f"*Auto-generated by `docs/update_arm_version_info.py` — "
           f"last hardware query **{now}** (read-only getters, "
           f"no motion commanded).*",
           "",
           f"| Item | {lh} | {rh} |",
           "|---|---|---|"]
    out += [f"| {name} | {l} | {r} |" for name, l, r in rows]
    out += ["", host_line, "",
            "### Live capability probe (read-only, both arms)", "",
            f"| Capability | {lh} | {rh} | Notes |",
            "|---|---|---|---|"]
    out += [f"| {name} | {lp[key]} | {rp[key]} | {note} |"
            for name, key, note in caps]
    out += ["", END]
    return "\n".join(out)


def update_doc(block: str, now: str) -> str:
    text = DOC_PATH.read_text()
    if BEGIN in text and END in text:
        pre = text.split(BEGIN)[0]
        post = text.split(END)[1]
        text = pre + block + post
    else:
        # First run on the hand-written doc: replace §1 up to §2.
        m1 = text.find("## 1. Current versions")
        m2 = text.find("\n## 2.")
        if m1 < 0 or m2 < 0:
            raise SystemExit("FATAL: cannot locate section 1 in "
                             f"{DOC_PATH} — markers missing and headings "
                             "not found; not touching the file")
        text = text[:m1] + block + text[m2:]
    # Refresh the intro's query date.
    text = re.sub(r"queried live over the API on \*\*[0-9-]+\*\*",
                  f"queried live over the API on **{now.split(' ')[0]}**",
                  text, count=1)
    DOC_PATH.write_text(text)
    return text


def main() -> int:
    print_only = "--print-only" in sys.argv[1:]
    print(f"Querying arms (read-only): left={LEFT_IP} right={RIGHT_IP}")

    left_robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    lh = left_robot.rm_create_robot_arm(LEFT_IP, ROBOT_PORT, 3)
    right_robot = RoboticArm()
    rh = right_robot.rm_create_robot_arm(RIGHT_IP, ROBOT_PORT, 3)
    try:
        missing = [ip for h, ip in ((lh, LEFT_IP), (rh, RIGHT_IP))
                   if h is None or h.id <= 0]
        if missing:
            print(f"FATAL: unreachable arm(s): {', '.join(missing)} — "
                  "both arms are required (a partial refresh would present "
                  "half-stale data as current). Doc left untouched.")
            return 1

        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        block = render(now,
                       query_arm(left_robot), query_arm(right_robot),
                       probe_arm(left_robot), probe_arm(right_robot),
                       host_side(left_robot))
        if print_only:
            print("\n" + block)
            return 0
        update_doc(block, now)
        print(f"Updated {DOC_PATH} (query time {now}). "
              "Review the change with: git diff docs/arm_version_info.md")
        return 0
    finally:
        for r in (left_robot, right_robot):
            try:
                r.rm_delete_robot_arm()
            except Exception:
                pass
        try:
            RoboticArm.rm_destroy()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
