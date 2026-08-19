"""Generate the points + sequence for a 1:1 Blockly version of a task.

Newton's experiment (2026-08-08): build a Blockly program identical in
every respect to one of the cleaning tasks, and see whether the controller
executes it. It isolates cleanly —

  Blockly MOVEL through OUR points fails too  -> the points/geometry are
      the problem, independent of our dispatcher
  Blockly succeeds, our dispatcher fails      -> the difference is in HOW
      we issue them (chaining, blend, connect flags, tool selection)

Neither answer is available from more offline analysis, and ZIGZAG01
already proves MOVEL chains work on this hardware with HAND-TAUGHT points.
What it does not prove is that our COMPUTED points work.

HOW BLOCKLY REFERENCES POINTS. A `point` block carries only a NAME
(`pointName`, `pointType 0`); the coordinates live in the controller's
global-waypoint store. So the poses do not go in the XML — they are
written to the controller, and the program refers to them. That store is
writable over the API (`rm_add_global_waypoint`), so this tool can push
all of them rather than have you type 27 poses into the GUI.

`rm_waypoint_t` carries point_name, joint[7], pose, work_frame AND
tool_frame — so each point records the frame it was defined against,
which is what makes the comparison honest.

WHAT MUST MATCH for the experiment to mean anything:
  * pole at 75 mm BEFORE the moves — the poses are resolved at that
    height, and the pole carries the arm base (215 mm at SRDF home)
  * tool frame = the task's ik_frame (L_glove_4 / R_glove_2), NOT `Hand`
    as ZIGZAG01 uses
  * work frame = World
  * speed 100 for the stroke, blend r=10 (the dispatcher's current values)

Usage:
    python3 blockly_points.py --task hinge_area_left
    python3 blockly_points.py --task toplid_right --prefix TLR
    python3 blockly_points.py --task hinge_area_left --push --mode REAL
"""

import math
import pathlib
import sys

import numpy as np

from cleaning_path import CleaningPath, R_to_euler, _Rx, _Ry, _Rz
from task_config import TaskConfig
from dual_arm_common import (
    handle_cli, LEFT_IP, RIGHT_IP, ROBOT_PORT, lift_hw_mm)
from segment_verifier import (
    arm_stages, load_plan, resolve_plan, stage_maps)

POLE_M = 0.075
DEFAULT_PREFIX = {"hinge_area_left": "HAL", "hinge_area_right": "HAR",
                  "toplid_left": "TLL", "toplid_right": "TLR"}


def _arg(flag, default=None):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def build(task, prefix=None):
    """Unique points (name -> pose, joints) plus the ordered sequence.

    The sequence REVISITS points — hinge_area_left walks 44 waypoint slots
    over 27 distinct points (point1 alone recurs many times). Blockly wants
    each point stored ONCE and referenced by name, so the two are returned
    separately.
    """
    cfg = TaskConfig.load(task)
    prefix = prefix or DEFAULT_PREFIX.get(task, task[:4].upper())
    path = CleaningPath(cfg)
    prog = path.movel_program(POLE_M)
    names = prog["waypoint_names"]
    poses = prog["poses"]

    # seed joints: MoveIt's own move_to_start solution, so the arm angle
    # of the first point matches what the dispatcher pins (F22: this is
    # the one thing the saved plan legitimately supplies)
    pref = "R_" if cfg.side == "right" else "L_"
    J = [f"{pref}joint{i}" for i in range(1, 8)]
    plan = load_plan(resolve_plan(f"{task}_ruckig_pro_only.json"))
    st = [s for s in arm_stages(plan, prefix=f"{pref}joint")
          if s["stage_name"] == "move_to_start"][0]
    seed = [math.degrees(stage_maps(st)[-1][j]) for j in J]

    joints = _solve_joints(cfg, prog, seed)

    unique, order = {}, []
    for i, (nm, p) in enumerate(zip(names, poses)):
        key = f"{prefix}_{nm}"
        if key not in unique:
            unique[key] = {"pose": p, "joint": joints[i], "src": nm}
        order.append(key)
    return cfg, prog, unique, order, seed


def _solve_joints(cfg, prog, seed):
    """Seeded IK per waypoint, with the global-tool-frame trap guarded.

    `rm_algo_set_toolframe` is process-wide: any Algo constructed after it
    resets it. Build everything first, set the tool frame LAST, then
    assert FK of the seed reproduces waypoint 0 before trusting a single
    number (this exact trap invalidated an entire analysis on 2026-08-08).
    """
    import pathlib as _pl   # SDK relative to this file, not a home dir
    sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[4]
                           / "Python"))
    from Robotic_Arm.rm_robot_interface import (
        Algo, rm_robot_arm_model_e, rm_force_type_e,
        rm_inverse_kinematics_params_t)
    from Robotic_Arm.rm_ctypes_wrap import (
        rm_frame_t, rm_pose_t, rm_position_t, rm_euler_t)
    from frame_alignment_offline import IK_FRAMES, ARM_TIP_TO_CONNECTOR_M
    algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,
                rm_force_type_e.RM_MODEL_RM_ISF_E)
    algo.handle = None
    xyz, rpy = IK_FRAMES[cfg.side][cfg.ik_frame]
    f = rm_frame_t()
    f.frame_name = b"ik"
    f.pose = rm_pose_t()
    f.pose.position = rm_position_t(xyz[0], xyz[1],
                                    xyz[2] + ARM_TIP_TO_CONNECTOR_M)
    f.pose.euler = rm_euler_t(*rpy)
    algo.rm_algo_set_toolframe(f)

    Rm = _Ry(math.radians(-CleaningPath.MOUNT_RY_DEG))

    def to_algo(p):
        pos = Rm @ np.asarray(p[:3])
        rx, ry, rz = R_to_euler(Rm @ (_Rz(p[5]) @ _Ry(p[4]) @ _Rx(p[3])))
        return [float(pos[0]), float(pos[1]), float(pos[2]),
                float(rx), float(ry), float(rz)]

    fk0 = list(algo.rm_algo_forward_kinematics(list(seed), 1))[:3]
    err = float(np.linalg.norm(np.asarray(to_algo(prog["poses"][0])[:3])
                               - np.asarray(fk0))) * 1000.0
    if err > 5.0:
        raise SystemExit(
            f"GUARD FAILED: waypoint 0 is {err:.1f} mm from FK of the "
            "move_to_start joints. The tool frame was reset under us — "
            "joints not trustworthy.")

    out, q = [], list(seed)
    for p in prog["poses"]:
        pr = rm_inverse_kinematics_params_t(q_in=list(q), q_pose=to_algo(p),
                                            flag=1)
        ret, sol = algo.rm_algo_inverse_kinematics(pr)
        if ret == 0:
            q = list(sol)[:7]
            out.append([round(float(x), 3) for x in q])
        else:
            out.append(None)          # no solution from this seed
    return out


def verify(cfg, unique, seed):
    """Cross-check every point: FK(its joints) must reproduce its pose.

    The poses come from the config chain and the joints from seeded IK —
    two different routes. If they agree, both are right; if they do not,
    one is wrong and the point must not be handed to anyone. Cheap, and
    it is the difference between "accurate" as an assurance and as a
    measurement.
    """
    import pathlib as _pl   # SDK relative to this file, not a home dir
    sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[4]
                           / "Python"))
    from Robotic_Arm.rm_robot_interface import (
        Algo, rm_robot_arm_model_e, rm_force_type_e)
    from Robotic_Arm.rm_ctypes_wrap import (
        rm_frame_t, rm_pose_t, rm_position_t, rm_euler_t)
    from frame_alignment_offline import IK_FRAMES, ARM_TIP_TO_CONNECTOR_M
    from segment_verifier import FORCE_MODEL_NAME
    algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,
                getattr(rm_force_type_e, FORCE_MODEL_NAME))
    algo.handle = None
    xyz, rpy = IK_FRAMES[cfg.side][cfg.ik_frame]
    f = rm_frame_t()
    f.frame_name = b"ik"
    f.pose = rm_pose_t()
    f.pose.position = rm_position_t(xyz[0], xyz[1],
                                    xyz[2] + ARM_TIP_TO_CONNECTOR_M)
    f.pose.euler = rm_euler_t(*rpy)
    algo.rm_algo_set_toolframe(f)
    Mf = _Ry(math.radians(CleaningPath.MOUNT_RY_DEG))
    out = {}
    for k, v in unique.items():
        if v["joint"] is None:
            out[k] = None
            continue
        fk = list(algo.rm_algo_forward_kinematics(list(v["joint"]), 1))[:3]
        got = Mf @ np.asarray(fk)          # back into the World frame
        out[k] = float(np.linalg.norm(got - np.asarray(v["pose"][:3]))) * 1000
    return out


def main() -> int:
    handle_cli(__doc__, extra_flags=("--push",),
               value_flags=("--task", "--prefix", "--out"))
    task = _arg("--task", "hinge_area_left")
    prefix = _arg("--prefix")
    cfg, prog, unique, order, seed = build(task, prefix)
    tool = prog["tool_frame"]
    unsolved = [k for k, v in unique.items() if v["joint"] is None]

    print("=" * 78)
    print(f"BLOCKLY 1:1 — {task}  [{cfg.side}]")
    print("=" * 78)
    print("SETUP (must match, or the experiment proves nothing):")
    print(f"    lift        -> {lift_hw_mm(cfg.side, POLE_M)} mm, BLOCKING, "
          "before any arm motion")
    print(f"    tool_frame  -> {tool}   (NOT 'Hand' — that is ZIGZAG01's)")
    print("    frame       -> World")
    print(f"    speed       -> {cfg.cleaning_speed_pct} for MOVEL, blend "
          f"r={prog['blend_pct']}")
    print(f"    arm         -> {LEFT_IP if cfg.side == 'left' else RIGHT_IP}")
    print()
    print(f"POINTS — {len(unique)} unique, poses in the CONTROLLER's World "
          "frame (mm, rad)")
    print(f"  {'blockly name':16s} {'src':9s} {'X':>9s} {'Y':>9s} {'Z':>9s}"
          f" {'RX':>8s} {'RY':>8s} {'RZ':>8s}")
    for k, v in unique.items():
        p = v["pose"]
        print(f"  {k:16s} {v['src']:9s} {p[0]*1000:9.3f} {p[1]*1000:9.3f} "
              f"{p[2]*1000:9.3f} {p[3]:8.4f} {p[4]:8.4f} {p[5]:8.4f}")
    if unsolved:
        print(f"\n  [WARN] no seeded IK solution for: {unsolved}")
        print("         those points may still be reachable from a "
              "different approach — the controller solves them itself")
    print()
    print(f"SEQUENCE — {len(order)} moves ({len(order)-1} MOVEL segments)")
    print(f"    1.  MOVEJ  {order[0]}     <- establishes the arm "
          "configuration")
    for i, k in enumerate(order[1:], start=2):
        print(f"    {i:<3d} MOVEL  {k}")
    print()
    print("NOTE the sequence REVISITS points — that is the task's own")
    print("cleaning_sequence (a chained polyline), not a mistake.")

    # ── accuracy check: two independent routes must agree ──
    resid = verify(cfg, unique, seed)
    vals = [r for r in resid.values() if r is not None]
    worst = max(vals) if vals else float("inf")
    print(f"ACCURACY — FK(point joints) vs the point's pose, all "
          f"{len(vals)} solved points")
    print(f"    worst residual {worst:.3f} mm"
          + ("   OK" if worst < 1.0 else "   <-- INVESTIGATE, do not use"))
    bad = [k for k, r in resid.items() if r is not None and r >= 1.0]
    if bad:
        print(f"    points over 1 mm: {bad}")
    print()

    out = _arg("--out")
    if out:
        _write(out, task, cfg, prog, unique, order, resid)

    if "--push" not in sys.argv:
        print("\n(--push writes these to the controller's global-waypoint "
              "store so Blockly can reference them by name)")
        return 0
    return _push(cfg, unique, tool)


def _write(path, task, cfg, prog, unique, order, resid):
    """Hand-off artifact for whoever writes the Blockly program."""
    from run_recorder import dumps
    doc = {
        "task_name": task,
        "side": cfg.side,
        "timestamp_human": __import__("time").strftime("%Y-%m-%d %H:%M:%S"),
        "setup": {
            "lift_mm": lift_hw_mm(cfg.side, POLE_M),
            "lift_blocking": True,
            "tool_frame": prog["tool_frame"],
            "work_frame": "World",
            "movel_speed_pct": cfg.cleaning_speed_pct,
            "blend_r_pct": prog["blend_pct"],
            "arm_ip": LEFT_IP if cfg.side == "left" else RIGHT_IP,
        },
        "units": "position mm, orientation rad, joints deg",
        "num_points": len(unique),
        "points": [
            {"name": k, "source_point": v["src"],
             "pose_mm_rad": [round(v["pose"][0] * 1000, 4),
                             round(v["pose"][1] * 1000, 4),
                             round(v["pose"][2] * 1000, 4),
                             round(v["pose"][3], 6), round(v["pose"][4], 6),
                             round(v["pose"][5], 6)],
             "joints_deg": v["joint"],
             "fk_residual_mm": (round(resid[k], 4)
                                if resid.get(k) is not None else None)}
            for k, v in unique.items()],
        "num_moves": len(order),
        "sequence": [{"index": i, "move": "MOVEJ" if i == 0 else "MOVEL",
                      "point": k} for i, k in enumerate(order)],
    }
    pathlib.Path(path).write_text(dumps(doc) + "\n")
    print(f"wrote {path}")


def _push(cfg, unique, tool):
    """Write the points to the controller's global-waypoint store."""
    from Robotic_Arm.rm_robot_interface import RoboticArm
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e, rm_waypoint_t
    ip = LEFT_IP if cfg.side == "left" else RIGHT_IP
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = robot.rm_create_robot_arm(ip, ROBOT_PORT, 3)
    if handle is None or handle.id <= 0:
        print(f"  [SKIP] hardware not reachable at {ip}")
        return 0
    ok = bad = 0
    try:
        for k, v in unique.items():
            p = v["pose"]
            wp = rm_waypoint_t(
                point_name=k,
                joint=[float(x) for x in (v["joint"] or [0.0] * 7)],
                pose=[float(p[0]), float(p[1]), float(p[2]),
                      float(p[3]), float(p[4]), float(p[5])],
                work_frame="World", tool_frame=tool)
            ret = robot.rm_add_global_waypoint(wp)
            if ret != 0:                       # already exists -> update
                ret = robot.rm_update_global_waypoint(wp)
            print(f"    {k:16s} ret={ret}")
            ok += (ret == 0)
            bad += (ret != 0)
        print(f"\n  {ok} written, {bad} failed")
        return 0 if bad == 0 else 1
    finally:
        try:
            robot.rm_delete_robot_arm()
            RoboticArm.rm_destroy()
        except Exception:
            pass


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
