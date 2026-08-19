"""C14 — Frame alignment survey. READ-MOSTLY (moves only with --poses).

The hinge cleaning points are FIXTURE-TAUGHT poses in URDF frames
(`butterfli_ref_frame`, ik_frame `R_glove_frame_4`), while the controller
executes in ITS work frame with ITS tool frame. If the two disagree, every
cleaning point lands offset — the worst failure mode, because it looks
like it works.

OFFLINE HALF ALREADY SOLVED (frame_alignment_offline.py, 2026-08-08):
URDF `*_ConnectorLink` == RealMan `Arm_Tip` + a constant (0, 0, 15.3 mm),
zero rotation, both arms, all configurations. (Was published as 32.5 mm
on 2026-08-08 and CORRECTED the same day: the offline rm_algo had been
configured as RM_MODEL_RM_B_E, the variant with no force sensor, whose
wrist is 17.2 mm short. The C14 hardware capture exposed it — see
segment_verifier.FORCE_MODEL_NAME.) The GUI's old hand-entered
tool frame (-35, 10, 260) matches NONE of the derived frames — replace it
with the generated ones (--create-frames) rather than trusting it.

This test needs ONLY the SDK on the lab laptop. It captures, per pose:

    joints (deg)                 rm_get_joint_degree
    controller pose              rm_get_current_arm_state (work frame)
    offline rm_algo FK           same joints through the offline solver
    tool frame in use            rm_get_current_tool_frame (name + offset)
    work frame in use            rm_get_current_work_frame

and prints a machine-parseable `C14CAP` line per pose, so the URDF-side
comparison (TF: butterfli_ref_frame -> R_glove_frame_4 at those joints)
runs OFFLINE afterwards against the captured file — no ROS needed at the
arm.

Default: capture the CURRENT pose only (nothing moves). With --poses,
movej through the named states (ready, rest, zero) capturing each — the
countdown and error gate apply.

--create-frames RECREATES THE URDF GLOVE/IK FRAMES IN THE CONTROLLER'S
TREE (Newton's C14 design). The offline half (frame_alignment_offline.py)
proved URDF `*_ConnectorLink` == RealMan `Arm_Tip` + a CONSTANT
(0, 0, 15.3 mm) offset with ZERO rotation on both arms, so each xacro
frame becomes a controller tool frame at (xacro offset + 15.3 mm Z),
relative to Arm_Tip, via rm_set_manual_tool_frame:

Frame NAMES track the URDF links (`frame_alignment_offline.IK_FRAMES`),
because these frames exist so a cleaning point expressed in
`R_glove_frame_4` can be commanded directly on the controller — that only
works if both sides call it the same thing. The controller's name field is
11 chars while `R_glove_frame_4` is 15, so one mechanical rule bridges
them: drop the `_frame` token.

    R_glove_frame_1..4  ->  R_glove_1..4   (e.g. R_glove_4 = 55, 7, 220.3 mm)
    R_tip_frame         ->  R_tip
    R_index_tip_frame   ->  R_index_tip

Existing frames are UPDATED (rm_update_tool_frame); only new ones are
created. rm_set_manual_tool_frame returns ret=1 on a name that already
exists, which is what failed on the second 2026-08-08 run. The invented
`glove1..glove4`/`tip` frames from the first run are deleted.

The payload of the CURRENT tool frame is copied onto the new frames (so
force compensation is unchanged), each frame is read back to verify, and
the originally-active tool frame is RESTORED before exit — creating
frames never leaves the arm on a different tool frame.

Checks: tool/work frame readable, controller pose vs offline rm_algo FK
agreement (flags gross frame mismatch immediately), capture completeness,
and (with --create-frames) frames written + verified + original restored.

Flags: --poses  (move through ready/rest/zero instead of capture-in-place)
       --create-frames  (write the glove tool frames to the controller)
Arm selection: RM_ARM=left (default) or RM_ARM=right.
"""

import json
import os
import sys
import time

from dual_arm_common import (
    handle_cli,
    com_mm, com_from_mm,
    preflight_error_gate,
    ARM_SPEED_PCT, ARM_TIMEOUT_S, DEV_JOINT, LEFT_IP, RIGHT_IP, ROBOT_PORT,
    ArrivalMonitor, ConceptArm, apply_run_mode, countdown, mode_label,
    parse_clear_errors_arg, parse_mode_arg, report_run_modes,
    restore_run_modes, state_deg, teardown,
)
from Robotic_Arm.rm_robot_interface import RoboticArm
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e

ARM_SIDE = os.environ.get("RM_ARM", "left").lower()
POSE_STATES = ("ready", "rest", "zero")
FK_TOL_M = 0.005              # controller pose vs offline rm_algo FK

_results = {"PASS": 0, "FAIL": 0, "SKIP": 0}
N_CHECKS = 5


def result(tag: str, name: str, detail: str = ""):
    _results[tag] += 1
    suffix = f"  ({detail})" if detail else ""
    print(f"  [{tag}] {name}{suffix}")


def _offline_fk(joints_deg, install=None, tool=None, work=None):
    """FK through the offline rm_algo lib, CONFIGURED LIKE THE CONTROLLER.

    `rm_get_current_arm_state()["pose"]` is reported through the arm's
    mounting angle, its active work frame and its active tool frame. A
    bare offline FK has none of those, so comparing the two directly is
    meaningless — the 2026-08-08 run reported 868 mm / 1128 mm "errors"
    that were nothing but the 90 deg mounting rotation plus the Hand tool
    offset. rm_algo exposes the same three settings, so mirror them and
    the comparison becomes a genuine model check.
    """
    try:
        from Robotic_Arm.rm_robot_interface import (
            Algo, rm_robot_arm_model_e, rm_force_type_e)
        from Robotic_Arm.rm_ctypes_wrap import (
            rm_frame_t, rm_pose_t, rm_position_t, rm_euler_t)
        # ISF, not B: measured against both controllers 2026-08-08 — the
        # base model is 17.2 mm short at the wrist (see segment_verifier).
        model = os.environ.get("RM_FORCE_MODEL", "RM_MODEL_RM_ISF_E")
        algo = Algo(rm_robot_arm_model_e.RM_MODEL_RM_75_E,
                    getattr(rm_force_type_e, model))
        algo.handle = None
        if install:
            algo.rm_algo_set_angle(*[float(v) for v in install])
        for setter, spec in ((algo.rm_algo_set_toolframe, tool),
                             (algo.rm_algo_set_workframe, work)):
            if not isinstance(spec, dict):
                continue
            pose = list(spec.get("pose") or [0.0] * 6)
            f = rm_frame_t()
            f.frame_name = str(spec.get("name", ""))[:11].encode()
            f.pose = rm_pose_t()
            f.pose.position = rm_position_t(*[float(v) for v in pose[:3]])
            f.pose.euler = rm_euler_t(*[float(v) for v in pose[3:6]])
            f.payload = float(spec.get("payload", 0.0))
            setter(f)
        return list(algo.rm_algo_forward_kinematics(list(joints_deg), 1))[:6]
    except Exception:
        return None


def _capture(robot, label):
    """One capture record; prints the C14CAP line."""
    cap = {"label": label, "side": ARM_SIDE}
    ret, joints = robot.rm_get_joint_degree()
    cap["joints_deg"] = list(joints) if ret == 0 else None
    ret, st = robot.rm_get_current_arm_state()
    cap["controller_pose"] = list(st["pose"][:6]) if ret == 0 else None
    for name, getter in (("tool_frame", "rm_get_current_tool_frame"),
                         ("work_frame", "rm_get_current_work_frame")):
        try:
            ret, frame = getattr(robot, getter)()
            cap[name] = frame if ret == 0 else f"ret={ret}"
        except Exception as exc:
            cap[name] = f"unavailable: {exc!r}"
    # rm_get_install_pose returns ONE dict {return_code,x,y,z} — not the
    # (ret, dict) tuple most getters use. Unpacking it as a tuple raised
    # ValueError on 2026-08-08, so the mounting angle was never applied and
    # the FK check compared two different frames.
    try:
        ip_ = robot.rm_get_install_pose()
        cap["install_pose"] = ([ip_.get("x"), ip_.get("y"), ip_.get("z")]
                               if ip_.get("return_code") == 0
                               else f"ret={ip_.get('return_code')}")
    except Exception as exc:
        cap["install_pose"] = f"unavailable: {exc!r}"
    cap["offline_fk"] = (_offline_fk(
        cap["joints_deg"],
        install=(cap["install_pose"]
                 if isinstance(cap["install_pose"], list) else None),
        tool=cap.get("tool_frame"), work=cap.get("work_frame"))
        if cap["joints_deg"] else None)
    print(f"  C14CAP {json.dumps(cap, default=str)}")
    return cap


# A payload centroid is an offset from the FLANGE to the centre of mass
# of whatever is bolted on. For an Inspire hand + glove that is O(100 mm);
# the arm's whole reach is under 1 m. Anything past this bound is a unit
# error, not a heavy tool — see the 2026-08-10 128-metre incident.
MAX_COM_MM = 500.0


def _arg(flag, default=None):
    argv = sys.argv[1:]
    for i, a in enumerate(argv):
        if a == flag and i + 1 < len(argv):
            return argv[i + 1]
        if a.startswith(flag + "="):
            return a.split("=", 1)[1]
    return default


def _create_glove_frames(robot):
    """Write the URDF glove/ik frames as controller tool frames.

    Offsets come from frame_alignment_offline (xacro offsets composed with
    the proven constant Arm_Tip->ConnectorLink 15.3 mm Z). Returns the
    (tag, name, detail) triple for result()."""
    import numpy as np
    from frame_alignment_offline import (
        ARM_TIP_TO_CONNECTOR_M, IK_FRAMES, controller_frame_name,
        frame_map, print_frame_map, _euler_zyx_to_R)
    from Robotic_Arm.rm_ctypes_wrap import rm_frame_t, rm_pose_t, \
        rm_position_t, rm_euler_t

    # keep the active frame + its payload; restore the frame afterwards.
    # rm_frame_t.to_dictionary() emits the ctypes field `frame_name` under
    # the key "name" — reading "frame_name" silently yields None and the
    # restore below is then skipped (that is what happened on the
    # 2026-08-08 run: both arms were left on the last frame written).
    try:
        ret, cur = robot.rm_get_current_tool_frame()
        original = (cur.get("name") or cur.get("frame_name")) \
            if ret == 0 else None
        payload = float(cur.get("payload", 0.0)) if ret == 0 else 0.0
        # METRES from the getter — convert, or the MAX_COM_MM guard below
        # is inert: it would see 0.128 and pass, and would ALSO pass the
        # 128 that means 128 metres, which is the very defect it exists to
        # stop.
        com, com_note = com_mm(cur)
    except Exception as exc:
        return ("FAIL", "glove tool frames created",
                f"cannot read the current tool frame: {exc!r}")

    # --- payload centroid: NEVER copy a value we have not sanity-checked.
    # 2026-08-10, from a GUI screenshot: the six L_* frames carried a
    # centroid of (-12000, 44000, 128000) mm — 128 METRES — while the
    # `Hand` frame they were copied from held the sane (-12, 44, 128) mm.
    # Exactly 1000x. The controller's gravity/dynamics model then predicts
    # a torque that the joints never produce, and calls the difference an
    # external collision: system 0x100D, raised ONLY while an L_glove_*
    # frame is active, which is exactly the `execute_path` stage and the
    # Blockly program. The movej stages run on `Hand` and pass.
    #
    # The write path is the suspect (rm_frame_t.x/y/z is documented mm,
    # but a read->write round trip inflated it), so the fix is not to
    # guess a scale factor: refuse anything physically impossible, say
    # so, and VERIFY the round trip below.
    com_override = _arg("--com")
    if com_override:
        com = tuple(float(v) for v in com_override.split(","))
        print(f"    payload centroid OVERRIDDEN to {com} mm (--com)")
    pay_override = _arg("--payload")
    if pay_override is not None:
        payload = float(pay_override)
        print(f"    payload OVERRIDDEN to {payload} kg (--payload)")
    worst = max(abs(v) for v in com) if com else 0.0
    if worst > MAX_COM_MM:
        return ("FAIL", "glove tool frames created",
                f"payload centroid read from {original!r} is "
                f"{tuple(round(v, 1) for v in com)} mm — {worst:.0f} mm "
                f"from the flange, beyond the {MAX_COM_MM:.0f} mm physical "
                "bound. REFUSING to copy it onto the glove frames: a "
                "centroid this far out makes the controller's torque model "
                "predict a force that is not there, which it reports as "
                "system 0x100D 'arm collision'. Fix the source frame in "
                "the GUI, or pass the true value with "
                "--com X,Y,Z (mm) --payload KG")
    print(f"    payload {payload} kg at centroid "
          f"{tuple(round(v, 1) for v in com)} mm  (source: {original!r}, "
          f"{com_note or 'plausible'})")
    if not original:
        return ("FAIL", "glove tool frames created",
                f"cannot identify the active tool frame (ret={ret}, "
                f"keys={sorted(cur) if ret == 0 else '-'}) — refusing to "
                "write frames we could not restore from")

    # What is already on the controller? rm_set_manual_tool_frame CREATES a
    # frame and returns ret=1 if the name is taken — which is why the second
    # run of 2026-08-08 failed on all five. Existing names must be UPDATED.
    try:
        total = robot.rm_get_total_tool_frame()
        existing = list(total.get("tool_names") or [])
        capacity = 10                       # rm_frame_name_t*10 in the SDK
    except Exception as exc:
        print(f"    [WARN] cannot list tool frames ({exc!r}) — assuming none")
        existing, capacity = [], 10
    print(f"    existing tool frames ({len(existing)}/{capacity}): "
          f"{existing}")

    # Drop the invented names written on 2026-08-08: wrong values (32.5 mm
    # instead of 15.3) AND wrong names (they must match the URDF links).
    removed = []
    for legacy in ("glove1", "glove2", "glove3", "glove4", "tip"):
        if legacy in existing and legacy != original:
            if robot.rm_delete_tool_frame(legacy) == 0:
                removed.append(legacy)
                existing.remove(legacy)
    if removed:
        print(f"    removed superseded frames: {', '.join(removed)}")

    wanted = {controller_frame_name(link): (link, xyz, rpy)
              for link, (xyz, rpy) in IK_FRAMES[ARM_SIDE].items()}

    # The controller's frame LIST truncates names to 10 chars —
    # `R_index_tip` (11) comes back as `R_index_ti` — so an 11-char name
    # never matches `existing` verbatim, the writer routes to CREATE on a
    # frame that exists, and gets ret=1 forever (both arms' logged runs
    # FAIL on exactly the two *_index_tip frames this way). Reads and
    # updates DO resolve the full name (the MATCH TABLE read them back at
    # 0.00 mm), so match existence on the 10-char form and route to update.
    def _held(fname):
        return any(e[:10] == fname[:10] for e in existing)

    new_count = len([n for n in wanted if not _held(n)])
    if len(existing) + new_count > capacity:
        return ("FAIL", "glove tool frames created",
                f"{len(existing)} frames on the controller + {new_count} new "
                f"> {capacity} capacity — delete unused frames first")

    print()
    print_frame_map(ARM_SIDE, indent="    ")
    print()
    residual = np.eye(4)
    residual[2, 3] = ARM_TIP_TO_CONNECTOR_M   # imported, never re-declared
    created, failed = [], []
    for fname, (link, xyz, rpy) in wanted.items():
        T = np.eye(4)
        T[:3, :3] = _euler_zyx_to_R(*rpy)
        T[:3, 3] = xyz
        T = residual @ T
        frame = rm_frame_t()
        frame.frame_name = fname.encode()
        frame.pose = rm_pose_t()
        frame.pose.position = rm_position_t(*[float(v) for v in T[:3, 3]])
        frame.pose.euler = rm_euler_t(*[float(v) for v in rpy])
        frame.payload = payload
        # com is in mm; the setter takes the getter's unit — see
        # com_from_mm(). The read-back below is what verifies it.
        frame.x, frame.y, frame.z = com_from_mm(com)
        update = _held(fname)
        try:
            ret = (robot.rm_update_tool_frame(frame) if update
                   else robot.rm_set_manual_tool_frame(frame))
        except Exception as exc:
            ret = repr(exc)
        (created if ret == 0 else failed).append(f"{fname}(ret={ret})")
        print(f"    {'update' if update else 'create'} {fname:12s} "
              f"<- {link:20s} at ({T[0, 3] * 1000:7.1f}, "
              f"{T[1, 3] * 1000:6.1f}, {T[2, 3] * 1000:6.1f}) mm  "
              f"payload {payload} kg  ret={ret}")
    # ── MATCH TABLE: read every frame BACK off the controller ──
    # Writing returned ret=0; that is not the same as the controller
    # holding the value we meant. Read each one back and compare against
    # the same frame_map() row the write came from, so the table printed
    # here is evidence rather than an assertion.
    print("\n    MATCH TABLE — URDF frame vs what the controller now holds")
    print(f"    {'URDF link':22s} {'controller':12s} "
          f"{'expected (mm)':>22s} {'read back (mm)':>22s}  d mm")
    print("    " + "-" * 88)
    mismatched = []
    for link, fname, _conn, tip, _rpy in frame_map(ARM_SIDE):
        try:
            gret, got = robot.rm_get_given_tool_frame(fname)
        except Exception as exc:
            gret, got = -1, {"error": repr(exc)}
        if gret != 0 or not isinstance(got.get("pose"), (list, tuple)):
            mismatched.append(f"{fname}(read ret={gret})")
            print(f"    {link:22s} {fname:12s} "
                  f"{tip[0] * 1000:7.1f}{tip[1] * 1000:7.1f}"
                  f"{tip[2] * 1000:7.1f}   {'UNREADABLE':>22s}")
            continue
        rb = list(got["pose"])[:3]
        d = max(abs(a - b) * 1000.0 for a, b in zip(rb, tip))
        ok = d <= 0.5                      # 0.5 mm: float round-trip only
        if not ok:
            mismatched.append(f"{fname}({d:.1f} mm off)")
        # The PAYLOAD and its CENTROID were never verified before
        # 2026-08-10, and that is exactly where the defect hid: the pose
        # table read 0.00 mm on every row while the centroid sat at
        # 128 metres. Check what we actually meant to write.
        rb_com = com_mm(got)[0]
        rb_pay = float(got.get("payload", 0.0))
        cd = max(abs(a - b) for a, b in zip(rb_com, com)) if com else \
            max(abs(v) for v in rb_com)
        if cd > 0.5 or abs(rb_pay - payload) > 1e-3:
            ratio = ""
            for a, b in zip(rb_com, com):
                if abs(b) > 1e-6 and abs(abs(a / b) - 1000.0) < 1.0:
                    ratio = "  <-- exactly 1000x: a UNIT mismatch " \
                            "between the setter and what the controller " \
                            "stores"
                    break
            mismatched.append(f"{fname}(payload/centroid: wrote "
                              f"{tuple(round(v, 1) for v in com)} mm / "
                              f"{payload} kg, reads "
                              f"{tuple(round(v, 1) for v in rb_com)} mm / "
                              f"{rb_pay} kg{ratio})")
        if max(abs(v) for v in rb_com) > MAX_COM_MM:
            mismatched.append(
                f"{fname}(centroid {tuple(round(v, 1) for v in rb_com)} mm "
                f"is beyond the {MAX_COM_MM:.0f} mm physical bound — this "
                "frame WILL make the controller report 0x100D collision)")
        print(f"    {link:22s} {fname:12s} "
              f"{tip[0] * 1000:7.1f}{tip[1] * 1000:7.1f}{tip[2] * 1000:7.1f}"
              f"   {rb[0] * 1000:7.1f}{rb[1] * 1000:7.1f}{rb[2] * 1000:7.1f}"
              f"  {d:5.2f} {'OK' if ok else '<-- MISMATCH'}")
    print("    " + "-" * 88)
    if mismatched:
        failed.append(f"readback({', '.join(mismatched)})")

    # Restore, then READ BACK — never report a restore we did not verify.
    restored = None
    try:
        rret = robot.rm_change_tool_frame(original)
        vret, now = robot.rm_get_current_tool_frame()
        restored = (now.get("name") or now.get("frame_name")) \
            if vret == 0 else None
        print(f"    active tool frame restored to {original!r} "
              f"(ret={rret}, now {restored!r})")
    except Exception as exc:
        failed.append(f"restore({exc!r})")
    if restored != original:
        failed.append(f"restore-verify(active={restored!r}, "
                      f"want={original!r})")
    if failed:
        return ("FAIL", "glove tool frames created",
                f"created {created}; FAILED {failed}")
    return ("PASS", "glove tool frames created",
            f"{', '.join(created)}; active frame verified back on "
            f"{original!r}")


def main() -> int:
    for k in _results:
        _results[k] = 0
    handle_cli(__doc__, extra_flags=("--poses", "--create-frames"),
               value_flags=("--com", "--payload"))
    forced = parse_mode_arg()
    clear_errs = parse_clear_errors_arg()
    with_poses = "--poses" in sys.argv
    create_frames = "--create-frames" in sys.argv
    ip = LEFT_IP if ARM_SIDE == "left" else RIGHT_IP

    print("=" * 68)
    print("C14  Frame alignment survey")
    print(f"    arm={ARM_SIDE} @ {ip}")
    print("    capture: " + (f"movej through {POSE_STATES} (ARM MOVES)"
                             if with_poses else
                             "current pose only (NOTHING MOVES)"))
    print(f"    mode: {mode_label(forced)}")
    print("    C14CAP lines are the deliverable — the URDF/TF comparison")
    print("    runs offline against them (frame_alignment_offline.py)")
    print("=" * 68)

    arm = None
    originals = {}
    caps = []
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

        if with_poses:
            countdown()
            for state in POSE_STATES:
                monitor.expect(arm.handle_id, DEV_JOINT)
                if robot.rm_movej(state_deg(ARM_SIDE, state),
                                  ARM_SPEED_PCT, 0, 0, 0) != 0:
                    print(f"    movej to {state} rejected — capturing "
                          "in place instead")
                else:
                    monitor.wait(arm.handle_id, DEV_JOINT, ARM_TIMEOUT_S)
                    time.sleep(0.3)          # settle before reading pose
                caps.append(_capture(robot, state))
        else:
            caps.append(_capture(robot, "as-found"))

        # ── checks ──
        frames_ok = all(isinstance(c.get("tool_frame"), dict)
                        or "unavailable" not in str(c.get("tool_frame"))
                        for c in caps)
        result("PASS" if frames_ok else "FAIL",
               "tool/work frame readable",
               "controller reports its active frames" if frames_ok else
               "frame getters failed — capture incomplete")

        # Controller pose vs offline rm_algo FK, in TWO tiers.
        #
        # The controller reports through its MOUNTING ANGLE (these arms sit
        # rotated ~90 deg about Y on the torso), so the raw vectors differ
        # by a rigid rotation even when the model is perfect — that is what
        # produced the bogus "868 mm mismatch" on 2026-08-08.
        #
        # Tier 1, always: |p|, the distance from the arm base. It is
        # INVARIANT under any base rotation or work frame, so it isolates
        # exactly one thing — is the ARM MODEL right? This is the tier that
        # caught the RM_B_E/ISF error (17.2 mm, F15).
        #
        # Tier 2, when the install pose is readable: the full vector.
        worst = worst_vec = None
        import math as _math
        for c in caps:
            if not (c["controller_pose"] and c["offline_fk"]):
                continue
            nc = _math.dist(c["controller_pose"][:3], (0, 0, 0))
            no = _math.dist(c["offline_fk"][:3], (0, 0, 0))
            err = abs(nc - no)
            worst = max(worst or 0.0, err)
            line = (f"    {c['label']:8s} |p| controller {nc * 1000:7.1f} mm "
                    f"vs rm_algo {no * 1000:7.1f} mm   d={err * 1000:5.2f} mm")
            if isinstance(c.get("install_pose"), list):
                vec = max(abs(a - b) for a, b in
                          zip(c["controller_pose"][:3], c["offline_fk"][:3]))
                worst_vec = max(worst_vec or 0.0, vec)
                line += f"   full-vector d={vec * 1000:.1f} mm"
            print(line)
        if worst_vec is None:
            print("    (install pose unreadable — orientation not checked; "
                  "the |p| tier still proves the arm model)")
        if worst is None:
            result("FAIL", "arm model matches the controller", "no data")
        elif worst <= FK_TOL_M:
            result("PASS", "arm model matches the controller",
                   f"|p| agrees within {worst * 1000:.2f} mm"
                   + (f", full vector within {worst_vec * 1000:.1f} mm"
                      if worst_vec is not None else " (reach only)"))
        else:
            result("FAIL", "arm model matches the controller",
                   f"|p| differs by {worst * 1000:.1f} mm — a base rotation "
                   "cannot cause this, so the MODEL is wrong. Check "
                   f"RM_FORCE_MODEL (currently "
                   f"{os.environ.get('RM_FORCE_MODEL', 'RM_MODEL_RM_ISF_E')}"
                   "): the force-sensor variant sets the wrist length, and "
                   "RM_MODEL_RM_B_E is 17.2 mm short on these arms")

        complete = all(c["joints_deg"] and c["controller_pose"] for c in caps)
        result("PASS" if complete else "FAIL", "capture complete",
               f"{len(caps)} C14CAP records for the offline comparison")

        # ── optional: recreate the URDF glove frames in the controller ──
        if not create_frames:
            result("SKIP", "glove tool frames created", "--create-frames "
                   "not given")
        else:
            result_tag = _create_glove_frames(robot)
            result(*result_tag)
        return 0 if _results["FAIL"] == 0 else 1
    finally:
        restore_run_modes(originals)
        teardown(arm)
        print(f"\n  Summary: {_results['PASS']} PASS, "
              f"{_results['FAIL']} FAIL, {_results['SKIP']} SKIP")


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    sys.exit(main())
