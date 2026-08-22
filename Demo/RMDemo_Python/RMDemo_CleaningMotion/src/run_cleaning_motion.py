#!/usr/bin/env python3
"""Run one cleaning motion on ONE arm, to understand how the hardware
executes it. ARM ONLY — no pole stage, no hand stage, ever.

THE SEQUENCE IS FOUR STEPS (Newton, 2026-08-22):

    movej    -> rest
    movej_p  -> start pose
    cleaning path (the cleaning sequence)
    movej    -> rest

That is the whole motion. There is no prestart hop and no approach movel:
the arm reaches the start pose with `movej_p`, which interpolates in JOINT
space, so the tool does not travel a commanded straight line and cannot be
dragged through anything on the way.

CONNECT AND BLEND RADIUS ARE THE POINT OF THIS BED, so they are ordinary
arguments you set per run — `--connect`, `--blend` — not buried constants:

    r        blend radius, a PERCENTAGE 0-100 of the shorter adjoining
             segment (vendor: jiao rong ban jing bai fen bi), not mm
    connect  1 = the moves join into one continuous trajectory
             0 = every move is discrete and the arm rests at each waypoint

The last move of a chain always closes it with `r=0, connect=0`; a chain
whose final move still says connect=1 never closes, and the run hangs.

THE CONFIG this reads carries the reference frame, the start pose relative
to it, the cleaning points as deltas from the start pose, the cleaning
sequence, and the ik_frame. The tool frame comes from the frames lookup
table (FRAME_MAP.md: the URDF link name with `_frame` removed). The pole is
ASSUMED AT ITS MINIMUM and never commanded — it carries the arm base, so
its height is baked into the transform, which is why `--no-pole` is refused
rather than ignored.

THE MODE IS YOURS TO SET. `--mode SIM` simulates, `--mode REAL` runs on
metal, and this script never changes it behind you.

USAGE
    # offline — no arm, no network
    python3 run_cleaning_motion.py --motion task:toplid_left --dry-run
    python3 run_cleaning_motion.py --motion ../motions/<file> --dry-run
    python3 run_cleaning_motion.py --selftest --dry-run

    # on the arm
    python3 run_cleaning_motion.py --motion task:toplid_left --mode SIM
    python3 run_cleaning_motion.py --motion task:toplid_left --mode REAL \\
            --speed 0.25 --blend 10 --connect 1

    # the two controls worth running back to back
    python3 run_cleaning_motion.py --motion task:toplid_left --mode REAL \\
            --speed 0.25 --connect 0          # discrete: stop at every point
    python3 run_cleaning_motion.py --motion task:toplid_left --mode REAL \\
            --speed 0.25 --blend 25           # rounder corners

    python3 run_cleaning_motion.py --list

FLAGS
    --motion PATH|task:NAME   the cleaning motion to run
    --fixture NAME    fixture for `task:` lookups (default commode_c)
    --format NAME     force an adapter instead of sniffing the file
    --side left|right override the side the motion declares

  Every flag below is stated by the config's `motion:` block; these
  override it for a one-off run.

    --blend R         blend radius %, every mid-chain move
    --connect 0|1     chain the moves together
    --block 0|1       blocking SDK calls
    --primitive movel|moves   the cleaning primitive
    --cleaning-v PCT  v % commanded on each cleaning move
    --transit-v PCT   v % for the movej / movej_p transits
    --loops N         repeat movej_p + cleaning path N times

    --speed X         one line speed [m/s], off-ladder
    --speeds a,b,c    a ladder, run in order, stopping on the first failure
    --line-acc X      pin the linear acceleration
    --angular-speed X pin the angular cap
    --coupling K      omega_cap = K * v
    --allow-over-limit  dispatch a rung the J4 screen refused. E-stop in hand.

    --dry-run         offline only: load, convert, screen, print
    --selftest        run a PROVEN concept path instead of a motion file
    --list            list the motion files and exit
    --no-record       skip the UDP recording (on by default)
    --no-return       leave the arm at the last waypoint instead of resting
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cm_frames                                              # noqa: E402
import cm_loader                                              # noqa: E402
import cm_speed                                               # noqa: E402
from cm_arm import CleaningArmController, build_program       # noqa: E402
from cm_common import (                                       # noqa: E402
    handle_cli, parse_mode_arg, mode_label, assert_arm_only,
    apply_run_mode, restore_run_modes, report_run_modes,
    ArrivalMonitor, preflight_error_gate, countdown,
    host_ip_for, motions_dir, state_deg,
    POLE_M, LEFT_IP, RIGHT_IP, ROBOT_PORT, UDP_PORT,
    RunRecorder, speed_limits, error_codes,
)

USAGE = __doc__[__doc__.index("USAGE"):]


def _val(flag, default=None, cast=str):
    if flag in sys.argv:
        i = sys.argv.index(flag) + 1
        if i >= len(sys.argv):
            raise SystemExit("%s needs a value" % flag)
        try:
            return cast(sys.argv[i])
        except ValueError:
            raise SystemExit("%s: %r is not valid" % (flag, sys.argv[i]))
    return default


class Options:
    """How the motion is executed — the knobs this bed exists to vary.

    THE CONFIG STATES THESE; the CLI overrides for a one-off run. That
    order matters: a config is the reproducible record of a motion, so a
    value it declares must not be silently overridden by a default that
    merely happens to live in this file. `defaults` is the config's
    `motion:` block; anything absent from it falls back to the values here.
    """

    def __init__(self, argv, defaults=None):
        d = dict(defaults or {})

        def pick(flag, key, fallback, cast=int):
            if flag in argv:
                return _val(flag, fallback, cast)
            if d.get(key) is not None:
                return cast(d[key])
            return fallback

        self.blend = pick("--blend", "blend_r", 0)
        self.connect = pick("--connect", "connect", 1)
        self.block = pick("--block", "block", 0)
        self.primitive = pick("--primitive", "primitive", "movel", str)
        self.cleaning_v = pick("--cleaning-v", "v", 100)
        self.transit_v = pick("--transit-v", "transit_v", 20)
        self.loops = pick("--loops", "loops", 1)
        self.source = "config + CLI" if d else "CLI"
        self.return_to_rest = "--no-return" not in argv
        if self.loops < 1:
            raise SystemExit("--loops must be at least 1, got %d" % self.loops)
        if not 0 <= self.blend <= 100:
            raise SystemExit("--blend is a PERCENTAGE 0-100 (not mm), got %d"
                             % self.blend)
        if self.connect not in (0, 1):
            raise SystemExit("--connect must be 0 or 1, got %r" % self.connect)
        if self.block not in (0, 1):
            raise SystemExit("--block must be 0 or 1, got %r" % self.block)
        if self.primitive not in ("movel", "moves"):
            raise SystemExit("--primitive must be movel or moves, got %r"
                             % self.primitive)
        for label, pct in (("--cleaning-v", self.cleaning_v),
                           ("--transit-v", self.transit_v)):
            if not 1 <= pct <= 100:
                raise SystemExit("%s is a percentage 1-100, got %d"
                                 % (label, pct))
        if self.blend and not self.connect:
            print("  [NOTE] --blend %d with --connect 0: a discrete move "
                  "cannot blend into anything, so r is forced to 0 on every "
                  "move. That is the control, not a bug." % self.blend)

    def describe(self):
        return "\n".join([
            "execution (%s)" % self.source,
            "  loops      %d" % self.loops,
            "  primitive  rm_%s" % self.primitive,
            "  connect    %d (%s)" % (self.connect,
                                      "chained into one trajectory"
                                      if self.connect else
                                      "discrete, rest at every waypoint"),
            "  blend r    %d %% of the shorter adjoining segment" % self.blend,
            "  block      %d (%s)" % (self.block,
                                      "SDK blocks per move" if self.block
                                      else "non-blocking, arrival events"),
            "  v          cleaning %d %%, transit %d %%"
            % (self.cleaning_v, self.transit_v),
        ])


def list_motions() -> int:
    found = cm_loader.discover(motions_dir())
    print("motions/ — %s" % motions_dir())
    if not found:
        print("  (empty) — drop cleaning-motion files here, or run a config "
              "from the concept tree directly with --motion task:<name>.")
        return 0
    for f in found:
        try:
            p = cm_loader.load(f)
            print("  %-40s %s, %d points, %d segments"
                  % (f.name, p.side, len(p.poses), p.n_moves))
        except SystemExit as exc:
            print("  %-40s [UNREADABLE] %s" % (f.name, exc))
    return 0


def load_motion():
    if "--selftest" in sys.argv:
        program = cm_loader.selftest_program()
        print("  [SELFTEST] %s — a PROVEN path with real recordings behind "
              "it. Exercises the bed; it is not a cleaning motion."
              % cm_loader.SELFTEST_PATH)
        return program
    src = _val("--motion")
    if not src:
        raise SystemExit(
            "no motion given. Use --motion <file>, --motion task:<name>, "
            "--selftest, or --list.")
    if src.startswith("task:"):
        return cm_loader.load_task(src, fixture=_val("--fixture",
                                                     "commode_c"))
    return cm_loader.load(src, fmt=_val("--format"))


def entry_and_chain(program, poses):
    """(entry_pose, movel_chain, label) — ONE definition, used by both the
    offline report and the dispatch, so what is printed is what is sent.

    The `movej_p` target is the config's declared `start_pose`. It is not
    always one of the cleaning points: the generated configs give no point a
    zero delta, so start_pose sits a few cm off the first stroke and the
    move onto that stroke is a real movel. Where start_pose IS the first
    point (older configs, point1 at a zero delta) the duplicate is dropped
    rather than commanded as a zero-length move.
    """
    entry = program.meta.get("entry_pose")
    if entry is None:
        return poses[0], poses[1:], "first cleaning point"
    if math.dist(entry[:3], poses[0][:3]) < 0.001:
        return entry, poses[1:], "start_pose (== the first cleaning point)"
    return (entry, list(poses),
            "start_pose, %.1f mm before the first cleaning point"
            % (1000 * math.dist(entry[:3], poses[0][:3])))


def offline(program, poses, plan, opts, note):
    """Everything knowable without an arm. Returns (ok, per-rung records)."""
    print("=" * 72)
    print(program.describe())
    print(cm_frames.describe_conversion(program, poses, note))
    print()
    print(opts.describe())
    print()
    print(plan.describe())
    print(cm_speed.summarise(program, poses))

    entry, chain, entry_label = entry_and_chain(program, poses)
    print("  entry      movej_p -> %s" % entry_label)
    prog = build_program(len(chain), v=opts.cleaning_v, r=opts.blend,
                         connect=opts.connect, v_list=program.v_list,
                         r_list=program.r_list,
                         overrides=program.meta.get("segment_overrides"))
    jumps = program.meta.get("jumps") or []
    if jumps:
        print("  ⚠ the cleaning_sequence does not join at %d place(s) (%s). "
              "Those moves are\n    implied by the sequence, not written "
              "down, and are traversed as ordinary\n    straight lines."
              % (len(jumps), ", ".join(jumps)))
    print("  program    %d moves; first %s, last %s (the chain-close rule)"
          % (len(prog), prog[0], prog[-1]))
    print()

    results, ok_all = [], True
    for rung in plan.rungs:
        lim, notes = plan.limits_for(rung)
        print("-" * 72)
        if plan.keep_controller:
            print("rung %.3f m/s  ASSUMED for the screens only — nothing "
                  "will be written to the\n     controller, and its own "
                  "settings decide the real speed." % rung)
        else:
            print("rung %.3f m/s   line_acc %.2f   angular %.3f rad/s"
                  % (rung, lim["line_acc"], lim["angular_speed"]))
            for n in notes or []:
                print("    %s" % n)
        worst = cm_speed.screen_j4(poses, program.tool_frame, rung,
                                   v_list=program.v_list,
                                   angular_cap=lim["angular_speed"])
        ok, line = cm_speed.verdict_j4(worst[0] if worst else None,
                                       plan.allow_over)
        print(line)
        print(cm_speed.audit_junctions(program, poses, rung, lim["line_acc"]))
        results.append({"rung": rung, "limits": lim,
                        "j4_pct": None if not worst else round(worst[0], 1),
                        "j4_segment": None if not worst else worst[1],
                        "screen_ok": ok})
        ok_all = ok_all and ok
    print("-" * 72)
    return ok_all, results


def run_sequence(ctl, program, poses, opts):
    """movej rest -> movej_p start -> cleaning path -> movej rest.

    The return to rest is attempted even when the cleaning path failed —
    an arm left mid-path is worse than one parked, and the failure is
    already recorded by the time this runs.
    """
    rest = state_deg(program.side, "rest")

    print("  1. movej   -> rest")
    if not ctl.movej(rest, v=opts.transit_v, block=opts.block):
        return False, []

    entry, chain, entry_label = entry_and_chain(program, poses)

    ok, prog = True, []
    for loop in range(opts.loops):
        if opts.loops > 1:
            print("  --- loop %d/%d ---" % (loop + 1, opts.loops))
        print("  2. movej_p -> %s" % entry_label)
        if not ctl.movej_p(entry, v=opts.transit_v, block=opts.block):
            ok = False
            break
        print("  3. cleaning path (%d movel segments)" % len(chain))
        ok, prog = ctl.cleaning_path(
            [entry] + chain, v=opts.cleaning_v, r=opts.blend,
            connect=opts.connect, block=opts.block,
            v_list=program.v_list, r_list=program.r_list,
            overrides=program.meta.get("segment_overrides"),
            primitive=opts.primitive)
        if not ok:
            break

    if opts.return_to_rest:
        print("  4. movej   -> rest")
        ctl.movej(rest, v=opts.transit_v, block=opts.block)
    return ok, prog


def select_tool_frame(ctl, tool):
    """Select the motion's tool frame, returning the one to restore.

    The tool frame decides what the commanded poses MEAN — dispatching a
    glove-frame program with a different frame active moves the arm
    somewhere else entirely, with no error. So this refuses rather than
    guesses. It is also global controller state shared with the Web GUI.
    """
    ret, current = ctl.robot.rm_get_current_tool_frame()
    original = current.get("name") if ret == 0 and isinstance(current, dict) \
        else None
    if original == tool:
        print("  tool frame %s already active" % tool)
        return original
    r = ctl.robot.rm_change_tool_frame(tool)
    if r != 0:
        raise SystemExit(
            "cannot select tool frame %r (ret=%s: %s).\n"
            "  The frame must exist on the controller — create it with "
            "RMDemo_DualArmConcept/src/test_frame_alignment.py --mode REAL "
            "--create-frames. Refusing to dispatch through the wrong frame."
            % (tool, r, error_codes.describe_api2_return(r)))
    print("  tool frame %s selected (was %s)" % (tool, original))
    return original


def main() -> int:
    handle_cli(__doc__,
               extra_flags=("--dry-run", "--selftest", "--list",
                            "--allow-over-limit", "--no-record",
                            "--no-return"),
               value_flags=("--motion", "--fixture", "--format", "--side",
                            "--blend", "--connect", "--block", "--primitive",
                            "--cleaning-v", "--transit-v", "--loops",
                            "--speed", "--speeds", "--line-acc",
                            "--angular-speed", "--coupling"),
               usage=USAGE, allow_common=True)
    # allow_common=True is what makes `--mode` work; `assert_arm_only`
    # then refuses --no-pole / --no-hands with a message that says WHY
    # they are meaningless here.
    assert_arm_only()

    if "--list" in sys.argv:
        return list_motions()

    program = load_motion()
    if "--side" in sys.argv:
        program.side = _val("--side")
        program.validate()

    poses, note = cm_frames.to_arm_base(program, pole_m=POLE_M)

    # THE CONFIG IS THE SOURCE OF THE PARAMETERS; the CLI overrides for a
    # one-off. A config that states line_acc or the angular cap has them
    # applied, rather than having this file's derivation quietly win.
    cfg_motion = program.meta.get("motion") or {}
    plan = cm_speed.resolve(sys.argv, program)
    if "--line-acc" not in sys.argv and cfg_motion.get("line_acc"):
        plan.line_acc_pinned = float(cfg_motion["line_acc"])
    if "--angular-speed" not in sys.argv and cfg_motion.get("angular_speed"):
        plan.angular_pinned = float(cfg_motion["angular_speed"])
    opts = Options(sys.argv, cfg_motion)

    screens_ok, per_rung = offline(program, poses, plan, opts, note)
    if "--dry-run" in sys.argv:
        print("dry run — nothing dispatched.")
        return 0 if screens_ok else 1
    if not screens_ok and "--allow-over-limit" not in sys.argv:
        print("REFUSED offline: a rung exceeds the J4 abort even at the most "
              "optimistic reading.\n  Lower the speed, or override with "
              "--allow-over-limit and an E-stop in hand.")
        return 1

    # ── the arm ─────────────────────────────────────────────────────────
    mode = parse_mode_arg(sys.argv)
    ip = LEFT_IP if program.side == "left" else RIGHT_IP
    ctl = CleaningArmController(ip, ROBOT_PORT, side=program.side)
    if not ctl.connected:
        return 0
    monitor = ArrivalMonitor()
    monitor.register(ctl.robot)
    ctl.attach_monitor(monitor)

    originals, limits_before, original_tool, rc = {}, None, None, 0
    try:
        originals = apply_run_mode(mode, ctl.arm)
        if originals is None:
            print("  [FAIL] requested mode did not engage — aborting before "
                  "any motion.")
            return 1
        report_run_modes(ctl.arm)
        ok_err, detail = preflight_error_gate(
            ctl.arm, clear="--clear-errors" in sys.argv)
        if not ok_err:
            print("  [FAIL] latched controller errors: %s" % detail)
            return 1

        # The mount angle is the controller's to state. Ask it now there is
        # a connection, and re-resolve if the poses came from a non-base
        # frame (a base-frame program is unaffected by the mount).
        res = cm_frames.FrameResolver(program.side, pole_m=POLE_M)
        if res.adopt_mount_from_controller(ctl.robot) is not None \
                and program.source_frame.lower() \
                not in cm_frames.ARM_BASE_ALIASES:
            poses, note = cm_frames.to_arm_base(program, pole_m=POLE_M,
                                                robot=ctl.robot)
            print("  re-resolved against the controller's install pose")

        original_tool = select_tool_frame(ctl, program.tool_frame)
        countdown()

        for i, rung in enumerate(plan.rungs):
            print("\n=== rung %d/%d — connect %d, r %d %% ==="
                  % (i + 1, len(plan.rungs), opts.connect, opts.blend))
            # Only what someone actually asked for is written. An empty set
            # means the arm keeps its controller's own settings, which is
            # the default (Newton, 2026-08-22) — and NOT writing the TCP
            # limits is materially different from writing back what they
            # already are, because they ratchet and are shared with the GUI.
            to_set = plan.limits_to_apply(rung)
            if to_set:
                limits_before = limits_before or speed_limits.read(ctl.robot)
                speed_limits.apply(ctl.robot, allow_raise=False, **to_set)
                print("  limits SET: %s" % to_set)
            else:
                print("  limits UNCHANGED — using the controller's own "
                      "settings")
            in_force = speed_limits.read(ctl.robot)
            print("  in force: line_speed %.3f m/s, line_acc %.2f, "
                  "angular %.3f rad/s, angular_acc %.2f"
                  % (in_force.get("line_speed", float("nan")),
                     in_force.get("line_acc", float("nan")),
                     in_force.get("angular_speed", float("nan")),
                     in_force.get("angular_acc", float("nan"))))

            rec = None
            if "--no-record" not in sys.argv:
                rec = RunRecorder(ctl.robot,
                                  "%s_v%03d_r%02d_c%d"
                                  % (program.name, round(rung * 1000),
                                     opts.blend, opts.connect),
                                  program.side, host_ip_for(ip), UDP_PORT)
                if not rec.start():
                    print("  [WARN] recorder did not start; this rung is not "
                          "introspectable afterwards")
                    rec = None

            t0 = time.perf_counter()
            ok, prog = run_sequence(ctl, program, poses, opts)
            dt = time.perf_counter() - t0

            if rec is not None:
                rec.meta.update({
                    "mode": mode_label(mode if mode is not None else -1),
                    "sim": (mode == 0),
                    "bed": "RMDemo_CleaningMotion (arm only)",
                    "sequence": ["movej rest", "movej_p start",
                                 "cleaning path", "movej rest"],
                    "motion": {
                        "name": program.name,
                        "source_path": program.source_path,
                        "source_format": program.source_format,
                        "source_frame": program.source_frame,
                        "frame_note": note,
                        "pole_m": POLE_M,
                        "pole_commanded": False,
                        "hand_commanded": False,
                        "meta": program.meta,
                    },
                    "execution": {
                        "primitive": opts.primitive,
                        "connect": opts.connect,
                        "blend_r_pct": opts.blend,
                        "block": opts.block,
                        "cleaning_v_pct": opts.cleaning_v,
                        "transit_v_pct": opts.transit_v,
                    },
                    "limits_in_force": speed_limits.read(ctl.robot),
                    "ladder": {"rung": i + 1, "of": len(plan.rungs),
                               "line_speed": rung,
                               "all_rungs": plan.rungs,
                               "screen": per_rung[i]},
                    "commanded": {
                        "tool_frame": program.tool_frame,
                        "num_waypoints": len(poses),
                        "segments": len(poses) - 1,
                        "waypoint_names": program.waypoint_names,
                        "poses": poses,
                        "program": [list(t) for t in prog],
                        "r_list": program.r_list,
                        "v_list": program.v_list,
                    },
                    "elapsed_s": round(dt, 3),
                    "ok": bool(ok),
                })
                print("  recorded -> %s" % rec.stop())

            print("  rung %s in %.2f s" % ("OK" if ok else "FAILED", dt))
            if not ok:
                rc = 1
                print("  stopping the ladder: a rung failed.")
                break
    finally:
        if original_tool and original_tool != program.tool_frame:
            r = ctl.robot.rm_change_tool_frame(original_tool)
            print("  tool frame restored to %s (ret=%s)" % (original_tool, r))
        restore_run_modes(originals)
        if limits_before:
            speed_limits.restore(ctl.robot, limits_before)
            print("  limits restored to %s" % limits_before)
        ctl.disconnect()
    return rc


if __name__ == "__main__":
    from log_utils import setup_log
    setup_log(__file__)
    raise SystemExit(main())
