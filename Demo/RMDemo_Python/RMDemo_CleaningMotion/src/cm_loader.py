"""Reading a cleaning-motion file into a MotionProgram. THIS IS THE SEAM.

Newton is supplying the trajectory format separately (2026-08-22). Rather
than guess it and bake a guess through the whole pipeline, everything
downstream — frames, speed, screens, dispatch, recording — consumes ONE
in-memory type, `MotionProgram`, and knows nothing about files. Teaching
this bed a new format is therefore exactly one function:

    @register("newtons_format")
    def _load_newtons_format(path, text) -> MotionProgram: ...

and nothing else changes. The adapters that already exist are proof the
seam holds, not a prediction about what the format will be.

WHAT AN ADAPTER MUST RETURN, precisely — get these wrong and the arm moves
somewhere unintended, so each is validated rather than trusted:

    poses         [[x, y, z, rx, ry, rz]] — METRES and RADIANS, in
                  `source_frame`. Not mm, not degrees. The controller takes
                  metres/radians and a program in mm is 1000x out.
    source_frame  "arm_base" if the poses are ALREADY what the controller
                  takes; otherwise the URDF frame they are expressed in
                  (e.g. "commode_c_link"), which `cm_frames` resolves at
                  the minimum pole height.
    side          "left" or "right" — decides which arm and which base link.
    tool_frame    the CONTROLLER tool-frame name, <= 11 characters
                  (`c_char_Array_12`). The URDF name minus the `_frame`
                  token, per FRAME_MAP.md. `L_glove_frame_4` -> `L_glove_4`.

`v_list` / `r_list` are per-MOVE, so their length is len(poses) - 1, and
move i governs the junction at poses[i+1] — `rm_movel`'s r blends at the
vertex the move ENDS on. Off-by-one here silently shifts every blend by one
corner, which is invisible in a plot and obvious only in the cut sizes.
"""

import ast
import dataclasses
import json
import math
import pathlib

MAX_TOOL_FRAME_CHARS = 11        # c_char_Array_12, one byte for the NUL


@dataclasses.dataclass
class MotionProgram:
    """One cleaning motion, format-agnostic. The unit of work in this bed."""

    name: str
    side: str
    tool_frame: str
    source_frame: str
    poses: list                          # [[x,y,z,rx,ry,rz]] m + rad
    waypoint_names: list = dataclasses.field(default_factory=list)
    v_list: list = None                  # per-move v %, len = n_moves
    r_list: list = None                  # per-move blend r %, len = n_moves
    speed_ladder: list = None            # default rungs [m/s], if declared
    source_path: str = ""
    source_format: str = ""
    meta: dict = dataclasses.field(default_factory=dict)

    # ── derived ──
    @property
    def n_moves(self) -> int:
        return max(0, len(self.poses) - 1)

    def segment_lengths_m(self) -> list:
        return [math.dist(self.poses[i][:3], self.poses[i + 1][:3])
                for i in range(self.n_moves)]

    def describe(self) -> str:
        L = self.segment_lengths_m()
        out = ["motion    %s" % self.name,
               "  source  %s  [%s]" % (self.source_path or "(synthetic)",
                                       self.source_format or "?"),
               "  side    %s      tool frame %s" % (self.side, self.tool_frame),
               "  frame   %s" % self.source_frame,
               "  points  %d  (%d movel segments)" % (len(self.poses),
                                                      self.n_moves)]
        if L:
            out.append("  lengths %.1f mm min / %.1f mm median / %.1f mm max, "
                       "%.0f mm total"
                       % (1000 * min(L), 1000 * sorted(L)[len(L) // 2],
                          1000 * max(L), 1000 * sum(L)))
        if self.v_list:
            out.append("  v_list  %s" % _compact(self.v_list))
        if self.r_list:
            out.append("  r_list  %s" % _compact(self.r_list))
        if self.speed_ladder:
            out.append("  ladder  %s m/s" % ", ".join(
                "%.2f" % v for v in self.speed_ladder))
        return "\n".join(out)

    def validate(self):
        """Refuse anything that would move the arm somewhere unintended.

        Every check here has a failure mode that is silent at dispatch: a
        program in millimetres is accepted by `rm_movel` and lands 1000x
        away; a tool-frame name one character too long is TRUNCATED by the
        controller and quietly selects a different frame; a per-move list
        of the wrong length shifts every blend by one corner.
        """
        p = "motion %r: " % self.name
        if self.side not in ("left", "right"):
            raise SystemExit(p + "side must be 'left' or 'right', got %r"
                             % self.side)
        if len(self.poses) < 2:
            raise SystemExit(p + "needs at least 2 waypoints, got %d"
                             % len(self.poses))
        for i, q in enumerate(self.poses):
            if len(q) != 6:
                raise SystemExit(p + "waypoint %d has %d values, expected 6 "
                                 "[x y z rx ry rz]" % (i, len(q)))
            if not all(isinstance(v, (int, float)) and math.isfinite(v)
                       for v in q):
                raise SystemExit(p + "waypoint %d has a non-finite value: %r"
                                 % (i, q))
        if not self.tool_frame:
            raise SystemExit(p + "no tool_frame — the controller would use "
                             "whatever frame happens to be active, and the "
                             "poses mean nothing without it")
        if len(self.tool_frame) > MAX_TOOL_FRAME_CHARS:
            raise SystemExit(
                p + "tool_frame %r is %d characters; the controller field "
                "holds %d and TRUNCATES silently, which would select a "
                "different frame. Use the URDF name with '_frame' removed "
                "(FRAME_MAP.md)."
                % (self.tool_frame, len(self.tool_frame),
                   MAX_TOOL_FRAME_CHARS))
        for label, seq in (("v_list", self.v_list), ("r_list", self.r_list)):
            if seq is not None and len(seq) != self.n_moves:
                raise SystemExit(
                    p + "%s has %d entries but there are %d movel segments. "
                    "These are per-MOVE, not per-waypoint; a mismatch shifts "
                    "every blend by one corner."
                    % (label, len(seq), self.n_moves))
        if self.waypoint_names and len(self.waypoint_names) != len(self.poses):
            raise SystemExit(p + "waypoint_names has %d entries for %d poses"
                             % (len(self.waypoint_names), len(self.poses)))
        self._sanity_units()
        return self

    def _sanity_units(self):
        """Catch millimetres-as-metres and degrees-as-radians before dispatch.

        Advisory, not fatal — a legitimate motion could sit outside these
        bounds — but silence here is how a 1000x program reaches the arm.
        """
        xyz = [q[:3] for q in self.poses]
        far = max(max(abs(v) for v in q) for q in xyz)
        rot = max(max(abs(v) for v in q[3:]) for q in self.poses)
        if far > 5.0:
            print("  [WARN] largest |coordinate| is %.1f. If this program is "
                  "in MILLIMETRES it must be divided by 1000 before "
                  "dispatch — the controller takes metres." % far)
        if rot > 2 * math.pi + 1e-6:
            print("  [WARN] largest |rotation| is %.2f, above 2*pi. If these "
                  "are DEGREES they must be converted — the controller takes "
                  "radians." % rot)


def _compact(seq):
    """'[100 x 12, 50, 100 x 3]' — long uniform lists are noise otherwise."""
    out, run, prev = [], 0, object()
    for v in list(seq) + [object()]:
        if v == prev:
            run += 1
            continue
        if run:
            out.append("%s x %d" % (prev, run) if run > 1 else "%s" % prev)
        prev, run = v, 1
    return "[%s]" % ", ".join(out)


# ── adapter registry ────────────────────────────────────────────────────
ADAPTERS = {}


def register(name):
    def deco(fn):
        ADAPTERS[name] = fn
        return fn
    return deco


@register("json")
def _load_json(path, text):
    """A JSON object carrying the waypoints. PROVISIONAL until the real
    format lands — it accepts a superset of the obvious spellings so that a
    hand-written file works today, and it is the template for the adapter
    Newton's format will need.

    Recognised keys (first spelling found wins):
        poses | waypoints | points        required
        side | arm                        required
        tool_frame | ik_frame | tool      required
        frame | source_frame | reference_frame   default "arm_base"
        v_list, r_list, speed_ladder, names, units, angle_units   optional

    `units: "mm"` and `angle_units: "deg"` are converted here, so the
    MotionProgram invariant (metres, radians) holds no matter what the file
    was written in.
    """
    obj = json.loads(text)
    if not isinstance(obj, dict):
        raise SystemExit("%s: top level must be a JSON object" % path)

    def pick(*keys, default=None, required=False):
        for k in keys:
            if k in obj:
                return obj[k]
        if required:
            raise SystemExit(
                "%s: none of %s present. This adapter is provisional — if "
                "the real format spells it differently, add the spelling to "
                "cm_loader._load_json or write a dedicated adapter."
                % (path, " / ".join(keys)))
        return default

    raw = pick("poses", "waypoints", "points", required=True)
    names = pick("names", "waypoint_names", default=[]) or []
    scale = 0.001 if str(pick("units", default="m")).lower() in ("mm",
                                                                "millimetre",
                                                                "millimeter") \
        else 1.0
    ang = math.pi / 180.0 if str(
        pick("angle_units", "rot_units", default="rad")).lower() in (
            "deg", "degree", "degrees") else 1.0

    poses, dict_names = [], []
    for i, q in enumerate(raw):
        if isinstance(q, dict):                      # {"xyz": [...], "rpy": [...]}
            xyz = q.get("xyz") or q.get("position") or q.get("p")
            rpy = q.get("rpy") or q.get("orientation") or q.get("r")
            if xyz is None or rpy is None:
                raise SystemExit("%s: waypoint %d needs xyz and rpy" % (path, i))
            dict_names.append(str(q.get("name") or "p%02d" % i))
            q = list(xyz) + list(rpy)
        if len(q) != 6:
            raise SystemExit("%s: waypoint %d has %d values, expected 6"
                             % (path, i, len(q)))
        poses.append([float(q[0]) * scale, float(q[1]) * scale,
                      float(q[2]) * scale,
                      float(q[3]) * ang, float(q[4]) * ang,
                      float(q[5]) * ang])

    return MotionProgram(
        name=str(pick("name", default=pathlib.Path(path).stem)),
        side=str(pick("side", "arm", required=True)).lower(),
        tool_frame=_controller_frame(str(pick("tool_frame", "ik_frame",
                                              "tool", required=True))),
        source_frame=str(pick("frame", "source_frame", "reference_frame",
                              default="arm_base")),
        poses=poses,
        waypoint_names=[str(n) for n in (names or dict_names)][:len(poses)],
        v_list=pick("v_list", "speeds"),
        r_list=pick("r_list", "blends"),
        speed_ladder=pick("speed_ladder", "ladder"),
        source_path=str(path),
        source_format="json",
        meta={k: v for k, v in obj.items()
              if k not in ("poses", "waypoints", "points")},
    )


@register("concept_path")
def _load_concept_path(path, text):
    """A `paths/*.py` module from RMDemo_DualArmConcept.

    Parsed with `ast`, never imported — a path file is data, and importing
    it would run whatever is in it. Poses there are POSES_MM: millimetres,
    already in the ARM BASE frame, with orientation carried per waypoint.

    This adapter is not a guess: it lets the whole pipeline be exercised
    today against paths that have real recordings behind them, which is how
    the bed gets tested before the first cleaning motion arrives.
    """
    tree = ast.parse(text, filename=str(path))
    ns = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            try:
                ns[node.targets[0].id] = ast.literal_eval(node.value)
            except (ValueError, SyntaxError):
                pass                       # computed — not data we can read
    if "POSES_MM" not in ns:
        raise SystemExit("%s: no POSES_MM — not a concept path module" % path)

    pm = ns["POSES_MM"]
    names = list(pm)
    poses = []
    for n in names:
        q = list(pm[n])
        if len(q) != 6:
            raise SystemExit("%s: waypoint %r has %d values, expected 6"
                             % (path, n, len(q)))
        poses.append([q[0] / 1000.0, q[1] / 1000.0, q[2] / 1000.0,
                      float(q[3]), float(q[4]), float(q[5])])

    tool = str(ns.get("TOOL_FRAME") or "")
    side = "left" if tool.startswith("L") else "right"
    return MotionProgram(
        name=str(ns.get("NAME") or pathlib.Path(path).stem),
        side=side,
        tool_frame=tool,
        source_frame="arm_base",           # concept paths are already base-frame
        poses=poses,
        waypoint_names=names,
        v_list=ns.get("V_LIST"),
        r_list=ns.get("R_LIST"),
        speed_ladder=ns.get("SPEED_LADDER"),
        source_path=str(path),
        source_format="concept_path",
        meta={k: v for k, v in ns.items()
              if k in ("ARC_LIST", "VIA_MM", "BLEND_SWEEP", "CHAIN_APPROACH")},
    )


@register("cleaning_config")
def _load_cleaning_config(path, text):
    """A self-contained cleaning config: frames, start pose, deltas,
    sequence and every motion parameter, in one file.

    All the resolution lives in `cm_config.CleaningConfig` — the start-pose
    format branching on the reference frame, the deltas, the transform into
    the arm's frame at the pinned minimum pole height. This adapter only
    turns the result into a MotionProgram.
    """
    from cm_config import CleaningConfig
    cfg = CleaningConfig.load(path)
    poses, names, note = cfg.poses_arm_base()
    motion = cfg.motion()

    return MotionProgram(
        name=cfg.name,
        side=cfg.side,
        tool_frame=cfg.tool_frame,
        # Already resolved into the arm's frame by `poses_arm_base`, so the
        # downstream identity pass-through is correct and says so.
        source_frame="arm_base",
        poses=poses,
        waypoint_names=names,
        # A ladder ONLY if the config actually states a line speed. When it
        # does not — which the generated configs do not — this stays None
        # and the arm keeps the speed its controller already holds.
        speed_ladder=([float(motion["line_speed"])]
                      if motion.get("line_speed") is not None else None),
        source_path=str(path),
        source_format="cleaning_config",
        meta={"config": cfg,
              "motion": motion,
              # The declared start_pose, transformed — the movej_p target.
              "entry_pose": list(cfg.entry_pose),
              "reference_frame": cfg.reference_frame,
              "ik_frame": cfg.ik_frame,
              "start_pose_form": cfg.start_pose_form,
              "frame_note": note,
              "segment_overrides": cfg.segment_overrides(),
              "jumps": [n for n, j in cfg.traversal() if j],
              "pole_m": POLE_M_FOR_META()},
    )


def POLE_M_FOR_META():
    from cm_common import POLE_M
    return POLE_M


@register("task_yaml")
def _load_task_yaml(path, text):
    """A cleaning-points YAML: ref frame + start pose + deltas + sequence.

    This is the format the cleaning configs are written in, and it is NOT
    re-parsed here. `TaskConfig` + `CleaningPath` in the concept tree are
    the faithful port of `TaskBase::resolveCleaningWaypoints` — verified
    against all three saved plans to a mean of 0.3-0.4 mm — so this adapter
    delegates to them and pins the pole.

    Three details that resolver gets right and a fresh parser would not:

      * translations are start_pose-origin deltas in the REFERENCE FRAME's
        axes, NOT rotated into the start pose's own frame;
      * rotations compose the other way round: R_final = R_delta * R_start,
        with R_delta = Rx(roll) Ry(pitch) Rz(yaw) in DEGREES;
      * `start_pose` is a QUATERNION, and the files carry both `xyzw` and
        `wxyz` orderings across live and commented lines.

    The waypoints are `cleaning_sequence[0].first` then every segment's
    `.second` — a chained polyline, asserted chained, because a break would
    silently drop a stroke.

    THE POLE IS PINNED AT ITS MINIMUM here, which is the whole premise of
    this bed: the pole carries the arm base, so `movel_program(pole_m)`
    resolves every waypoint against that height. The returned poses are
    therefore already in the ARM BASE frame — `source_frame` says so, and
    `cm_frames` passes them through as an explicit identity.
    """
    from cm_common import POLE_M
    task, fixture = _task_and_fixture(path)
    try:
        from task_config import TaskConfig
        from cleaning_path import CleaningPath
    except Exception as exc:                                  # noqa: BLE001
        raise SystemExit(
            "cannot import the task-config resolver (%r).\n"
            "  This format is resolved by RMDemo_DualArmConcept's "
            "TaskConfig + CleaningPath rather than re-parsed here." % exc)

    cfg = TaskConfig.load(task, fixture=fixture)
    prog = CleaningPath(cfg).movel_program(pole_m=POLE_M)

    return MotionProgram(
        name=task,
        side=cfg.side,
        # The frames lookup table decides this, not a guess: the controller
        # name is the URDF link with the `_frame` token removed, and every
        # name fits the 11-character field (FRAME_MAP.md).
        tool_frame=prog["tool_frame"],
        source_frame="arm_base",
        poses=[list(q) for q in prog["poses"]],
        waypoint_names=list(prog["waypoint_names"]),
        source_path=str(path),
        source_format="task_yaml",
        meta={"fixture": fixture,
              "ik_frame": cfg.ik_frame,
              "reference_frame": cfg.reference_frame,
              "pole_m": POLE_M,
              "resolved_by": "TaskConfig + CleaningPath.movel_program",
              "segments": prog["segments"]},
    )


def _task_and_fixture(path):
    """(task, fixture) from a cleaning-points path.

    `.../task_configs/commode_c/toplid_left_cleaning_points.yaml`
        -> ("toplid_left", "commode_c")
    """
    p = pathlib.Path(path)
    task = p.stem
    for suffix in ("_cleaning_points", "_points"):
        if task.endswith(suffix):
            task = task[: -len(suffix)]
            break
    fixture = p.parent.name or "commode_c"
    return task, fixture


def load_task(spec, fixture="commode_c") -> MotionProgram:
    """Load by NAME — `task:toplid_left` — rather than by file path."""
    name = spec.split(":", 1)[1] if spec.startswith("task:") else spec
    from cm_common import concept_path
    cand = concept_path("task_configs", fixture,
                        "%s_cleaning_points.yaml" % name)
    if not cand.is_file():
        alt = concept_path("task_configs", fixture, "%s.yaml" % name)
        if not alt.is_file():
            raise SystemExit("no task config for %r under fixture %r; tried:"
                             "\n  %s\n  %s" % (name, fixture, cand, alt))
        cand = alt
    return _load_task_yaml(cand, "").validate()


def _controller_frame(name: str) -> str:
    """URDF link name -> controller tool-frame name.

    The one mechanical rule from FRAME_MAP.md: drop the `_frame` token.
    Reversible, collision-free, and every name fits the 11-character field.
    Truncating instead would map all four glove frames onto one name.
    Delegates to the concept tree when it is importable, so there is a
    single implementation of the rule.
    """
    try:
        from frame_alignment_offline import controller_frame_name
        return controller_frame_name(name)
    except Exception:
        return name.replace("_frame", "")


def sniff(path) -> str:
    """Which adapter reads this file. Extension first, then content."""
    p = pathlib.Path(path)
    if p.suffix == ".py":
        return "concept_path"
    if p.suffix in (".json", ".jsn"):
        return "json"
    if p.suffix in (".yaml", ".yml"):
        # Two YAML shapes exist. A task_configs file is marked by its
        # `task_parameters:` block and is resolved by name against that
        # directory layout; anything else carrying cleaning points is a
        # self-contained cleaning config.
        #
        # Discriminating on `task_parameters` rather than on the presence
        # of `reference_frame` matters for the ERROR: a self-contained
        # config that forgot its reference_frame must be told that, not
        # sent down the task_configs path to fail with a list of task
        # directories it was never in.
        # The discriminator is whether the file STATES ITS OWN reference
        # frame — at the top level or inside `task_parameters`, which is
        # where the generated configs put it. A file that does is
        # self-describing and is read directly; one that does not needs the
        # task_configs directory layout (its reference frame lives in the
        # defaults file beside it) and goes through `task_yaml`.
        try:
            import yaml
            head = yaml.safe_load(p.read_text()) or {}
            if isinstance(head, dict):
                tp = head.get("task_parameters") or {}
                if (head.get("reference_frame") or head.get("ref_frame")
                        or tp.get("reference_frame")):
                    return "cleaning_config"
                # A file with cleaning points but NO anchor and NO frame is
                # the other format: RMDemo_PointSequence's absolute world
                # poses. The two look almost identical and mean opposite
                # things, so say which tool owns it rather than failing in
                # the task-config lookup with a list of directories it was
                # never in.
                if (head.get("cleaning_points") or head.get("points")) \
                        and not head.get("cartesian_poses") \
                        and not head.get("start_pose") and not tp:
                    raise SystemExit(
                        "%s looks like an ABSOLUTE points file "
                        "(RMDemo_PointSequence), not a cleaning config: it "
                        "declares cleaning points but no `reference_frame` "
                        "and no `start_pose` to measure them from.\n"
                        "  Here a translation is a DELTA from the start "
                        "pose; there it is an absolute world pose. Run it "
                        "with the tool that owns that format:\n"
                        "    cd ../../RMDemo_PointSequence/src\n"
                        "    python3 point_sequence.py --points %s --dry-run"
                        % (p, p))
        except SystemExit:
            raise
        except Exception:                                     # noqa: BLE001
            pass
        return "task_yaml"
    head = p.read_text(errors="replace")[:400].lstrip()
    if head.startswith("{"):
        return "json"
    raise SystemExit(
        "cannot tell what format %s is.\n"
        "  known: %s\n"
        "  Pass --format <name>, or add an adapter — cm_loader.register()."
        % (p, ", ".join(sorted(ADAPTERS))))


def load(path, fmt=None) -> MotionProgram:
    """Read one cleaning-motion file. Always validated before it is returned."""
    p = pathlib.Path(path)
    if not p.is_file():
        raise SystemExit("no such motion file: %s" % p)
    fmt = fmt or sniff(p)
    if fmt not in ADAPTERS:
        raise SystemExit("unknown format %r — known: %s"
                         % (fmt, ", ".join(sorted(ADAPTERS))))
    return ADAPTERS[fmt](p, p.read_text()).validate()


def discover(folder) -> list:
    """Every motion file in `motions/`, sorted. Skips READMEs and dotfiles."""
    d = pathlib.Path(folder)
    if not d.is_dir():
        return []
    return sorted(f for f in d.iterdir()
                  if f.is_file() and not f.name.startswith(".")
                  and f.suffix.lower() in (".json", ".jsn", ".py",
                                           ".yaml", ".yml")
                  and f.name.lower() != "readme.md")


SELFTEST_PATH = "paths/blend_corner_001.py"


def selftest_program() -> MotionProgram:
    """A PROVEN path, for exercising the bed before any motion file exists.

    Deliberately NOT a synthetic one. An invented square is easy to write
    and worthless: the first attempt here used a plausible-looking in-box
    rectangle with a made-up orientation and screened at 252 % of the J4
    limit, i.e. it was unrunnable and said nothing about the plumbing. A
    path with real recordings behind it screens at 65 % and exercises the
    `concept_path` adapter at the same time.

    `blend_corner_001` is the concept tree's blend reference: 65 mm
    segments, CONSTANT orientation (so nothing angular-throttles and a
    speed dip means "corner"), inside the hardware-proven box. It is a
    PLUMBING test, not a cleaning motion — it proves loader -> frames ->
    screens -> dispatch -> recording works end to end.
    """
    from cm_common import concept_path
    p = concept_path(SELFTEST_PATH)
    if not p.is_file():
        raise SystemExit(
            "selftest path missing: %s\nExpected it in the concept tree; "
            "without it there is no proven path to exercise the bed with." % p)
    prog = load(p)
    prog.name = "selftest_" + prog.name
    prog.meta["purpose"] = "plumbing selftest (proven path, real recordings)"
    return prog
