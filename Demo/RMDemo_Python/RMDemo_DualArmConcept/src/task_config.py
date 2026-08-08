"""Phase 2 — the task CONFIG is the specification, not the saved trajectory.

A cleaning task is defined by TWO YAML files (Newton, 2026-08-08):

    <category>/task_parameters_defaults.yaml   category-level defaults —
        frames, step size, velocity/acceleration/jerk scaling, pole scaling,
        and `arm_defaults.{left,right}` (planning groups, hand/pole group,
        default ik_frame)
    <category>/<fixture>/<task>_cleaning_points.yaml   the task itself —
        overrides those defaults, and adds `cartesian_poses`,
        `cleaning_points`, `cleaning_sequence`, `stage_sequence`

plus one shared file that defines what contact is legal:

    contact_links.yaml    contact_link_groups[ik_frame] x
                          contact_surface_objects[contact_surface]

Task values override category defaults; the arm is selected by the
`_left` / `_right` suffix on `primary_planning_group`.

WHY THIS MODULE EXISTS. The earlier approach subsampled the saved joint
trajectory to get "targets" for `rm_movel`, which is backwards: movel takes
CARTESIAN poses, and the Cartesian path is right here — `cleaning_sequence`
is an ordered list of straight strokes between named `cleaning_points`.
The saved JSON is Ruckig's time-parameterized IK interpolation OF those
strokes; reproducing it is not the goal, executing them is.

SERIALIZATION IS ALREADY THE DESIGN. Every entry in `stage_sequence` names
one `planning_group` — `pole_right`, `inspire_hand_right`, `rm75_arm_right`
— and stages are strictly ordered. The F9 constraint (never move pole and
arm together in the planned domain) is therefore the config's own
structure. `enforce_serialization` below turns that from a property into a
checked invariant.

Usage:
    cfg = TaskConfig.load("hinge_area_right", fixture="commode_c")
    cfg.side, cfg.ik_frame, cfg.arm_speed_pct, cfg.stage_sequence
    cfg.strokes()                # ordered Cartesian strokes for movel
"""

import pathlib

import yaml

from segment_verifier import WS

CONFIG_ROOT = WS / "cleaning_tasks" / "config"
CONTACT_LINKS = CONFIG_ROOT / "contact_links.yaml"

# Controller speed is a PERCENTAGE; the config carries 0..1 scalings.
# A scaling of 1.0 does not mean "100 % of the controller maximum" is safe
# to command blind — the controller's own TCP/joint limits (F10) already
# bound it, so the scaling is applied to a configured ceiling.
ARM_SPEED_CEILING_PCT = 20      # what every hardware gate has run at
POLE_SPEED_CEILING_PCT = 50


def _deep_merge(base: dict, over: dict) -> dict:
    """`over` wins, recursively — task values override category defaults."""
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _deep_merge(out[k], v)
        else:
            out[k] = v
    return out


class TaskConfig:
    """Merged view of the two config files plus the contact contract."""

    def __init__(self, task, fixture, category, defaults, task_doc, contact):
        self.task = task
        self.fixture = fixture
        self.category = category
        self._contact = contact or {}

        d_params = (defaults or {}).get("task_parameters", {})
        t_params = (task_doc or {}).get("task_parameters", {})
        merged = _deep_merge(d_params, t_params)

        # Which arm? The task may name its planning group; otherwise the
        # task NAME carries the side (toplid_left / hinge_area_right).
        group = merged.get("primary_planning_group", "")
        if group.endswith("_left") or task.endswith("_left"):
            self.side = "left"
        elif group.endswith("_right") or task.endswith("_right"):
            self.side = "right"
        else:
            raise SystemExit(f"cannot determine the arm for task {task!r}")

        # arm_defaults fill in whatever the task did not state
        arm_def = ((defaults or {}).get("arm_defaults", {})
                   .get(self.side, {}).get("task_parameters", {}))
        self.params = _deep_merge(arm_def, merged)

        self.doc = task_doc or {}
        self.stage_sequence = self.doc.get("stage_sequence", [])
        self.cleaning_points = self.doc.get("cleaning_points", {})
        self.cleaning_sequence = self.doc.get("cleaning_sequence", [])
        self.cartesian_poses = self.doc.get("cartesian_poses", {})

    # ── the parameters the dispatcher needs ──
    @property
    def ik_frame(self):
        return self.params["ik_frame"]

    @property
    def reference_frame(self):
        return self.params.get("reference_frame", "butterfli_ref_frame")

    @property
    def arm_group(self):
        return self.params.get("primary_planning_group")

    @property
    def hand_group(self):
        return self.params.get("hand_group")

    @property
    def pole_group(self):
        return self.params.get("pole_group")

    def _pct(self, key, ceiling, default=1.0):
        return max(1, min(100, int(round(
            float(self.params.get(key, default)) * ceiling))))

    @property
    def arm_speed_pct(self):
        return self._pct("max_velocity_scaling", ARM_SPEED_CEILING_PCT)

    @property
    def approach_speed_pct(self):
        """Approach to start_pose is deliberately slower (default 0.5)."""
        return self._pct("start_pose_velocity_scaling",
                         ARM_SPEED_CEILING_PCT, 0.5)

    @property
    def pole_speed_pct(self):
        return self._pct("pole_max_velocity_scaling", POLE_SPEED_CEILING_PCT)

    # ── the contact contract ──
    def contact_links(self):
        """Hand links permitted to touch the declared surface."""
        return list(self._contact.get("contact_link_groups", {})
                    .get(self.ik_frame, []))

    def contact_objects(self, surface):
        """Fixture objects a given `contact_surface` allows (config names)."""
        objs = self._contact.get("contact_surface_objects", {})
        return list(objs.get(surface, []))

    def contact_window(self):
        """(surface, {stage names}) that run under `allow_collision`.

        The sequence brackets the cleaning with allow_collision(surface)
        ... forbid_collision. Stages inside that bracket are CONTACT
        stages; everything outside must stay clear of the fixture.
        Returns (None, set()) when the task never allows contact.
        """
        surface, inside, stages = None, False, set()
        for st in self.active_stages():
            name = st.get("stage")
            if name == "allow_collision":
                surface = st.get("contact_surface", "all")
                inside = True
                continue
            if name == "forbid_collision":
                inside = False
                continue
            if inside and self.device_of(st) == "arm":
                stages.add(name)
        return surface, stages

    # ── stages ──
    def stage_enabled(self, stage) -> bool:
        """Resolve a stage's `conditional` / `condition_param` guard.

        toplid_* sets `use_prestart_stage: false` while still LISTING
        move_to_pre_start, so a runner that ignores the guard would drive
        the arm to a pose the task deliberately skips.
        """
        if not stage.get("conditional"):
            return True
        key = stage.get("condition_param")
        return bool(self.params.get(key, False)) if key else True

    def active_stages(self):
        """The stage sequence the task actually runs."""
        return [st for st in self.stage_sequence if self.stage_enabled(st)]

    # ── serialization: already true of the config, now CHECKED ──
    def device_of(self, stage):
        """arm | hand | pole | scene — from the stage's planning_group."""
        g = stage.get("planning_group", "")
        if g == self.pole_group:
            return "pole"
        if g == self.hand_group:
            return "hand"
        if g == self.arm_group:
            return "arm"
        return "arm" if stage.get("stage") == "execute_path" else "scene"

    def enforce_serialization(self, enabled=True):
        """Verify no stage drives two devices, and report the device order.

        F9 (vendor-confirmed) makes concurrent pole+arm motion unusable in
        the planned domain, and RealMan's own workaround is serialization.
        The task files already satisfy it — this turns that from a happy
        property into an invariant that fails loudly if a future task
        introduces a combined stage.

        Returns (ok, order, problems). With `enabled=False` the check still
        reports, but never blocks — the parameter Newton asked for.
        """
        order, problems = [], []
        for st in self.active_stages():
            dev = self.device_of(st)
            if dev != "scene":
                order.append((st.get("stage"), dev))
            groups = [v for k, v in st.items() if k.endswith("planning_group")]
            if len(set(groups)) > 1:
                problems.append(
                    f"stage {st.get('stage')!r} names {len(set(groups))} "
                    f"planning groups {sorted(set(groups))} — a single stage "
                    "must drive ONE device (F9)")
        return (enabled and not problems) or not enabled, order, problems

    # ── loading ──
    @classmethod
    def load(cls, task, fixture="commode_c", category="commode_cleaning"):
        cat = CONFIG_ROOT / category
        defaults = yaml.safe_load(
            (cat / "task_parameters_defaults.yaml").read_text())
        tpath = cat / fixture / f"{task}_cleaning_points.yaml"
        if not tpath.exists():
            raise SystemExit(f"no task config: {tpath}")
        task_doc = yaml.safe_load(tpath.read_text())
        contact = (yaml.safe_load(CONTACT_LINKS.read_text())
                   if CONTACT_LINKS.exists() else {})
        return cls(task, fixture, category, defaults, task_doc, contact)


def _describe(cfg):
    print(f"  task            {cfg.task}  [{cfg.side}]  fixture "
          f"{cfg.fixture}")
    print(f"  groups          arm={cfg.arm_group}  hand={cfg.hand_group}  "
          f"pole={cfg.pole_group}")
    print(f"  frames          ik={cfg.ik_frame}  ref={cfg.reference_frame}")
    print(f"  speeds          arm={cfg.arm_speed_pct}%  "
          f"approach={cfg.approach_speed_pct}%  pole={cfg.pole_speed_pct}%")
    print(f"  path            {len(cfg.cleaning_points)} points, "
          f"{len(cfg.cleaning_sequence)} strokes")
    ok, order, problems = cfg.enforce_serialization()
    print(f"  contact         {len(cfg.contact_links())} links may touch "
          f"the declared surface")
    print("  stage order     " + " -> ".join(f"{n}[{d}]" for n, d in order))
    print(f"  serialization   {'OK — no stage drives two devices' if ok else problems}")


if __name__ == "__main__":
    import sys
    tasks = sys.argv[1:] or ["hinge_area_right", "toplid_right",
                             "toplid_left"]
    for t in tasks:
        print("=" * 70)
        _describe(TaskConfig.load(t))
