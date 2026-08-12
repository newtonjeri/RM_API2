# Running and introspecting a new cleaning task

*2026-08-12. All 24 commode_c cleaning tasks now run through `stage_runner`
and through every introspection tool, the same way `toplid` and
`hinge_area` always have.*

---

## What was actually missing

Only one thing: **the saved plan.** `stage_runner` resolves it through
`segment_verifier.resolve_plan`, which reads `../plans/` and **nowhere
else** — no search, no fallback. That is deliberate (see the docstring): on
2026-08-08 the lab machine's ROS workspace held a *different* plan under the
same filename, so one machine rehearsed a plan the other had never verified,
and both runs looked fine.

`plans/` held 4 files. The other 20 were sitting in `butterfli_ws` and
`stage_runner` would not look at them, so a new task died on a bare
`FileNotFoundError`.

**Fixed two ways.** The 20 plans are now bundled — verified byte-identical
to `butterfli_ws/.../plans/commode_c/hardware` before copying — and
`resolve_plan` now explains a missing plan instead of raising a traceback:

```
$ python3 stage_runner.py --task deep_left --dry
no bundled plan for task 'deep_left'.
  looked for : .../plans/deep_left_ruckig_pro_only.json
  bundled    : 24 task(s) — bottomlid_left, bottomlid_right, …
  Plans are versioned artifacts of THIS repo (see resolve_plan);
  copy the one you want in deliberately, e.g.
    cp <ws>/cleaning_tasks/plans/commode_c/hardware/deep_left_… .../plans/
```

**All 24 now dry-run clean**, `2 PASS 0 FAIL 0 SKIP` each.

Eight tasks still have no plan anywhere — `deep_*`, `front_*`, `lid_side_*`,
`lidsides_back_*`. They have configs but were never planned. `front_right`
additionally raises: *"cleaning_sequence is NOT chained at segment(s) [3]"*.

---

## Where the task configuration is used

This was under-explained in the earlier write-ups. The two inputs are:

| input | source | what it provides |
|---|---|---|
| **task config** | `butterfli_ws/.../config/commode_cleaning/commode_c/<task>_cleaning_points.yaml` | the Cartesian cleaning points, `ik_frame`, speed scalings, pole height, hand steps |
| **saved plan** | `plans/<task>_ruckig_pro_only.json` (bundled) | joint values for the named poses, and the start configuration that pins the redundancy |

`TaskConfig.load(task, fixture="commode_c")` reads the config; its
`CONFIG_ROOT` already points into `butterfli_ws`, so **no copying is needed
for configs** — only for plans. `CleaningPath(cfg).movel_program(pole_m)`
then turns the config's points into the chained `rm_movel` program, and that
program — not the plan's 1800 waypoints — is what gets dispatched.

Both are needed. The config says *where the tool goes*; the plan says *which
of the infinitely many arm configurations to start from*.

---

## The workflow, per task

### 1. Screen it offline — before any motion

```bash
cd src
python3 predict_task.py --ls 0.45 --la 1.6      # predicted REAL utilisation
python3 orientation_cost.py                      # full kinematic audit
```

`predict_task` multiplies the plan's peak joint utilisation by a ratio
calibrated on all 13 REAL recordings (MODE_CHARACTERIZATION.md §4). **The
plan named the REAL binding joint 13/13**, so this is the screen to trust.

⚠ **Do not use `emu_stroke.py` for this.** Its rank correlation against the
plan is +0.09 and it names the right joint 6/24. The reason is structural,
not a bug — MODE_CHARACTERIZATION.md §3.

### 2. Dry run — no controller needed

```bash
python3 stage_runner.py --task <task> --dry
```

Checks serialization, resolves the plan, builds the movel program. Expect
`2 PASS, 0 FAIL, 0 SKIP`.

### 3. SIM run — the free pre-flight

```bash
python3 stage_runner.py --task <task> --mode SIM
```

**A SIM run predicts REAL's binding joint 9/9 and its timing to 0.07 %.** It
flagged the H45 silent stop before it happened: at `line_speed 0.80` SIM read
J4 at 100 % of limit and completed, while REAL stopped dead.

Read it with the SIM-aware tool, because **SIM's `speed{n}` column is dead**
(~0.4 °/s while the arm moves 56° of joint travel) — the position channel is
the faithful one:

```bash
python3 mode_compare.py --channels     # what each mode measures
python3 mode_compare.py                # SIM vs REAL, every matched pair
```

### 4. REAL run

```bash
python3 reset_limits.py --side <side> --apply         # limits RATCHET; do this first
RM_LINE_SPEED=0.450 RM_ANGULAR_SPEED=1.200 RM_LINE_ACC=1.6 \
  python3 stage_runner.py --task <task> --mode REAL
python3 reset_limits.py --side <side>                 # and verify after
```

### 5. Introspect the recording

```bash
python3 dip_report.py ../runs/<run_id>
```

First-order channels only — joint speed, joint current, event durations. It
refuses SIM runs outright, reports the time budget, the dip/ramp split, peak
joint speed against limits, peak current against the 7.3 A / 16.7 A
baselines, and the predicted-vs-measured J4 check.

---

## Safety gates that apply to every new task

**Run untested tasks at `line_speed 0.45 / line_acc 1.6`.** At `line_acc
3.6` — our current toplid operating point — **11 of the 24 are predicted
over a joint speed limit**; at 1.6, none are.

| at `line_speed 0.45` | predicted over limit | ≥ 95 % |
|---|---|---|
| `line_acc` **1.6** | **0** | 0 |
| `line_acc` 2.4 | 0 | 8 |
| `line_acc` 3.6 | **11** | 0 |

**Eight tasks plan at 99.8–99.9 % of a joint limit** — `lid_seat_hinge_area_*`,
`seat_ring_bottom_*`, `side_*`, `bottomlid_*`. They have never been run.
Treat them as unproven at any speed.

**The controller does not stop itself.** At `line_speed 0.80` the arm whipped
four joints in 80 ms at 16.7 A and reported nothing on any channel; at 0.60
it stopped mid-path with no error and no arrival event. E-stop in hand.

**`line_speed` and `line_acc` are persistent controller state and they
ratchet** — `reset_limits.py` before and after, every session.

---

## Quick reference

```bash
cd src

python3 predict_task.py --ls 0.45 --la 1.6     # screen all 24
python3 orientation_cost.py                     # kinematic audit, all 24
python3 stage_runner.py --task <t> --dry        # no controller
python3 stage_runner.py --task <t> --mode SIM   # pre-flight
python3 reset_limits.py --side <s> --apply      # before REAL
python3 stage_runner.py --task <t> --mode REAL  # with env limits above
python3 dip_report.py ../runs/<id>              # introspect
python3 mode_compare.py                         # SIM vs REAL fidelity
```

Every one of these accepts `-h` and prints its documentation without writing
a log or touching the arm.
