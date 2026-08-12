# Speed investigation — what the `line_speed 0.8` run actually did

**Status**: 2026-08-11. Supersedes the retracted `line_acc` analysis (see
[speed_limits.py](src/speed_limits.py) §JOINT for the retraction and its four
defects). Everything below is computed from `runs/` and states its channel.

**Ground truth from the operator**: at `line_speed 0.800` the arm *"gave in and
moved violently"*. The run reported no error. Both halves of that are confirmed
by the recording, and the second half is the more serious.

---

## 0. The corpus, honestly

24 recordings. **18 are `sim=True`** and report joint speed ≈ 0.4 °/s while
still producing 5.6 m of TCP path — they cannot support any dynamics claim.
**Six are REAL.** Of those, three (`20260810T221245/221548/221832`) are the
pendant-override ladder at a fixed `line_speed 0.250`, achieving 38 / 68 / 94 %
of cap; the pendant slider, not any controller limit, is what varies across
them.

That leaves **two REAL runs at a deliberately set `line_speed`**:

| run | `line_speed` | `line_acc` | outcome |
|---|---|---|---|
| `20260811T183500` | 0.450 | 2.4 | PASS, 21.4 s |
| `20260811T184109` | 0.800 | 2.4 | **FAIL**, 13.5 s |

`line_acc` is **2.4 in every run that records it**. It has never been varied.

---

## 1. The violence is real, and current is the cleanest evidence

Joint current is measured directly — no differentiation, no filter, no
derived quantity. Units are **mA** (`rm_ctypes_wrap.py:1896`, 关节电流，单位mA).
Peak over `execute_path`, same task, same path, same tool:

| joint | `ls=0.450` (PASS) | `ls=0.800` (FAIL) | ratio |
|---|---|---|---|
| J1 | 5.2 A | 9.4 A | 1.8× |
| J2 | 5.3 A | 10.4 A | 1.9× |
| J3 | 5.1 A | 11.8 A | 2.3× |
| **J4** | **7.3 A** | **16.7 A** | **2.3×** |
| J6 | 2.0 A | 4.8 A | 2.4× |

Mean current barely moved (J4: 2.0 A → 2.1 A). This is not a heavier duty
cycle — it is a small number of very large transients.

### The transient, sample by sample

`dt` is a flat **10.0 ms** through this whole window (checked: 9.9–10.1 ms),
so none of it is sampling artifact.

```
  t      status  pos4      speed4    current4   current3
  18.28    0     38.983    -223.3     -4831       6348
  18.29    1     36.164    -225.1     -4093       2951
  18.30    1     33.906    -149.6    +14196     +11491
  18.31    1     33.387     -90.4    +16737     +11824     <- peak
  18.32    1     32.920     -15.2     +2852       7067
  18.33    1     32.905     +11.5      3636       3102
```

J4 goes **−225 °/s to +12 °/s in 40 ms**. And 250 ms later, four joints
reverse together:

```
  t=18.64   J1 -145.0   J3 +142.9   J4  -16.8   J6   +7.9
  t=18.72   J1  +62.0   J3  -33.8   J4 +224.4   J6  -85.7
```

J4 swings 241 °/s and slams into its 225 °/s limit. That 80 ms window is the
violent motion.

---

## 2. Nothing reported it

Across the **entire** recording of `20260811T184109`:

- `err1..err7` — **zero on every sample**. No joint fault bit, no
  undervoltage, no position-step warning.
- `arm_status` — only `0` (`RM_IDLE_E`), `1` (`RM_MOVE_L_E`), `2`
  (`RM_MOVE_J_E`), `14`. **Never `9` (`RM_STOP_E`, 急停), never `10`
  (`RM_SLOW_STOP_E`), never `12` (`RM_PAUSE_E`).**
- no collision code. `collision_stage` was **3** and `avoid_singularity`
  was **1** — both protections were on.
- the SDK error read at failure returned clean.

The **only** signal was the `movel` arrival event returning failure. A caller
that trusted return codes and error bits would have recorded a clean run.

Two joints also exceeded `rm_get_joint_max_speed` — J1 at 186 °/s against 180,
J4 at 238 against 225 — **and the controller neither clamped nor faulted**.
That is consistent with `joint_max_speed` being a *planning* parameter rather
than a drive protection, which would explain why `rm_get_joint_max_speed` and
`rm_get_joint_drive_speed` return identical values and neither bounds
execution. **Hypothesis — not established.** Test in §4.

---

## 3. Mechanism: the path contains a 144° elbow swing, and `line_speed` sets how fast it happens

The RM75 is 7-DOF. Its redundancy is one scalar — the **arm angle**, the
elbow's rotation about the shoulder–wrist axis. `rm_movel` takes a Cartesian
target; the arm angle is free, and **nothing in the Cartesian speed/accel
limits bounds its rate**.

Computed with `rm_algo_calculate_arm_angle_from_config_rm75`, unwrapped
(the raw value wraps at ±180° and produces a spurious 38 000 °/s):

| source | arm-angle excursion | peak rate | p99 rate |
|---|---|---|---|
| **planned** (`plans/toplid_left_ruckig_pro_only.json`, 1820 wp) | **144.0°** | — | — |
| measured `ls=0.250` | 146° | 478 °/s | 198 °/s |
| measured `ls=0.450` | 145° | 709 °/s | 279 °/s |
| measured `ls=0.800` | 151° | **702 °/s** | **455 °/s** |

Two things follow.

**The excursion is ours.** MoveIt/Ruckig planned a 144° elbow swing; hardware
reproduces 145–151° at every speed. The controller is not inventing it.

**`line_speed` scales its rate, and the TCP cap does not see it.** The p99
more than doubles (198 → 455 °/s) while the excursion stays fixed. In
`184109` the peak arm-angle rate lands at **t = 18.56 s** — inside the
18.29–18.72 s violent window, alongside the 16.7 A spike.

A 144° null-space excursion is a lot of elbow travel for a wiping stroke, and
it is the most likely thing to fix. It costs nothing at 0.25 m/s and is
dangerous at 0.8.

---

## 4. Where the time actually goes — which knob is worth turning

Measured on `20260811T183500` (`ls=0.450`, PASS, 21.37 s, 5.848 m):

| band | time | share of time | share of path |
|---|---|---|---|
| > 90 % cruise | 8.8 s | 41 % | **72 %** |
| 50–90 % | 3.5 s | 17 % | 15 % |
| **< 50 % (dips)** | **9.0 s** | **42 %** | **11 %** |

**42 % of the time covers 11 % of the path.** The cost is the dips, not the
cruise. Splitting the dip time by what each knob can reach — measured from
dip *durations*, which is first-order timing and needs no differentiation:

```
28 dips, 7.18 s below half cruise
  dwell below 10 mm/s   0.88 s  (12 %)   command gap; line_acc cannot fix
  ramping               6.30 s  (88 %)   line_acc CAN fix
  implied effective ramp accel: 1.58 m/s^2   against line_acc = 2.4 (66 %)
```

Two levers, comparable size, **independent failure modes**:

| lever | headroom | saves | fails by |
|---|---|---|---|
| `line_acc` 2.4 → ~4.8 (halve ramp time) | untested — never varied | ~15 % | torque transient (watch J4 current) |
| cruise → the joint-speed ceiling | +26 % | ~14 % | joint overspeed (predictable, below) |

### The joint-speed ceiling is now a number

Peak J4 speed against **achieved** p95 TCP speed — regressing on achieved
rather than commanded cancels the pendant override entirely:

```
p95 mm/s    114   201   278   486   731
J4 peak      63   105   123   195   238
J4 % limit  28%   47%   55%   87%  106%

J4peak = 0.368 x p95_mm_s        R2 = 0.85, n = 5
  -> J4 reaches its 225 limit at p95 ~ 610 mm/s
  -> 183500 sat at 486 = 79 % of it;  184109 at 731 = 120 %, and it failed
```

R² 0.85 on n=5 makes ~610 an estimate, not an edge. It is enough to say the
headroom above 0.450 is real and roughly +25 %, and enough to explain 0.800.

**Raising `line_acc` at fixed `line_speed` does not raise peak joint speed** —
cruise is unchanged, and peak joint speed is set by cruise through the
Jacobian. So acceleration does not push toward the limit that was actually
breached in §1. It is the lower-risk of the two.

---

## 4b. LADDER RESULTS, 2026-08-11 22:0x — both arms, and the ceiling is J4

Eleven new runs. Both arms ran the `line_acc` ladder at `line_speed 0.450`,
and the right arm has REAL runs for the first time.

### `line_acc` works. It was wrong to demote it.

Durations below are the **`execute_path` stage**, not the whole run. An
earlier version of this table used `duration_s` (which includes hand, pole,
transit and rest stages) and that made every comparison against the plan
wrong by ~7 s.

| arm | `ls / la` | `execute_path` | whole run | cruise | J4 peak |
|---|---|---|---|---|---|
| left | 0.45 / 1.6 | 23.80 s | 31.5 s | 282 mm/s | 177 |
| left | 0.45 / 2.4 | 21.28 s | 29.0 s | 347 | 194 |
| left | 0.45 / 3.6 | **20.22 s** | 28.0 s | 394 | 215 |
| right | 0.45 / 1.6 | 24.20 s | 33.3 s | 283 | 169 |
| right | 0.45 / 2.4 | 21.53 s | 29.1 s | 353 | 188 |
| right | 0.45 / 3.6 | **20.14 s** | 27.5 s | 396 | 207 |
| right | 0.50 / 3.6 | 19.62 s | 27.1 s | 396 | 218 |
| right | 0.60 / 3.6 | — | — | — | **225.0 → STALLED** |

**−15 % (left) / −17 % (right) of stroke time from `line_acc` alone**, at
unchanged `line_speed`.

### How close are the COMPLETING runs, really?

Samples in `execute_path` at or above a fraction of `joint_max_speed`:

| run | `ls/la` | ≥80 % | ≥90 % | ≥95 % | ≥99 % |
|---|---|---|---|---|---|
| left `220322` | 0.45/1.6 | 0 | 0 | 0 | 0 |
| left `220453` | 0.45/2.4 | 14 (0.7 %) | 0 | 0 | 0 |
| left `220617` | 0.45/3.6 | 22 (1.1 %) | 7 (0.3 %) | 1 | 0 |
| right `221720` | 0.45/1.6 | 0 | 0 | 0 | 0 |
| right `221929` | 0.45/2.4 | 5 (0.2 %) | 0 | 0 | 0 |
| right `222049` | 0.45/3.6 | 14 (0.7 %) | 2 (0.1 %) | 0 | 0 |
| right `222354` | 0.50/3.6 | 18 (0.9 %) | 5 (0.3 %) | 2 (0.1 %) | 0 |

**Zero samples above 99 % in any completing run**, and the fastest touches
95 % for 20 ms out of 19.6 s. So "we are at the limit" is true of a *single
instant*, not of the stroke: 99.7 % of it runs below 90 %.

### THE HEADLINE: we are at the joint velocity limit

`rm_get_joint_max_speed` reports 225 °/s for J4. The 0.600 run measured
**225.0** and the arm stopped. This is not a torque problem, a collision
problem, or a Cartesian-limit problem — **the task, at this configuration,
saturates J4.** Everything else in this document is downstream of that.

### Correction: `line_acc` DOES raise peak joint speed

§4 argued that raising `line_acc` at fixed `line_speed` would not push J4
toward the limit that failed. **That was wrong.** J4 rose 169 → 207 °/s
(+38) across the ladder, because higher acceleration lets the arm actually
reach cruise on short segments — measured cruise rose 283 → 396 mm/s under
an unchanged 450 mm/s cap. Both knobs spend the same J4 budget.

### The law, fitted and validated on both arms independently

```
J4peak = C + 47.0 * ln(line_acc / 3.6) + 220 * (line_speed - 0.45)

  C = 206.9 (right)    C = 215 (left, runs 8 deg/s hotter)
  ln-slope fitted separately per arm: 47.0 and 46.9   <- independent agreement
  residuals across all four right-arm points: <= 0.5 deg/s
```

Both knobs are near saturation, and the ceilings are tight:

| | left | right |
|---|---|---|
| max `line_speed` at `la=3.6` | **0.50** | **0.53** |
| max `line_acc` at `ls=0.45` | **4.5** | **5.3** |

The 0.600 run was at a model ceiling of 0.53 — **13 % past it**, and it
stalled. The law predicted the failure.

### `line_speed` is a bad trade; `line_acc` is a good one

From `0.45 / 3.6` on the right arm:

```
ls 0.45 -> 0.50    -0.4 s  (-1 %)    costs J4 +11 deg/s
la 1.6  -> 3.6     -5.8 s  (-17 %)   costs J4 +38 deg/s
```

Per degree of J4 headroom spent, acceleration buys about 4× the time.

### The binding constraint is a fixed point on the path, not either knob

The J4 peak, the peak elbow rate, and the stall all land in the same place
regardless of speed or acceleration:

```
J4 peak            74.4 - 75.6 % of path   (five runs, 1.6 <= la <= 3.6)
peak elbow rate    75.1 - 77.9 %
the 0.600 stall    78.7 %
```

**There is a kinematic hot spot at ~75 % of the stroke.** Its *location* is
fixed by geometry; speed and acceleration only scale the joint rate demanded
there. Both knobs are now within 10 % of saturating J4 at that one point, so
**neither has meaningful headroom left and fixing the hot spot is the only
thing that unlocks more.** That is the 144–149° null-space excursion of §3,
now localised.

### H45 — a worse failure mode than H39

The 0.600 run did not report a failure. It **stalled**: TCP motion stopped
dead at t+11.6 s, 4.629 m into a 5.88 m path, and stayed at zero for 16 s
until the operator interrupted. Throughout: `err1..err7` all zero,
`arm_status` only 0/1, no collision code — and **the arrival event never
fired at all**, so `monitor.wait()` blocked forever. H39's run at least
returned a failed arrival event. This one returns nothing and hangs.

⚠ `capabilities_as_found` on the right arm shows **`collision_stage: 0`** for
every one of these runs — collision detection was OFF, unlike the left arm's
3. Any collision-related conclusion from the right-arm ladder is void.

---

## 5. What to run next

### Step 0 — preconditions, every run

- **Pendant global override at 100 %**, verified by eye. It is unreadable by
  the SDK and it silently scaled every timing number we have (`183500`
  achieved 79 % of cap, `184109` 55 %).
- REAL only. SIM cannot answer any of this.
- Free space, no commode. E-stop in hand — §2 says the controller will not
  stop itself.
- **One knob at a time.** If speed and acceleration both move and the arm
  whips, the run tells us nothing.

### Step 1 — the `line_acc` ladder at fixed `line_speed 0.450`

Biggest measured lever (88 % of the lost time), never varied, and it does not
approach the limit that failed. Three REAL runs, `line_speed` held at
**0.450**, `line_acc` ∈ {1.6, 2.4, 3.6}. Ratio constraint needs
`line_acc ≥ 3 × line_speed = 1.35`, so all three are legal.

*Measure* effective ramp acceleration by dip duration (§4) — first-order, and
it sidesteps the estimator problem that wrecked the earlier analysis. Watch
**J4 current** live; 7.3 A is the passing baseline, 16.7 A was the violent
event. Abort on a reversal.

*Falsifiable*: effective ramp accel should track `line_acc`. It currently sits
at 1.58 against a commanded 2.4 — only 66 %. **If it does not move across the
ladder, `line_acc` is inert for `movel` ramps** and the dips are a
trajectory-connect problem instead — which is exactly H35, settled by the same
run.

### Step 2 — settle whether `joint_max_speed` protects anything

One `rm_movej` on a single joint, free space, commanding a speed that implies
> 225 °/s on J4, recorded. Does the controller clamp, fault, or comply?

*Falsifiable*: comply → it is a planning parameter only, and **every joint
speed bound must be enforced by us, before dispatch**. Clamp or fault → §2's
hypothesis is wrong and the `184109` overspeed needs another explanation.

### Step 3 — walk `line_speed` up to the predicted ceiling

Only after Step 2. `line_speed` ∈ {0.50, 0.55} at whatever `line_acc` Step 1
selected. The regression predicts J4 stays under 225 while p95 stays under
~610 mm/s.

*Falsifiable*: J4 peak should land within ±30 °/s of `0.368 × p95`. A miss
means the linear law breaks down near the limit and the ceiling is not where
we think it is — stop there.

### Step 4 — reduce the null-space excursion, then re-test 0.800

Needs a replan first, so it is blocked on code. Re-plan `toplid_left` with the
arm angle constrained so the excursion is well under 144°, then re-run at
`ls=0.450` and `ls=0.800`.

*Falsifiable*: if §3's mechanism is right, a plan with a ≤40° excursion runs
at `ls=0.800` with J4 peak current at or below the 7.3 A seen at 0.450, and no
reversal. If current still spikes to ~16 A, §3 is wrong.

### A standing rule for any acceleration number

The four defensible estimators of joint acceleration disagree by 2.6×–16.6× on
existing data, and `speed{n}` is not a clean derivative of `position{n}`
(median 12 °/s apart, up to 123 °/s on J4). Either characterise that filter
against a known input, or report only **first-order** quantities — joint
speed, joint current, and event durations — which is how §1, §3 and §4 were
obtained.

### Not yet a question for RealMan

The earlier draft asked about a `set_arm_max_line_acc` ceiling. That question
came from an analysis that did not hold. **The question worth their time is
§2**, and only after we have measured it ourselves:

> During `rm_movel` we recorded J1 at 186 °/s (limit 180) and J4 at 238 °/s
> (limit 225), with a 16.7 A transient and four joints reversing inside
> 80 ms. No joint error bit was set, `arm_status` never entered `RM_STOP_E`
> or `RM_SLOW_STOP_E`, and collision detection at stage 3 did not trigger.
> Is `rm_get_joint_max_speed` enforced during execution, or is it a planning
> parameter only? If the latter, what *does* protect against an overspeed?

---

## 5. Recorded, for the port

- **H39** — at `line_speed 0.800` the left arm executed a 4-joint reversal in
  80 ms with a 16.7 A J4 transient (2.3× the passing run) and **reported no
  error on any channel**: no joint fault bit, no collision code, no
  `RM_STOP_E`. Only the arrival event failed. Operator-confirmed violent
  motion. *(`runs/20260811T184109_toplid_left_left`, t = 18.28–18.72)*
- **H40** — the cleaning plans carry a **144° arm-angle (null-space)
  excursion**, reproduced 145–151° on hardware at every speed. Cartesian
  speed and acceleration limits do not bound its rate: p99 rises 198 →
  455 °/s as `line_speed` goes 0.250 → 0.800.
- **H41** — J1 and J4 exceeded `rm_get_joint_max_speed` during `movel`
  without clamp or fault. Suggests it is a planning parameter, not a drive
  protection. **Unconfirmed** — Step 2 settles it.
- **H42** — `stream.csv` is shape-identical between SIM and REAL; only the
  near-zero `speed{n}` columns distinguish them. Any analysis reading
  `stream.csv` must gate on `run.json["sim"]` first. 18 of 24 recordings are
  SIM.
- **H43** — the cleaning stroke spends **42 % of its time covering 11 % of its
  path**, and **88 % of that lost time is ramping**, not dwelling at rest.
  Effective ramp acceleration measures **1.58 m/s² against a `line_acc` of
  2.4**. Acceleration is the largest single lever in the run and has never
  been varied. *(`20260811T183500`, 28 dips, measured by dip duration)*
- **H46** — **the joint speed is spent holding ORIENTATION, not translating.**
  Numerical 6×7 Jacobian at the binding instant of `20260811T222049`
  (`q = [-16.4, -30.5, 25.0, 58.5, -6.0, 27.6, 17.5]`, flange
  `Ry = 58.1°` — 31.9° clear of gimbal lock):
  * TCP is translating at **336 mm/s** and rotating at only **6 °/s**
  * min-norm joint rates for the identical 6-DOF twist: **205 °/s**;
    the controller actually used **207** — **within 1 % of optimal**, so
    the 7-DOF redundancy has essentially nothing to give back here
  * the same 336 mm/s with orientation **free**: **70 °/s**
  * → **holding tool orientation costs 135 of the 205 °/s, i.e. 66 % of the
    budget**, while the orientation barely changes
  * Jacobian singular values 0.0106 / 0.0064 / 0.0025 m per °/s —
    **4.2× anisotropy**, and the stroke direction sits near the worst axis
  * per-axis relaxation (rx 194, ry 75, rz 186 °/s) is in **Euler-rate**
    space, so which physical axis carries the cost needs the proper
    angular-velocity form before acting on it. The 205-vs-70 comparison
    does not depend on that parameterisation.
  ⚠ A position-only (3×7) Jacobian says 70 °/s and implies the controller
  wastes 196 % on null-space motion. **That is an artifact of letting
  orientation drift** — with the real 6-DOF constraint the controller is
  near-optimal. Do not quote the 3-DOF number as waste.
- **H47** — **the orientation cost is universal across all four cleaning
  tasks, and the hinge tasks are worse.** `src/orientation_cost.py`, offline,
  over each saved plan (stride 20):

  | task | median cost | max | at peak demand | peak joint util |
  |---|---|---|---|---|
  | toplid_left | 1.7× | 3.8× @ 77 % | 1.6× @ 55 % | **83 % J4** |
  | toplid_right | 1.6× | 3.5× @ 89 % | 1.7× @ 63 % | **77 % J4** |
  | hinge_area_left | 2.1× | **70.4× @ 17 %** | 2.7× @ 56 % | **88 % J3** |
  | hinge_area_right | 2.1× | **34.7× @ 17 %** | 2.1× @ 56 % | **90 % J1** |

  Three things follow.
  * **It is not a toplid quirk.** Holding tool orientation costs 39–63 % of
    the joint-speed budget at the peak-demand point of every task.
  * **The binding joint is task-specific** — J4 on toplid, J3 on
    `hinge_area_left`, J1 on `hinge_area_right`. There is no single weak
    joint to design around.
  * ⚠ **The hinge tasks plan 6–13 points closer to the limit than toplid
    (88–90 % vs 77–83 %), and only toplid has ever been speed-tested.**
    `toplid_right` plans at 77 % and *executed* at 97 % at `ls=0.500`.
    PREDICTION: the hinge tasks will saturate at a lower `line_speed` than
    toplid. Start them at 0.250, not at toplid's settings.
  * the 70.4× and 34.7× spikes at 17 % of both hinge paths are near-singular
    configurations — worth locating before any hinge speed work.
- **H44** — peak J4 speed is linear in achieved TCP speed:
  `J4peak = 0.368 × p95_mm_s`, R² 0.85 over all five REAL runs. Regressing on
  **achieved** rather than commanded speed cancels the unreadable pendant
  override. Predicts J4 reaching its 225 °/s limit at p95 ≈ 610 mm/s; the
  failed run sat at 731 (120 %), the passing run at 486 (79 %).
