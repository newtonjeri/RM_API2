# Speed investigation — complete assessment

**Status**: 2026-08-12. Restructured from the 2026-08-11 working notes after three
rounds of answers from RealMan support. Everything below is computed from `runs/`
and states its channel. Findings are numbered in the shared H-series used across
`TASK_KINEMATICS.md` and `MODE_CHARACTERIZATION.md` — this document owns H39–H47
and H57–H64; do not reuse a number another document defines.

**The question this document exists to answer**: how do we command a predictable
TCP speed on a chained `rm_movel`, and why did the arm stop — twice, in two
different ways — when we pushed it.

**Ground truth from the operator**: at `line_speed 0.800` the arm *"gave in and
moved violently"*. The run reported no error. Both halves of that are confirmed
by the recording, and the second half is the more serious.

---

## 0. Findings summary

| # | Finding | Status |
|---|---|---|
| **H39** | At `ls=0.800` the left arm ran a 4-joint reversal in 80 ms with a 16.7 A J4 transient (2.3× the passing run) and **reported no error on any channel** — no fault bit, no collision code, never `RM_STOP_E`. Only the arrival event failed. | **Confirmed** |
| **H45** | At `ls=0.600 / la=3.6` the right arm **stalled**: TCP motion stopped dead at 4.629 m of 5.88 m and stayed at zero 16 s until interrupted. All err flags zero, `arm_status` only 0/1, **and the arrival event never fired at all** — `monitor.wait()` blocks forever. Strictly worse than H39, which at least returned a failed event. | **Confirmed** |
| **H40** | The cleaning plans carry a **144° arm-angle (null-space) excursion**, reproduced 145–151° on hardware at every speed. Cartesian limits do not bound its rate: p99 rises 198 → 455 °/s as `ls` goes 0.250 → 0.800. | **Confirmed** |
| **H43** | The stroke spends **42 % of its time covering 11 % of its path**, and **88 % of that lost time is ramping**, not dwelling. Effective ramp accel measured **1.58 m/s² against a commanded `line_acc` 2.4**. | **Confirmed** |
| **H57** | **`line_acc` is the dominant lever on achieved TCP speed.** At fixed `v=100` and fixed `ls=0.45`, median TCP rises **62.6 → 77.0 → 87.6 % of cap** as `line_acc` goes 1.6 → 2.4 → 3.6 (left; right replicates 62.8 → 78.4 → 88.1) while **p95 stays pinned at the cap**. The tool reaches the commanded speed and cannot hold it. **The shortfall is acceleration-bound.** | **Confirmed, both arms** |
| **H58** | **The pendant's global override IS the vendor's "real-time speed adjustment", and it multiplies.** Our own override ladder at fixed `ls=0.250` gives p95/cap = 0.454 / 0.803 / 1.111 → slider ≈ 41 / 72 / 100 %. | **Confirmed** |
| **H59** | **At 100 % override the achieved p95 EXCEEDS the commanded cap by 5–11 %** in every completing run (474–495 mm/s against a 450 cap; 539 against 500). The TCP cap is not a hard ceiling. Corollary: the override was at 100 % for every 2026-08-11 run, so **it is not the explanation for the shortfall**. | **Confirmed** |
| **H60** | **The binding constraint is a fixed point on the path, not a knob.** J4 peak (74.4–75.6 % of path), peak elbow rate (75.1–77.9 %) and the stall (78.7 %) all land in the same place across five runs at 1.6 ≤ `la` ≤ 3.6. A kinematic hot spot at **~75 % of the stroke**. | **Confirmed** |
| **H46** | **The joint speed is spent holding ORIENTATION, not translating.** At the binding instant the TCP translates 336 mm/s and rotates 6 °/s, yet needs 205 °/s of joint rate; with orientation free the same translation needs **70 °/s**. Holding orientation costs **66 % of the budget**. | **Confirmed** |
| **H47** | The orientation cost is **universal across all four cleaning tasks**, and the hinge tasks plan 6–13 points closer to the limit than toplid (88–90 % vs 77–83 %). Only toplid has ever been speed-tested. | **Confirmed (offline)** |
| **H44** | Peak J4 speed is linear in **achieved** TCP speed: `J4peak = 0.368 × p95_mm_s`, R² 0.85, n=5. Predicts J4 hits 225 °/s at p95 ≈ 610 mm/s. | **Confirmed, weak n** |
| **H63** | **DWELL AT SATURATION, NOT PEAK, PREDICTS THE FAILURE.** Time spent at ≥98 % of a joint limit separates every outcome with no overlap: **0 ms in all six completing runs, 110 ms in the stall, 330 ms in the violent run.** Peak alone does not — 96.9 % completes, 99.8 % stalls. The stall **never exceeded 100 %**. | **Confirmed** — §4.9 |
| **H41** | J1 (186/180) and J4 (238/225) **exceeded** `rm_get_joint_max_speed` during `movel` with no clamp and no fault — suggesting it is a planning parameter, not a drive protection. | **Open** — Step 1 settles it |
| **H62** | **RealMan advises AGAINST modifying the TCP speed/acceleration values**, warning it degrades accel/decel behaviour and the immediate-stop response. This is guidance against the entire `line_acc`-raising programme. See §1.3. | **Open — reconcile** |
| **H64** | **THE FACTORY DEFAULTS ARE 0.250 m/s / 1.600 m/s² / 0.600 rad/s / 4.000 rad/s²** — read back after pressing the pendant's "Default" button on a **never-used arm** of the same model *and* on **our own working arm**, with identical results. Identical also to the F10 read of both arms, so both were at shipped state on 2026-08-07, `reset_limits.py --apply` is a genuine restore, and the ladder's 1.6 rung **is** the default. | **Confirmed, two arms** — §1.6 |
| **H65** | **The real-time override is visible and settable in the pendant toolbar** (`Speed ——●—— NN%`) and is **not always 100 %** — the reference arm was found at 50 %. **All of our own runs were made at 100 %** (Newton, 2026-08-12), which confirms H59's inference by direct observation. Still unreadable through the SDK. The same page documents the joint setter ranges — speed **1–180 °/s**, acceleration **1–600 °/s²** — so our joint values are simultaneously the defaults *and* the maxima. | **Confirmed** |
| **H69** | **THE J4 SCREEN IS SOUND BUT INCOMPLETE — and a saved plan is the complete one.** Sweeping the arm angle with `rm_algo_inverse_kinematics_rm75_for_arm_angle`: J4 **0.0 %** spread, J6 1.5 %, J5/J7 8–9 %, J2 89 %, **J1/J3 62 %** (and they diverge near the shoulder singularity, moving in equal opposite steps). So J4 alone gives a *no-false-alarm* reject test, but per `TASK_KINEMATICS` H49 **J1 binds on 11 of 24 tasks and J4 on only 4** — a J4-only screen passes the majority blind. **A saved plan pins the redundancy, so all seven joints become predictable from its own `velocities`** — that read gives toplid_left J4 **83 %**, matching `TASK_KINEMATICS` exactly. | **Confirmed** — §4.11 |
| **H66** | **J4 IS REDUNDANCY-INVARIANT — the one joint predictable offline.** On an S-R-S 7-DOF arm the elbow angle is a function of the shoulder→wrist distance alone, which the commanded Cartesian pose fixes. Re-solving a segment from 12 randomised arm configurations (±35° on J1/J3/J5/J7) gives the identical J4 demand — **0.000 % spread**, independently reproduced 2026-08-12. This is why `predict_task.py` names the REAL binding joint 13/13. | **Confirmed** — §4.10 |
| **H67** | **The 0.60 rad/s angular cap silently protects high-rotation segments, and a LOW `rot°` is therefore a WARNING, not comfort.** A segment demanding > 0.60 rad/s of tool rotation gets time-scaled by the controller — which incidentally keeps its joints safe. A segment that barely rotates runs at the full commanded speed, straight into joint saturation. | **Confirmed** |
| **H68** | **Reshaping the ORIENTATION PROFILE is a third lever — and it does not touch the limits RealMan advise against changing (H62).** On `test_motion_001`, changing **one number** (point 8 `ry` −0.218 → −0.400) took two segments from hand-tuned 60 %/90 % back to **100 %/100 %**, and the row got *faster in wall-clock* (0.83 s vs 1.34 s). Total tool rotation across the row was unchanged — only its distribution. **Hardware-confirmed.** | **Confirmed** |
| **H70** | **THE SINGLE-WAYPOINT FIX DOES NOT TRANSFER TO `toplid`.** `test_motion_001` had *one* smooth outlier segment (4.75° rotation, ω 0.23 rad/s) far below the angular cap. `toplid`'s long segments **all** sit at 25–30° with ω between 0.50 and 0.65 — clustered on the cap, six of them just under it and each demanding 89–125 % of J4. Sweeping the worst point's tilt moves the binding segment to its neighbour and gains ~1 point (125 % → 124 %). **Its overload is distributed, not localised**, so it needs a multi-waypoint or replanning fix. | **Confirmed** — §4.12 |
| **H71** | **DO NOT RAISE `rm_set_arm_max_angular_speed` — it is currently the elbow's only protection.** Modelled across six tasks: raising 0.60 → 2.40 rad/s leaves the worst J4 unchanged or makes it **worse** (`bottomlid_left` 81 % → 199 %, `seat_ring_left` 235 % → 291 %) while throttled segments collapse from 15/27 to 1/27. Lowering it is a real safety lever that costs time roughly linearly: `toplid_left` 126 % → 103 % → 69 % → 46 % at caps 0.60 / 0.45 / 0.30 / 0.20, for stroke times 16.6 → 21.4 → 31.9 → 47.8 s. | **Modelled** — §4.13 |
| **H72** | **SUBDIVIDING A SEGMENT WITH LINEAR ORIENTATION IS A PROVABLE NO-OP** — `toplid_left` reads 283 °/s at 1, 2, 4 and 8 sub-segments, identical, because splitting preserves the path, ω and the elbow sweep exactly. It only creates *handles*: shaping those points' orientation NON-linearly does work, but is bounded by the next segment (toplid 126 % → 124 %). And the **shape discriminator says it cannot save the tasks that need it** — of the 10 tasks over the J4 limit, **none** is concentrated in one segment; the concentrated ones are all already under 90 %. | **Confirmed** — §4.14 |
| **H42** | `stream.csv` is shape-identical between SIM and REAL; only the near-zero `speed{n}` columns distinguish them. Any analysis must gate on `run.json["sim"]` first. | **Standing rule** |
| ~~H61~~ | ~~Vendor contradiction on what `v` scales.~~ **RETRACTED 2026-08-12** — an artifact of misreading round 1 as applying the joint baseline to *both* commands. Round 1 distributes: `movel`→end-effector speed, `movej`→joint angular velocity. **The two answers agree.** | **Retracted** |

---

## 1. What RealMan has told us

Three rounds, 2026-08-12, as interpreted by Newton (native reader — an earlier
machine reading of round 1 manufactured a contradiction that is not there, see §1.4).

### 1.1 Round 1 — `v` in `movel` and `movej` scale different things

> *"v在rm_movel与rm_set_arm_max_line_speed中是不一样的，movej、movel的V是关节的最大
> 角速度，另一边的v是机械臂末端的速度（TCP），TCP是工具中心点单位mm/s、m/s，是空间
> 笛卡尔线速度。"*
>
**Reading (Newton):** the sentence *distributes* — the `v` in `rm_movel` and
`rm_movej` represent, respectively, **maximum end-effector speed** and **maximum
joint angular velocity**. It is not saying the joint baseline governs both. This
is the trivial, already-known part; the substance is in round 2.

Also given: **max `line_speed` = 1.8 m/s**, so the corresponding acceleration
ceiling is **3 × 1.8 = 5.4 m/s²** by their ratio rule, and **`line_acc ≥ 3 ×
line_speed` is intended design**, not a quirk. ⚠ Whether either maximum is
*documented* anywhere is unconfirmed.

### 1.2 Round 2 — the baseline differs per command, and there is a multiplier

> *"如果是movej关节空间的话，这个速度比例系数是接口设置的速度\*实时调速的速度，然后速度
> 的基准就是安全配置里关节的速度；在这个基础上算法做轨迹规划；
> 如果是movel笛卡尔空间的话，这个速度比例系数是接口设置的速度\*实时调速的速度，然后速度
> 的基准是TCP的速度约束；在这个基础上算法做轨迹规划；"*
>
> "For **movej** (joint space), the speed ratio coefficient is the interface-set
> speed × the real-time speed adjustment, and the **baseline is the joint speed in
> the safety configuration**. For **movel** (Cartesian space), the coefficient is
> the interface-set speed × the real-time speed adjustment, with the **baseline
> being the TCP speed constraint**."

So, per round 2:

```
effective movej speed  =  v_interface  ×  realtime_adjust  ×  joint_speed_limit
effective movel speed  =  v_interface  ×  realtime_adjust  ×  TCP_speed_constraint
```

### 1.3 Round 3 — do not change the TCP values

> *"不建议修改TCP的值，默认值是咱们算法设计的比较合理的值，点击停止能够立刻停下来；修改了
> tcp的速度和加速度，会影响机械臂加速和减速运动，可能会存在加速很慢，减速很慢，时间就会更长"*
>
> "Modifying the value of TCP is **not recommended**. The default value was designed
> by our algorithm to be fairly reasonable. Clicking stop can halt immediately.
> Changing the speed and acceleration of TCP affects the arm's acceleration and
> deceleration, potentially resulting in very slow acceleration and deceleration,
> which would prolong the overall time."

### 1.4 The settled model — and a retraction (~~H61~~)

**The two answers agree.** Round 1 distributes across the two commands; round 2
states the same split in full, adding the multiplier. The model is:

```
movej:  effective = v_interface  x  realtime_adjust  x  JOINT speed limit
movel:  effective = v_interface  x  realtime_adjust  x  TCP speed constraint
```

> **RETRACTED (H61).** An earlier revision of this document claimed the two
> rounds contradicted each other, on a machine reading of round 1 that applied
> the joint baseline to *both* commands. That reading was wrong: the sentence
> distributes. There was never a contradiction, and nothing needs reconciling
> with RealMan. Recorded rather than deleted because the false claim briefly
> reached the code and the questions doc.

**What this settles.** `movel`'s `v` is a percentage of the TCP speed constraint,
i.e. of `rm_set_arm_max_line_speed`. That is what `src/rm_emulator.py` and
`src/speed_limits.py` now encode, and it is what the codebase assumed all along.

**What it leaves open.** The formula ends *"based on this, the algorithm performs
trajectory planning"* — both times. That frames every one of these speeds as a
**planning input** and says nothing about what bounds *execution*, which is
precisely our failure case (H41, H63). It is weak support for H41 and not an
answer to it.

**Also unanswered by it**: which constraint governs a rotation-dominated segment —
the linear cap or `rm_set_arm_max_angular_speed`. "TCP speed constraint" does not
say.

### 1.5 ⚠ Round 3 versus our entire programme (H62)

Our single most effective measured lever is raising `line_acc` (H57: −15 %/−17 %
of stroke time, median 62.6 → 87.6 % of cap). **RealMan explicitly advises against
modifying the TCP speed and acceleration values.** Two parts of that warning
deserve different weight:

* *"may result in very slow acceleration and deceleration, prolonging the time"* —
  this is about **lowering** the values. We have measured the opposite direction
  and it behaves as expected. Low concern.
* *"点击停止能够立刻停下来"* — the defaults preserve the ability to **stop
  immediately** on command. **This is a safety claim, and it is the one that
  matters.** We have never measured stop latency at raised `line_acc`, and both of
  our failures are stop-behaviour failures. Treat raised `line_acc` as carrying an
  unquantified stopping-distance cost until this is answered.

**Do not read H57 as a licence to raise `line_acc` on hardware** until §5 Step 0
is satisfied and the stop-latency question is put to RealMan.

### 1.6 ✔ The factory defaults, settled (H64)

RealMan advise using the defaults. **We now know what they are**, established by
Newton on 2026-08-12 by pressing the pendant's **"Default"** button under
Configuration → Security Config → TCP Speed Limit and reading the values back —
first on an arm **that had never been used** (so uncontaminated by any program of
ours), then on **our own working arm**, with identical results:

| | linear velocity | linear acceleration | angular velocity | angular acceleration |
|---|---|---|---|---|
| **factory default** | **0.250 m/s** | **1.600 m/s²** | **0.600 rad/s** | **4.000 rad/s²** |

**These are exactly the values the F10 read returned from both our arms on
2026-08-07.** So the earlier worry — that our own programs had written the
Cartesian limits and we were mistaking our own settings for the shipped state —
does not hold for these values. Three consequences, all favourable:

1. **`reset_limits.py --apply` is a genuine factory restore.** Its target was
   right all along.
2. **Both arms were at their shipped state when characterised**, so the whole
   investigation has a clean baseline.
3. **The `line_acc` ladder's 1.6 rung IS the default** — which makes 2.4 and 3.6
   genuinely *above* default, and makes the −15 %/−17 % gain exactly the trade
   RealMan caution against in §1.5. The caution applies squarely to what we did.

The ratchet is still real and still matters: the census of 2026-08-11 20:09 found
the left arm at **0.800 / 2.400**, the settings of the violent run, left behind by
an aborted program. Read before every session; the defaults just give us a correct
value to restore *to*.

**Still unidentified: the API equivalent of that button.** `rm_set_arm_tcp_init`
remains the candidate — it sits in the `ArmTipVelocityParameters` group alongside
the four setters — but it is untested. The pendant path works today.

**And the joint limits are simultaneously the defaults and the maxima.** The same
page documents the setter ranges as speed **1–180 °/s** and acceleration
**1–600 °/s²**, against our read of 180 / 600. There is no headroom to raise them
into, which removes one option from §5 Step 1.

### 1.7 The real-time override — ours were all at 100 % (H65)

The pendant carries the real-time speed adjustment in its toolbar header —
`Speed ——●—— NN%`. **Every run in this document was made with the slider at 100 %**
(Newton, 2026-08-12). That is a direct observation, and it independently confirms
what H59 inferred from the data: p95 landing 5–11 % *above* the commanded
constraint is what a 100 % slider looks like.

Two things still follow from it. **It is not always 100 %** — the reference arm was
found at 50 %, which under §1.4 would halve every commanded speed on both `movej`
and `movel`. And it is **still unreadable through the SDK**, so a run cannot
self-describe: the only defences are verifying by eye before the run (§5 Step 0)
and the H59 p95 check afterwards.

---

## 2. The assessment — what we are confident of

**The shortfall was never a defect.** Achieved TCP sits below the commanded cap
because the stroke is **acceleration-bound**: 42 % of the time covers 11 % of the
path (H43), 88 % of that is ramping, and raising `line_acc` alone recovers it
(H57) while p95 sits *at or above* the cap throughout (H59). The tool reaches the
speed we ask for; it cannot hold it between corners. Ramp distance is v²/2a, so at
`la=2.4` a 450 mm/s cap needs ~84 mm of accel+decel against a ~230 mm mean segment,
while an 800 mm/s cap needs ~267 mm — more than the segment, so cruise is never
reached at all. That is why the shortfall *worsens* as the cap rises, and it is
geometry, not a controller fault. **No question for RealMan here.**

**The real-time speed adjustment is real, it multiplies, and it was at 100 %.**
Round 2 names a `× 实时调速` term; our override ladder measured it (H58); and p95
exceeding the cap by 5–11 % proves it was not derating the 2026-08-11 runs (H59).
It remains unreadable from the API, which is why every timing measurement needs
the slider verified by eye.

**Both knobs are spent, and they were never the real constraint.** `line_speed` and
`line_acc` both drive J4 toward 225 °/s at *the same fixed point* — ~75 % of the
stroke (H60). The fitted law puts the ceilings at `ls` 0.50/0.53 and `la` 4.5/5.3,
and the 0.600 run sat 13 % past its ceiling and stalled. Per degree of J4 headroom
spent, acceleration buys ~4× the time that speed does — but neither has meaningful
headroom left.

**The actual constraint is the plan, not the controller — and we now know how to
fix it.** The hot spot is where a **144° null-space excursion** (H40) meets an
orientation constraint that eats 66 % of the joint-speed budget (H46), a cost that
recurs on *every* cleaning task with the untested hinge tasks planning closer to
the limit than toplid (H47). **Fixing the path is the only thing that unlocks more
speed** — everything else is within 10 % of saturating J4 at one point.

**And that fix is now demonstrated, on hardware.** `test_motion_001` (§4.10) had
the same shape of problem and was cured by moving **one waypoint's tilt**: two
segments went from a hand-tuned 60 %/90 % back to 100 %, and the row got *faster*
(0.83 s vs 1.34 s). The enabling facts are that **J4 is redundancy-invariant** so
its demand can be computed offline exactly (H66), and that **a segment which barely
rotates is the dangerous one** because nothing time-scales it (H67). This is the
only lever that does not touch the TCP limits RealMan advise against changing.

**The failure criterion is dwell at saturation, and we can now predict it.** Time
at ≥98 % of a joint limit separates every run we have: 0 ms completes, 110 ms
stalls, 330 ms goes violent (H63). The stall never crossed 100 % — it gave in
*below* its ceiling — so there is no amplitude margin to spend, and the peak-based
ceilings of §4.5 understate the risk. **Screen candidate plans on predicted dwell,
not on predicted peak.**

**The controller will not protect us.** Two distinct silent failures (H39, H45),
neither reported on any error channel, one of which hangs the caller forever. Two
joints exceeded their reported limits with no clamp and no fault (H41), and the
vendor's own description of `v` ends "the algorithm performs trajectory
**planning**" — which is where these limits appear to live. Until Step 1 settles
it, **assume nothing bounds execution, keep the E-stop in hand, and validate joint
rates before dispatch.**

**Our baseline is clean, and the trade is now explicit.** The factory defaults are
0.250 / 1.600 / 0.600 / 4.000 (H64, confirmed on two arms), which is exactly what
both arms read at characterisation — so nothing in this document was measured from
a contaminated starting point. It also sharpens §1.5: the ladder's 1.6 rung is the
default, so the −15 %/−17 % we gained came from running **above** it, which is the
configuration RealMan advise against. That is a decision to take deliberately, not
one we have already taken by accident.

**What we still cannot explain**: the stall itself (H45) — what internal state the
controller is in, why no event fires, and what to poll to see it; and the 103 %/106 %
overspeed past a commanded 100 % (H41). Both are with RealMan.

---

## 3. The corpus, honestly

35 recordings, of which **18 are `sim=True`** and report joint speed ≈ 0.4 °/s while
still producing 5.6 m of TCP path — they cannot support any dynamics claim (H42).

**Every recording dispatches `cleaning_pct = v = 100`.** `v` has never been varied
in this project. Under either vendor reading that is the loosest setting available,
so `v` is an entire untouched axis of this investigation.

The REAL runs that carry deliberate settings:

| run | `ls` | `la` | outcome |
|---|---|---|---|
| `20260810T221245/221548/221832` | 0.250 | 1.6 | pendant-override ladder — 38/68/94 % of cap (H58) |
| `20260811T183500` | 0.450 | 2.4 | PASS, 21.4 s |
| `20260811T184109` | 0.800 | 2.4 | **FAIL — violent, H39** |
| `20260811T220322/220453/220617` | 0.450 | 1.6/2.4/3.6 | left `line_acc` ladder (H57) |
| `20260811T221720/221929/222049` | 0.450 | 1.6/2.4/3.6 | right `line_acc` ladder (H57) |
| `20260811T222354` | 0.500 | 3.6 | PASS, best time |
| `20260811T222451` | 0.600 | 3.6 | **FAIL — stalled, H45** |

⚠ `capabilities_as_found` on the right arm shows **`collision_stage: 0`** for every
right-arm ladder run — collision detection was OFF, unlike the left arm's 3. **Any
collision-related conclusion from the right-arm ladder is void.**

---

## 4. Evidence

### 4.1 The violence is real, and current is the cleanest evidence (H39)

Joint current is measured directly — no differentiation, no filter. Units **mA**
(`rm_ctypes_wrap.py:1896`). Peak over `execute_path`, same task, same path, same tool:

| joint | `ls=0.450` (PASS) | `ls=0.800` (FAIL) | ratio |
|---|---|---|---|
| J1 | 5.2 A | 9.4 A | 1.8× |
| J2 | 5.3 A | 10.4 A | 1.9× |
| J3 | 5.1 A | 11.8 A | 2.3× |
| **J4** | **7.3 A** | **16.7 A** | **2.3×** |
| J6 | 2.0 A | 4.8 A | 2.4× |

Mean current barely moved (J4: 2.0 → 2.1 A). Not a heavier duty cycle — a small
number of very large transients. `dt` is a flat **10.0 ms** through this window
(9.9–10.1 ms), so none of it is sampling artifact.

```
  t      status  pos4      speed4    current4   current3
  18.28    0     38.983    -223.3     -4831       6348
  18.29    1     36.164    -225.1     -4093       2951
  18.30    1     33.906    -149.6    +14196     +11491
  18.31    1     33.387     -90.4    +16737     +11824     <- peak
  18.32    1     32.920     -15.2     +2852       7067
  18.33    1     32.905     +11.5      3636       3102
```

J4 goes **−225 °/s to +12 °/s in 40 ms**. 250 ms later four joints reverse together:

```
  t=18.64   J1 -145.0   J3 +142.9   J4  -16.8   J6   +7.9
  t=18.72   J1  +62.0   J3  -33.8   J4 +224.4   J6  -85.7
```

J4 swings 241 °/s and slams into its 225 °/s limit. That 80 ms window is the
violent motion.

### 4.2 Nothing reported it (H39, H41)

Across the **entire** recording of `20260811T184109`:

- `err1..err7` — **zero on every sample**. No fault bit, no undervoltage, no
  position-step warning.
- `arm_status` — only `0`/`1`/`2`/`14`. **Never `9` (`RM_STOP_E`), never `10`
  (`RM_SLOW_STOP_E`), never `12` (`RM_PAUSE_E`).**
- No collision code, with `collision_stage` **3** and `avoid_singularity` **1** —
  both protections on.
- The SDK error read at failure returned clean.

The **only** signal was the `movel` arrival event returning failure. A caller
trusting return codes and error bits would have recorded a clean run.

Two joints also exceeded `rm_get_joint_max_speed` — J1 at 186 against 180, J4 at
238 against 225 — **and the controller neither clamped nor faulted** (H41). That is
consistent with `joint_max_speed` being a *planning* parameter rather than a drive
protection, which would explain why it and `rm_get_joint_drive_speed` return
identical values and neither bounds execution. **Hypothesis, not established** —
§5 Step 2.

### 4.3 The path contains a 144° elbow swing (H40)

The RM75 is 7-DOF; its redundancy is one scalar — the **arm angle**, the elbow's
rotation about the shoulder–wrist axis. `rm_movel` takes a Cartesian target; the
arm angle is free, and **nothing in the Cartesian speed/accel limits bounds its
rate**. Computed with `rm_algo_calculate_arm_angle_from_config_rm75`, unwrapped:

| source | excursion | peak rate | p99 rate |
|---|---|---|---|
| **planned** (`plans/toplid_left_ruckig_pro_only.json`, 1820 wp) | **144.0°** | — | — |
| measured `ls=0.250` | 146° | 478 °/s | 198 °/s |
| measured `ls=0.450` | 145° | 709 °/s | 279 °/s |
| measured `ls=0.800` | 151° | **702 °/s** | **455 °/s** |

**The excursion is ours** — MoveIt/Ruckig planned it; hardware reproduces it at
every speed. **`line_speed` scales its rate and the TCP cap does not see it**: p99
more than doubles while the excursion stays fixed. In `184109` the peak arm-angle
rate lands at **t = 18.56 s** — inside the violent window, alongside the 16.7 A spike.

### 4.4 Where the time goes (H43)

Measured on `20260811T183500` (`ls=0.450`, PASS, 21.37 s, 5.848 m):

| band | time | share of time | share of path |
|---|---|---|---|
| > 90 % cruise | 8.8 s | 41 % | **72 %** |
| 50–90 % | 3.5 s | 17 % | 15 % |
| **< 50 % (dips)** | **9.0 s** | **42 %** | **11 %** |

**42 % of the time covers 11 % of the path.** Splitting dip time by dip *duration*
— first-order timing, no differentiation:

```
28 dips, 7.18 s below half cruise
  dwell below 10 mm/s   0.88 s  (12 %)   command gap; line_acc cannot fix
  ramping               6.30 s  (88 %)   line_acc CAN fix
  implied effective ramp accel: 1.58 m/s^2   against line_acc = 2.4 (66 %)
```

### 4.5 The `line_acc` ladder, and the law (H57)

Eleven runs, both arms. Durations are the **`execute_path` stage**, not the whole
run — an earlier version of this table used `duration_s` (which includes hand, pole,
transit and rest) and made every comparison against the plan wrong by ~7 s.

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

**−15 % (left) / −17 % (right) of stroke time from `line_acc` alone**, at unchanged
`line_speed`. As a fraction of cap the medians go **62.6 → 77.0 → 87.6 %** (left)
and **62.8 → 78.4 → 88.1 %** (right) while p95 moves only 474 → 492 and 477 → 495.

**Correction to an earlier draft**: §4.4's claim that raising `line_acc` would not
push J4 toward the limit **was wrong**. J4 rose 169 → 207 °/s across the ladder,
because higher acceleration lets the arm actually reach cruise on short segments.
Both knobs spend the same J4 budget.

How close are the *completing* runs? Samples in `execute_path` at or above a
fraction of `joint_max_speed`:

| run | `ls/la` | ≥80 % | ≥90 % | ≥95 % | ≥99 % |
|---|---|---|---|---|---|
| left `220322` | 0.45/1.6 | 0 | 0 | 0 | 0 |
| left `220453` | 0.45/2.4 | 14 (0.7 %) | 0 | 0 | 0 |
| left `220617` | 0.45/3.6 | 22 (1.1 %) | 7 (0.3 %) | 1 | 0 |
| right `221720` | 0.45/1.6 | 0 | 0 | 0 | 0 |
| right `221929` | 0.45/2.4 | 5 (0.2 %) | 0 | 0 | 0 |
| right `222049` | 0.45/3.6 | 14 (0.7 %) | 2 (0.1 %) | 0 | 0 |
| right `222354` | 0.50/3.6 | 18 (0.9 %) | 5 (0.3 %) | 2 (0.1 %) | 0 |

**Zero samples above 99 % in any completing run**, and the fastest touches 95 % for
20 ms out of 19.6 s. "We are at the limit" is true of a *single instant*, not of the
stroke: 99.7 % of it runs below 90 %.

The fitted law, validated on both arms independently:

```
J4peak = C + 47.0 * ln(line_acc / 3.6) + 220 * (line_speed - 0.45)

  C = 206.9 (right)    C = 215 (left, runs 8 deg/s hotter)
  ln-slope fitted separately per arm: 47.0 and 46.9   <- independent agreement
  residuals across all four right-arm points: <= 0.5 deg/s
```

| | left | right |
|---|---|---|
| max `line_speed` at `la=3.6` | **0.50** | **0.53** |
| max `line_acc` at `ls=0.45` | **4.5** | **5.3** |

The 0.600 run was **13 % past** the model ceiling of 0.53, and it stalled. **The law
predicted the failure.** And the trade is lopsided — from `0.45/3.6` on the right:

```
ls 0.45 -> 0.50    -0.4 s  (-1 %)    costs J4 +11 deg/s
la 1.6  -> 3.6     -5.8 s  (-17 %)   costs J4 +38 deg/s
```

Per degree of J4 headroom spent, acceleration buys about **4×** the time.

### 4.6 The binding constraint is a fixed point on the path (H60)

The J4 peak, the peak elbow rate and the stall all land in the same place
regardless of speed or acceleration:

```
J4 peak            74.4 - 75.6 % of path   (five runs, 1.6 <= la <= 3.6)
peak elbow rate    75.1 - 77.9 %
the 0.600 stall    78.7 %
```

**A kinematic hot spot at ~75 % of the stroke.** Its *location* is fixed by
geometry; speed and acceleration only scale the joint rate demanded there. Both
knobs are within 10 % of saturating J4 at that one point, so **neither has
meaningful headroom left and fixing the hot spot is the only thing that unlocks
more.** That is the 144–151° excursion of §4.3, now localised.

### 4.7 The joint speed is spent holding orientation (H46, H47)

Numerical 6×7 Jacobian at the binding instant of `20260811T222049`
(`q = [-16.4, -30.5, 25.0, 58.5, -6.0, 27.6, 17.5]`, flange `Ry = 58.1°` — 31.9°
clear of gimbal lock):

* TCP translating at **336 mm/s**, rotating at only **6 °/s**
* min-norm joint rates for the identical 6-DOF twist: **205 °/s**; the controller
  used **207** — **within 1 % of optimal**, so the 7-DOF redundancy has essentially
  nothing left to give here
* the same 336 mm/s with orientation **free**: **70 °/s**
* → **holding tool orientation costs 135 of the 205 °/s, i.e. 66 % of the budget**,
  while the orientation barely changes
* Jacobian singular values 0.0106 / 0.0064 / 0.0025 m per °/s — **4.2× anisotropy**,
  and the stroke direction sits near the worst axis
* per-axis relaxation (rx 194, ry 75, rz 186 °/s) is in **Euler-rate** space, so
  which physical axis carries the cost needs the proper angular-velocity form
  before acting on it. The 205-vs-70 comparison does not depend on that.

⚠ A position-only (3×7) Jacobian says 70 °/s and implies the controller wastes
196 % on null-space motion. **That is an artifact of letting orientation drift** —
with the real 6-DOF constraint the controller is near-optimal. **Do not quote the
3-DOF number as waste.**

Universal across tasks (`src/orientation_cost.py`, offline, each saved plan, stride 20):

| task | median cost | max | at peak demand | peak joint util |
|---|---|---|---|---|
| toplid_left | 1.7× | 3.8× @ 77 % | 1.6× @ 55 % | **83 % J4** |
| toplid_right | 1.6× | 3.5× @ 89 % | 1.7× @ 63 % | **77 % J4** |
| hinge_area_left | 2.1× | **70.4× @ 17 %** | 2.7× @ 56 % | **88 % J3** |
| hinge_area_right | 2.1× | **34.7× @ 17 %** | 2.1× @ 56 % | **90 % J1** |

* **Not a toplid quirk** — orientation costs 39–63 % of the budget at the
  peak-demand point of every task.
* **The binding joint is task-specific** — J4 on toplid, J3 on `hinge_area_left`,
  J1 on `hinge_area_right`. No single weak joint to design around.
* ⚠ **The hinge tasks plan 6–13 points closer to the limit than toplid (88–90 % vs
  77–83 %), and only toplid has ever been speed-tested.** `toplid_right` plans at
  77 % and *executed* at 97 % at `ls=0.500`. **PREDICTION: the hinge tasks saturate
  at a lower `line_speed` than toplid. Start them at 0.250, not toplid's settings.**
* The 70.4× and 34.7× spikes at 17 % of both hinge paths are near-singular
  configurations — locate them before any hinge speed work.

### 4.8 The joint-speed ceiling as a number (H44)

Peak J4 speed against **achieved** p95 TCP — regressing on achieved rather than
commanded cancels the real-time override entirely:

```
p95 mm/s    114   201   278   486   731
J4 peak      63   105   123   195   238
J4 % limit  28%   47%   55%   87%  106%

J4peak = 0.368 x p95_mm_s        R2 = 0.85, n = 5
  -> J4 reaches its 225 limit at p95 ~ 610 mm/s
  -> 183500 sat at 486 = 79 % of it;  184109 at 731 = 120 %, and it failed
```

R² 0.85 on n=5 makes ~610 an estimate, not an edge.

### 4.9 It is DWELL at saturation, not peak, that predicts the failure (H63)

Measured 2026-08-12 directly from `stream.csv` on every REAL run: per sample, the
maximum over joints of `|speed_n| / limit_n` against `[180,180,225,225,225,225,225]`.
Dwell is quantised at the 10 ms sample interval.

| run | `ls` | outcome | peak | ms ≥90 % | ms ≥95 % | **ms ≥98 %** | ms ≥100 % |
|---|---|---|---|---|---|---|---|
| `220322` | 0.45 | completed | 78.7 % | 0 | 0 | **0** | 0 |
| `221929` | 0.45 | completed | 83.7 % | 0 | 0 | **0** | 0 |
| `222049` | 0.45 | completed | 91.9 % | 20 | 0 | **0** | 0 |
| `220617` | 0.45 | completed | 95.6 % | 70 | 10 | **0** | 0 |
| `222354` | 0.50 | completed | 96.9 % | 50 | 20 | **0** | 0 |
| `222451` | 0.60 | **STALLED** | 99.8 % | 130 | 120 | **110** | 0 |
| `184109` | 0.80 | **VIOLENT** | 105.9 % | 420 | 380 | **330** | 70 |

**The ≥98 % band separates every outcome with no overlap**: 0 ms in all six
completing runs, 110 ms in the stall, 330 ms in the violent run — and it is
monotone in severity. Peak alone does not separate nearly as well: 96.9 %
completes and 99.8 % stalls, a 2.9-point gap.

Three things follow.

**The stall never exceeded the limit.** `222451` peaked at 99.8 % and spent **0 ms**
above 100 %. It gave in *below* its ceiling. So "the arm tolerates some overspeed
before tripping" is not what happens, and there is **no evidence of a ~20 % design
margin in amplitude** — nothing in any recording approached 120 %, and the highest
value ever seen (105.9 %) belongs to a run that failed too.

**The tolerance is in TIME, not amplitude.** This is what a threshold detector on a
continuous signal must look like: you cannot trip at exactly 225 °/s, so there is a
dwell before action. Touching 96.9 % for 20 ms is survivable; sitting at ≥98 % for
110 ms is not.

**The ordering is causal, not coincidental.** In `222451` the ≥95 % samples span
t+17.03 → t+17.14 s and motion ceases at **t+17.37 s** — 0.23 s *after* saturation
ends. The recording then holds at zero for a further 15.7 s.

**Usable as a pre-dispatch guardrail**: predict the joint rates for a candidate
plan and require **zero predicted time at ≥98 % of any joint limit**. That is a
sharper and more actionable criterion than the peak-based ceilings of §4.5.

⚠ Caveats: **n = 2 failures**. Dwell is quantised at 10 ms. And faster runs
naturally spend longer near the limit, so dwell and speed are correlated — against
that, the step `ls` 0.50 → 0.60 moved peak by only 2.9 points but dwell by **6×**,
so dwell is the far more sensitive of the two. `speed{n}` is a first-order channel,
which the standing rule at the end of §5 permits.

### 4.10 J4 is predictable offline, and the orientation profile is a lever (H66–H68)

From the `test_motion_001` work of 2026-08-12 (`paths/README.md`), re-verified here
independently with `rm_algo`.

**The mechanism.** `test_motion_001` would not run at 100 %: segments `9→8` and
`8→7` had to be hand-tuned down to 60 and 90. The cause was J4, and it was
predictable *before* touching hardware:

| segment | `rot°` | tool ω at 0.6 m/s | v after the ω cap | outcome |
|---|---|---|---|---|
| **9→8** | **4.75** | **0.23 rad/s** | **0.600 m/s** — nothing throttles it | **J4 far over limit** |
| 8→7 | 25.44 | 1.54 rad/s | 0.234 m/s — capped | J4 at 9 % |

**`9→8` failed *because* it was the smoothest segment.** Every other long segment
demands more than the controller's 0.60 rad/s angular cap, so the controller
time-scales it — and that incidentally keeps its elbow safe. `9→8` asked for only
0.23 rad/s, so nothing slowed it down. **Read a low `rot°` as a warning (H67).**

**J4 is redundancy-invariant — verified independently.** Re-solving `9→8` from 12
seeds with ±35° kicks on J1/J3/J5/J7, 400 interpolation steps:

| point 8 `ry` | avg J4 | **peak J4** | peak/avg | spread over 12 seeds |
|---|---|---|---|---|
| −0.218 (before) | 365.9 °/m | **808.4 °/m** | 2.21× | **0.000 %** |
| −0.400 (after) | 291.3 °/m | **484.5 °/m** | 1.66× | **0.000 %** |

**0.000 % spread from every seed** confirms H66: the redundancy cannot absorb J4,
so it is the one joint an offline tool can pin down without knowing the
controller's own resolution scheme. **The fix lowers J4 demand 20 % on average and
40 % on peak, and it also *flattens* the profile** (peak/avg 2.21 → 1.66) — which
matters more than the average, given H63.

⚠ **Use these figures comparatively, not absolutely.** My average (365.9 °/m)
differs from the 548.1 °/m recorded in `paths/README.md`, and neither converts
cleanly to a measured deg/s: the numbers are sensitive to how orientation is
interpolated between waypoints (linear-Euler here; the controller's own scheme is
not documented). What is robust is the **invariance** and the **ratio between
configurations** — which is all the lever needs.

**Why this matters here.** It is a **third lever, and the only one that does not
fight RealMan's advice**: it changes the *path*, not the controller's TCP limits
(H62). On `test_motion_001` one number bought back 40 % of the wall-clock that
speed tuning could not. §4.6's hot spot at ~75 % of the cleaning stroke is the same
shape of problem — a fixed point where the elbow is asked for too much — so the
same treatment should apply.

### 4.11 How far the offline screen generalises, and how to screen toplid (H69)

**Sweeping the redundancy directly.** `rm_algo_inverse_kinematics_rm75_for_arm_angle`
solves IK at a *specified* arm angle (radians), so the redundancy can be swept
rather than inferred. Peak per-joint demand across the reachable arm-angle range,
on a single segment:

| joint | spread over arm angle | reading |
|---|---|---|
| **J4** | **0.0 %** | invariant — the redundancy cannot move it |
| J6 | 1.5 % | nearly fixed |
| J5, J7 | 8–9 % | partly absorbable |
| J1, J3 | 62 % | absorbable; **diverge near the shoulder singularity**, moving in equal and opposite steps (wrist-over-shoulder self-motion) |
| J2 | 89 % | absorbable |

**So the J4 screen is SOUND but INCOMPLETE.** Sound: if J4 alone exceeds its limit,
the segment fails whatever the controller chooses — no false alarms. Incomplete:
`TASK_KINEMATICS` H49 records **J1 binding on 11 of 24 tasks and J4 on only 4**, so
a J4-only screen would pass the majority of tasks blind. The J4 story is specific
to `toplid`.

**The complete screen is the saved plan.** A plan pins the redundancy, so every
joint is predictable from its own `velocities` array — no IK, no tool frame, no
interpolation model. `plans/toplid_left_ruckig_pro_only.json`, `execute_path`
stage, 1820 waypoints:

| joint | peak | % of limit | at % of path |
|---|---|---|---|
| **J4** | **187.8 °/s** | **83 %** | 55 % |
| J3 | 149.2 | 66 % | 70 % |
| J1 | 118.6 | 66 % | 70 % |

**83 % J4 matches `TASK_KINEMATICS` exactly**, which is the check that this read is
right. Two caveats for anyone using it: the plan puts J4's peak at **55 %** of the
path while hardware measured it at **~75 %** (§4.6) — the controller re-plans the
27 `movel` segments and does not reproduce MoveIt's timing — and the plan predicts
187.8 °/s where hardware measured 215 (×1.14).

⚠ **A wrong turn worth recording.** A first attempt computed per-segment J4 from
the *commanded Cartesian poses* by seeded IK and reported 400–550 % of limit on
segments that completed on hardware. The cause: the commanded poses are TCP poses
in the `L_glove_frame_2` tool frame, and the IK was solved for the **flange** — the
~220 mm glove offset was never applied. `FRAME_MAP.md` warns about exactly this.
**Any Cartesian-side analysis must set the tool frame first**; `orientation_cost.py`
already does it correctly and is the thing to extend, rather than a fresh script.

### 4.12 Screening toplid with the tilt sweep, and why the cheap fix fails (H70)

`orientation_cost.py --segments` (added 2026-08-12) screens the **Cartesian
waypoints a program dispatches**, follows the traversal rather than the
declaration order, and can sweep one point's tilt while re-scoring the whole path.

**It is validated on the case with a known answer.** On `test_motion_001` the sweep
independently recovers **ry = −0.400** as the optimum — the value derived by hand
and confirmed on hardware — and places the superseded −0.218 at ~148 % of the J4
limit against the 146 % computed there by a different route. The sweep also shows
*why* it is an optimum: below −0.400 the binding segment is `9→8`, above it the
binding segment flips to `8→7`. Tilt is being moved between neighbours, not removed
— which is why total rotation across the row was unchanged.

**Applied to toplid, the answer is negative, and usefully so.**

| | `test_motion_001` | `toplid_left` |
|---|---|---|
| shape of the problem | **one** outlier: 4.75° rotation, ω 0.23 rad/s, far under the cap | **six** long segments at 25–30°, ω **0.50–0.65** — clustered on the cap |
| worst segment | 146 % of J4 | 125 % of J4 |
| sweeping the worst point | 146 % → 59 % | 125 % → **124 %** |
| why | one segment carried all the overload | the binding segment just moves to its neighbour |

**toplid's overload is distributed, not localised.** Its long segments all sit
either side of the 0.60 rad/s angular cap, so several fall *just* under it and run
unthrottled at 89–125 % of J4 each. There is no single waypoint to move. It needs
a multi-waypoint redistribution across the row, or the null-space replan of Step 4b.

⚠ **Absolute values over-read by ~1.3×.** Against `20260811T220617` (J4 peak 215 °/s
measured) the tool predicts 282, because it assumes each segment holds `v_eff`
throughout and ignores corner blending. **Rank with it; do not quote its deg/s.**
And it is J4-only (H69), so a pass is not a clearance — J1 is invisible to it.

### 4.13 The angular cap is the elbow's safety valve — do not raise it (H71)

RealMan's ratio rule (`acc >= 3 x speed`) applies to the angular pair as much
as the linear one, so `rm_set_arm_max_angular_speed` can be raised the same way
`line_speed` can. **It should not be.** Modelled with
`orientation_cost.py --screen-all --angular-cap <c>` at `line_speed 0.45`:

**Raising it** — worst J4 as a % of limit:

| task | 0.60 | 0.90 | 1.20 | 1.80 | 2.40 |
|---|---|---|---|---|---|
| toplid_left | 126 % | 126 % | 126 % | 126 % | 126 % |
| hinge_area_left | 114 % | 114 % | 114 % | 114 % | 114 % |
| side_left | 190 % | 255 % | 255 % | 255 % | 255 % |
| seat_ring_left | 235 % | 255 % | 291 % | 291 % | 291 % |
| **bottomlid_left** | **81 %** | 81 % | 100 % | 149 % | **199 %** |

Nothing improves; several get much worse. `bottomlid_left` goes from the safest
task in the set to badly over. Meanwhile throttled segments collapse — toplid
**15/27 → 1/27**. **The cap is doing the protecting** (H67), and raising it
simply removes that protection from the segments that currently depend on it.

**Lowering it** is a genuine safety lever, and it trades against time roughly
linearly:

| task | 0.20 | 0.30 | 0.45 | 0.60 |
|---|---|---|---|---|
| toplid_left | 46 % | 69 % | 103 % | 126 % |
| seat_ring_left | 80 % | 119 % | 179 % | 235 % |
| bottomlid_left | 48 % | 73 % | 81 % | 81 % |
| *est. stroke time, toplid_left* | *47.8 s* | *31.9 s* | *21.4 s* | *16.6 s* |

Halving the cap roughly halves the elbow demand and roughly doubles the stroke.

**Two tasks it cannot save, and they are the informative ones.**
`side_left` sits flat at 162 % from 0.20 through 0.45 — its worst segment barely
rotates, so the angular cap never touches it no matter where it is set. That is
the pure H67 case. `front_left` is still at 239 % even at 0.20. Both need a path
change (§4.10) or a lower `line_speed`; no angular setting reaches them.

⚠ This whole section rests on H67 — that exceeding the angular limit
time-scales the segment — which is **our inference, not vendor-confirmed**. It
is §6 Q1 in [QUESTIONS_FOR_REALMAN.md](QUESTIONS_FOR_REALMAN.md). If the
mechanism is not what we think, these numbers do not hold.

### 4.14 Intermediate waypoints: a no-op alone, and the shape test (H72)

**Question (Newton, 2026-08-13): would inserting intermediate points along the
long segments, interpolating orientation linearly, reduce the elbow demand?**

**No — and it is provable rather than merely measured.** Splitting a segment into
N pieces with linear orientation gives each piece `rot/N` over `L/N`, so ω is
unchanged, the geometric path is unchanged, and the elbow sweeps the same angle
over the same distance. `toplid_left`, every segment subdivided:

| points per segment | waypoints | worst J4 | throttled |
|---|---|---|---|
| original | 28 | **283 °/s (126 %)** | 15/27 |
| 2 | 55 | **283 (126 %)** | 30/54 |
| 4 | 109 | **283 (126 %)** | 62/108 |
| 8 | 217 | **283 (126 %)** | 124/216 |

Identical at every level, and the throttled *fraction* holds at ~55 %. More
points describe the same curve; they do not change it. (They do add corners,
which — if blending is as poor as C19 is testing — costs time for nothing.)

**What subdivision does give you is a handle.** Inserting one midpoint on
toplid's worst segment and varying how much of the turn it carries:

| rotation carried | 0.0 | 0.3 | **0.5 (linear)** | 0.7 | 1.0 |
|---|---|---|---|---|---|
| worst J4 | 124 % | 124 % | **126 %** | 151 % | 190 % |

So the mechanism is real — front-loading the rotation helps, back-loading is
badly worse — but the gain stops at 124 % because the binding segment jumps to
another one. That is H70 again.

**The shape discriminator: which tasks could a waypoint fix save?** Compare a
task's worst segment against its next-worst. `test_motion_001` was
*concentrated* — one outlier, and moving one tilt took it 146 % → 59 %.

| shape | tasks |
|---|---|
| **CONCENTRATED** (one segment, next < 70 %) | `bottomlid_*` 81/79 %, `bowl_inside_front_right` 65 %, `bowl_inside_ring_*` 33 % |
| partly concentrated | `front_right` 399/351, `seat_ring_right` 264/217, `seat_ring_left` 235/200, `hinge_area_*`, `bowl_inside_rim_*` |
| **distributed** (3+ segments within 90 % of worst) | `front_left` 527/527/491, `side_left` 190/190/190, `side_right` 146×3, `toplid_left` 126/124/121, `toplid_right` 109/107/107 |

**The conclusion is unwelcome and worth stating plainly: of the ten tasks that
project OVER the J4 limit, not one is concentrated.** Every concentrated task is
already under 90 %. So the cheap single-waypoint fix — the one that worked so
well on `test_motion_001` — **does not apply to any task that needs it.** The
partly-concentrated ones (`front_right`, both `seat_ring`) might buy 15–20 % from
one edit, which does not reach safe on its own.

That leaves the null-space replan (§5 Step 4b), a lower `line_speed`, or a lower
angular cap (§4.13) as the levers for the over-limit set.

---

## 5. What to run next

### Step 0 — preconditions, every run

- **Pendant real-time speed adjustment at 100 %**, verified by eye in the toolbar
  header (`Speed ——●—— NN%`). **Do not assume it** — the reference arm was found at
  50 % (H65). It is unreadable by the SDK, it multiplies everything (H58), and H59
  is the after-the-fact check: at 100 % the p95 lands *at or slightly above* the cap.
- **Cartesian limits at the factory defaults** unless the run is deliberately
  varying them — `reset_limits.py --side both` reports, `--apply` restores
  0.250 / 1.600 / 0.600 / 4.000 (H64, now confirmed correct).
- REAL only. SIM cannot answer any of this (H42).
- Free space, no commode. **E-stop in hand** — §4.2 says the controller will not
  stop itself.
- **One knob at a time.**
- ⚠ **Align `collision_stage` across both arms first.** The right arm ran the whole
  ladder at 0, the left at 3.

### Step 1 — settle whether `joint_max_speed` protects anything (H41)

**Promoted to first.** One `rm_movej` on a single joint, free space, commanding a
speed that implies > 225 °/s on J4, recorded. Does the controller clamp, fault, or
comply?

*Falsifiable*: comply → it is a planning parameter only, and **every joint speed
bound must be enforced by us before dispatch**. Clamp or fault → H41 is wrong and
the `184109` overspeed needs another explanation.

This now gates everything else: both failure modes are the controller declining to
protect itself, and we should not push further until we know which it is.

### ~~Step 2 — establish the true default limits~~ ✔ DONE (H64)

Closed 2026-08-12 via the pendant's "Default" button on two arms — see §1.6.
Defaults are 0.250 / 1.600 / 0.600 / 4.000, `reset_limits.py --apply` is correct,
and our baseline was clean. **Optional follow-on, no motion**: call
`rm_set_arm_tcp_init` and read all four back, to find out whether it is the API
equivalent of that button. Worth 30 seconds the next time an arm is up.

### Step 3 — vary `v`, the untouched axis

`v` has never been varied in 35 recordings (§3). Three REAL runs at `ls=0.450`,
`la=2.4`, `v ∈ {100, 75, 50}`.

*Falsifiable*: achieved TCP should scale with `v` against the **450 mm/s
constraint** (≈450/338/225 target), per §1.4. If it does not — if it tracks a
joint ceiling instead — our motion model is wrong and `rm_emulator` and
`speed_limits` both need revisiting.

Also record dwell at ≥98 % (H63) at each `v`: if lowering `v` collapses dwell, it
is a safer derating lever than lowering `line_acc`, which RealMan advise against
touching (H62).

### Step 4 — reshape the path at the hot spot (H40, H46, H60, H66–H68)

**The only lever with real headroom left, and the one demonstrated to work.**
Two routes, cheapest first:

**(a) Orientation profile, as on `test_motion_001` (§4.10).** Compute J4 demand
along the stroke offline — exact, because J4 is redundancy-invariant (H66) — find
the ~75 % hot spot, and redistribute tool tilt into the *low-rotation* segments
around it. Flag any segment whose `rot°` implies < 0.60 rad/s: nothing will
time-scale it (H67). Verify offline first; the whole point is that it costs no
hardware time to evaluate.

**(b) Null-space excursion.** Re-plan `toplid_left` with the arm angle constrained
so the 144° excursion is well under it. Needs a replan, so it is blocked on code.

*Falsifiable*: if §4.3's mechanism is right, a plan with a ≤40° excursion runs at
`ls=0.800` with J4 peak current at or below the 7.3 A seen at 0.450, and no
reversal. If current still spikes to ~16 A, §4.3 is wrong.

*Success criterion is H63, not peak*: the reshaped path should show **zero
predicted time at ≥98 %** of any joint limit — and note from §4.10 that peak J4 runs
1.7–2.2× the segment average, so an average-based check will pass a path that
still fails.

### Step 5 — stop latency at raised `line_acc` (H62)

**New, and safety-relevant.** RealMan warns the TCP defaults preserve immediate-stop
behaviour. Measure `rm_set_arm_stop` latency and stopping distance at `la` 1.6 vs
3.6, free space, before any further use of raised acceleration.

### Step 6 — walk `line_speed` to the predicted ceiling

Only after Steps 1 and 5. `ls ∈ {0.50, 0.55}` at the selected `line_acc`. The
regression predicts J4 stays under 225 while p95 stays under ~610 mm/s.

*Falsifiable*: J4 peak within ±30 °/s of `0.368 × p95`. A miss means the linear law
breaks near the limit — stop there.

**Abort criterion, from H63 rather than from peak**: stop the moment any run shows
**non-zero time at ≥98 % of a joint limit.** `222354` at `ls=0.50` recorded 0 ms
and completed; `222451` at 0.60 recorded 110 ms and stalled. There is no amplitude
headroom to trade — the stall happened below 100 %.

### A standing rule for any acceleration number

The four defensible estimators of joint acceleration disagree by 2.6×–16.6× on
existing data, and `speed{n}` is not a clean derivative of `position{n}` (median
12 °/s apart, up to 123 °/s on J4). Either characterise that filter against a known
input, or report only **first-order** quantities — joint speed, joint current, and
event durations — which is how §4.1, §4.3, §4.4 and §4.5 were obtained.

---

## 6. Open with RealMan

Full text in [QUESTIONS_FOR_REALMAN.md](QUESTIONS_FOR_REALMAN.md). Ranked:

1. **The stall (H45) and the missing arrival event** — what internal state, and what
   to poll to detect it. A caller blocked on the callback waits forever. Most
   blocking item we have. **Now sharper**: we can show the arm stalled at 99.8 % of
   a joint limit having never crossed it, after 110 ms at ≥98 % (H63).
2. **Is `joint_max_speed` enforced at execution (H41)** — and if there is a dwell
   or debounce before the controller acts on a joint at its ceiling, what is it?
   H63 says the behaviour is time-dependent; we would like their number.
3. **What are the factory default Cartesian limits (H64)**, and does
   `rm_set_arm_tcp_init` restore them? We cannot follow "use the defaults" without
   this.
4. **Stop latency versus `line_acc` (H62)** — does raising it compromise the
   immediate-stop behaviour they cited, and is there a documented maximum for
   `rm_set_arm_max_line_acc`?
5. **Is the real-time speed adjustment readable/settable via the API (H58)**, and is
   `rm_set_plan_speed` that same quantity?
6. **Which constraint bounds a rotation-dominated `movel` segment** — the linear cap
   or `rm_set_arm_max_angular_speed`? Unanswered by §1.4.
