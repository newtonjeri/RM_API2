# Commanding TCP speed on `rm_movel`, and a stop with no error — questions

**Hardware** RM75-6FB, Gen-3 controller, firmware **V1.7.4**, RM_API2 C API
**1.1.6**, Python SDK. Motion is in **free space** — no contact with any
object at any point in these runs.

**Reproduction** `rm_movel_repro.py`, attached. One file, no dependencies
beyond the SDK.

```
python3 rm_movel_repro.py --ip <arm-ip> --sdk <path-to-RM_API2/Python> --tool <tool-frame>
```

## Why we are asking

We are trying to establish **how to command a predictable TCP speed on a
chained `rm_movel`**. The tool never reaches the speed we ask for, and the
shortfall grows the more we ask — so we raised the limits to find where the
real ceiling is. That investigation is what produced the failure in questions
2 and 3: at one setting the arm stops mid-path and reports nothing at all.

So question 1 is the one we actually need answered; the rest are what we hit
on the way.

## What the script does

It executes a **cleaning stroke** — one continuous tool path of 28 Cartesian
waypoints, 5.78 m long, that wipes a surface — as a **chained `rm_movel`**:
every segment but the last is queued with `trajectory_connect=1`, and the
final segment is sent with `trajectory_connect=0`, which plans and executes
the whole chain and produces a single completion event. We refer to that
event below as the **arrival event**; it is what
`rm_get_arm_event_call_back()` delivers.

The script runs that identical path **twice**, changing **only**
`rm_set_arm_max_line_speed`:

| | `line_speed` | `line_acc` | result |
|---|---|---|---|
| **Run A** | 0.50 m/s | 3.6 m/s² | completes, 19.6 s |
| **Run B** | 0.60 m/s | 3.6 m/s² | **stops mid-path and never resumes** |

Everything else is identical in both runs: same tool frame, same 28
waypoints, and the same starting joint configuration — reached with
`rm_movej` before the stroke so that the arm's **redundancy** is fixed. (The
RM75 has seven joints but a Cartesian pose needs only six, so one degree of
freedom is left over — the elbow can swing while the tool stays put. Starting
from a commanded joint configuration removes that variable between runs.)
Every segment is `rm_movel(pose, v=100, r=10, connect=1, block=0)`, with
`r=0, connect=0` on the last. **All 27 calls return 0 in both runs.**

We record joint position, joint speed, joint current and `arm_current_status`
at 100 Hz from the controller's **UDP realtime state push** for every run
quoted below.

---

## 1. What does `v` in `rm_movel` scale, and can the global speed override be read?

**What we observe.** We call `rm_movel(pose, v=100, ...)` after setting
`rm_set_arm_max_line_speed`. The TCP speed we measure is consistently well
below the **cap** — the value we passed to `rm_set_arm_max_line_speed`.

Two figures are quoted, both computed from the recorded tool position over
the stroke. We difference consecutive tool positions to get a speed for each
10 ms sample, smooth over ±2 samples (50 ms), and discard samples below 10 %
of the cap so that corner stops and dwell do not drag the figure down. Of
what remains:

* **median** — the middle value, i.e. the speed the tool holds for half the
  time it is actually moving
* **95th percentile (p95)** — the value exceeded by only 5 % of samples,
  i.e. how fast it gets at its fastest, without being a single noisy sample

| commanded cap | median achieved | 95th percentile | median as fraction of cap |
|---|---|---|---|
| 450 mm/s | 355 mm/s | 486 mm/s | 79 % |
| 800 mm/s | 440 mm/s | 731 mm/s | 55 % |

Note the shortfall **worsens** as the cap rises — so it is not a fixed
scaling factor.

We understand the teach pendant carries a **global speed override** — a
slider that scales all motion the controller executes — and we have found no
API to read or set it, so we cannot tell whether the shortfall is that
slider, the segment geometry, or something else.

**Our questions.**

* Is `v` a percentage of `rm_set_arm_max_line_speed`, or of something else?
* `rm_movel` derives both linear and angular motion from the single `v`.
  When a segment is limited by its **rotation** rather than its translation,
  does `v` still scale against the linear cap, or against
  `rm_set_arm_max_angular_speed`?
* **Is the pendant's global speed override readable or settable through the
  API?** Without it we cannot make any timing measurement reproducible, and
  we cannot tell a controller limit from a slider position.

---

## 2. What is the stopped state in Run B, and what should we poll to see it?

**What we observe.** The arm executes **4.47 m of the 5.78 m** path (77 %) in
**11.9 s**, then stops completely and stays stopped. We left it a further
**15.7 s** before interrupting. During the stop:

* `rm_get_joint_err_flag()` → **all seven flags 0**
* `rm_get_current_arm_state()` → **`err` empty**
* `arm_current_status` (UDP push) → only **0 (`RM_IDLE_E`)** and
  **1 (`RM_MOVE_L_E`)**; never **9 (`RM_STOP_E`)**, never
  **10 (`RM_SLOW_STOP_E`)**
* no controller system error code appears on any interface we read

The last motion before the stop, sampled every 10 ms, shows **joint 4 at
224.6 °/s** against the **225 °/s** that `rm_get_joint_max_speed` reports for
that joint — 99.8 % of it.

**Our question.** What is the controller's internal state here, and **which
API call or register reports it?** We would like to detect this condition
rather than infer it from a timeout.

---

## 3. Why does the arrival event never fire, and how should we detect an abandoned chain?

**What we observe.** We register `rm_get_arm_event_call_back()` and wait for
the single arrival event produced by the closing segment. In Run B **no event
of any kind is delivered** — not success, not failure. A caller blocked on
that callback waits indefinitely.

This differs from another failure we recorded on the same path at
`line_speed 0.80`, where the event **was** delivered, carrying
`trajectory_state = 0` (failure). So the controller has two distinct
behaviours on this path, and only one of them is visible through the
callback.

**Our question.** Is the missing event expected in this state? **What is the
recommended way for an application to determine that a queued `movel` chain
has been abandoned** — is there a queue-depth or trajectory-status query we
should poll alongside the callback?

---

## 4. Does the joint speed limit constrain execution, or only planning?

**Note on scope.** We have **only ever read** these values, with
`rm_get_joint_max_speed`. **We have never called
`rm_set_joint_max_speed`** — the limits below are as the arms shipped. The
question is therefore about the parameter itself: what the value written by
`rm_set_joint_max_speed` and reported by `rm_get_joint_max_speed` is
understood to govern.

**What we observe.** Both of our arms report:

```
rm_get_joint_max_speed  = [180, 180, 225, 225, 225, 225, 225]   °/s
rm_get_joint_max_acc    = [600, 600, 600, 600, 600, 600, 600]   °/s²
```

and `rm_get_joint_drive_speed` / `rm_get_joint_drive_acc` return **identical
values**.

During `rm_movel` we measure joint speeds **above** those figures, with no
clamping and no fault. Each figure below is the largest single sample over
the stroke, from the 100 Hz UDP push:

| run | `line_speed` | measured peak | reported limit |
|---|---|---|---|
| Run B | 0.60 | **J4 224.6 °/s** | 225 |
| other arm | 0.80 | **J1 186 °/s** | 180 |
| other arm | 0.80 | **J4 238 °/s** | 225 |

The 0.80 run also drew **16.7 A on joint 4** (against 6.1 A in Run A) and
reversed the direction of four joints within 80 ms. It reported no joint
error and never entered `RM_STOP_E`.

**Our questions.**

* Is the joint speed limit **enforced at execution**, or is it a parameter
  the planner consults when generating a trajectory?
* If it is not enforced at execution, **what protects the drives against an
  overspeed**, and should we be validating joint rates ourselves before
  dispatching a `movel`?
* Would calling `rm_set_joint_max_speed` with a **lower** value change the
  behaviour in Run B — i.e. would the controller then refuse or derate the
  trajectory rather than stop silently?

---

## 5. What are the maxima for `rm_set_arm_max_line_speed` and `rm_set_arm_max_line_acc`?

**What we observe.** The only constraint we have found the controller
enforce is the ratio **`line_acc / line_speed ≥ 3`** — a pair violating it is
rejected with `ret=1`. Beyond that, both setters accept values well above the
shipped defaults (0.25 m/s and 1.6 m/s²) and return 0. We have run
`line_acc` up to 3.6 m/s² with no rejection.

Raising `line_acc` from 1.6 to 3.6 m/s² at a fixed `line_speed` of 0.45 m/s
shortened the stroke by 17 % (24.2 s → 20.1 s), so it is a useful control and
we would like to know how far it may legitimately be taken.

**Our questions.**

* Is there a documented maximum for `rm_set_arm_max_line_acc`, and for
  `rm_set_arm_max_line_speed`?
* Do these settings interact with the joint speed and acceleration limits, or
  are the Cartesian and joint limits independent?
* Are these values **persistent controller state**? We observe them surviving
  between separate program runs — an arm left at `line_speed 0.80` still read
  0.80 hours later — which means a program that raises them and exits
  uncleanly leaves the arm faster than the next operator expects. Is that
  intended, and is there a documented way to restore the shipped defaults?

---

## Summary of what would help most

1. **What `v` scales, and whether the global speed override is reachable
   from the API** — this is the question we set out to answer.
2. A way to **detect the Run B state** from the API instead of a timeout.
3. Confirmation of whether the **joint speed limit is enforced at
   execution**.
4. The **maxima** for the Cartesian speed and acceleration setters.
