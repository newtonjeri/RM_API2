# `rm_movel` — a chain that stops with no error. Questions.

**Hardware** RM75-6FB, Gen-3, firmware **V1.7.4**, RM_API2 **1.1.6** (we know this
should be 1.1.4 for V1.7.4 — being corrected). All motion in **free space**, no
contact. Pendant speed slider at **100 %** for every run below.

**Reproduction** `rm_movel_repro.py` (attached, one file):

```
python3 rm_movel_repro.py --ip <arm-ip> --sdk <path-to-RM_API2/Python> --tool <tool-frame>
```

**What it does.** One cleaning stroke — 28 Cartesian waypoints, 5.78 m — as a
**chained `rm_movel`**: every segment `rm_movel(pose, v=100, r=10, connect=1,
block=0)`, the last with `r=0, connect=0`, producing one arrival event via
`rm_get_arm_event_call_back()`. Same tool frame, same waypoints, same start joints
(set by `rm_movej` first, so the 7-DOF redundancy is fixed). **All 27 calls return
0 in every run.** We record joint position, speed, current and `arm_current_status`
at 100 Hz from the UDP push.

| | `line_speed` | `line_acc` | result |
|---|---|---|---|
| **Run A** | 0.50 | 3.6 | completes, 19.6 s |
| **Run B** | 0.60 | 3.6 | **stops mid-path, never resumes** |
| earlier run | 0.80 | 2.4 | **violent motion, event returns failure** |

**Already answered — thank you, not re-asking:** what `v` scales for each command
(`movej` → joint speed, `movel` → TCP speed constraint, both × real-time
adjustment); max `line_speed` 1.8 m/s; `line_acc ≥ 3 × line_speed` is by design.
We also found the factory defaults ourselves via the pendant's "Default" button:
**0.250 / 1.600 / 0.600 / 4.000**.

---

## 1. Run B stops. What state is it in, and what do we poll to see it?

The arm executes **4.47 m of 5.78 m (77 %)** in 11.9 s, then stops and stays
stopped. We waited a further 15.7 s before interrupting. During the stop:

* `rm_get_joint_err_flag()` → **all seven flags 0**
* `rm_get_current_arm_state()` → **`err` empty**
* `arm_current_status` → only **0** (`RM_IDLE_E`) and **1** (`RM_MOVE_L_E`);
  never **9** (`RM_STOP_E`), never **10** (`RM_SLOW_STOP_E`)
* no system error code on any interface we read

**Q: What is the controller's internal state here, and which API call or register
reports it?** We want to detect this, not infer it from a timeout.

---

## 2. No arrival event fires at all. How should we detect an abandoned chain?

In Run B **no event is delivered** — not success, not failure. A caller blocked on
the callback waits forever. The 0.80 run behaved differently: there the event *was*
delivered, carrying `trajectory_state = 0`. So there are two failure behaviours and
only one is visible through the callback.

**Q: Is the missing event expected in this state? What is the recommended way to
determine that a queued `movel` chain has been abandoned** — is there a
queue-depth or trajectory-status query we should poll alongside the callback?

---

## 3. Is the joint speed limit enforced at execution — and is there a dwell?

`rm_get_joint_max_speed` reports `[180,180,225,225,225,225,225]` °/s. We have only
ever read it, never written it. During `rm_movel` we measure speeds above it, with
no clamping and no fault:

| run | measured peak | limit | |
|---|---|---|---|
| Run B | J4 **224.6** °/s | 225 | 99.8 % — and the arm stopped here |
| 0.80 run | J1 **186** °/s | 180 | 103 % |
| 0.80 run | J4 **238** °/s | 225 | 106 % |

The 0.80 run also drew **16.7 A on J4** (6.1 A in Run A) and reversed four joints
within 80 ms, with no error reported.

**The behaviour looks time-dependent.** Taking the maximum over joints of
`|speed| / limit` at each 10 ms sample:

| runs | outcome | peak | ms ≥95 % | **ms ≥98 %** | ms ≥100 % |
|---|---|---|---|---|---|
| six runs, `ls` 0.45–0.50 | all completed | 78.7–96.9 % | 0–20 | **0** | 0 |
| Run B, `ls` 0.60 | **stopped** | 99.8 % | 120 | **110** | **0** |
| `ls` 0.80 | **violent** | 105.9 % | 380 | **330** | 70 |

Note **Run B never exceeded 100 %** — it stopped below its limit, after 110 ms at
≥98 %. A run that touched 96.9 % for 20 ms completed normally.

**Q1: Is the joint speed limit enforced at execution, or only used by the planner?**

**Q2: If the controller acts on a joint near its limit, is there a dwell or
debounce first, and what is it?** If that is a documented value it becomes an
offline check we can run before dispatching.

**Q3: If it is not enforced at execution, what protects the drives** — and should
we validate joint rates ourselves before dispatch? Against peak, or against
sustained time near the limit?

**Q4: Does the controller reject or derate a `movel` whose inverse kinematics
demand a joint rate above the limit, or does it attempt it?** Our evidence says it
attempts it. We can predict elbow (J4) demand offline exactly — on an S-R-S arm the
elbow angle is fixed by the commanded pose, independent of how the redundancy is
resolved — so if you do not check this, we can. We would rather know which of us
is responsible for it.

---

## 4. `line_acc` — is there a maximum, and does raising it affect stopping?

You advise against modifying the TCP values, noting the defaults keep 点击停止能够
立刻停下来. We take that seriously, and ask because our measurements point the other
way: raising `line_acc` from the default **1.6 → 3.6** at unchanged `line_speed
0.45` cut our stroke time by **15–17 %** on both arms. Your warning describes very
slow acceleration *prolonging* the time, which we read as a caution about
**lowering** the values.

**Q1: Does raising `line_acc` degrade the immediate-stop response** — longer
latency, longer stopping distance, or a softer deceleration ramp?

**Q2: Is there a documented maximum for `rm_set_arm_max_line_acc`?** Your ratio
rule gives a lower bound only. The same question applies to
`rm_set_arm_max_angular_speed` / `_acc`.

---

## 5. Reading the speed settings from the API

**Q1: Is the real-time speed adjustment (实时调速) readable or settable through the
API?** We can see and set it on the pendant, but our programs cannot read it — so
no timing measurement is self-describing. Is `rm_set_plan_speed(1-100)`, documented
as 全局规划速度比例, this same quantity? We find no `rm_get_plan_speed`.

**Q2: Is there an API equivalent of the pendant's "Default" button** for the TCP
speed limits? `rm_set_arm_tcp_init` sits in the same interface group, but we would
rather ask than guess at a call that writes global state.

**Q3:** These limits survive power-off — an arm left at `line_speed 0.80` still
read 0.80 hours later. Is that intended? A program that raises them and exits
uncleanly leaves the next operator a faster arm with no indication.

---

## 6. How is the angular limit applied — and does it time-scale the segment?

This has become the most useful thing we could learn, so we will show our working.

We believe the controller **time-scales a whole `movel` segment** when its tool
rotation would exceed `rm_set_arm_max_angular_speed` (0.60 rad/s here). Worked
example from a 10-point path, at a commanded 0.60 m/s:

| segment | tool rotation | implied ω at 0.6 m/s | speed we observe |
|---|---|---|---|
| A | 25.4° over 173 mm | 1.54 rad/s — over the cap | **0.234 m/s** — throttled |
| B | 4.75° over 215 mm | 0.23 rad/s — under the cap | **0.600 m/s** — full speed |

**The consequence is counterintuitive and it cost us a failure.** Segment B, the
*smoothest* one, is the one that broke: nothing throttled it, so it ran at full
speed and drove joint 4 past its limit. Segment A, which rotates five times more,
was automatically slowed and its joints stayed comfortable. We fixed it by moving
one waypoint's tilt to redistribute rotation between A and B — both then ran at
100 %, and the pair got **faster** in wall-clock.

**Q1: Is that correct — does exceeding the angular limit time-scale the entire
segment, reducing its linear speed proportionally?**

**Q2: When both the linear and angular constraints apply, is the segment scaled to
satisfy the more binding one?**

**Q3: Is there any pre-execution check on the resulting joint rates?** A segment
that satisfies both TCP constraints can still demand more than a joint can give —
that is exactly what happened to us, and nothing reported it.

We ask because if Q1 is confirmed, a segment's rotation becomes a **safety
indicator we can compute offline**: a low rotation means nothing will slow that
segment down for us, and we should check its joint demand before running it.

**And it decides whether we may raise the angular limit.** Your ratio rule lets
`rm_set_arm_max_angular_speed` be raised the same way `line_speed` can, and we
would like the time it would buy. But if Q1 is right, that limit is currently the
only thing protecting the elbow on our rotation-heavy segments — modelling
0.60 → 2.40 rad/s across six of our tasks leaves the worst joint-4 demand
unchanged or **worse** (one task goes from 81 % to 199 % of the joint limit) while
the number of throttled segments falls from 15 of 27 to 1 of 27.

**Q4: Is that the right reading — is the angular limit effectively acting as joint
protection during `movel`, such that raising it removes protection rather than
just permitting faster rotation?** If so we will leave it alone, and we would
suggest the manual say so.

---

## Ranked by what blocks us

1. §1 and §2 — the stop, and detecting an abandoned chain. A caller waits forever.
2. §6 — does the angular limit time-scale the segment? Confirming this turns a
   segment's rotation into an offline safety check, which is the cheapest fix
   available to us and needs nothing from the controller.
3. §3 — enforcement at execution, and the dwell.
4. §4 — stopping behaviour at raised `line_acc`, and its maximum.
5. §5.
