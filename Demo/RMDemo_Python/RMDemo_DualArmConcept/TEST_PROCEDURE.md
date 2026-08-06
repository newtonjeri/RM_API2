# Dual-Arm Concept Test Procedure (Concept #1)

**Hardware**: 2 × RealMan RM75-6FB (Gen-3 controllers, V1.7.1) + 2 × pole lifts, Butterfli dual-arm setup
**API**: RM_API2 Python SDK 1.1.6 (repo `Python/` package, no vendored copy)
**Scope**: Concept validation of dual-arm execution semantics — **parallel locked**, **chained**, and **free** — plus the **arm–pole synchronization feature** (duration-matched concurrent arm+lift motion, the butterfli_hw `bench_sync` contract at command level) and **Inspire dexterous hand** steps. Uses the SRDF `rest_pose`/`ready`/`zero_pose` arm states, `pole_*` lift states, and `inspire_hand_*` hand states, executed with controller-planned moves (`rm_movej`, `rm_set_lift_height`, `rm_set_hand_angle`), arrival-event completion (devices 0/2/3), and per-process event demuxing across two handles.

---

## 1. Prerequisites

### 1.1 Hardware

All endpoints are **environment-overridable** (defaults below are the network-verified butterfli setup) — no code edits needed when addresses change:

| Item | Default | Override |
|---|---|---|
| Left arm IP | 192.168.1.10 | `RM_LEFT_IP` |
| Right arm IP | 192.168.1.103 (re-addressed 2026-08-05; was .11) | `RM_RIGHT_IP` |
| TCP port | 8080 | `RM_ROBOT_PORT` |
| Host IP (UDP push target, C5) | 192.168.1.239 (lab laptop; robot main host is .235) | `RM_HOST_IP` |
| UDP push port (C5) | 8095 | `RM_UDP_PORT` |

Example: `RM_LEFT_IP=192.168.2.10 RM_RIGHT_IP=192.168.2.11 RM_HOST_IP=192.168.2.50 python3 test_dual_connect.py`. The emulator honours the same variables, so emulated runs stay consistent with any addressing.

| Item | Value |
|---|---|
| Lift stroke (physical) | 0–0.3 m |
| Lift safe range (SRDF states) | 0.01–0.29 m → 7–193 hw-mm |
| Lift gearing — LEFT (V1.7.4) | **1:1, true mm, travel 0–330** (physically confirmed 2026-08-06: commanding 193 under the old 2/3 assumption left the pole mid-rail). `full_length` 0.29 m → **290** | env `RM_LEFT_LIFT_GEAR` (`1to1`/`2to3`) |
| Lift gearing — RIGHT (V1.7.1) | 2:3 geared, hw 0–200 (`hw = m × 2000/3`). `full_length` 0.29 m → **193**. **Flip to `1to1` via `RM_RIGHT_LIFT_GEAR` after its upgrade** | env `RM_RIGHT_LIFT_GEAR` |
| Controller-side lift ceiling | Left (V1.7.4): Min 0 / Max **330** true mm — `full_length` command 290 stays inside it. Right (V1.7.1): hw max 200. `lift_hw_mm()` hard-asserts the per-side ceiling. Motor params observed on left: 1250 rpm / 5000 rpm/s / RR 0.005 / joint ID 7 — the V1.7.1-measured speed map (1.85 phys mm/s per %) and start latencies should be revalidated on the upgraded arm before trusting sync-finish numbers. |
| Arm speed used | 20 % (`rm_movej` v) |
| Lift speed used | 50 % standalone; **duration-matched** on sync steps (see §1.5) |
| Hand | Inspire RH56DFX-2L on each arm, protocol path `rm_set_hand_angle`, values 0–1000 (1000 = open), SDK order [little, ring, middle, index, thumb_flex, thumb_rot] |

### 1.2 Software

- Python 3, this repo checked out; scripts run from `src/` (`cd src && python3 <script>.py`).
- SDK imported from `RM_API2/Python` via `parents[4]` — no per-demo vendored SDK.
- Thread mode: `RM_TRIPLE_MODE_E` (required for arrival-event callbacks). One `rm_init` per process; the second `RoboticArm()` is constructed with no mode to skip re-init.

### 1.3 Joint states (provenance: `butterfli_moveit_config/config/butterfli_alix.srdf`)

Values stored in **radians verbatim** from the SRDF; converted to degrees at dispatch (plain rad→deg — butterfli_hw confirms no sign flips or offsets between URDF and controller conventions).

| State | Right arm (rad) | Left arm (rad) |
|---|---|---|
| `zero_pose` | all 0.0 | all 0.0 except **J7 = 3.1416** |
| `ready` | 0, −1.885, 0, 1.798, 0, 1.379, 0 | 0, −1.885, 0, 1.798, 0, 1.379, **3.1416** |
| `rest_pose` | 0, −1.431, −0.4538, 1.7103, 0.1047, 1.0821, 1.2043 | 0, −1.431, **+0.4538**, 1.7103, **−0.1047**, 1.0821, 1.9373 |
| Lift `half_length` | 0.15 m → **100** (2:3) | left: **150** (1:1) |
| Lift `full_length` | 0.29 m → **193** (2:3) | left: **290** (1:1) |

### 1.4 Concept sequence (both arms, per run)

**Every motion run first pre-positions the pole(s) to `full_length` (0.29 m — left 290 @ 1:1, right 193 @ 2:3)** — a deterministic start state with maximum clearance — dispatched concurrently after the countdown and arrival-verified; a failed homing aborts the run with all arms halted. Then:

`arm→ready` → `hand→release` → `sync(arm→zero + pole→half)` → `hand→grasp` → `sync(arm→ready + pole→full)` → `hand→half_grasp` → `arm→rest`

First step doubles as homing from any start pose. Hand states come from the SRDF `inspire_hand_*` groups via butterfli_hw's `hand_rad_to_hw` (cross-checked against the bench §3.8 ANGLE_SET echo: grasp = `33/33/33/33/133/944`). The sequence is collision-free by construction; collision gating for arbitrary free-running tasks is future work.

### 1.5 Arm–pole synchronization (the feature under test)

Sync steps port the butterfli_hw sync contract (`TECHNICAL_INTERFACE.md` §"SYNCHRONIZATION", `bench_sync`) to the RM_API2 command level: both devices are dispatched back-to-back, and the lift speed is **duration-matched** to the arm move — `v = distance / (arm_duration − 0.38 s start latency)`, `pct = ceil(v / 1.85)`, floored at 4 % — with ROUND-UP quantization (early finish is benign, the device waits; late is the failure mode). Per step the test reports dispatch start skew and joint-vs-lift finish skew, mirroring `bench_sync`'s `pa_start`/`pa_finish`.

**Hand dispatch method (hardware-validated 2026-08-06, butterfli_hw
`acked_angle` semantics)**: both hand paths WORK on both arms (protocol and
modbus RTU — C7). Two constraints shape the design: **device-2 arrival
events never reach the user event callback** (arm/lift events do), and
**blocking `rm_set_hand_angle` fails with −4 when an arm move is in flight**
(it consumes the arm's arrival push — observed in C6). All hand steps
therefore use the butterfli-proven **non-blocking send + duration-based
completion**, with the dwell sized by the measured stroke law
(0.115 s + 373·span/SPEED_SET, ×1.5 margin, capped by `RM_HAND_DWELL_S`;
no-op sends dwell ~0.15 s). Feedback is echo — modbus + ANGLE_ACT
(`test_hand_only.py`) remains the path with true measured feedback.
**`--no-hands`** on any motion test strips all hand parts at runtime.

**⚠ Hand/modbus exclusivity (fw 1.7.x)**: `rm_set_hand_angle` is the hand *protocol* path and is mutually exclusive with end-port modbus mode — it returns −5 and degrades the modbus session if the port is in modbus mode. Do **not** run these tests while the butterfli_hw ALL-MODBUS stack is attached; if the end port was left in modbus mode, call `rm_close_modbus_mode(1)` first.

---

## 2. File Structure

```
src/
├── log_utils.py           stdout/stderr tee logger (same as RMDemo_LiftBenchmark)
├── dual_arm_common.py     constants, SRDF states, conversions, ArrivalMonitor,
│                          ConceptArm, connect/teardown, run_locked/chained/free
├── rm_emulator.py         in-process SDK emulator of both arms (see EMULATOR.md)
├── run_dry_run.py         offline logic verification with mock robots (run first)
├── run_emulated_suite.py  full C1–C4 suite against the emulator (no hardware)
├── test_dual_connect.py   C1 — connectivity + state pre-check (NO MOTION)
├── test_pole_only.py      C8 — pole diagnostic: state dump + lift acceptance
│                          probe + recovery (POLE ONLY; read-only with
│                          --diagnose-only)
├── test_hand_only.py      C7 — hand-alone: protocol probe + modbus RTU
│                          with measured ANGLE_ACT feedback (HAND ONLY)
├── test_sim_motion_visibility.py  C5 — sim-mode motion visibility probe
│                          (NO PHYSICAL MOTION — moves only the simulated arm)
├── test_dual_locked.py    C2 — parallel locked mode        (MOVES BOTH ARMS)
├── test_dual_chained.py   C3 — chained mode, left leads    (MOVES BOTH ARMS)
└── test_dual_free.py      C4 — free-running mode           (MOVES BOTH ARMS)
```

---

## 3. Running the Offline Verification (No Hardware)

```bash
cd src
python3 run_dry_run.py          # logic checks with instant mocks
python3 run_emulated_suite.py   # the four test scripts, unmodified, against
                                # the emulator (see EMULATOR.md), ~40 s
```

Expected: `68/68 passed`, then every suite entry `exit 0 (OK)` — including
the **C8 locked-pole drill**, which injects the 2026-08-06 lift-rejection
fault on the emulated left arm, requires C8 to FAIL with the diagnosis,
then requires a `--clear-errors` run to recover and go green. It checks: rad→deg values against the SRDF, lift m→hw-mm mapping and range guard, sequence integrity, `ArrivalMonitor` demux (wrong handle ignored, `trajectory_connect=1` non-completion, failure reporting), locked-mode barrier invariant and partner-stop, chained ordering + pipelining, free-mode completion + partner-stop, and the endpoint-configuration plumbing (defaults, and `RM_*` env overrides reaching `dual_arm_common`, C5, and the emulator — probed in clean-environment subprocesses).

---

## 4. Individual Test Cases

### C1 — `test_dual_connect.py` (run first; **no motion**)

**Purpose**: validate the two-handle single-process topology before any motion.
**Duration**: ~5 s.
**Steps**: connect both arms → per arm: robot info, arm error state, lift state, run mode → distinct handle IDs → register the process-global event callback.

| ID | Check | Pass condition |
|---|---|---|
| DC1/DC2 | Robot info (L/R) | ret 0, RM_75, 7-DOF |
| DC3/DC4 | Arm state clean (L/R) | ret 0, no NONZERO error codes (fw pads `err_len=1`, code `'0'` on a clean arm — observed 2026-08-06) |
| DC5/DC6 | Lift state (L/R) | ret 0, `err_flag` 0, pos ∈ [0, 200] hw-mm |
| DC7 | Handles distinct | left id ≠ right id |
| DC8 | Event callback registered | no exception |
| DC9 | Completion | no motion commanded |

SKIP: all 9 checks skip (exit 0) if either arm is unreachable.

### C5 — `test_sim_motion_visibility.py` (**no physical motion** — sim-mode probe)

**Purpose**: answers the open question — with the arm in SIMULATION mode (the
Web GUI sim toggle), is the simulated motion programmatically accessible?
The controller provably runs its planner in sim mode (fence/self-collision
are sim-only, the pendant animates the 3D model), but no doc states whether
the UDP push, arrival events, or TCP polling carry the simulated states.
**Steps**: engage sim mode and VERIFY by readback (aborts before any
dispatch otherwise) → configure UDP push to this host → dispatch J7 +5° at
v=10 → record UDP frames, TCP polls, and the arrival event → print the
three-channel VERDICT → return J7 → restore the original run mode.

| ID | Check | Pass condition |
|---|---|---|
| SM1 | Sim mode engaged | readback confirms mode 0 before any motion |
| SM2 | UDP push delivering | ≥1 frame within 2 s — **silent UDP = FAIL-TO-RUN by default** (wrong HOST_IP is accepted by the controller and delivers nothing; override with `RM_ALLOW_NO_UDP=1`) |
| SM3 | Probe dispatched | movej accepted in sim mode |
| SM4 | Definitive verdict | all three channels answered YES/NO |
| SM5 | Cleanup | J7 returned, original run mode restored |

The verdict itself (YES/NO per channel) is the *finding*, not a pass/fail:
if UDP or TCP sweeps, the sim-rehearsal record-and-FCL-verify pipeline is
viable; if only events fire, it is not. Note: under the emulator sim mode
behaves identically to real (documented limitation), so C5 answers YES
trivially there — the hardware run is the authoritative answer.

### C8 — `test_pole_only.py` (**only the pole moves** — no arm, no hand; read-only with `--diagnose-only`)

**Purpose**: diagnose and recover the **lift-rejection state** first seen
2026-08-06 20:38 — both controllers suddenly rejected every
`rm_set_lift_height` / `rm_set_lift_speed` with ret=1
(`[rm_set_lift_height] set_state: false`) although the identical commands
had physically worked minutes earlier (left 290 @ 20:15, right 193 @
19:59). ret=1 means "controller returned false: parameter error **or
arm-state error**" — with proven-good parameters, that is an arm-state
error. The test reads first (power, controller/joint errors, raw lift
state) and moves second (zero-distance acceptance probe → 10 hw-mm stroke
→ home to full length), so the check that FAILs names the blocking
condition. `RM_ARM=left|right` selects the pole; the lift does **not**
execute in SIM — run `--mode REAL`.

| ID | Check | Pass condition |
|---|---|---|
| D1 | Connected | handle valid |
| D2 | Arm power | ON (OFF ⇒ e-stop chain / power the arm on) |
| D3 | Controller/joint errors | clean (latched codes FAIL unless `--clear-errors` is clearing them) |
| D4 | Lift state | readable; driver `err_flag` 0 (else stall/overcurrent latched) |
| D5 | `rm_clear_system_err` | ret 0 (SKIP unless `--clear-errors`) |
| D6 | Acceptance probe | command CURRENT pos (no motion) → ret 0 + arrival event |
| D7 | Small stroke | 10 hw-mm away and back, both events |
| D8 | Home to full length | the standard pre-run state reached |

On D6 failure the `[DIAG]` state dump prints automatically (also printed
by every motion test when pole homing is rejected with ret=1). Recovery
ladder: release/reset the physical e-stop → `--clear-errors` → Web GUI
lift panel → power cycle.

### C7 — `test_hand_only.py` (**only the hand moves** — no arm, no pole)

**Purpose**: determine which hand-control path works and exercise it with
REAL measured feedback. Phase P probes the PROTOCOL path (one blocking
`rm_set_hand_angle`): ret 0 = end port free, arrival events usable by
C2/C6; **ret −5 = end port in MODBUS mode (the butterfli_hw ALL-MODBUS
state) — the exact C6 hand-failure signature**. Phase M then opens modbus
RTU (`rm_set_modbus_mode(1, 115200)`), writes SPEED_SET, drives
release→grasp→half_grasp→release via ANGLE_SET register writes and
verifies each with polled **ANGLE_ACT** reads — the only genuine measured
hand feedback on fw 1.7.x. Probe order is safety-critical (protocol first;
a failed protocol call DURING a modbus session degrades the session). On
exit the port is restored to protocol mode unless `RM_KEEP_MODBUS=1`.
Registers (butterfli_hw map): ANGLE_SET 1486, FORCE_SET 1498, SPEED_SET
1522, ANGLE_ACT 1546, FORCE_ACT 1582; device `RM_HAND_MODBUS_DEVICE`
(default 1). `RM_ARM=left|right` selects the hand.

| ID | Check | Pass condition |
|---|---|---|
| HB1 | Connected | handle valid |
| HB2 | Protocol probe | verdict resolved (WORKS / BLOCKED −5 / other) |
| HB3 | Modbus session | `rm_set_modbus_mode` ret 0 |
| HB4 | ANGLE_ACT readable | 6 measured values |
| HB5 | SPEED_SET written | ret 0 |
| HB6 | Command + feedback | all 4 states verified via ANGLE_ACT (stopped-short-on-obstacle counts as grasp success) |
| HB7 | FORCE_ACT readable | informational |
| HB8 | Port state | restored (or kept per `RM_KEEP_MODBUS=1`) |

### C2 — `test_dual_locked.py` (**moves both arms and both poles**)

**Purpose**: parallel locked semantics — per step, dispatch both arms back-to-back non-blocking, then barrier on **both** arrivals before the next step. Measures dispatch skew and per-arm completion delta at every boundary.
**Duration**: ~1–2 min at 20 %/50 % speeds.
**Steps**: connect → register monitor → 5 s countdown → `run_locked` over the sequence → report.

| ID | Check | Pass condition |
|---|---|---|
| PL1 | All dispatches accepted | every `rm_movej`/`rm_set_lift_height` ret 0 |
| PL2 | Sequence completed | all 7 steps, barrier honored, both arms ok |
| PL3 | Dispatch skew | max < 50 ms (baseline: single call ≈ 8 ms) |
| PL4 | Arrivals confirmed | arm/lift via events; **hand via acked_angle** (non-blocking + stroke-law dwell — device-2 events are not delivered to the user callback, and blocking fails with −4 under concurrent motion) |
| PL5 | Arm–pole sync finish | pole never finishes > 0.5 s LATE vs the arm (early is benign) |

On any failure: partner arm is halted (`rm_set_arm_stop` + `rm_set_lift_speed(0)`), test fails.

### C3 — `test_dual_chained.py` (**moves both arms and both poles**)

**Purpose**: chained semantics — left leads; right dispatches step k only after left completes step k. Leader advances freely (pipelined: follower k overlaps leader k+1). The follower performs the *following* task; it is not a synchronized copy.
**Duration**: ~2–3 min.

| ID | Check | Pass condition |
|---|---|---|
| CH1 | All dispatches accepted | ret 0 everywhere |
| CH2 | Both chains completed | all 7 leader + 7 follower steps ok |
| CH3 | Ordering invariant | follower dispatch(k) ≥ leader done(k), all k |
| CH4 | Follower gate latency | max < 1.0 s from gate-open to dispatch |

### C6 — `test_single_arm_planned.py` (**moves one arm, its pole, and its hand**)

**Purpose**: single-arm control through the controller's PLANNED functions
only (no passthrough), with the Inspire hand commanded **CONCURRENTLY with
every arm motion** (dispatched back-to-back via the `combo` step kind, both
arrivals awaited): `ready`+`release` → `rest_pose`+`grasp` → **+20 cm X via
`rm_movej_p`**+`half_grasp` → `ready`+`release`. Verifies the Cartesian
displacement from the controller's own pose feedback and reports per-phase
arm/hand durations plus the **hand-vs-arm finish skew** (negative = hand
finished during arm motion — measured from true event-arrival timestamps).
Arm selection: `RM_ARM=left` (default) or `RM_ARM=right`. The hand caveat
of §1.5 applies (end port must not be in modbus mode); in SIM mode the
lift and hand do not simulate, so this test is meaningful on REAL.

| ID | Check | Pass condition |
|---|---|---|
| SA0 | Pole pre-positioned | full_length reached (per-side gearing) |
| SA1 | movej ready + hand release | both arrivals, ok |
| SA2 | movej rest_pose + hand grasp | both arrivals, ok |
| SA3 | movej_p +X + half_grasp | `dx = +0.20 ± 0.02 m`, `|dy|,|dz| ≤ 0.03 m`, hand ok (movej_p ret 1 ⇒ IK/unreachable, fails cleanly) |
| SA4 | movej back to ready + release | both arrivals, ok |
| SA5 | Planned pipeline | movej ×3 + movej_p ×1 + hand ×4, no passthrough |
| SA6 | Hand concurrency | every phase yields both arrivals; skews reported |

All motion tests now print each arm's **run mode** before the countdown and
WARN loudly on SIMULATION (dispatches succeed and events fire in sim, but
nothing physical moves — root cause of the 2026-08-06 "no motion" run).
PL4 was also corrected: it now FAILS when a dispatched device never
delivers an arrival event (the hand has no position fallback).

### C4 — `test_dual_free.py` (**moves both arms and both poles**)

**Purpose**: free execution — both arms run the sequence independently, no cross-arm gates. Valid only because this sequence is collision-free by construction.
**Duration**: ~1–2 min.

| ID | Check | Pass condition |
|---|---|---|
| FR1 | All dispatches accepted | ret 0 everywhere |
| FR2 | Independent completion | both arms finish all 7 steps |
| FR3 | Concurrent free run | execution-window overlap ≥ 50 % of the shorter run |

---

## 5. Running the Full Suite

Every script accepts **`-h`/`--help`** (prints its documentation and the shared usage/env reference, exits without touching the arms) and **rejects unknown arguments with exit 2** — a silently ignored typo would otherwise start a motion run. Every motion test (C2/C3/C4/C6) additionally accepts **`--mode SIM|REAL`**: the requested
mode is engaged on the arm(s) and VERIFIED by readback before any dispatch
(refusal aborts the run before motion), and the pre-run mode is restored on
exit. `--mode SIM` runs the full test virtually on the real controller (a
rehearsal — dispatches, planning, and events are real; nothing physical
moves). Without the flag the test runs in whatever mode the arms are in,
warning loudly on SIMULATION.

```bash
cd src
python3 test_dual_locked.py --mode SIM      # controller-side rehearsal
python3 test_dual_locked.py --mode REAL     # physical run
RM_ARM=right python3 test_single_arm_planned.py --mode REAL
```

```bash
cd src
python3 run_dry_run.py           # offline, must pass 33/33 first
python3 run_emulated_suite.py    # offline, full suite on the emulator
python3 test_dual_connect.py     # no motion — validates topology
python3 test_sim_motion_visibility.py  # no physical motion — sim-mode probe
python3 test_hand_only.py        # hand only — resolves the hand-path question
python3 test_dual_locked.py   # motion — barrier semantics
python3 test_dual_chained.py  # motion — pipeline semantics
python3 test_dual_free.py     # motion — independent semantics
```

Recommended order rationale: dry run proves the logic, C1 proves the plumbing, C2 is the most conservative motion mode (step barriers bound divergence), C3 and C4 progressively relax coupling. Total hardware runtime ≈ 6–10 min. Hand steps require the end port NOT in modbus mode (§1.5).

**Before any motion run**: clear space around both arms, both poles free to travel 0.01–0.29 m, e-stop within reach. Each motion test prints a banner and a 5-second countdown before the first dispatch.

---

## 6. Expected Summary Results

| Test | PASS | FAIL | SKIP | Notes |
|---|---|---|---|---|
| run_dry_run | 68 | 0 | 0 | offline |
| test_dual_connect | 9 | 0 | 0 | 9 SKIP if arms off |
| test_pole_only | 7+1 SKIP | 0 | 1 | D5 SKIPs without `--clear-errors` |
| test_sim_motion_visibility | 5 | 0 | 0 | verdict lines are the finding |
| test_hand_only | 8 | 0 | 0 | HB2 verdict is the finding |
| test_dual_locked | 6 | 0 | 0 | incl. pole pre-position + PL5 sync-finish |
| test_dual_chained | 5 | 0 | 0 | incl. pole pre-position |
| test_dual_free | 4 | 0 | 0 | incl. pole pre-position |
| test_single_arm_planned | 7 | 0 | 0 | one arm + pole + hand |
| **Total** | **119** | **0** | **1** | |

---

## 7. Log Files

| Script | Log file |
|---|---|
| run_dry_run.py | `src/run_dry_run.log` |
| run_emulated_suite.py | `src/run_emulated_suite.log` |
| test_dual_connect.py | `src/test_dual_connect.log` |
| test_pole_only.py | `src/test_pole_only.log` |
| test_hand_only.py | `src/test_hand_only.log` |
| test_sim_motion_visibility.py | `src/test_sim_motion_visibility.log` |
| test_dual_locked.py | `src/test_dual_locked.log` |
| test_dual_chained.py | `src/test_dual_chained.log` |
| test_dual_free.py | `src/test_dual_free.log` |
| test_single_arm_planned.py | `src/test_single_arm_planned.log` |

Logs append across runs with a timestamped banner per run (same `log_utils` as RMDemo_LiftBenchmark).

---

## 8. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| `socket connect err` + all SKIP | Arm(s) powered off or network down | Power arms, `ping 192.168.1.10 / .11` |
| Arrival always via position fallback (WARN) | Event push not reaching host (thread mode, or firmware) | Confirm `RM_TRIPLE_MODE_E`; check DC8; note firmware V1.7.1 |
| PL3 dispatch skew ≥ 50 ms | Host load, TCP latency spike | Re-run; check `mean_lat_us` in LiftBenchmark baseline |
| One arm stops mid-run | Partner-stop triggered by the other arm's failure | Read the per-step table above the summary for the failing side |
| Lift never arrives, arm steps fine | Lift command queued behind previous lift motion | Controller queues rather than preempts (see LiftBenchmark report §5.3) — ensure lift idle before run |
| Run mode WARN = SIMULATION | Arm left in sim mode | `rm_set_arm_run_mode(1)` from GUI/API if real motion intended — sim runs also double as an event-in-sim probe |
| Pole homing FAILED ret=1, `set_state: false` (seen BOTH arms 2026-08-06 20:38) | Controller in an arm-state error: e-stop chain, latched system/joint error, or lift driver error — NOT a parameter/script problem (identical commands physically worked minutes earlier) | Read the auto-printed `[DIAG]` dump, then `RM_ARM=<side> python3 test_pole_only.py --mode REAL` (add `--clear-errors` to recover); ladder: e-stop → clear errors → Web GUI lift panel → power cycle |

---

## Concept roadmap

- **Concept #1 (this)**: dual-arm operation — locked / chained / free semantics over rest/ready/zero + pole strokes.
- **Concept #2 (next)**: cleaning test program for the hinge area (right arm), reusing the same dispatch/arrival machinery with task-specific poses.
- **Future**: collision gating for arbitrary free-running task pairs (this concept's sequence is collision-free by construction).
