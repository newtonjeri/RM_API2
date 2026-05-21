# RMDemo_LiftBenchmark — Test Procedure

**Hardware:** RM-75 robotic arm with telescopic pole axis  
**API:** RM_API2 Python bindings (`rm_robot_interface.py`)  
**Scope:** Pole/lift axis — control, timing, UDP feedback, trajectory replay, preemption

---

## 1. Prerequisites

### 1.1 Hardware
| Requirement | Value |
|---|---|
| Robot IP | `192.168.1.10` |
| Robot port | `8080` |
| Host machine IP | `192.168.1.11` |
| UDP listen port | `8089` |
| Pole travel | 300 mm physical (0–300 mm), safe range 10–290 mm |
| Hardware scale | physical mm × 2/3 = hardware units |

Ensure the robot is powered, arm is in a safe configuration, and the host machine can ping `192.168.1.10`.

### 1.2 Software
- Python 3.8+
- RM_API2 shared library built for the host architecture (`libapi_c.so` under `Python/Robotic_Arm/libs/`)
- All test scripts and support files in `src/` (see §3)

### 1.3 Baseline file
`src/hw_baseline.json` must be populated before running tests that use adaptive thresholds (tests 2–6). Populate it by running `test_lift_connect.py` and `test_lift_udp_feedback.py` on the target hardware at least once and saving the baseline:

```bash
cd src
python3 -c "import hw_baseline; hw_baseline.save()"
```

Baseline values (current reference):

| Metric | Value |
|---|---|
| Mean command latency | 7621 µs |
| UDP interval (cycle=2) | 10.00 ms ± 0.48 ms |
| Connect time | 65.65 ms |
| Speed map 100% | 66.8 mm/s effective |

---

## 2. File Structure

```
src/
├── hw_baseline.py              Shared baseline store (read/write hw_baseline.json)
├── hw_baseline.json            Measured hardware reference values
├── lift_trajectory.json        MoveIt-format trajectory (26 waypoints, 75→295 mm)
├── log_utils.py                Shared logging helper (appends to <script>.log)
├── run_dry_run.py              Offline logic/math verification (no hardware)
├── test_lift_connect.py        T1 — SDK init and basic state read
├── test_lift_nonblocking_rate.py   T2 — Non-blocking command rate
├── test_lift_trajectory_rate.py    T3 — Trajectory replay and UDP validation
├── test_lift_udp_feedback.py       T4 — UDP push latency and intervals
├── test_lift_blocking_vs_nonblocking.py  T5 — Blocking vs non-blocking timing
├── test_lift_speed_param.py    T6 — Speed parameter characterisation
├── test_lift_motion_profile.py T7 — Commanded vs actual velocity profile
└── test_lift_preemption.py     T8 — Mid-trajectory command preemption
```

Each script appends its full output to `src/<script_name>.log` on every run.

---

## 3. Running the Offline Verification (No Hardware)

Before any hardware run, verify all math and module imports are correct:

```bash
cd src
python3 run_dry_run.py
```

**Expected result:** `75/75 passed  (0 failed)`  
**Log file:** `src/run_dry_run.log`

This checks:
- Scale-factor round-trips (PHYS_TO_HW × HW_TO_PHYS)
- `safe_height()` boundary conditions
- `hw_height()` and `phys_height()` conversions
- Trajectory conversion helpers (`pos_m_to_mm`, `vel_m_s_to_pct`)
- Clean import of all 8 test modules with mock robot

---

## 4. Individual Test Cases

Run each test from the `src/` directory:

```bash
cd src
python3 <test_script>.py
```

Output is printed to stdout and appended to `src/<test_script>.log`.

---

### Test 1 — `test_lift_connect.py`
**Purpose:** Verify SDK initialisation, TCP connection, and basic lift state read. No motion commanded.

**Duration:** ~2 s

**Steps:**
1. Initialise `RoboticArm` in TRIPLE thread mode.
2. Call `rm_create_robot_arm("192.168.1.10", 8080, 3)` and measure connect time.
3. Call `rm_get_robot_info()` — verify `arm_dof == 7`.
4. Call `rm_get_lift_state()` — verify position in [0, 300] mm and `err_flag == 0`.
5. Call `rm_delete_robot_arm()` and `rm_destroy()`.

**Pass / Fail criteria:**

| ID | Check | Pass condition |
|---|---|---|
| T1 | SDK init (TRIPLE mode) | No exception raised |
| T2 | `rm_create_robot_arm` | `handle.id > 0`; connect time recorded |
| T3 | `rm_get_robot_info` | `ret == 0` and `arm_dof == 7` |
| T4 | `rm_get_lift_state` | `ret == 0` and position ∈ [0, 300] mm |
| T5 | `err_flag` | `err_flag == 0` |
| T6 | en_flag | **SKIP** (not available via polling; use UDP) |
| T7 | Teardown | Both delete and destroy return `0` |

**If hardware is unreachable:** T2–T7 are automatically SKIP; exits 0.

---

### Test 2 — `test_lift_nonblocking_rate.py`
**Purpose:** Measure the maximum rate at which `rm_set_lift_height(..., block=False)` can be called without errors. Also sweeps 10 / 25 / 50 Hz to show acceptable call rates.

**Duration:** ~60 s (homing + burst + sweep)

**Steps:**
1. Home to 150 mm (blocking) at 50%.
2. Issue 200 non-blocking commands alternating between 140 mm and 160 mm, measuring per-call latency.
3. Sweep at 10, 25, 50 Hz (50 waypoints each), recording error counts.

**Pass / Fail criteria:**

| ID | Check | Pass condition |
|---|---|---|
| R1 | Mean per-call latency | ≤ `hw_baseline.mean_lat_us × 2.0` µs (≤ ~15.2 ms) |
| R2 | Burst pass rate | ≥ 80% of 200 calls returned `0` |

*Rate sweep (10/25/50 Hz) is informational only — no assertion.*

---

### Test 3 — `test_lift_trajectory_rate.py`
**Purpose:** Replay a full MoveIt JointTrajectory (loaded from `lift_trajectory.json`) using the timed-waypoint scheduler. Validates command rate, timing jitter, UDP feedback, and that the lift actually moves to target.

**Duration:** ~15 s

**Trajectory:** 26 waypoints, 75 → 295 mm over 2.4 s at 100 mm/s plateau (ramp-up/down included).

**Steps:**
1. Home to 75 mm (blocking).
2. Enable UDP push at cycle=2 (100 Hz, `192.168.1.11:8089`).
3. Replay all 26 waypoints using the timed scheduler (`time.perf_counter`-based).
4. Collect UDP samples for 5 s post-replay.
5. Evaluate latency, jitter, UDP intervals, packet count, and final position.

**Pass / Fail criteria:**

| ID | Check | Pass condition |
|---|---|---|
| TR1 | Mean command latency | ≤ baseline × 2.0 µs |
| TR2 | Command pass rate | ≥ 80% of waypoints returned `0` |
| TR3 | Timing jitter | Max jitter ≤ 50 ms |
| TR4 | Lift moved to target | Final position > start AND position ∈ [10, 290] mm |
| TR5 | UDP interval | Mean interval ∈ [baseline_mean ± 3σ] ms |
| TR6 | UDP packet count | ≥ 300 packets received |
| TR7 | Height reached | Position ≥ 230 mm within 5 s |

---

### Test 4 — `test_lift_udp_feedback.py`
**Purpose:** Characterise UDP realtime-state push: measure command-to-feedback latency and validate packet intervals at three cycle settings (200 Hz / 100 Hz / 50 Hz).

**Duration:** ~45 s

**Steps:**
1. Home to 50 mm (blocking).
2. For each cycle setting (1, 2, 4): configure UDP push, collect 2 s of idle samples, print interval statistics.
3. Main test at cycle=2: issue non-blocking move 50 → 250 mm at 50%, collect UDP until 230 mm or 10 s timeout.
4. Evaluate UDP interval, packet count, and latency to reach 230 mm.

**Pass / Fail criteria:**

| ID | Check | Pass condition |
|---|---|---|
| U1 | UDP interval (cycle=2) | Mean ∈ [baseline ± 3σ] ms (≈ 9.5–10.5 ms) |
| U2 | UDP packet count | ≥ 300 packets |
| U3 | Height reached | ≥ 230 mm within 5 s of command |

**If no UDP data received:** U1 and U2 both FAIL.

---

### Test 5 — `test_lift_blocking_vs_nonblocking.py`
**Purpose:** Compare blocking vs non-blocking call timing. Verify that the event callback fires for both call types and that `rm_set_arm_stop()` halts motion.

**Duration:** ~30 s

**Steps:**
1. **T1 — Blocking timing:** Move 80 → 220 mm at 50% blocking. Measure wall time.
2. **T2 — Non-blocking timing:** Move 220 → 80 mm at 50% non-blocking. Measure call-return time only.
3. **T3 — Blocking event callback:** Confirm callback fires (or confirm clean return with no spurious event).
4. **T4 — Non-blocking event callback:** Wait for event (`event_type==1`, `device==3`, `trajectory_state==True`).
5. **T5 — Stop command:** Issue `rm_set_arm_stop()` mid-travel; confirm mode returns to 0 within 2 s.

**Pass / Fail criteria:**

| ID | Check | Pass condition |
|---|---|---|
| BN1 | Blocking wall time | ∈ [2.0, 5.0] s (140 mm @ 50 mm/s ≈ 2.8 s) |
| BN2 | Non-blocking call time | ≤ baseline mean_lat × 3.0 (≤ ~22.9 ms) |
| BN3 | Blocking call return | `ret == 0` |
| BN4 | Non-blocking event | Fires with correct `event_type`, `device`, `trajectory_state` |
| BN5 | Stop command | `rm_set_arm_stop()` returns `0` |
| BN6 | Mode returns to idle | Mode == 0 within 2 s after stop (SKIP if not detected) |

---

### Test 6 — `test_lift_speed_param.py`
**Purpose:** Characterise how the `speed_pct` parameter (10 / 20 / 30 / 50 / 70 / 100%) maps to effective travel velocity in mm/s.

**Stroke:** 80 → 230 mm (150 mm fixed distance), repeated for each speed.

**Duration:** ~120 s (6 speeds × up to 20 s each)

**Steps:**
1. For each speed in [10, 20, 30, 50, 70, 100]:
   a. Home to 80 mm (blocking at 100%).
   b. Issue non-blocking move to 230 mm at the test speed.
   c. Poll `rm_get_lift_state()` until position ≥ 225 mm or timeout.
   d. Record travel time, compute effective mm/s.
2. Print comparison table: `speed% | travel_s | eff mm/s | expected mm/s | ratio`.
3. Evaluate assertions.

**Pass / Fail criteria:**

| ID | Check | Pass condition |
|---|---|---|
| SP1 | Monotonic travel times | Each higher speed completes in less time |
| SP2 | speed=100%: upper bound | Effective ≤ 110 mm/s |
| SP3 | speed=100%: lower bound | Effective ≥ 85 mm/s |
| SP4 | speed=10%: travel time | ≥ 5 s for 150 mm stroke |
| SP5 | All speeds in physical range | 0 < effective ≤ 150 mm/s for all speeds |

---

### Test 7 — `test_lift_motion_profile.py`
**Purpose:** Q2 — Does the motor actually follow the commanded velocity profile? Replays `lift_trajectory.json` at 200 Hz UDP capture and cross-references commanded vs actual position.

**Duration:** ~15 s

**Steps:**
1. Home to 75 mm (blocking).
2. Enable UDP at cycle=1 (200 Hz).
3. Replay trajectory (same timed scheduler as Test 3).
4. Compute rolling velocity from UDP position stream (100 ms window).
5. Print side-by-side table: `# | t_sched | cmd_mm | cmd_% | udp_mm | vel(mm/s) | err(mm)`.
6. Print profile summary: motor-start latency, peak velocity, shape heuristic.
7. Evaluate assertions.

**Pass / Fail criteria:**

| ID | Check | Pass condition |
|---|---|---|
| MP1 | All waypoints dispatched | Every `rm_set_lift_height` returns `0` |
| MP2 | Motor moved | Peak rolling velocity > 5 mm/s |
| MP3 | Motor-start latency | First motion detected within 500 ms of first command |
| MP4 | Max tracking error | Max |commanded − actual| ≤ 30 mm throughout trajectory |

---

### Test 8 — `test_lift_preemption.py`
**Purpose:** Q3 — MoveIt trajectory preemption. Replays the forward trajectory (75 → 295 mm), then at each of three injection points abandons it and issues a reverse trajectory (→ 75 mm). Measures whether the motor redirects.

**Duration:** ~60 s (3 sub-tests × ~20 s each, including homing between runs)

**Injection points:**

| Label | Inject at | Position at inject |
|---|---|---|
| T1 — early | 0.3 s | ~20 mm into stroke (~95 mm) |
| T2 — mid | 1.2 s | ~80 mm into stroke (~155 mm) |
| T3 — late | 2.2 s | ~147 mm into stroke (~222 mm) |

**Steps (per injection point):**
1. Home to 75 mm (blocking).
2. Start replaying forward trajectory; break at inject time.
3. Read current position from last UDP sample.
4. Build reverse trajectory from current position → 75 mm at 100 mm/s.
5. Issue reverse trajectory (same timed scheduler).
6. Collect UDP for 3 s post-inject.
7. Detect reversal: position drops ≥ 5 mm below post-inject peak.
8. Record verdict (REDIRECT / NO_REDIRECT) and latency from inject command to first reversal.

**Pass / Fail criteria:**

| ID | Injection point | Pass condition |
|---|---|---|
| PR1 | T1 — early | Motor redirects: position drops ≥ 5 mm below peak within 3 s |
| PR2 | T2 — mid | Same |
| PR3 | T3 — late | Same |

**SKIP:** If fewer than 2 UDP samples captured for a sub-test.

---

## 5. Running the Full Hardware Suite

```bash
cd src

python3 test_lift_connect.py
python3 test_lift_nonblocking_rate.py
python3 test_lift_trajectory_rate.py
python3 test_lift_udp_feedback.py
python3 test_lift_blocking_vs_nonblocking.py
python3 test_lift_speed_param.py
python3 test_lift_motion_profile.py
python3 test_lift_preemption.py
```

**Recommended order:** Run in the sequence above. Tests 1–4 populate `hw_baseline.json` values used as adaptive thresholds by Tests 5–6.

**Total expected run time:** ~5–7 minutes

---

## 6. Expected Summary Results

| Test | PASS | FAIL | SKIP | Notes |
|---|---|---|---|---|
| T1 connect | 6 | 0 | 1 | T6 (en_flag) always SKIP |
| T2 nonblocking rate | 2 | 0 | 0 | |
| T3 trajectory rate | 6–7 | 0 | 0 | UDP checks conditional |
| T4 UDP feedback | 3 | 0 | 0 | |
| T5 blocking vs nonblocking | 5 | 0 | 0–1 | T5 mode check may SKIP |
| T6 speed param | 5 | 0 | 0 | |
| T7 motion profile | 4 | 0 | 0 | |
| T8 preemption | 3 | 0 | 0 | May FAIL if motor queues |
| **Total** | **34–35** | **0** | **1–2** | |

---

## 7. Log Files

Each script appends its complete output (stdout + stderr) to a log file in `src/`:

| Script | Log file |
|---|---|
| `run_dry_run.py` | `src/run_dry_run.log` |
| `test_lift_connect.py` | `src/test_lift_connect.log` |
| `test_lift_nonblocking_rate.py` | `src/test_lift_nonblocking_rate.log` |
| `test_lift_trajectory_rate.py` | `src/test_lift_trajectory_rate.log` |
| `test_lift_udp_feedback.py` | `src/test_lift_udp_feedback.log` |
| `test_lift_blocking_vs_nonblocking.py` | `src/test_lift_blocking_vs_nonblocking.log` |
| `test_lift_speed_param.py` | `src/test_lift_speed_param.log` |
| `test_lift_motion_profile.py` | `src/test_lift_motion_profile.log` |
| `test_lift_preemption.py` | `src/test_lift_preemption.log` |

Each run is separated in the log by a timestamped header:

```
========================================================================
  Script : test_lift_connect.py
  Log    : test_lift_connect.log
  Started: 2026-05-21 14:30:22
========================================================================
```

---

## 8. Troubleshooting

| Symptom | Likely cause | Action |
|---|---|---|
| All T2–T7 SKIP | Robot unreachable | Check network, ping `192.168.1.10` |
| T4 FAIL: `err_flag != 0` | Robot fault state | Power-cycle robot, check estop |
| T2 FAIL: latency > threshold | Baseline stale or CPU load | Re-populate `hw_baseline.json` |
| T3/T4 FAIL: no UDP data | Wrong host IP or firewall | Confirm host IP is `192.168.1.11`, check iptables |
| T7/T8 FAIL: tracking error > 30 mm | Motor slow to respond | Check pole mechanical condition; verify `cycle=1` is supported |
| T8 FAIL: NO_REDIRECT | Command queuing | Expected behaviour — motor finishes current command before redirecting |
