# RM API Emulator (`src/rm_emulator.py`)

An in-process emulator of the RealMan RM_API2 Python SDK surface, modeled on
the actual Butterfli hardware: **two RM75-6FB arms, Gen-3 controllers,
V1.7.1 firmware, pole lifts** (left `192.168.1.10`, right `192.168.1.11`).
It lets the concept tests — and any simple test written against the same SDK
subset — run end-to-end with zero hardware and zero network.

The distinction from `run_dry_run.py`: the dry run verifies *logic* with
instant mocks; the emulator verifies the *test programs themselves*,
unmodified, against realistic motion timing, arrival-event semantics, and
failure behavior.

## Quick start

```bash
cd src
python3 run_emulated_suite.py                  # full C1–C4 suite, ~40 s
RM_EMU_TIME_SCALE=1 python3 run_emulated_suite.py   # real-time durations
```

For your own scripts:

```python
import rm_emulator
rm_emulator.set_time_scale(10)     # optional; or RM_EMU_TIME_SCALE env
rm_emulator.install()              # BEFORE importing Robotic_Arm / test code

from Robotic_Arm.rm_robot_interface import RoboticArm   # ← the emulator
```

`install()` must run before the code under test first imports `Robotic_Arm`;
it plants emulated `rm_robot_interface` / `rm_ctypes_wrap` modules in
`sys.modules` (same technique as the LiftBenchmark dry run).

## Fidelity model — what behaves like the real system

| Aspect | Emulated behavior | Source of truth |
|---|---|---|
| Identity getters | `RM_75`, 7-DOF, `6FB`, controller gen 3, ctrl/plan `V1.7.1-emu` | live queries of both arms (2026-08-04) |
| Arm motion | synchronized joint interpolation, duration = max Δq / (180 °/s × v%) | RM75 URDF velocity limits, movej semantics |
| Mid-motion state | `rm_get_current_arm_state` returns time-interpolated joints | controller streams real positions |
| Lift motion | duration = 0.38 s start latency + distance / measured speed map (`10%→17.3` … `100%→66.89` phys mm/s); **commands queue rather than preempt** | LiftBenchmark `hw_baseline.json` + report §5.3 |
| Arrival events | process-global callback, `handle_id`/`event_type=1`/`device 0|3`/`trajectory_state`/`trajectory_connect=0`, fired from completion thread | rm_define event struct + observed behavior |
| Command latency | ~5 ms per call (measured mean was ~8 ms) | LiftBenchmark baseline `mean_lat_us` |
| `rm_set_arm_stop` | halts arm **and** pole, motion freezes mid-path, **no** arrival event | benchmark: only reliable pole halt |
| `rm_set_lift_speed(0)` | halts pole only (current + queued) | lift API semantics |
| Failed trajectory | stops at ~60 % of the path, event carries `trajectory_state=False` | plausible physical failure |
| Unreachable arm | `rm_create_robot_arm` prints socket error, returns handle id −1 | observed 2026-08-05 |
| UDP push (subset) | periodic frames: `joint_status.joint_position/joint_speed`, `liftState.height/pos`, `arm_current_status` (0 idle / 2 moving), `errCode`, `arm_ip` | rm_define push struct (partial) |
| Two-handle process | one global event callback, demux by `handle_id`; distinct ids; second `RoboticArm()` skips init | SDK global-callback design |
| Inspire hand | `rm_set_hand_angle` (6 values 0–1000, −1 = hold), stroke time = 115 ms latency + 373 × span / SPEED_SET ms, device-2 arrival event, `rm_set_hand_speed/force` stored | butterfli_hw bench §3.7/§3.8 stroke law + measured latency |
| **UDP wrong-IP trap** | push to an IP not in `EMU_HOST_IPS` (default `{RM_HOST_IP or "192.168.1.235"}`): ret 0, clear `[emu]` log, **no frames delivered** — exactly like real hardware; override with `emu_set_host_ips(...)` or `RM_HOST_IP` | controller accepts any target IP; silent void push observed in the field |
| Push field gating | `joint_speed`/`liftState` populate only when enabled in `custom_config`; `handState` always zeros | bench_udp_fields (fw 1.7.2: handState absent) |
| Joint limits | `rm_movej` rejects targets beyond RM75 limits (±177.6/±130/±177.6/±135/±177.6/±128/±360°) with ret 1 | RM75 URDF limits |
| Push persistence | push survives `rm_delete_robot_arm` (controller state); only disable or `rm_destroy` stops it | controller-side config semantics |
| Single-thread mode | `RoboticArm(RM_SINGLE_MODE_E)` prints a warning and events are never delivered | real SDK @attention |
| Getter latency | `rm_get_current_arm_state`/`rm_get_lift_state` cost one command latency | TCP round trip |

**Not emulated** (extend when needed): Cartesian moves (`rm_movel`,
`rm_movej_p`), pose in arm state (zeros), force-sensor data (zeros in push
frames), hand modbus registers and the modbus/protocol exclusivity, CANFD passthrough, online programming, fence/collision
config, sim-mode behavioral differences (`rm_set_arm_run_mode` stores the
flag; motion is identical), and `rm_movej`'s `r` (blend) / `connect`
(chaining) arguments — accepted but every move executes immediately with an
exact stop.

**Contract details** (post-review hardening): `rm_movej` rejects joint lists
that are not exactly 7 long and `v` outside 1–100 (ret 1); blocking calls
return −5 on wait timeout and blocking `rm_movej` returns 1 when the motion
failed; `rm_set_lift_height` rejects heights outside the documented 0–2600
range (ret 1); `rm_set_lift_speed(±n)` emulates the open-loop jog toward the
travel end; `rm_get_lift_state` `mode` is direction-aware (2 up / 4 down /
0 idle); `rm_get_realtime_push` returns the stored config with the real
keys; the installed `rm_thread_mode_e` is a real IntEnum, so the
`rm_thread_mode_e(2)` style used by the LiftBenchmark scripts works; and
`rm_get_arm_event_call_back` / `rm_realtime_arm_state_call_back` also exist
as module-level functions, matching the real `rm_ctypes_wrap`.

## Fault injection

Per-arm knobs via `rm_emulator.emu_controller(ip)`:

```python
ctrl = rm_emulator.emu_controller("192.168.1.10")
ctrl.reject_next_dispatch = True   # next motion command returns ret 1
ctrl.fail_next_motion = True       # next arrival: trajectory_state=False,
                                   #   position stops at 60% of the path
ctrl.drop_next_event = True        # arrival event lost -> exercises the
                                   #   position-verify fallback path
ctrl.command_latency_s = 0.05      # inflate per-command latency

rm_emulator.emu_power_off("192.168.1.11")   # connect refused (socket err)
rm_emulator.emu_power_on("192.168.1.11")
```

Verified effects on the mode runners: rejection stops the run and halts the
partner; a dropped event is recovered by the measured-position fallback
(`event=False, verified=True`, WARN); a failed motion fails the step (the
arm genuinely isn't at the target, so the fallback correctly refuses it).

## Configurable addressing

The emulated arms live at `RM_LEFT_IP` / `RM_RIGHT_IP` (defaults
192.168.1.10/.11) and the emulated "this host" for UDP delivery follows
`RM_HOST_IP` — the same environment variables the tests read, so changing
the robot's addressing keeps emulated and real runs consistent with zero
code edits.

## Initial conditions

Left arm parked at its SRDF zero (J7 = 180°), right arm at zero, both lifts
at 100 hw-mm (`half_length`). State persists across `RoboticArm` create /
delete cycles within one process — consecutive tests continue from where the
previous one ended, like real hardware. `RoboticArm.rm_destroy()` halts all
motion but keeps state.

## Emulated API surface

`rm_create_robot_arm`, `rm_delete_robot_arm`, `rm_destroy`,
`rm_get_robot_info`, `rm_get_arm_software_info`, `rm_get_arm_run_mode`,
`rm_set_arm_run_mode`, `rm_get_current_arm_state`, `rm_get_joint_degree`,
`rm_get_lift_state`, `rm_movej`, `rm_set_lift_height`, `rm_set_lift_speed`,
`rm_set_arm_stop`, `rm_set_hand_angle`, `rm_set_hand_speed`, `rm_set_hand_force`,
`rm_get_arm_event_call_back`,
`rm_realtime_arm_state_call_back`, `rm_set_realtime_push`,
`rm_get_realtime_push`.
