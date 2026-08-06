# RealMan RM75 Dual-Arm — Version Information

Dual-arm setup (Butterfli robot). Both arms queried live over the API on **2026-08-04** using read-only `rm_get_*` calls (RM_API2 SDK 1.1.6, no motion commands issued).

## 1. Current versions

| Item | Arm 1 — `192.168.1.10` | Arm 2 — `192.168.1.11` |
|---|---|---|
| Arm model | RM_75, 7-DOF | RM_75, 7-DOF |
| Product version | RM75-6FB (6-axis force sensor) | RM75-6FB (6-axis force sensor) |
| Controller generation | Gen-3 (`robot_controller_version: 3`) | Gen-3 (`robot_controller_version: 3`) |
| Controller software (`ctrl`) | **V1.7.1**-f95e896 (build 2025/06/16) | **V1.7.1**-f95e896 (build 2025/06/16) |
| Planning layer (`plan`) | **V1.7.1**-39d1b2f (build 2025/06/16) | **V1.7.1**-39d1b2f (build 2025/06/16) |
| Controller algorithm library | 1.5.5-c0d52d18 | 1.5.5-c0d52d18 |
| Dynamics model version | 2 | 2 |
| Joint firmware (J1–J6 / J7) | 54544 × 6, 58640 | 54544 × 6, 58640 |
| Serial number via API | not supported on this firmware | not supported on this firmware |

**Host-side (this repo):** RM_API2 C API **1.1.6** (SDK header `v1.1.6`), offline algorithm library **1.6.0**-701ee1e6.

### Live capability probe (read-only, both arms)

| Capability | `.10` | `.11` | Notes |
|---|---|---|---|
| Dynamics collision detection (`collision_stage`) | **level 2** | **level 0 (OFF)** | ⚠ configuration differs between arms |
| Static-state collision switch | supported, off | supported, off | added in V1.7.1 |
| Singularity-avoidance switch | supported, off | supported, off | 7-axis support only lands in V1.7.3 — likely ineffective on RM75 at V1.7.1 |
| Arm self-collision detection | supported, off | supported, off | simulation-mode only |
| Payload/end-effector self-collision | supported, off | supported, off | simulation-mode only |
| Electronic fence | supported, off | supported, off | simulation-mode only |
| Manual collision-release (`collision_remove_enable`) | **no response** | **no response** | V1.7.4 feature |
| Joint torque data (`rm_get_torque_data`) | **no response** | **no response** | N/A on RM75-6FB hardware (wrist force sensor, no joint torque sensors) |
| SN read (`rm_get_sn`) | **no response** | **no response** | needs newer firmware |

## 2. Newer controller releases and their features (Gen-3)

Current release line per RealMan release notes (last updated 2026-04-29). Both arms are **4 releases behind**.

| Version | Date | Key features | Required pairings |
|---|---|---|---|
| V1.7.1 *(installed)* | 2025-06-10 | Manual load identification; singularity-avoidance enable switch; static-state collision-detection mode | End board V1.9.9; joints Vd5.1.0 / Ve5.1.0 |
| V1.7.2 | 2025-06-17 | Load identification default/manual modes | End board V1.9.9; API2 v1.1.1 |
| V1.7.3 | 2025-11-04 | **Cartesian velocity passthrough** (`rm_movev_canfd`); **7-axis singularity avoidance** (RM75-relevant); end-device register R/W; **UDP speed-reporting fix** | End board V1.9.9; API2 v1.1.3; ROS1 v2.6.0; ROS2 v1.6.0 |
| V1.7.4 | 2025-12-12 | Manual collision-release mode; current-loop drag near joint limits; user-configurable singularity protection | **End board V2.0.0** (separate `.bin` flash); API2 v1.1.4 |
| **V1.7.5** *(latest)* | 2026-04-29 | Force-control teach safety check; soft-start current monitoring; new model support; **one-key upgrade** (controller + joints + end board in one pass, single restart) | API2 v1.1.5; joints Vd5.1.0 / Ve5.1.0 |

### What staying on V1.7.1 costs

- **No controller-side singularity protection for a 7-axis arm** during Cartesian-linear moves (7-axis support arrived in V1.7.3).
- **UDP joint-velocity reporting carries a known bug** fixed in V1.7.3 — treat pushed `joint_speed` values with caution.
- No Cartesian velocity passthrough interface (`rm_movev_canfd` / `rm_set_movev_canfd_init`).
- No manual collision-release mode, configurable singularity protection (V1.7.4), or force-control teach safety checks (V1.7.5).

Not affected by any firmware upgrade (Gen-3 hardware ceiling): `rm_movel_offset`, latched Gen-4 e-stop, named trajectory-file playback, flowchart APIs, `rm_run_tool_action`.

## 3. Upgrade path

- Firmware packages (`.realman` files) are **not publicly downloadable** — request them from RealMan technical support / after-sales, quoting: model **RM75-6FB**, **Gen-3 controller**, current versions from the table above, target version. A package is model-specific.
- Upgrade is performed through the **web teach pendant** (browse to the arm's IP): Configuration → Robotic Arm Config → Version Information → Select File → Start Upgrade (~4–5 min) → wait for continuous beeping → restart → **Ctrl+F5** to clear the cached UI.
- Targeting ≥ V1.7.4 additionally requires flashing the end interface board to V2.0.0 (`.bin` upload); V1.7.5's one-key upgrade folds this into a single pass.
- Undocumented (confirm with support before upgrading): whether saved programs/configs survive, and whether downgrades are possible. Back up online-programming projects, tool/work frames, payload and network settings first.
- Upgrade both arms to the **same version** to keep the dual-arm setup consistent, and align the client SDK to the paired API2 version afterwards.

### Sources

- Release notes: <https://develop.realman-robotics.com/robot/releaseNotes/releaseNotes/>
- Gen-3 upgrade procedure: <https://develop.realman-robotics.com/en/robot/teachingPendant/systemUpgrade/>
- Version pairing table: <https://develop.realman-robotics.com/en/robot/releaseNotes/versionComparisonTable/>
- Package distribution policy: <https://develop.realman-robotics.com/robot/download/redevelopment/>, forum thread [bbs #224](https://bbs.realman-robotics.cn/question/224.html)
- Upgrade walkthrough with end-board steps: forum thread [bbs #323](https://bbs.realman-robotics.cn/question/323.html)
