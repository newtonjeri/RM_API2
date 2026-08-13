# RM75-6FB — the robot description, system-wide

*The single file every other program should take its robot facts from.
Kinematics, dynamics, limits, singularities, and the frames — each number
carrying its source and how it was checked. Nothing here is inferred from a
drawing.*

**Both arms are RM75-6FB**, left `192.168.1.10` and right `192.168.1.103`.
Measured, not assumed — §5.

**Re-verify with** `python3 src/verify_robot_model.py` (offline, no arm).

---

## 0. Sources, and what each is authoritative for

| source | authoritative for | how it was checked |
|---|---|---|
| **SDK algorithm library** `rm_algo_*` (v1.6.0-701ee1e6) | DH, joint limits, singularity test | it *is* the controller's IK; runs offline |
| **[RM75 Ontology Parameters](https://develop.realman-robotics.com/en/robot/robotParameter/RM75OntologyParameters/)** | published specs, MDH, variants, singularity types, load curves | cross-checked against SDK and URDF |
| **[rm_models URDF](https://github.com/RealManRobot/rm_models/tree/main/RM75/urdf)** | link mass, COM, inertia, effort | masses sum to the published self-weight |
| **`butterfli.urdf`** (our workspace) | how the arms are *mounted and dressed* | kinematics agree; **inertials do not** — §7 |
| **our recordings** `runs/*/stream.csv` | which variant we actually have | 0.01 mm TCP reproduction |

The `rm_algo_*` calls need **no arm connection**: `Algo(RM_MODEL_RM_75_E)`
with `handle=None` answers from the model.

---

## 1. Published specifications

| | value |
|---|---|
| degrees of freedom | 7, humanoid configuration |
| payload | 5 kg |
| working radius | **627 mm (6FB)** · 610 mm (B) · 638.5 mm (6F) |
| repeatability | ±0.05 mm |
| self-weight | 7.8 kg (7.9 kg for the 6-axis-force version) |
| max TCP linear speed | **≤ 1.8 m/s** |
| power | ≤200 W typical, ≤1000 W peak |
| base diameter | φ107 mm |
| installation angle | any |
| operating environment | 0–45 °C, 25–85 % RH non-condensing |

Workspace is a sphere of the working radius **plus the cylindrical volume
directly above and below the base**; the vendor's cross-section marks an
annular region of reduced manoeuvrability. Load capacity vs reach is
published as per-variant load curves (B / 6F / 6FB) — consult those before
assuming the full 5 kg at full extension.

---

## 2. Kinematics

### 2.1 Standard DH — from `rm_algo_get_dh()`

| i | d (m) | a (m) | α (°) | offset (°) |
|---|---|---|---|---|
| 1 | 0.2405 | 0 | −90 | 0 |
| 2 | 0 | 0 | +90 | 0 |
| 3 | 0.2560 | 0 | −90 | 0 |
| 4 | 0 | 0 | +90 | 0 |
| 5 | 0.2100 | 0 | −90 | 0 |
| 6 | 0 | 0 | +90 | 0 |
| 7 | 0.1612 | 0 | 0 | 0 |

Convention: `T_i = Rz(θ_i + offset_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)`.

Every `a` and every `offset` is zero — a pure **S-R-S** chain: spherical
shoulder (J1–J3), revolute elbow (J4), spherical wrist (J5–J7). That is the
structural reason **J4's angle is fixed by the commanded pose regardless of
how the redundancy is resolved**, which is what lets our offline elbow screen
be exact while the other six joints need a saved plan.

**Verified: 0.00008 mm worst error vs `rm_algo_forward_kinematics()` over 200
poses.** At zero the flange sits at (0, 0, 0.8677) m.

### 2.2 Modified DH — as RealMan publish it

Same arm, other convention. The `d` column is identical; α is the same
sequence indexed from the previous row. RealMan also publish an SDH table
which matches §2.1.

| i | aᵢ₋₁ (mm) | αᵢ₋₁ (°) | dᵢ (mm) | θᵢ/offsetᵢ (°) |
|---|---|---|---|---|
| 1 | 0 | 0 | 240.5 | 0 |
| 2 | 0 | −90 | 0 | 0 |
| 3 | 0 | +90 | 256 | 0 |
| 4 | 0 | −90 | 0 | 0 |
| 5 | 0 | +90 | 210 | 0 |
| 6 | 0 | −90 | 0 | 0 |
| 7 | 0 | +90 | **d₇, see §5** | 0 |

`offset` is "the offset of the joint zero position from the model zero
position", applied as `model angle = joint angle + offset`. Ours are all
zero, so joint readings and model angles are the same number.

### 2.3 Limits

| | J1 | J2 | J3 | J4 | J5 | J6 | J7 |
|---|---|---|---|---|---|---|---|
| range (°) | ±178 | ±130 | ±178 | ±135 | ±178 | ±128 | ±360 |
| max speed (°/s) | 180 | 180 | 225 | 225 | 225 | 225 | 225 |
| URDF effort (N·m) | 60 | 60 | 30 | 30 | 10 | 10 | 10 |

Ranges agree across SDK, vendor page and URDF (≤0.04°). Speeds agree across
SDK, vendor page and the live controller.

**Two that disagree — use the controller's, not the library's or the URDF's:**

- **Joint acceleration.** Algorithm library returns **100 °/s²**; the
  controller reports **600 °/s²** (`joint_drive_max_acc`, both arms). Neither
  is validated against motion.
- **URDF velocity is a uniform 3.14 rad/s = 180 °/s on all seven**,
  understating J3–J7 by 25 %. The URDF is for visualisation and dynamics.

### 2.4 Reach

`Σd = 240.5 + 256 + 210 + 161.2 = 867.7 mm` base to flange. Less the 240.5 mm
base rise: **627.2 mm**, matching the published 627 mm for the 6FB. Two
independently published numbers that only agree if `d₇ = 161.2`.

---

## 3. Motion singularities

**Four types, from the vendor page.** `x` = any value.

| type | condition | pattern | vendor example |
|---|---|---|---|
| **1** | q2 = 0 **and** q6 = 0 | `[x,0,x,x,x,0,x]` | `[0,0,0,90,0,0,0]` |
| **2** | q4 = 0 (**elbow straight**) | `[x,x,x,0,x,x,x]` | `[0,60,0,0,0,90,0]` |
| **3** | q2 = 0 **and** q3 = ±90 | `[x,0,±90,x,x,x,x]` | `[0,0,90,90,0,90,0]` |
| **4** | q6 = 0 **and** q5 = ±90 | `[x,x,x,x,±90,0,x]` | `[0,90,90,90,90,0,0]` |

Type 2 is the one to hold in mind day to day: **a straight elbow is
singular**, and our paths live near full extension. Types 1/3 are shoulder
configurations; type 4 is a wrist configuration.

### 3.1 Testing for it — which SDK call, and which not

| call | joints | use on RM75? |
|---|---|---|
| `rm_algo_universal_singularity_analyse(q, limit)` | **7** | **yes** — Jacobian minimum singular value |
| `rm_algo_kin_robot_singularity_analyse(q)` | 6 | **NO** — documented 仅支持六自由度 (6-DOF only), and the Python wrapper packs `c_float * 6`. Passing 7 joints raises `IndexError` |

`universal` returns `0` normal, `-1` singular at the given threshold, `-2`
computation failed. Threshold is the minimum singular value, range 0–1,
**default 0.01**.

**Validated**: it flags all four of RealMan's example configurations as `-1`
and a benign pose as `0`.

The 6-DOF-only call is still worth knowing about because its return codes
name the families — `-1` shoulder (肩部奇异), `-2` elbow (肘部奇异), `-3`
wrist (腕部奇异) — and it returns the distance from the wrist centre to the
shoulder singularity plane. Its thresholds, from
`rm_algo_kin_singularity_thresholds_init()`:

    limit_qe = 10°     elbow
    limit_qw = 10°     wrist
    limit_d  = 0.05 m  wrist-centre-to-shoulder-plane distance

### 3.2 A continuous measure, and where our runs actually sit

The SDK returns only a verdict. To get a number, **bisect the threshold its
analyser accepts** — that recovers the SDK's own minimum singular value
exactly, with no assumption about how it normalises the Jacobian.
`robot_model.sigma_min()` does this.

⚠ **Do not take the SVD of the 6×7 Jacobian yourself.** It was tried and it
is wrong: the matrix mixes metres with radians, so its singular values shrink
whenever the arm is merely **retracted** — a reach effect, not a rank
deficiency. On `20260811T222451` sample 1766 the hand-rolled value read
0.0034, apparently singular, while the SDK called the same pose fine and the
arm was standing still. An 8-configuration validation missed it because those
poses were all either deep in a singularity or far from one; the two measures
diverge only in between, which is exactly the region worth reporting.

| pose | σ_min | SDK @0.01 |
|---|---|---|
| all four documented types | ≤0.0001 | −1 |
| q4 = 5° | 0.0081 | −1 |
| q4 = 20° | 0.0322 | 0 |
| a benign pose | 0.0921 | 0 |
| our blend-path start | 0.1089 | 0 |

Our recorded runs:

| run | σ_min | median |
|---|---|---|
| `20260813T204650` test_motion @0.25 | 0.0554 | 0.1067 |
| `20260813T204720` test_motion @0.35 | 0.0476 | 0.1104 |
| **`20260811T222451` toplid_right** | **0.0285** | 0.0757 |

**Nothing we have run comes near singular** — the worst is `toplid_right` at
2.9× the SDK's threshold, and 48 % of its samples sit below a 0.03 watch
level. It is the least well-conditioned motion we have, on the arm with no
clean REAL run, which makes it worth watching; it is not a fault.

**σ_min is NOT a predictor of joint load, and must not gate anything.** It is
tempting: on the two `test_motion` runs it correlates −0.40 and −0.47 with
peak joint utilisation. But on `toplid_right` the correlation is **+0.49**,
with peak joint demand occurring at σ_min 0.107 against a run median of
0.076 — a *better* conditioned pose than typical. The sign is path-dependent.
Report it; do not infer from it.

---

## 4. Dynamics

**Source: the RealMan URDF only.** The SDK has no dynamics call — all 55
`rm_algo_*` functions checked, none returns mass, inertia or torque — and
the vendor page publishes no per-joint rated torque. RealMan do publish
kinetic parameters per variant (B / 6F / 6FB) in §2.4 of their page; the
figures below are the URDF's.

### 4.1 Mass and centre of mass

COM in each link's own frame, metres.

| link | mass (kg) | COM x | COM y | COM z |
|---|---|---|---|---|
| base_link | 1.862 | 0.00049987 | 0.000052709 | 0.060019 |
| link_1 | 1.574 | 0.000241 | −0.013273 | −0.009950 |
| link_2 | 1.217 | −0.000357 | −0.106789 | 0.005329 |
| link_3 | 1.110 | 0.000003 | −0.013980 | −0.011324 |
| link_4 | 0.685 | −0.000005 | −0.084658 | 0.004747 |
| link_5 | 0.619 | 0.000078 | −0.012937 | −0.008781 |
| link_6 | 0.602 | −0.000014 | −0.078524 | 0.002819 |
| link_7 | 0.144 | 0.001094 | −0.000077 | −0.010119 |

**Total 7.813 kg** against the published **7.8 kg** — independent
confirmation these inertials describe the real arm. Moving mass excluding
`base_link`: **5.951 kg**.

### 4.2 Inertia tensors

kg·m², about each link's COM, in the link frame.

| link | ixx | iyy | izz | ixy | ixz | iyz |
|---|---|---|---|---|---|---|
| base_link | 0.0017232 | 0.0017051 | 0.00090158 | −3.1058e−06 | −3.7924e−05 | 1.3691e−06 |
| link_1 | 0.002487573 | 0.002321038 | 0.001450554 | 9.663e−06 | −7.909e−06 | 1.79393e−04 |
| link_2 | 0.003494121 | 0.000892721 | 0.003444080 | 2.921e−06 | −5.613e−06 | −5.83884e−04 |
| link_3 | 0.001836663 | 0.001498875 | 0.001062545 | 2.259e−06 | −4.216e−06 | 3.7167e−05 |
| link_4 | 0.001282444 | 0.000373013 | 0.001256177 | −5.51e−07 | −6.30e−07 | −2.32084e−04 |
| link_5 | 0.000627336 | 0.000542455 | 0.000370291 | 1.636e−06 | −1.345e−06 | 3.4970e−05 |
| link_6 | 0.000780774 | 0.000289973 | 0.000763955 | −1.21e−07 | −4.69e−07 | −1.20513e−04 |
| link_7 | 0.000044123 | 0.000035078 | 0.000065445 | −6.4e−08 | 3.0e−07 | −2.9e−08 |

### 4.3 URDF joint tree

| joint | origin xyz (m) | origin rpy (rad) | limits (rad) | effort | vel |
|---|---|---|---|---|---|
| joint_1 | 0, 0, 0.2405 | 0, 0, 0 | ±3.106 | 60 | 3.14 |
| joint_2 | 0, 0, 0 | −1.5708, 0, 0 | ±2.2689 | 60 | 3.14 |
| joint_3 | 0, −0.256, 0 | 1.5708, 0, 0 | ±3.106 | 30 | 3.14 |
| joint_4 | 0, 0, 0 | −1.5708, 0, 0 | ±2.356 | 30 | 3.14 |
| joint_5 | 0, −0.21, 0 | 1.5708, 0, 0 | ±3.106 | 10 | 3.14 |
| joint_6 | 0, 0, 0 | −1.5708, 0, 0 | ±2.234 | 10 | 3.14 |
| joint_7 | 0, −0.1612, 0 | 1.5708, 0, 0 | ±6.28 | 10 | 3.14 |

The non-zero origins are the same four numbers as the DH `d` column, reached
by a different route.

### 4.4 Torque, and why current cannot be converted to it

J4 is rated 30 N·m, J1/J2 60 N·m. Our recordings: a clean 0.25 m/s pass of
`test_motion_001` peaks at **5.8 A on J4**; the aborted `20260813T205319`
hit **26.35 A on J4** at the instant the controller stopped it. **No torque
constant is published and the SDK has no torque channel**, so current stays
a *relative* indicator. This is what blocks the F30 payload-residual test in
`analyse_run.py`, and it is a specific ask for RealMan: `K_t` per joint, or
a torque readout.

---

## 5. Variant — five models differ ONLY in d₇

| variant | d₇ (mm) |
|---|---|
| RM75-B | 144 |
| **RM75-6FB** | **161.2** |
| RM75-B-V | 166.8 |
| RM75-6F | 172.5 |
| RM75-6FB-V | 184 |

Same DH otherwise, same limits, same masses. Choose wrong and every Cartesian
target is displaced along the tool axis by up to 40 mm.

**Ours are 6FB — both.** Reconstructing the TCP from the recorded joint
angles of a **right-arm** run (`20260811T222451`, 2666 samples) and a
**left-arm** run (`20260811T220617`, 2774 samples):

| d₇ | right-arm median error | left-arm |
|---|---|---|
| **0.1612 (6FB)** | **0.01 mm** | **0.01 mm** |
| 0.1845 (6FB-V) | 23.30 mm | — |

23.30 mm is exactly the difference between the two. The `-6FB-V` URDF's extra
length is its **camera mount folded into link 7**, not kinematic length —
that URDF also carries `camera_rolink` (0.0197 kg) and `camera_link`
(0.0392 kg), the D435 that PHASE_PLAN F32 records as why the right arm's
payload reads 0.711 kg against the left's 0.567 kg.

**The right arm carries the camera; its kinematics are plain 6FB.** Model the
camera as payload, not as link length.

⚠ `rm_algo_set_dh()` mutates library state for the whole process. Anything
that calls it must restore it in a `finally`, or every later FK/IK in that
process is silently wrong. `verify_robot_model.py` does, and checks it did.

---

## 6. The right arm as our workspace models it

Per the instruction to use our saved `butterfli.urdf` for the right arm.
Its chain, traced:

| joint | type | parent | xyz (m) | rpy | axis |
|---|---|---|---|---|---|
| R_joint1 | revolute | R_base_link | 0, 0, 0.2405 | −1.5708, 0, 0 | 0 −1 0 |
| R_joint2 | revolute | R_Link1 | 0, 0, 0 | 1.5708, 0, 0 | 0 1 0 |
| R_joint3 | revolute | R_Link2 | 0, 0, 0.256 | −1.5708, 0, 0 | 0 −1 0 |
| R_joint4 | revolute | R_Link3 | 0, 0, 0 | 1.5708, 0, 0 | 0 1 0 |
| R_joint5 | revolute | R_Link4 | 0, 0, 0.21 | −1.5708, 0, 0 | 0 −1 0 |
| R_joint6 | revolute | R_Link5 | 0, 0, 0 | 1.5708, 0, 0 | 0 1 0 |
| R_joint7 | revolute | R_Link6 | 0, 0, **0.114** | 0, 0, 1.5708 | 0 0 1 |
| R_Link7CameraLink_Joint | fixed | R_CameraHolderLink | 0, 0, **0.05** | 0, 0, −1.5708 | — |
| R_ConnectorLink7_Joint | fixed | R_Link7 | 0, 0, 0.0125 | 0, 0, 0 | — |

Links 1–6 match RealMan exactly. The wrist is modelled differently: joint 7's
axis is placed at 0.114 and the camera holder carries the remaining 0.05, so
**joint 6 → flange is 0.164 m against the measured 0.1612 m — this URDF is
2.8 mm long at the flange.**

Placing joint 7's origin anywhere along its own axis is kinematically free,
so the 0.114 split is fine; the **2.8 mm total is not**. It will show up as a
constant offset between ROS-planned and controller-executed targets along the
tool axis. Reconcile it to 0.1612 before the bridge relies on either.

**And its inertials are not RealMan's** — see §7.

---

## 7. `butterfli.urdf` understates arm mass by 62 %

Both sides carry the same figures:

| link | butterfli.urdf (kg) | RealMan (kg) | ratio |
|---|---|---|---|
| Link1 | 0.5936 | 1.574 | 0.38 |
| Link2 | 0.4328 | 1.217 | 0.36 |
| Link3 | 0.4313 | 1.110 | 0.39 |
| Link4 | 0.2896 | 0.685 | 0.42 |
| Link5 | 0.2394 | 0.619 | 0.39 |
| Link6 | 0.2188 | 0.602 | 0.36 |
| Link7 | 0.0649 | 0.144 | 0.45 |
| **arm total** | **2.271** | **5.951** | **0.38** |

`R_CameraHolderLink` and `R_ConnectorLink` are both exactly **0.5 kg** —
round numbers, i.e. placeholders rather than measurements.

Its kinematics are sound (bar §6's 2.8 mm); its **dynamics are ~2.6× too
light**. Any torque, gravity-compensation or effort figure from that file is
wrong by that factor. Use §4.

---

## 8. Verification summary

| claim | checked against | result |
|---|---|---|
| DH reproduces the SDK's FK | 200 poses vs `rm_algo_forward_kinematics` | **0.00008 mm** |
| DH `d` == vendor MDH table | RealMan parameter page | exact |
| DH `d` == URDF joint origins | rm_models | exact |
| Σd − d₁ == published reach | 627.2 vs 627 mm | agrees |
| URDF masses == published self-weight | 7.813 vs 7.8 kg | agrees |
| joint ranges | SDK / vendor / URDF | agree ≤0.04° |
| joint speeds | SDK / vendor / controller | exact |
| singularity analyser flags all 4 vendor types | `rm_algo_universal_singularity_analyse` | 4/4 |
| our σ_min == the SDK's verdict | 8 configurations | 8/8 |
| **both arms are 6FB** | recordings, both sides | **0.01 mm** |
| tool+mount transform | `orientation_cost.selfcheck()` | 29.2 µm |

**Known gaps, stated rather than filled:** no per-joint rated torque; no
torque constant, so amps cannot become newton-metres; no dynamics call in the
SDK, so gravity compensation must be computed from §4 by our own code; the
library's 100 °/s² joint acceleration disagrees with the controller's
600 °/s² and neither is validated against motion; RealMan's per-variant
kinetic parameters (their §2.4) have not been transcribed — the URDF was used
instead, and the two have not been diffed.
