# Frame map — URDF ↔ RealMan controller

**Generated. Do not hand-edit** — regenerate with:

```bash
python3 src/frame_alignment_offline.py --map
```

The single source is `frame_alignment_offline.IK_FRAMES` (verbatim from
`butterfli_description/urdf/ik_frames.xacro`) plus
`ARM_TIP_TO_CONNECTOR_M`. The same `frame_map()` feeds this document, the
frame writer, and the read-back verifier, so the three cannot disagree.

## Why the names differ

The cleaning points name their `ik_frame` in URDF terms
(`ik_frame: R_glove_frame_4`). For a point to be commanded **directly** on
the controller, the controller must hold a tool frame meaning the same
thing. Its name field is `c_char_Array_12` — **11 usable characters** —
while `R_glove_frame_4` is 15, so one mechanical rule bridges them:

> **controller name = URDF link name with the `_frame` token removed**

Reversible, collision-free, and every name fits (`R_index_tip` lands
exactly on the limit). Truncating instead would map all four glove frames
onto `R_glove_fra` — four frames, one name.

## Why the offsets differ by 15.3 mm

The URDF offsets are measured from `*_ConnectorLink`; controller tool
frames are measured from `Arm_Tip`. Those two points are the same physical
location plus a constant:

> `ConnectorLink = Arm_Tip + (0, 0, 15.3 mm)`, zero rotation, both arms

proved over five joint configurations with 0.00 mm spread and 0.000°
rotation (C14 offline). The value depends on the arm variant: it must be
computed with `RM_MODEL_RM_ISF_E`, since `RM_MODEL_RM_B_E` has no wrist
force sensor and is 17.2 mm short (F15).

## The table

```
RIGHT arm — URDF frame <-> controller tool frame
  URDF link (MoveIt / TF)  controller frame    from ConnectorLink (mm)        from Arm_Tip (mm)
  --------------------------------------------------------------------------------------------
  R_glove_frame_1          R_glove_1             50.0     0.0   145.0      50.0    0.0  160.3
  R_glove_frame_2          R_glove_2             13.5     0.0   165.0      13.5    0.0  180.3
  R_glove_frame_3          R_glove_3             75.0     7.0   170.0      75.0    7.0  185.3
  R_glove_frame_4          R_glove_4             55.0     7.0   205.0      55.0    7.0  220.3
  R_tip_frame              R_tip                 15.0     5.0   230.0      15.0    5.0  245.3
  R_index_tip_frame        R_index_tip           24.2    28.8   225.0      24.2   28.8  240.3
  --------------------------------------------------------------------------------------------
  rule: controller name = URDF link with '_frame' removed  |  Arm_Tip -> ConnectorLink = 15.3 mm on Z, zero rotation

LEFT arm — URDF frame <-> controller tool frame
  URDF link (MoveIt / TF)  controller frame    from ConnectorLink (mm)        from Arm_Tip (mm)
  --------------------------------------------------------------------------------------------
  L_glove_frame_1          L_glove_1            -50.0     0.0   145.0     -50.0    0.0  160.3
  L_glove_frame_2          L_glove_2            -13.5     0.0   165.0     -13.5    0.0  180.3
  L_glove_frame_3          L_glove_3            -75.0     7.0   170.0     -75.0    7.0  185.3
  L_glove_frame_4          L_glove_4            -55.0     7.0   205.0     -55.0    7.0  220.3
  L_tip_frame              L_tip                -15.0     5.0   230.0     -15.0    5.0  245.3
  L_index_tip_frame        L_index_tip          -24.2    28.8   225.0     -24.2   28.8  240.3
  --------------------------------------------------------------------------------------------
  rule: controller name = URDF link with '_frame' removed  |  Arm_Tip -> ConnectorLink = 15.3 mm on Z, zero rotation
```

**Left-arm values re-cut to MIRROR the right (alix_ws, corrected here
2026-08-20):** `L_glove_1` z 140 → 145, `L_glove_2` x −20 → −13.5. REAL
logs from 2026-08-10/11 predate the re-cut — the controller held the OLD
values then, so do **not** back-fit those two frames from those logs
(`L_glove_4` did not move, which is why it validates). The 15.3 mm
Arm_Tip → ConnectorLink offset is hardware-confirmed: Kabsch fit of
emulator FK to recorded controller TCP, residuals 0.008 mm (R) / 0.019 mm
(L) at +2.8 mm over the URDF's 12.5 (alix-ws-54, 2026-08-20).

## Pending controller update — APPROVED (Newton, 2026-08-20)

The controllers hold the PRE-recut left values (`L_glove_1` at 155.3 z,
`L_glove_2` at −20.0 x). Newton approved the update pass and the right-arm
payload correction. Rehearsed against the emulator 2026-08-20, two full
passes, both arms: update routing correct, `*_index_tip` 10-char collision
resolved, match tables 0.00, restores verified.

**Left arm — ready to run as-is** (values come from the corrected table):

```bash
cd Demo/RMDemo_Python/RMDemo_DualArmConcept/src
RM_ARM=left python3 test_frame_alignment.py --mode REAL --create-frames
```

All six frames must print `update … ret=0` (a `create` on the left pass
means the name list changed — stop and look). The MATCH TABLE must be all
OK **including payload/centroid**; an "exactly 1000x" flag is a unit
mismatch — do not select any frame or run any movel until it is resolved.
Left payload stays 0.706 kg copied from `Hand` (unchanged).

**Right arm — parked on ONE input:** the frames carry payload 0.0 kg
(copied from `Arm_Tip`), so right-side force compensation runs unloaded.
Needs the TRUE right hand+glove mass and centroid from Newton — if the
right assembly mirrors the left `Hand` (0.706 kg at (−12, 44, 128) mm),
the mirrored centroid is (+12, 44, 128) mm, but that is a HYPOTHESIS to
confirm, not a value to write. Then:

```bash
RM_ARM=right python3 test_frame_alignment.py --mode REAL --create-frames \
        --payload <KG> --com <X,Y,Z in mm>
```

## Verification

`test_frame_alignment.py --create-frames` writes these frames, then reads
every one **back off the controller** and prints a match table with a
per-row delta. Tolerance is 0.5 mm (float round-trip); anything larger
fails the gate. Writing returning `ret=0` is not evidence the controller
holds the value you meant — the read-back is.
