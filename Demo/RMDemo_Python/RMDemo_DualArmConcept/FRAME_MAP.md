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

**Payloads are NOT copied — pass them explicitly on BOTH arms.** The
writer's default copies the ACTIVE frame's payload/centroid onto every
frame it writes; on the left the active frame is `Hand` (0.706 kg at
(−12, 44, 128)), which would overwrite the glove frames' **calibrated
consensus** (`payload_audit.py`, measured per-arm 2026-08-10/11,
flange-relative — the earlier "mirror the left" idea in this section is
REFUTED by that measurement: centroids do not mirror, CX is negative on
both arms, and the right is 25 % heavier because a D435 rides distal of
J7):

    left   0.567 kg at (−25.2, 41.2, 224.0) mm    (4 of 6 frames agreed)
    right  0.711 kg at (−23.6, 25.4, 164.8) mm    (5 of 6 frames agreed)

```bash
cd Demo/RMDemo_Python/RMDemo_DualArmConcept/src
RM_ARM=left  python3 test_frame_alignment.py --mode REAL --create-frames \
        --payload 0.567 --com "-25.2,41.2,224.0"
RM_ARM=right python3 test_frame_alignment.py --mode REAL --create-frames \
        --payload 0.711 --com "-23.6,25.4,164.8"
```

All six frames must print `update … ret=0` (a `create` means the name
list changed — stop and look). The MATCH TABLE must be all OK **including
payload/centroid**; an "exactly 1000x" flag is a unit mismatch — do not
select any frame or run any movel until it is resolved. After both
passes, `python3 payload_audit.py` re-reads both arms and must report the
same consensus back. (The right side's payload state was MIXED at the
last audit read — 5 of 6 frames at 0.711, and a later frame write stamped
0.0 from `Arm_Tip` — the explicit values above normalize it either way.)

## Verification

`test_frame_alignment.py --create-frames` writes these frames, then reads
every one **back off the controller** and prints a match table with a
per-row delta. Tolerance is 0.5 mm (float round-trip); anything larger
fails the gate. Writing returning `ret=0` is not evidence the controller
holds the value you meant — the read-back is.
