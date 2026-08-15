# Cleaning-motion design spec — 0.8 m/s program, wiping action, 4-minute activity

*2026-08-15. Research basis: commode_c fixture configs (32 task YAMLs, 20
regions, meshes + region annotations), glove_frames.yaml, the measured motion
laws in MOTION_FINDINGS §9, and a J4 screen of every task family. Targets set
by Newton: line speed toward 0.8 m/s, human-like wiping action, proximal
glove surfaces where primary contact degrades below half, whole activity
under 4 minutes. Task-config structure may change if speed is not lost.*

---

## 1. The budget arithmetic — what 4 minutes actually requires

Parsed from every commode_c cleaning config: **64.3 m (left) / 65.4 m
(right) of commanded cleaning path**, split across 16 tasks per arm, mostly
RL_paired (arms parallel). 240 s minus realistic overheads (task entries,
lid/seat articulation, UGV standoff moves at 0.25↔0.38 m) leaves roughly
**180 s of cleaning motion → a sustained average TCP speed ≥ 0.36 m/s.**
Best measured today (toplid rev 3 at the 0.45 baseline): **0.21 m/s
average**. The gap is ×1.7 — and it cannot be closed by one knob. Three
levers must move together:

1. **speed where physics allows it** (§2),
2. **path-length and overhead rationalization** (§5),
3. **the angular cap** — the quiet dominant limit (§2b).

## 2. Where 0.8 m/s is real — the per-family speed-ceiling table

J4 screen (exact, redundancy-invariant) of every left task at 0.25 m/s;
`v_ceil` = commanded line speed putting the worst segment at 90 % of J4.
(Screen vs measured calibration on toplid rev 3: screen 71 %, measured
59.5 % — the screen over-predicts ~15 %, so ceilings are conservative.)

| family | tool | worst J4 @0.25 | v_ceil m/s | throttled segs | verdict |
|---|---|---|---|---|---|
| lidsides_back | glove_3 | 18 % | **1.28** | 4/12 | 0.8 REAL |
| bowl_inside_back | glove_2 | 21 % | **1.10** | 11/11 | cap-bound (§2b) |
| seat_body_hinge | glove_2 | 21 % | **1.07** | 0/10 | 0.8 REAL |
| lid_seat_hinge | glove_2 | 26 % | **0.87** | 0/6 | 0.8 REAL |
| bowl_inside_ring | glove_1 | 33 % | 0.69 | 31/31 | cap-bound |
| seat_ring_bottom | glove_2 | 43 % | 0.53 | 41/55 | cap-bound |
| bottomlid | glove_2 | 50 % | 0.45 | 35/45 | mixed |
| lid_side | glove_1 | 50 % | 0.45 | 9/13 | mixed |
| bowl_inside_rim | glove_3 | 59 % | 0.38 | 22/23 | cap-bound |
| top | glove_2 | 65 % | 0.35 | 38/42 | cap-bound |
| hinge_area | glove_4 | 64 % | 0.35 | 5/43 | J4-bound |
| toplid | glove_2 | 70 % | 0.32 | 7/27 | J4-bound |
| **deep** | glove_1 | **118 %** | **0.19** | 41/44 | OVER at 0.25 |
| **side** | glove_1 | **142 %** | **0.16** | 18/45 | OVER at 0.25 |
| **seat_ring** | glove_2 | **162 %** | **0.14** | 35/52 | OVER at 0.25 |
| **front** | glove_1 | **293 %** | **0.08** | 17/40 | OVER at 0.25 |

Three consequences:

* **"0.8 m/s" is a per-move property, not a global setting.** Four families
  take it as-is; the per-move-v architecture (verified §9.3d) delivers it
  surgically. A global 0.8 would abort exactly like the old 0.45 toplid did.
* **Four families are illegal at today's 0.25** — `front`, `seat_ring`,
  `side`, `deep` (23.5 m, ~36 % of the path). These, not toplid, decide the
  4-minute question. Their J4 blowups are pose-family problems (frame_1
  X-press at low/awkward reach), fixable in order of preference: stroke
  DIRECTION change (J4/m is direction-dependent), ik_frame change with the
  §4 contact rule, UGV standoff change (the F49 lesson), pose-family
  redesign. Each needs its own screen-guided redesign like toplid got.
* **The angular cap 0.6 rad/s binds more path than J4 does** (§2b).

### 2b. The angular cap is the bigger half of the speed program

10 of 16 families have most segments throttled by TCP_ANGULAR_VELOCITY
0.6 rad/s — on those, raising line speed changes nothing. The cap is a
configurable limit exactly like line_speed (same vendor-advisory caveats,
H62, ratchet rule). Raising it toward 1.0–1.2 rad/s would directly raise
`v_eff` on curved/rotating strokes — the bulk of seat_ring, top, bowl and
rim work — and its joint-space cost lands mostly on the wrist (J5–J7),
which every measurement shows loafing (≤36 % in all REAL runs). **This is
the highest-leverage untested knob in the entire program.** Screen exactly
like the speed ladder: SIM first, one task, angular cap stepped 0.6 → 0.8
→ 1.0, H63 dwell rule on every joint.

### 2c. Acceleration and the scrub limit

acc = max(1.6, 3v) → at 0.8 the ramp is v²/2a = 133 mm each way: only
segments ≥ ~350 mm ever touch 0.8. Short strokes are acceleration-bound:
peak speed on a stroke of length L is √(aL) (triangle profile) — 0.44 m/s
on an 80 mm scrub at a=2.4. Human-scrub cadence lives here: an 80 mm
back-and-forth cycle costs ~0.4 s → **~2.5 Hz scrubbing is feasible** —
if the retraces stay crisp (measured: exact-180 reversals cross zero in
<200 ms; the apex reversal in rev 3 was <80 ms).

## 3. Wiping action — human motion mapped to controller primitives

What a hand does on a fixture, and the primitive that reproduces it:

| human element | robot primitive | status |
|---|---|---|
| long sweeping strokes on open surfaces | blended movel serpentine, padded turns | VERIFIED (§9.3g) |
| curved strokes following contours; rim/edge following | **`rm_movec(pose_via, pose_to, v, r, loop, connect, block)`** — circular arcs, chainable, blendable | EXISTS, UNTESTED |
| scrub cycles on soiled spots | short exact-180 retraces at max accel (§2c), 2–3 Hz | components verified |
| flowing direction changes, no dead stops | blend chains + turnarounds in padding | VERIFIED |
| varying contact pressure | NOT commandable (position control + glove compliance); hover offset is the proxy | out of scope |

The single biggest wiping-action win is **movec for the annular regions**.
`seat_ring`, `bowl_inside_rim`, `bowl_inside_ring` are circles authored as
10–20-segment polygons: every vertex is a corner that fights the angular
cap and the blend engine. As 2–4 chained arcs they have NO corners — the
tool sweeps continuously like a hand wiping a ring, and the angular cap
becomes the only (honest) speed limit. This also directly attacks the
`seat_ring` family's 0.14 m/s ceiling: its J4 spikes live at polygon
vertices where translation direction snaps.

**Required screen before design relies on it — `chain_semantics_006`:**
does movec accept connect=1 into a movel chain; does r blend arc→line; does
the first-corner exemption apply; what does `loop` do mid-chain; does the
UDP stream trace the commanded arc. Same one-geometry/three-outcomes
pattern as 001–003.

## 4. Proximal glove surfaces — the contact-area rule

Glove pads (glove_frames.yaml): frames 1/2 = 35×80 mm, t = 20 mm
compliance; frames 3/4 = 20×80 mm, t = 18 mm. Contact model (flat pad,
locally planar surface, misalignment θ between press axis and −normal):
the pad conforms up to depth t, so the contact band along the tilted axis
of dimension D is `min(D, t/sin θ)`:

    contact fraction  f = min(1, t / (D · sin θ))

* Tilt about the pad's SHORT axis (band runs along L=80): **f < ½ at
  θ > asin(2t/L) = 30.0°** (frames 1/2) or **26.7°** (frames 3/4).
* Tilt about the LONG axis (band across W=35): f = t/(W sinθ) ≥ 0.57 even
  at 90° — the width never drops below half. **The ½ rule therefore binds
  on lengthwise tilt only: keep the pad's LONG axis within 30° of the
  surface, or switch frames.**
* Curvature corrections (be careful — Newton's warning): on a CONVEX
  surface of radius R the flat-pad contact length is ~2√(2tR) — for
  R ≥ 80 mm this exceeds L and changes nothing; below R ≈ 40 mm it, not
  tilt, caps contact (seat-ring outer edges, rim lips). CONCAVE surfaces
  only help. Combined rule: f = min(f_tilt, 2√(2tR)/L on convex work).
* The 1.5 cm dimensional tolerance already derates the footprint for
  coverage spacing; the ½-area rule above is about PRESSURE/wipe quality,
  a separate check. Both must pass.

**Where it applies now:** the four over-limit families all press with
frame_1 (X) at stretched poses. Candidate: clean `front`/`side` upper
bands with frame_2 (Z-press, wrist rolled 90°) whose pose family the J4
screen scores far lower — legality decided by computing θ per cleaning
point (surface normal from the region OBJ) against the achievable tool
orientation, applying f ≥ 0.5. This computation belongs in the generator,
not in hand-tuning: emit `ik_frame` per STAGE, chosen by best f.

## 5. Path & overhead rationalization

* **Regenerate paths at glove-complete density with the §9 recipe**
  (two-pass serpentine, padded turns, per-move v/r, chained approach). The
  toplid case: original 6.2 m was under-covered; the correct-coverage
  redesign is 7.4 m but at a far higher achievable average. Some configs
  carry redundant double-passes (toplid's old rim down+up) — regeneration
  recovers those meters.
* **Task chaining.** The rest→prestart→approach entry per task was a TEST
  protocol. In production, consecutive tasks in the same articulation
  state should chain: task-end → next prestart directly. At 16 tasks/arm
  and ~5–8 s per full entry, this is **60–90 s of wall clock** — possibly
  the single largest line item after the slow families.
* **Schedule around articulation**: group closed-lid regions, then
  lid_open, then lid_seat_open (regions.yaml already encodes states);
  overlap UGV standoff moves with the other arm's cleaning where the
  pairing allows.

## 6. Task-config schema extension (backward compatible)

Bare `[a, b]` sequence entries stay legal (defaults apply). Extended form:

```yaml
cleaning_sequence:
  - [point1, point2]                          # legacy: task defaults
  - {seg: [point2, point3], v: 100, r: 35}    # per-move speed/blend
  - {arc: [point3, point5], via: point4, v: 80, r: 25}   # movec
  - {scrub: [point5, point6], cycles: 3, v: 100}         # retrace n times
stage_sequence:                                # per-stage frame switch
  - {stage: upper_band, ik_frame: L_glove_frame_2}
  - {stage: lower_band, ik_frame: L_glove_frame_1}
```

`v` is % of the task baseline (movel semantics), `r` per move; a scrub
expands to n exact-180 retraces (coverage-safe, §9.2). The generator owns
frame selection per stage via the §4 rule. stage_runner/test harness read
the same structure the screens already understand (R_LIST/V_LIST map 1:1).

## 7. The projected budget (honest, with today's numbers)

Efficiency factor measured (avg TCP / commanded cap): 0.70 at 0.25,
0.72 at the v45 mixed baseline. Applying family ceilings × 0.7, per arm:

| group | path | assumed avg | time |
|---|---|---|---|
| fast four (0.8 program) | ~4.6 m | 0.45 | ~10 s |
| medium (at ceiling, angular cap 0.6) | ~36 m | 0.22 | ~165 s |
| slow four (AFTER redesign, target ≥0.25 ceiling) | ~23.5 m | 0.17 | ~140 s |
| **total motion** | **64 m** | | **~315 s** |

**Over budget at today's angular cap — by design honesty.** The two
closures: (a) the angular-cap raise (§2b) plausibly moves the medium group
0.22 → 0.30+ (−45 s) and slow group similarly (−30 s); (b) task chaining
recovers 60–90 s of overhead that the 240 s must also contain. With both,
the activity lands at ≈ 210–240 s. **The 4-minute target is reachable,
and it is reachable ONLY with the angular-cap program and the slow-family
redesigns — not with line speed alone.**

## 8. Execution order (each step SIM-gated like everything in §9)

1. `chain_semantics_006` — movec chain/blend semantics (SIM).
2. Angular-cap ladder on one cap-bound task (top_left): 0.6/0.8/1.0, SIM
   then REAL, H63 dwell on all joints.
3. Redesign the four over-limit families (direction/frame/standoff via §4
   rule + §5 recipe); screen; SIM; REAL at each family's ceiling.
4. Regenerate annular tasks as movec arcs; A/B against polygon versions.
5. Task-chaining entry in stage_runner (same-articulation groups).
6. Full-activity dual-arm dry-run in SIM; walltime audit vs §7 table.

Data gaps this closes en route: movec semantics, angular-cap scaling law,
J4-vs-direction maps for the slow families, dual-arm chained dispatch
(EMULATOR_ROADMAP B9).
