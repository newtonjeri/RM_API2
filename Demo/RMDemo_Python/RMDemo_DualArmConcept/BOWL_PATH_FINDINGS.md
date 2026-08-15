# Bowl coverage path — findings against the real system

Answers to the four kickoff questions and the two dual-arm questions from the handoff
brief, now that the machine is reachable. Every number below is reproduced by
`bowl_audit.py` against the actual `commode_c` meshes and the actual `alix_taskgen`
pipeline; nothing here is carried over from the brief's idealised profiles.

---

## 0. What the brief got right, and the one thing it got wrong

**Right, and better than it had any right to be.** The curvature-specified idealised
profile predicted the bowl to within a few percent of the real mesh:

| quantity | brief (idealised) | real mesh | error |
|---|---|---|---|
| bowl interior wetted area | 1.80 ft² | 1.87 ft² | +4 % |
| bowl depth | 239 mm | 197 mm | −18 % |
| main interior wall \|k_perp\| | 21 m⁻¹ | 13.7–21.7 m⁻¹ | in range |
| rim region \|k_perp\| | 50–100 m⁻¹ | 65–96 m⁻¹ | in range |
| rim fillet radius | 10–20 mm | 10–15 mm | in range |

R1–R4 all survive contact with the meshes. The competence-envelope idea, the
two-principal-curvatures split, and "the rim is a tooling problem, not a path problem"
are all confirmed on real geometry.

**Wrong: the objective.** The brief assumed adaptive pitch was needed to *rescue
coverage*. Measured, the shipped planner already reaches **99.5–100 %** coverage on
every bowl-interior region. What it does not do is reach it cheaply: redundancy is
**2.5–6.5×** against the project's own 1.6 target and its 1.25 over-covered flag.

That matters because of the other thing the machine says: the bowl is
**angular-velocity-bound, not speed-bound**. At the 0.60 rad/s TCP cap the bowl runs at
**30–124 mm/s** and no controller setting changes that — raising the cap to 1.0 gives
50–207 mm/s, still far under the **0.36 m/s sustained** the 4-minute budget needs.

> **The bowl cannot be made faster. It can only be made shorter.**
> Which makes redundancy reduction — the adaptive-pitch payoff — the only lever that
> works there. The brief's machinery is right; point it at path length, not coverage.

---

## 1. Regions vs the R0/R1/R2 decomposition

The real decomposition is **not** depth-banded. `models/regions.yaml` cuts the bowl
interior **azimuthally as well as by depth**, and allocates the azimuthal halves to
different arms:

| region | id | arm | area | maps to brief |
|---|---|---|---|---|
| `bowl_inside_rim` | 15 | RL_paired | 0.0506 m² | **R0** (rim) |
| `bowl_inside_ring` | 9 | RL_paired | 0.0276 m² | R0/R1 boundary |
| `bowl_inside_front` | 7 | **R only** | 0.0772 m² | R1+R2, front sector |
| `bowl_inside_back` | 8 | **L only** | 0.0180 m² | R1+R2, back sector |
| `deep` | 20 | RL_paired | — | R2 (compound: ring→back) |
| `top` | 6 | RL_paired | 0.0514 m² | rim top face |

Three consequences:

- **R1 is falsified as stated.** The brief's "closed in azimuth ⇒ single continuous
  helix with zero lift-offs" cannot hold: the interior is already cut into open
  per-arm sectors. Each arm gets an open region and pays turnarounds. The zero-lift-off
  helix only returns if the two arms' sectors are stitched into one path, which the
  data model has no way to express.
- **`bowl_inside_back` is too small to plan.** 0.0180 m² ≈ 3 pad footprints. With the
  30 mm blend floor enforced you cannot place a raster in it at all — my prototype
  drops to 86.7 % there. This is an argument *for* the existing `deep` compound region
  (ring + back planned as one chain) and against planning `bowl_inside_back` alone.
- **`bowl_inside_rim` has an articulation-state bug** (see §5.1).

---

## 2. Real curvature and the competence envelope

Local quadric fit, 30 mm neighbourhood, resolved into along-stroke (`k_par`) and
across-stroke (`k_perp`) for contour-parallel strokes:

| region | k_par p90 | **k_perp p90** | k_perp p95 | fillet R | usable swath @η=0.6 |
|---|---|---|---|---|---|
| `bowl_inside_front` | 13.5 | 13.7 | 19.0 | 73 mm | 68 mm |
| `bowl_inside_back` | 20.4 | 21.7 | 28.5 | 46 mm | 54 mm |
| `bowl_inside_ring` | 15.5 | **65.0** | 77.2 | 15 mm | **31 mm** |
| `bowl_inside_rim` | 10.6 | **69.8** | 81.8 | 14 mm | **30 mm** |
| `top` | 7.0 | **96.2** | 146.0 | 10 mm | **26 mm** |
| `sides` | 30.2 | 17.7 | 30.5 | 57 mm | 60 mm |

**R2 is confirmed and is strongly anisotropic.** On ring/rim/top the across-stroke
curvature is **4–9×** the along-stroke curvature. Contour-parallel strokes therefore run
along the low-curvature direction, which is correct — but it means across-stroke
conformance is what binds.

**The envelope threshold changes, because the pad is not what the brief assumed.**
Real pad, from `config/glove_frames.yaml` + `alix.urdf`:

| frame | footprint W×L | compliance t | press axis |
|---|---|---|---|
| glove_1 | 35 × 80 mm | 20 mm | **±X — lateral** |
| glove_2 | 35 × 80 mm | 20 mm | +Z — axial |
| glove_3 / _4 | 20 × 80 mm | 18 mm | +Z — axial |

vs the brief's assumed 90 × 60 mm, δ₀ = 8 mm. With `l ≤ √(2(1−η)t/|k|)`, the threshold
at which the full 80 mm across-stroke edge stops conforming is

    k* = 2(1−η)t / (L/2)²  =  2(0.4)(0.020) / 0.040²  =  **10 m⁻¹**

**not 28 m⁻¹.** The 80 mm edge is nearly 3× more curvature-sensitive than the brief's
90 mm edge because the compliance gain (20 mm vs 8 mm) does not offset the length. So
**every bowl region is outside the envelope for the full 80 mm swath** — front and sides
mildly (68 / 60 mm usable), ring/rim/top badly (26–31 mm usable, a **2.6–3.1× shortfall**
against the 80 mm the planner assumes).

The project's coverage model does not see this, because its conformance test is a
**±20 mm depth band** — equivalent to η ≥ 0, i.e. accepting *zero contact pressure* at
the swath edge. Coverage passes at 99.5 % on the rim while the outer 25 mm of each pass
is making no useful pressure. That gap between "geometrically swept" and "actually
wiped" is the single most important thing the coverage number is hiding.

---

## 3. URDF: wrist clearance and the cranked mount

**R4's recommendation is already implemented — on `glove_frame_1` only.**

All glove frames hang off `*_ConnectorLink` with **`rpy = "0 0 0"`** (confirms
`orientation_cost.py`'s zero-rotation model is correct — that was an open question):

```
R_glove_frame_1   xyz = 0.05   0.0    0.145     press ±X   -> shank axial, pad LATERAL
R_glove_frame_2   xyz = 0.0135 0.0    0.165     press +Z   -> shank axial, pad AXIAL
R_glove_frame_3   xyz = 0.075  0.007  0.17      press +Z
R_glove_frame_4   xyz = 0.055  0.007  0.205     press +Z
```

`glove_frame_1` extends **145 mm axially** and presses **sideways** — exactly the brief's
"crank the mount so the shank runs parallel to the fixture axis with the pad presented
radially; it then never crosses the cavity." So the 79 mm sump-neck clearance problem
**does not arise for glove_1**, and the regions that most need it already use it
(`bowl_inside_ring`, `deep`).

`bowl_inside_rim` uses **glove_3** — axial press, and the *narrow* 20 × 80 mm pad. That is
the wrong presentation for a rim and the worst pad for it. Switching the rim to a
lateral-press frame is the highest-value single change in this document.

Also from the URDF: `R_base_joint rpy = (0, π/2, 0)` confirms the Ry(+90°) install pose,
and **`glove_frame_4` exists as a real frame** — contradicting `glove_frames.yaml`'s
comment that it is "not built yet — commented out below".

---

## 4. RM_API2: how the path ships

**There is no continuous Cartesian servo mode.** The only streaming primitive is
`rm_movej_canfd` at 100 Hz, **joint space**, used solely for the arm↔pole sync
workaround. Cartesian motion is discrete queued `rm_movel` / `rm_movec`.

This kills the brief's §2.2 waypoint-discretisation table outright:

- **`r` is a percentage, not a length** — of the *shorter adjoining segment*, with
  `cut ≈ 1.3–1.5 × (r/100) × min(L_in, L_out)` and the design rule `r ≈ 133·δ/L`. The
  brief's "keep the blend radius below the local pitch" has no referent.
- **`trajectory_connect` is a latch, not a blend.** With `r=0` the tool stops dead at
  every waypoint even though the chain is connected — median speed retained **2.4 %**.
- **There is a blend floor at ~25–30 mm.** Below it the controller substitutes a full
  stop — measured 29 stops in 40 corners on one task — and it is *not* a clean length
  threshold (at 0.25 m/s a 21.3 mm hop blended while 22.0 / 24.6 / 27.5 mm stopped).
- **Short segments are acceleration-limited**: peak speed on a stroke of length L is
  √(aL), so an 8 mm segment tops out at ~0.11 m/s regardless of commanded speed.
- **`r ≥ 25` on dense paths triggers multi-second planner freezes** (22.5 s of a 60 s
  run in nine freezes), reproduced on hardware. Operating guidance is **r = 10** on
  dense task paths.

The brief proposed 8–30 mm chord spacing. **Every segment would sit in the dead zone.**

**And the shipped paths are already partly in it:** 8–12 % of segments in every bowl
region are under 30 mm today, with minima of 1.5–11 mm. That is a live defect, not just
a risk for the proposal.

**What to do instead.** The brief's variable-pitch helix should ship as **chained
`rm_movec` arcs**, which the project has already hardware-verified: radial error median
0.37–0.43 mm, junctions crossed at 93–212 mm/s with **no stops**, roughly **2× the corner
speed** of blended `movel`. A bowl contour is locally an arc; 2–4 arcs per turn gives
arc chords of 100–180 mm — far above the blend floor — and collapses the move count from
hundreds to ~20, inside the queue depth of 20 that C10 actually verified. There is also
an unscreened `loop` parameter on `rm_movec` that would wipe a ring N times in one
command with zero junctions.

---

## 5. Defects found (ranked, all checkable)

### 5.1 `bowl_inside_rim` is declared unreachable
`regions.yaml:118-125` sets `lid_state: closed, seat_state: closed` on
`bowl_inside_rim`, directly under the comment (line 91) stating *"Bowl-interior regions
are cleaned through the seat opening with BOTH the lid and the seat raised."* Every
other `bowl_inside_*` region is `open/open`. With the seat down, the seat ring physically
covers the bowl rim. Almost certainly should be `open/open`.

### 5.2 Row spacing is metered in the projection plane, not on the surface
`boustrophedon.py` rasterises in the region's PCA best-fit plane, so the true
on-surface row pitch is `nominal / |n_face · n_plane|`:

| region | inflation p50 | p90 | true pitch at 50 mm nominal |
|---|---|---|---|
| `bowl_inside_ring` | **5.38×** | 36.2× | **269 mm** |
| `sides` | **9.35×** | 52.0× | **467 mm** |
| `bowl_inside_rim` | 2.61× | 98.0× | 131 mm |
| `bowl_inside_front` | 1.56× | 2.14× | 78 mm |
| `top`, `bowl_inside_back` | 1.00–1.01× | ≤1.10× | 50 mm ✓ |

One nominal number means 50 mm on a flat patch and 269 mm on the bowl ring. This is the
root cause of **both** failure modes: narrow wrapped regions get rows piled on top of
each other (ring redundancy 6.48), wide wrapped regions get gaps (`sides` **87.8 %**,
below the 95 % gate; `top` **85.8 %**).

### 5.3 The competence envelope is unmodelled
Covered in §2. The ±20 mm depth band is an η ≥ 0 test where the wipe-quality rule needs
η ≥ 0.5–0.6. `CLEANING_MOTION_SPEC` already derives the tilt half of this
(`f = min(1, t/(D·sinθ))`) but the curvature half is only stated for *convex* work
("concave surfaces only help") — which is true for contact *length* and false for
contact *pressure*. Both belong in the generator.

### 5.4 `glove_frames.yaml` contradicts itself
The spec table (lines 47–52) says `glove_frame_2  t = 30` and `glove_frame_4  80×35, t=10,
not built yet — commented out below`. The `frames:` block declares frame_2 at
`thickness_m: 0.020` and frame_4 at `[0.020, 0.08], 0.018` — and frame_4 is neither
commented out nor absent from the URDF. Since `t` *is* the conformance depth, this
propagates straight into any envelope calculation.

### 5.5 Left/right tool frames are asymmetric in the URDF
`glove_frame_1` Z: L = 0.140, R = 0.145 (**5 mm**). `glove_frame_2` X: L = −0.020,
R = +0.0135 (**6.5 mm**). Every other frame mirrors exactly. Either a real physical
asymmetry that should be documented, or a URDF bug — worth a tape measure.

### 5.6 `surface_type` detection is winding-dependent
`concave_fraction` uses raw face normals, so flipping face orientation changes the
answer (`sides`: double_curved → concave_interior in my run). Harmless today because
both branches route to boustrophedon, but the `concave_fraction > 0.6` gate fires
*before* the annular test, so `top` (hole 0.62, spread 13.6°) and `seat_ring` never reach
`racetrack`. `plan_contour_parallel` is imported and **never dispatched to**.

---

## 6. Prototype: on-surface, curvature-adaptive rows

`surface_spacing.py` — level-set contours metered **on the surface**, pitch set by local
`k_perp` at η ≥ 0.6 with 20 % overlap, level-set field chosen automatically from the
region's own orientation (depth contours for walls, radial contours for shelves/rims),
and the 30 mm blend floor enforced by construction. Scored with the project's own
`compute_coverage`:

| region | planner | n | cov % | redundancy | path m | <30 mm |
|---|---|---|---|---|---|---|
| `bowl_inside_front` | shipped | 34 | 100.0 | 2.46 | 2.59 | 9 % |
| | **prototype** | 39 | 99.6 | **1.93** | **1.94** | **0 %** |
| `bowl_inside_ring` | shipped | 36 | 99.8 | 6.48 | 2.70 | 11 % |
| | **prototype** | 20 | 97.7 | **3.62** | **1.51** | **0 %** |
| `top` | shipped | 49 | **85.8** | 4.96 | 3.86 | 8 % |
| | **prototype** | 71 | **94.7** | 5.91 | 4.12 | **0 %** |
| `bowl_inside_back` | shipped | 17 | 100.0 | 4.52 | 1.10 | 12 % |
| | prototype | 11 | **86.7** ✗ | 2.32 | 0.56 | 0 % |
| `bowl_inside_rim` | shipped | 53 | 99.5 | 5.12 | 4.37 | 10 % |
| | prototype | 5 | **15.8** ✗ | 0.68 | 0.78 | 0 % |
| **total** | shipped | 189 | | | **14.62** | |
| | **prototype** | 146 | | | **8.92 (−39 %)** | |

**Works on 3 of 5.** Ring is the headline: **−44 % path at −2.1 points of coverage**, and
short segments eliminated. `top` gains **+8.9 points** and clears the 95 % gate. Front
holds coverage at −25 % path.

**Fails on 2, for understood reasons.** `bowl_inside_rim` is a band that wraps in both
depth and radius, so neither pure level-set field works — it needs a genuine geodesic
field. `bowl_inside_back` is 0.018 m², smaller than a raster can occupy at a 30 mm blend
floor; it should be planned inside `deep`, not alone.

**Time implication.** Holding commanded speed fixed, the bowl group's contribution falls
from ≈165 s to ≈100 s. Against a 240 s total budget of which these five regions are 23 %
of the path but ~69 % of the time, that is the largest single saving available — and it
comes from geometry, not from any controller setting.

---

## 7. The two dual-arm questions

**Does the second arm hold or clean?** It cleans. There is no "hold" mode anywhere in
the data model — `arm_coverage` is only `R | L | RL_paired | RL_compound`. So the
brief's hope that arm 2 could present a rim tool while arm 1 sweeps is **not expressible
today**; it would need a new coverage class and a mechanism template. Given §3, the
cheaper route to the same end is to switch `bowl_inside_rim` from `glove_3` to a
lateral-press frame — one config change instead of an architecture.

**Do two arms remove the need to reposition the base?** Partly, and it is already
assumed: `bowl_inside_front → R`, `bowl_inside_back → L` *is* an azimuthal split of the
bowl between arms. But it is asserted by hand in `regions.yaml` and has **never been
verified**, because the reachability module cannot verify it — `reachability.py`
voxelises tip **position** only (0.04 m, 70×70×50), discards orientation entirely, and
then runs `binary_closing` + `fill_holes`, which erases genuine interior voids. It
cannot answer "reachable *with the orientation this waypoint demands*", which is exactly
the azimuthal question. The one recorded per-region reach fact in the whole system is
F49 (`seat_ring_sides_front`: right 0/13, left 14/14 at 0.25 m, fixed by standoff 0.38 m).

**So the honest answer is: unknown, and unknowable with the current tool.** An
orientation-aware reach check is the prerequisite for settling it.

---

## 8. What I'd do next, in order

1. **Fix `bowl_inside_rim`'s `lid_state`/`seat_state`** — one line, and it may be
   invalidating the region's whole plan.
2. **Switch `bowl_inside_rim` to a lateral-press frame** (`glove_1`-style). Biggest
   wipe-quality gain per unit effort; §2 + §3.
3. **Meter row spacing on the surface.** `sides` and `top` are below the 95 % gate today
   purely from projection distortion, and the ring wastes 44 % of its path to it.
4. **Add the conformance gate to the generator**: usable swath `2√(2(1−η)t/k_perp)`
   capped at the pad edge, η ≥ 0.6. This is the brief's contribution and it drops in
   cleanly; it is what turns the redundancy number honest.
5. **Re-emit the annular regions as `rm_movec` arcs** rather than 10–20-segment polygons.
   Already hardware-verified, and it removes the blend-floor exposure entirely.
6. **Build an orientation-aware reach check** before trusting the front/back arm split.

Not worth doing: the brief's variable-pitch *helix* as a single continuous stroke. The
azimuthal region split, the absence of Cartesian streaming, and the blend floor each
independently prevent it. The pitch *schedule* is the valuable part; the helix
topology is not.
