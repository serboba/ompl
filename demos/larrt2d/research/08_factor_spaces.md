# Heterogeneous factor spaces — metrics, sampling measure, and workspace normalization for LA-RRT

> Context: LA-RRT plans in a compound space `Q = Q_1 × … × Q_n` whose factors are of *different
> kinds*: a 1-DOF revolute door (bounded interval on SO(2), arc-length geometry), a 1-DOF prismatic
> slider, a 2-DOF translating box (rectangle in R²), and a 3-DOF box in SE(2) (R² × SO(2)).
> Extensions move ONE factor at a time, range-limited by an L1 metric summed over that factor's
> dims — today **a raw radian counts exactly like a raw meter** (`LARRT.cpp` `growTree`, lines
> 354–361). The user's concern is exactly the classic one: a revolute-only factor behaves
> completely differently from two prismatic DOF, and mixing them biases the search.
> This briefing surveys (1) metric/weight choice in compound spaces, (2) RRT's specific
> sensitivity to it, (3) how composite/multi-robot planners sidestep it, (4) the sampling-measure
> mismatch between factors, and (5) workspace-displacement metrics as the principled
> normalization; then gives concrete LA-RRT recommendations. Extends briefing
> `06_constraint_sampling_deep_dive.md` (which covered *where* to sample — goals/buffers/quotients;
> this one covers *in what geometry* to measure, step, and allocate). Citations verified;
> unverified items flagged. Produced by a research agent, 2026-07-04.

---

## 1. Where our code stands today (the concrete problem)

Facts from the tree (all paths absolute in this worktree):

- **Extension range**: `LARRT.cpp:352–361` picks a random differing factor `g` and computes
  `dg = Σ_idx |n[idx] − r[idx]|` over `g`'s dims, then caps the step at `maxDistance_ / dg`.
  Two issues: (i) *units* — for the door factor `dg` is radians, for a box it is meters, and the
  same `maxDistance_` caps both; (ii) *wrap bug-in-waiting* — unlike `groupDistance()`
  (`LARRT.cpp:179–200`, which wraps SO(2) dims via `FactoredStateSpace::isAngleDim`), the `dg`
  computation and the linear interpolation at line 361 use **raw coordinate differences with no
  2π wrap**, so a seam-crossing door sample would be measured and interpolated the long way
  around. Harmless while doors live in a bounded interval well inside (−π, π], but it is a
  latent inconsistency between the NN metric and the extension geometry.
- **Range default**: `SelfConfig::configurePlannerRange` (`src/ompl/tools/config/src/SelfConfig.cpp:98`)
  sets `maxDistance_ = 0.2 × maximumExtent` of the **joint** space; for a compound space
  `maximumExtent = Σ_i w_i · extent_i` (`StateSpace.cpp:1006–1007`). So the per-factor step cap is
  scaled by the size of the *whole arrangement space* (grows with n), not by any single factor —
  adding objects silently enlarges every object's step.
- **Factor selection**: uniform over differing factors (`LARRT.cpp:352`). For a random continuous
  sample, essentially all factors differ, so this is uniform over objects — a 1-DOF door gets the
  same extension budget as a 3-DOF box regardless of how large or "important" its factor is.
- **OMPL's compound machinery** (what we inherit if we lean on it):
  - distance = weighted sum `Σ w_i d_i` (`StateSpace.cpp:1080`);
  - `sampleUniform` on a compound sampler samples **every component independently from its own
    uniform measure** — the subspace weights play *no role at all* in uniform sampling
    (`src/ompl/base/src/StateSampler.cpp:47–52`); weights only scale the per-component radius in
    `sampleUniformNear`/`sampleGaussian` (`StateSampler.cpp:54–71`);
  - stock weights are eyeballed constants: SE(2) = {R²: 1.0, SO(2): **0.5**}
    (`SE2StateSpace.h:110–111`), SE(3) = {R³: 1.0, SO(3): 1.0} (`SE3StateSpace.h:119–120`). There
    is no principled rule in the library; the manual expects the user to set weights.

So: the L1-radians-as-meters range, the joint-extent-scaled default, and the uniform factor pick
are all *choices by omission*, and every one of them is exactly the kind of choice the literature
below says planners are sensitive to.

---

## 2. Metrics on compound / SE(n) spaces — what theory allows and practice does

### 2a. There is no canonical metric; any mix of meters and radians embeds a length scale

- **Park, "Distance Metrics on the Rigid-Body Motions with Applications to Mechanism Design",
  ASME J. Mechanical Design 117(1):48–54, 1995.**
  The foundational negative result: **no bi-invariant metric exists on SE(3)**, and because
  physical space has no natural length scale, *any* metric on SE(3) necessarily depends on a
  chosen length scale. Left-invariant metrics can be built, parameterized by that scale.
  Consequence for us: the question is never "what is *the* metric" but "which length scale is
  *task-appropriate*" — the literature's task-appropriate answer turns out to be a body-dependent
  one (§6).
- **Žefran, Kumar & Croke, "Metrics and Connections for Rigid-Body Kinematics", IJRR 18(2), 1999.**
  Differential-geometric companion: the family of admissible Riemannian structures on SE(3) and
  their dependence on the choice of scale/frame.
- **Di Gregorio, "Metrics proposed for measuring the distance between two rigid-body poses:
  review, comparison, and combination", Robotica 42(1):302–318, 2024 (doi:10.1017/S0263574723001388).**
  Recent review of the whole pose-metric zoo. Its blunt summary of weighted-sum metrics
  `c₁·rot + c₂·trans`: because the two parts carry different units, homogenizing constants are
  *always* necessary and make the formula "intrinsically arbitrary" — unless the constants are
  derived from the body's geometry (which is precisely what §6's displacement metrics do).

### 2b. Empirical metric studies in sampling-based planning

- **Amato, Bayazit, Dale, Jones & Vallejo, "Choosing Good Distance Metrics and Local Planners for
  Probabilistic Roadmap Methods", ICRA 1998, pp. 630–637; journal version IEEE T-RA 16(4), 2000.**
  The classic empirical study for rigid bodies in 3D. Two findings matter here: (i) metrics that
  reflect *workspace displacement* of the body predict edge feasibility better than raw C-space
  formulas; (ii) **the right translation-vs-rotation weighting is environment-dependent — the
  relative importance of translation grows as the environment gets more crowded.** A static
  hand-tuned weight is wrong somewhere; a geometry-derived one (§6) at least gets the units right.
- **Kuffner, "Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning", ICRA 2004,
  pp. 3993–3998.**
  The practice-defining paper for SE(3) planning: uniform quaternion sampling (Haar measure on
  SO(3)) and a weighted metric `d = w_t·‖Δt‖ + w_r·(quaternion geodesic)`, with explicit guidance
  that the rotation weight should reflect the **maximum displacement rotation can induce on the
  body** (bounded by the body's radius). This is the earliest clean statement of the
  normalization we want: *convert the rotational term into meters via the body's size*.
- **LaValle, *Planning Algorithms*, Cambridge Univ. Press 2006, §5.1.**
  Textbook treatment: products of metric spaces get metrics `d = Σ w_i d_i` (or max / weighted
  Lp), and §5.1.4 introduces the **robot displacement (pseudo)metric**
  `DISP(q,q′) = max_{a∈A} ‖τ(q,a) − τ(q′,a)‖` — the maximum workspace displacement of any body
  point — as the principled, unit-free way to compare heterogeneous DOF (details in §6). §5.1.2–5.1.4
  and the sampling sections also cover Haar measure: "uniform" on a rotation group must mean
  Haar-uniform, not uniform in coordinates.

**Takeaway:** theory (Park) says a scale choice is unavoidable; practice (Amato, Kuffner, LaValle)
converged on making that scale the *body's own workspace geometry* rather than a global constant.
OMPL's SE(2) `0.5` weight is a fossilized compromise, not a rule.

---

## 3. RRT is *specifically* sensitive to this — Voronoi bias distortion

Why the metric matters more for RRT than for PRM: RRT's exploration engine is the **Voronoi
bias** — the probability a node is extended is the measure of its Voronoi cell *under the metric
and sampling measure used by the NN lookup*. Distort the metric and you distort the bias.

- **LaValle & Kuffner, "Rapidly-Exploring Random Trees: Progress and Prospects", WAFR 2000
  (Algorithmic and Computational Robotics: New Directions, pp. 293–308).**
  The authors themselves list the metric as RRT's primary vulnerability: with a poor metric the
  Voronoi bias pulls the tree toward states that *look* close but are hard to connect. (Listed on
  lavalle.pl/rrtpubs.html; substance restated in the two papers below.)
- **Cheng & LaValle, "Reducing Metric Sensitivity in Randomized Trajectory Design", IROS 2001;
  and "Resolution Complete Rapidly-Exploring Random Trees", ICRA 2002, pp. 267–272.**
  The direct study: RRT performance degrades sharply under mis-scaled metrics; their fix
  (penalizing repeatedly-failing nodes, collision-tendency bookkeeping) is an *algorithmic patch
  for a metric problem* — evidence that if the metric can be fixed at the source (our case:
  we know each factor's geometry exactly), it should be.
- **Lindemann & LaValle, "Incrementally Reducing Dispersion by Increasing Voronoi Bias in RRTs",
  ICRA 2004.** Formalizes the Voronoi-bias view of RRT (exact Voronoi-diagram-driven variants);
  useful as the theoretical lens: whatever we choose as NN metric *is* the exploration bias.
- **Shkolnik, Walter & Tedrake, "Reachability-Guided Sampling for Planning Under Differential
  Constraints", ICRA 2009, pp. 2859–2865.**
  Under differential constraints the C-space metric wildly misrepresents reachability; restricting
  the Voronoi bias to the *reachable set* buys an order of magnitude. Analogy for us: a factor's
  one-step reachable set is `{that factor moved ≤ range, others frozen}` — the Voronoi bias should
  be computed in a metric where those sets are *comparable across factors* (same workspace
  volume-of-effect), which raw radians/meters L1 does not give.
- **Şucan & Kavraki, "Kinodynamic Motion Planning by Interior-Exterior Cell Exploration"
  (KPIECE), WAFR 2008 (Algorithmic Foundation of Robotics VIII, Springer STAR 57, 2009).**
  The other escape route: **give up on metrics entirely** — drive exploration by coverage of a
  low-dimensional *projection* (grid cells, interior/exterior distinction), not by NN distances.
  OMPL ships this as the default answer to "my space's metric is untrustworthy". Relevance: a
  *workspace* projection (object footprints) is metric-free and factor-commensurable by
  construction; a KPIECE-style per-factor coverage grid is a cheap fallback if metric
  normalization proves insufficient. Related in spirit: **Shkolnik & Tedrake, "Path Planning in
  1000+ Dimensions Using a Task-Space Voronoi Bias", ICRA 2009, pp. 2061–2067** — put the Voronoi
  bias in task/workspace coordinates where units are homogeneous, and let the C-space follow.

**Takeaway:** RRT's exploration *is* its metric. Our current L1 makes the door's Voronoi
influence tiny (its factor diameter is ~2 rad ≈ "2 meters" of L1) next to a 10 m × 10 m box
factor, while simultaneously making a *capped door step* move the blade tip much farther through
the workspace than a capped box step of equal L1. Both distortions have the same fix (§6).

---

## 4. How composite / multi-robot / multilevel planners sidestep the problem

Planners facing a product of heterogeneous factors have three known dodges:

1. **Discretize each factor separately, then search the product of graphs.**
   - **Solovey, Salzman & Halperin, "Finding a Needle in an Exponential Haystack: Discrete RRT for
     Exploration of Implicit Roadmaps in Multi-Robot Motion Planning", IJRR 35(5):501–513, 2016
     (WAFR 2014 special issue; arXiv:1305.2889).** dRRT builds a PRM **per robot in its own
     C-space with its own metric**, then runs a *discrete* RRT on the implicit tensor-product
     roadmap. The compound-metric problem evaporates: cross-factor comparisons happen only through
     a *direction oracle* on the product graph, and each factor's geometry is honored by its own
     roadmap. Demonstrated to 60 DOF.
   - **Shome, Solovey, Dobson, Halperin & Bekris, "dRRT*: Scalable and Informed
     Asymptotically-Optimal Multi-Robot Motion Planning", Autonomous Robots 44(3–4):443–467, 2020
     (arXiv:1903.00994).** Asymptotically-optimal, informed version; same per-factor-roadmap
     architecture.
   - **Wagner & Choset, "Subdimensional Expansion for Multirobot Path Planning", Artificial
     Intelligence 219:1–24, 2015.** M*: plan each factor independently in its own space; only
     where factors *interact* (collision) does the search locally expand into the joint space.
     Same lesson from the discrete side: **the joint metric is never used — factors are coupled
     through conflicts, not through a common distance.**
   - *Relation to LA-RRT:* our one-factor-at-a-time extension is already "dRRT-flavored" — each
     edge lives in a single factor. The lesson to import is that **nothing in the algorithm needs
     a cross-factor exchange rate except (i) the NN lookup and (ii) the step cap** — so those are
     the only two places a normalization has to be injected, and it can be per-factor.
2. **Order the factors into levels (quotients/bundles)** — Orthey's line, covered in briefing 06
   §4 (QRRT ISRR 2019, fiber-bundle IJRR 2023/24). One addition beyond 06: the levels in that
   framework are explicitly *heterogeneous* (base SE(2) below, arm R^k above) and each level uses
   **its own metric and its own sampler**; cross-level interaction is only through projection and
   lift, never through a mixed metric. Same structural dodge as dRRT, vertically instead of
   horizontally. See also **Orthey, Chamzas & Kavraki, "Sampling-Based Motion Planning: A
   Comparative Review", Annual Review of Control, Robotics, and Autonomous Systems 7, 2024
   (arXiv:2309.13119)** — the current survey; its treatment of metrics/projections/samplers is the
   up-to-date reference list for everything in §§2–3.
3. **Couple subspaces only where needed, with a region-dependent sampling split.**
   - **Kang et al., "Harmonious Sampling for Mobile Manipulation Planning", IROS 2019
     (arXiv:1809.07497).** Mobile manipulator = product of a base SE(2) factor and an arm factor.
     They sample *mostly the base subspace* in easy regions and switch to coupled full-space
     sampling only in difficult regions (near the goal, in narrow passages), with regions
     identified in the low-dimensional base space. Up to 5.6× faster to first solution. This is
     the closest published instance of **state-dependent per-subspace sampling allocation** — the
     geometric ancestor of the bandit allocation we defer to briefing 07.

---

## 5. The measure mismatch: uniform sampling over a product of unequal factors

Two distinct mathematical facts get conflated under "sampling bias"; both apply to us:

1. **Within a factor — coordinate-uniform ≠ geometry-uniform.** On SO(2) an interval's uniform
   (Haar) measure *is* proportional to arc length, so coordinate-uniform sampling of the door
   angle is fine. The subtlety only bites for SO(3) (quaternions, Haar measure — Kuffner ICRA
   2004; **Yershova, Jain, LaValle & Mitchell, "Generating Uniform Incremental Grids on SO(3)
   Using the Hopf Fibration", IJRR 29(7):801–812, 2010** for the deterministic version;
   LaValle *Planning Algorithms* §5.1–5.2 for the Haar-measure framing). For our 2D factors the
   within-factor measure is unproblematic — worth stating so we stop worrying about it.
2. **Across factors — the product measure allocates *nothing*; the algorithm allocates.** Uniform
   sampling of the product samples *every* factor's coordinates each draw (OMPL does exactly this,
   `StateSampler.cpp:47–52`); there is no notion of "the door gets fewer samples because its
   factor is smaller". Allocation across factors happens *only* through whatever the planner does
   with the sample — in LA-RRT, through the uniform random pick of the extended factor
   (`LARRT.cpp:352`) and through the metric's effect on NN. The "door's 1-DOF arc has tiny measure
   next to a box's R² area" intuition is therefore really a statement about the **metric** (its
   Voronoi cells, §3) and the **factor-pick distribution**, not about the sampler.
   - Literature status: we found **no dedicated measure-theoretic treatment of per-factor
     sampling probabilities in compound C-spaces** — the field's answers are structural (dRRT/M*/
     multilevel, §4), region-driven (Harmonious Sampling, §4), experience-driven (Kingston &
     Kavraki's *discrete lead* over mode transitions, T-RO 39(1):128–146, 2023 — verified in
     briefing 06 §3a, applies verbatim to "which factor next"), or bandit-driven (briefing 07).
     Flagging this as a genuine gap: a principled *static* allocation exists only in the weak
     sense of "proportional to normalized factor diameter" (a dispersion argument: to cover factor
     `Q_i` of normalized diameter `D_i` with steps of length `r` takes ~`(D_i/r)^{dim_i}`
     extensions — allocation proportional to that is the uniform-coverage ideal, and it is
     *exponentially* lopsided toward high-dim factors; use it as an ablation arm, not a default).

---

## 6. Workspace-displacement metrics — the candidate normalization

The one metric family that makes a door radian and a box meter *commensurable by construction*
measures a configuration change by **how far the body's points move in the workspace**:

- **DISP (maximum displacement pseudometric)** — LaValle, *Planning Algorithms* §5.1.4:
  `DISP_i(q,q′) = max_{a∈A_i} ‖τ_i(q,a) − τ_i(q′,a)‖`. For our factors this is closed-form:
  | factor | DISP | tight upper bound (swept/arc form) |
  |---|---|---|
  | revolute door, blade reach `R` (hinge→farthest blade point) | `2R·sin(|Δθ|/2)` | arc `R·|Δθ|` |
  | prismatic slider | `|Δs|` | `|Δs|` |
  | translating box (R²) | `‖Δt‖₂` | `‖Δt‖₂` |
  | box in SE(2), circumradius `ρ` (center→farthest corner) | `≤ ‖Δt‖₂ + 2ρ·sin(|Δθ|/2)` | `‖Δt‖₂ + ρ·|Δθ|` |
- **C-DIST** — **Zhang, Kim & Manocha, "C-DIST: Efficient Distance Computation for Rigid and
  Articulated Models in Configuration Space", ACM Symposium on Solid and Physical Modeling
  2007.** Exact DISP computation for general rigid/articulated models; key structural results:
  DISP is realized at a **vertex of the convex hull** of the body (via Chasles' screw-motion
  theorem). For our OBBs that means "check the 4 corners" — DISP is 4 point-norm evaluations.
- **Schwarzer, Saha & Latombe, "Adaptive Dynamic Collision Checking for Single and Multiple
  Articulated Robots in Complex Environments", IEEE T-RO 21(3):338–353, 2005.**
  The *swept-length* sibling: their exact path checker needs, for each body, an upper bound
  `λ(q,q′)` on the **length of the workspace curve traced by any body point** along the local
  path. For a rigid 2D body moving `(Δt, Δθ)` that bound is exactly `‖Δt‖ + ρ·|Δθ|` — i.e. the
  arc-form column above. So the "L1 with the angle weighted by the body's radius" is not an ad-hoc
  fix; it is the standard certified bound from the collision-checking literature, and adopting it
  as the factor metric simultaneously gives the *correct per-factor collision-checking
  resolution* (`longestValidSegment` becomes a workspace length, uniform across factors).
- Precedent as a *planning* metric, not just a checking bound: Amato 1998 found
  workspace-displacement metrics predict local-planner success best (§2b); Kuffner 2004's
  recommended rotation weight is the body-radius scale (§2b); Shkolnik-Tedrake's task-space
  Voronoi bias (§3) is the same idea applied to the exploration bias.

**Assessment for LA-RRT: adopt it.** The arc-form bound `d_i^W` (last column) is the right
variant for us, for three reasons: (i) it is an L1-style sum, so it drops into the existing
range-cap code with only per-dim weights (angle dims get weight `R_i`/`ρ_i`, translation dims
weight 1 — plus the missing SO(2) wrap); (ii) it upper-bounds swept displacement, so
"range = 0.5" literally means "no single action sweeps any point more than ~0.5 m", which makes
the step cap a *collision-relevant* quantity and makes edges of equal cost equally checkable;
(iii) it is exactly the unit in which "how much did this action move the world" should be
measured, i.e. the natural reward scale for the bandit layer (briefing 07) and a defensible
tie-breaker/secondary cost in `PathDefragmenter` (primary cost stays action count).
The chord-form DISP is marginally tighter for large rotations but loses additivity along a
rotation (not a path length); the arc form is additive and is what Schwarzer et al. certify. Note
DISP-style metrics are **pseudometrics on the factor** only when the body has symmetries; our
OBBs with distinct corners make them true metrics — and the door factor's chart is an interval,
so no quotient subtleties arise.

---

## 7. Concrete recommendations for LA-RRT

**(a) Per-factor extension range — replace raw L1 with workspace-normalized L1.**
Give `FactoredStateSpace` per-dimension scale factors `s_idx` (already knows `isAngleDim`):
`s_idx = R_i` (hinge→farthest-point) for a revolute dim, `s_idx = ρ_i` (circumradius) for the
rotation dim of an SE(2) factor, `s_idx = 1` for translation/prismatic dims. In `growTree`,
`dg = Σ_idx s_idx · wrap(|n[idx] − r[idx]|)` (add the SO(2) wrap — see §1's latent bug), and
interpret `maxDistance_` as a **workspace length** (e.g. 0.3–0.5 of the narrowest corridor
width), set explicitly instead of `configurePlannerRange`'s `0.2 × joint extent` (which grows
with object count, §1). Effect: a capped door step and a capped box step sweep comparable
workspace volume; the door stops being simultaneously over-aggressive per step and under-weighted
in NN.

**(b) Per-factor selection probabilities.**
Keep **uniform over factors as the default and the control arm** — it is the only choice with no
assumptions, and one-factor-per-edge already gives every object a floor of attention. Add two
alternatives behind a switch, for the ablation and for briefing 07 to build on:
(i) *measure-weighted*: `p_i ∝ (D_i^W / range)^{dim_i}` with `D_i^W` the factor's
workspace-normalized diameter — the dispersion/coverage allocation of §5.2 (expected to
over-serve the SE(2) box badly; that is the point of the arm);
(ii) *adaptive*: Kingston-Kavraki-style discrete lead / bandit over factors, reward measured in
the §6 units — details deferred to briefing 07. Literature support for anything *static* beyond
uniform is thin (§5): say so in the paper rather than inventing a rule.

**(c) Nearest-neighbor distance for the trees.**
Use the **sum over factors of the workspace-normalized factor distances**:
`d(q,q′) = Σ_i d_i^W(q_i, q_i′)` — i.e. `groupDistance` with the same `s_idx` weights and wrap as
(a), so the NN metric and the extension geometry agree (they currently agree only by both being
unweighted). Rationale: the sum is the natural "total work to reconfigure the arrangement"
under one-factor-at-a-time motion (an L1 over factors matches the action semantics; a max/L∞
over factors would measure makespan, wrong here), and it fixes the door's Voronoi starvation
(§3) at the only two places cross-factor exchange rates enter the algorithm (§4, dRRT lesson).
Keep the OMPL `CompoundStateSpace` weights consistent (`setSubspaceWeight(i, ·)` per factor with
internally normalized factor metrics) so `distance()`, `maximumExtent()`, and
`longestValidSegmentFraction` all see the same geometry the planner uses.

**(d) What to log to show the normalization matters (ablation design).**
Conditions: {raw-L1 (today), diameter-normalized (each factor scaled to unit diameter — the
"cheap" fix a reviewer will propose), workspace-DISP-normalized (§6)} × {uniform, measure-weighted
factor pick} on scenes that stress heterogeneity (door + slider + free boxes; `swing_door`,
`buffer_swap`, `three_alcoves`). Log per run:
1. per-factor extension attempts vs. accepted edges (starvation/over-service directly);
2. per accepted edge: its factor, its metric length, and its *measured* max corner/tip
   displacement (validates that the normalized cap equalizes real sweep across factors —
   under raw L1 the door-edge sweep histogram should visibly dominate);
3. time and iterations to first solution; action count before/after defrag; success rate at
   fixed budget (the headline table);
4. NN-metric quality: correlation between `d(q,q′)` and edge-validation outcome (Amato's
   criterion — a good metric's short pairs should connect more often);
5. per-factor dispersion proxy (KPIECE-style occupancy of a per-factor grid) over time — shows
   whether the door's factor actually gets *covered* rather than merely touched.
Prediction to test: raw L1 hurts most on door-gated scenes (door under-explored in NN, blade
sweeps too far per step → wasted collision checks), and diameter normalization recovers only part
of the gap because it ignores that a radian's workspace effect depends on blade length, not on
the interval's width.

---

## 8. Verification summary

**Verified (publisher/arXiv/IEEE page or local source confirmed; title+authors+venue match):**
- Park, *Distance Metrics on the Rigid-Body Motions…*, ASME J. Mech. Design 117(1):48–54, 1995 —
  ASME page; no-bi-invariant-metric + length-scale claims from abstract.
- Žefran, Kumar & Croke, *Metrics and Connections for Rigid-Body Kinematics*, IJRR 18(2), 1999 —
  SAGE listing.
- Di Gregorio, *Metrics proposed for measuring the distance between two rigid-body poses…*,
  Robotica 42(1):302–318, 2024, doi:10.1017/S0263574723001388 — Cambridge Core + journal PDF.
- Amato, Bayazit, Dale, Jones, Vallejo, ICRA 1998 pp. 630–637 (+ IEEE journal version 864240);
  "translation importance grows with clutter" finding confirmed from search abstract.
- Kuffner, *Effective Sampling and Distance Metrics for 3D Rigid Body Path Planning*, ICRA 2004,
  vol. 4 pp. 3993–3998 — IEEE 1308895, CMU RI page.
- LaValle, *Planning Algorithms*, 2006 — §5.1 metric-space and DISP definition confirmed from
  lavalle.pl book pages (node188/node189: `disp(q,q′) = max_a ‖τ(q,a) − τ(q′,a)‖`).
- Cheng & LaValle, *Reducing Metric Sensitivity…*, IROS 2001; *Resolution Complete RRTs*, ICRA
  2002 pp. 267–272 — lavalle.pl publication list + citations.
- Lindemann & LaValle, *Incrementally Reducing Dispersion by Increasing Voronoi Bias in RRTs*,
  ICRA 2004 — IEEE 1308755.
- LaValle & Kuffner, *RRTs: Progress and Prospects*, WAFR 2000 — from lavalle.pl/rrtpubs.html
  listing only; the "metric is RRT's Achilles heel" phrasing is our paraphrase of its discussion.
- Şucan & Kavraki, KPIECE, WAFR 2008 (Springer STAR proceedings 2009) — Kavraki Lab page,
  Springer chapter.
- Shkolnik, Walter & Tedrake, *Reachability-Guided Sampling…*, ICRA 2009 pp. 2859–2865; Shkolnik &
  Tedrake, *Path Planning in 1000+ Dimensions Using a Task-Space Voronoi Bias*, ICRA 2009
  pp. 2061–2067 — IEEE 5152638, MIT DSpace.
- Solovey, Salzman & Halperin, dRRT, IJRR 35(5):501–513, 2016 (arXiv:1305.2889); Shome et al.,
  dRRT*, Autonomous Robots 44(3–4):443–467, 2020 (arXiv:1903.00994, Springer).
- Wagner & Choset, M*/subdimensional expansion, Artificial Intelligence 219:1–24, 2015.
- Kang et al., *Harmonious Sampling for Mobile Manipulation Planning*, IROS 2019 —
  arXiv:1809.07497, IEEE 8967721; base/arm region-dependent allocation + 5.6× claim from abstract.
- Yershova, Jain, LaValle & Mitchell, *Uniform Incremental Grids on SO(3) via the Hopf
  Fibration*, IJRR 29(7):801–812, 2010 (WAFR 2008 version confirmed).
- Zhang, Kim & Manocha, *C-DIST*, ACM SPM 2007 — UNC/UMD PDFs; convex-hull-vertex and Chasles
  results from the paper page.
- Schwarzer, Saha & Latombe, *Adaptive Dynamic Collision Checking…*, IEEE T-RO 21(3):338–353,
  2005 — IEEE 1435478, Stanford PDF.
- Orthey, Chamzas & Kavraki, *Sampling-Based Motion Planning: A Comparative Review*, Annu. Rev.
  Control Robot. Auton. Syst. 7, 2024 — Annual Reviews page, arXiv:2309.13119.
- Orthey QRRT / fiber-bundle line and Kingston & Kavraki T-RO 2023 (discrete lead) — verified
  previously in briefing 06; reused here without re-verification.
- **Local-source facts** (this worktree): SE2 weights 1.0/0.5 (`SE2StateSpace.h:110–111`); SE3
  1.0/1.0 (`SE3StateSpace.h:119–120`); compound distance/extent/measure (`StateSpace.cpp:1006,
  1080`); uniform compound sampling ignores weights (`StateSampler.cpp:47–71`); range default
  0.2 × extent (`MagicConstants.h:73`, `SelfConfig.cpp:98`); LARRT raw-L1 cap and unwrapped
  interpolation (`LARRT.cpp:352–361`) vs. wrapped `groupDistance` (`LARRT.cpp:179–200`).

**Flagged / partially verified:**
- The exact `2R·sin(|Δθ|/2)` / `‖Δt‖ + ρ·|Δθ|` formulas in §6 are **our own elementary
  derivations** (chord length; triangle inequality on the screw decomposition), consistent with
  C-DIST's and Schwarzer et al.'s general statements but not quoted from them — recheck the
  Schwarzer T-RO paper's Lemma for the precise per-body λ bound before citing it with a formula
  in a paper.
- The dispersion-based allocation `p_i ∝ (D_i/r)^{dim_i}` in §5/§7b is a folklore covering
  argument, not a citable result — presented as our reasoning.
- Amato et al.'s finding is taken from the abstract/summary, not a re-read of the full tables.
- The claim that no measure-theoretic per-factor allocation literature exists (§5.2) is a
  negative result from our searches (July 2026), not a certainty.
