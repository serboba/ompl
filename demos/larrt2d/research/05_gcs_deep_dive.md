# GCS Deep Dive — Can Graphs of Convex Sets Give Global Optimality for OBB Rearrangement?

> Deep-dive briefing extending `02_optimality_and_sampling.md` §4 (do not re-read that section as
> current; this file supersedes it on GCS specifics). Question: can a GCS formulation give a
> **global** optimality certificate on **action count / buffer usage** for our 2D OBB world
> (n movable OBBs, some on prismatic/revolute joints, single "hand", one object moved at a time)?
> Every citation was checked against arXiv / publisher / author pages on 2026-07-04; anything not
> fully confirmed is flagged. Produced by a research agent, 2026-07-04.

**Executive answer.** Yes-with-asterisks. GCS gives global optimality **of a convex-set model**,
and action count is directly expressible (constant edge costs). But the paper closest to our
problem — the WAFR 2026 shortest-*walks* paper, now positively identified (§1) — is itself
**NP-hard and solved only approximately**: the one GCS variant that can express rearrangement's
revisit structure is exactly the one that *loses* the global certificate in practice. A certificate
for n≤6 is realistic only via an **explicit product-graph GCS solved by branch-and-bound**
(GCSOPT-style, §5), and only for coarse decompositions and roughly n≤3–4 objects; beyond that the
honest architecture is: exact discrete search over a region abstraction for the count certificate,
GCS underneath for continuous placement optimality (converging with Option A+B of briefing 02 §5).

---

## 1. The "WAFR 2026 pick-and-place over convex sets" paper — identified

**Confirmed.** The paper the agenda refers to is:

- **Mixed Discrete and Continuous Planning using Shortest Walks in Graphs of Convex Sets** —
  Savva Morozov, Tobia Marcucci, Bernhard Paus Graesdal, Alexandre Amice, Pablo A. Parrilo,
  Russ Tedrake. **WAFR 2026** (venue listed on Marcucci's publications page; the arXiv page itself
  carries no venue string). https://arxiv.org/abs/2507.10878

Caveat on framing: the abstract says collision-free motion planning / **skill chaining** / hybrid
control — not "pick-and-place" verbatim. But the **skill-chaining experiment is a planar
pick-and-place/rearrangement domain**: a suction-cup gripper with fixed vertical orientation sorts
rectangular objects into a target region, with three skills — arm motion, center-grasp
pick-and-place, and corner-grasp **flipping** (swaps the rectangle's width/height). So "the WAFR
2026 pick-and-place-over-convex-sets paper" is a fair informal description. Facts that matter
for us (from the arXiv full text):

1. **Shortest-Walk Problem (SWP) in GCS**: vertices may be **revisited** with *different continuous
   points each visit* — no layered vertex duplication. This is the structural feature rearrangement
   needs (an object/mode recurs; a door opens and closes).
2. **Cost structure fits action counting**: total cost = Σ vertex costs + Σ edge costs; their
   skill-chaining cost is literally **"1 + arm horizontal displacement" per skill** — i.e. an
   action-count term plus a motion term. Constant per-edge costs are first-class in GCS.
3. **No global optimality**: SWP in GCS is **NP-hard (reduction from 3SAT)**. Their solver is
   multi-step-lookahead **greedy** search guided by a piecewise-quadratic cost-to-go **lower bound**
   synthesized offline by SDP, plus post-processing (re-optimization, cycle elimination). The
   authors state it "lacks guarantees of completeness or optimality".
4. **Scale**: skill-chaining GCS has **22 vertices, 120 edges** (skills are non-convex due to
   collision avoidance and get decomposed into convex sub-skills); setup 20 s, SDP cost-to-go
   synthesis tens of seconds (reusable across queries), per-query planning sub-second.
5. Non-convex skills ⇒ convex **sub**-skills: the same trick we would need for OBB rotations.

**Takeaway.** The flagship WAFR 2026 paper *models* exactly our objective class (unit cost per
skill + revisits) but *certifies nothing globally*. The lower bound (their SDP cost-to-go) is,
however, an **admissible bound** — usable in exactly the branch-and-bound / bracket role our
agenda's Q5 asks for.

---

## 2. The Marcucci–Tedrake GCS stack, mapped

### 2a. Core formulation
- **Shortest Paths in Graphs of Convex Sets** — Marcucci, Umenberger, Parrilo, Tedrake,
  *SIAM J. Optimization* 34(1):507–532, 2024. https://arxiv.org/abs/2101.11565 ·
  https://doi.org/10.1137/22M1523790. SPP where each vertex holds a continuous point in a convex
  set, edge lengths are convex functions of endpoints. **NP-hard.** Contribution: a strong,
  lightweight **mixed-integer convex** formulation via perspective (homogenization) operators whose
  **convex relaxation is empirically very tight** for motion-planning-style instances ⇒ cheap
  rounding usually recovers a certified (near-)global optimum; exact B&B available when it doesn't.
- **Motion Planning around Obstacles with Convex Optimization** — Marcucci, Petersen, von Wrangel,
  Tedrake, *Science Robotics* 8(84), 2023. https://arxiv.org/abs/2205.04422 ·
  https://doi.org/10.1126/scirobotics.adf7843. GCS-Trajopt: convex decomposition of C-free (IRIS
  regions) + Bézier curves per region; certified optimality gaps; outperformed sampling planners on
  7+ DoF problems. Code: https://github.com/RobotLocomotion/gcs-science-robotics.
- **A Unified and Scalable Method for Optimization over Graphs of Convex Sets** — Marcucci, 2025,
  arXiv:2510.20184, under review at *Mathematical Programming*. https://arxiv.org/abs/2510.20184.
  Generalizes *any* graph optimization (SPP, **TSP**, spanning tree, …) to GCS, auto-generates the
  MICP via homogenization, solves **to global optimality with off-the-shelf branch-and-bound**.
  Ships **GCSOPT**, an open-source Python library. ⇒ *This is the tool with which a small explicit
  rearrangement GCS would actually be solved to certified optimality.*

### 2b. Search-based / implicit variants (for graphs too big to enumerate)
- **GCS\*: Forward Heuristic Search on Implicit Graphs of Convex Sets** — Chew Chia, Jiang,
  Graesdal, Kaelbling, Tedrake. WAFR 2024; Springer proceedings chapter
  https://link.springer.com/chapter/10.1007/978-3-032-09967-9_5 · https://arxiv.org/abs/2407.08848.
  A\* generalized to GCS where the graph is generated on the fly; the crux is **domination
  checking** between partial paths whose cost/feasibility depend on the continuous points along the
  *whole* prefix. Exact domination checks (set-containment style) preserve completeness/optimality
  but are expensive; the practical implementation approximates them by **sampling**, weakening the
  guarantee **[detail confidence: medium — from abstract + summaries, full proof conditions not
  re-derived here]**. Demonstrated on planning-through-contact-style domains.
- **Multi-Query Shortest-Path in GCS** — Morozov, Marcucci, Amice, Graesdal, Bosworth, Parrilo,
  Tedrake, WAFR 2024. https://arxiv.org/abs/2409.19543. Precomputation for repeated queries on one
  GCS — relevant if we solve many puzzles in one scene.

### 2c. Region generation (the "where do the convex sets come from" line)
- **IRIS** — Deits & Tedrake, WAFR 2014 (obstacle-free convex region inflation, task space).
- **IRIS-NP** — Petersen & Tedrake: *Growing Convex Collision-Free Regions in Configuration Space
  using Nonlinear Programming*, https://arxiv.org/abs/2303.14737. Nonlinear-programming
  hyperplane search ⇒ regions in **configuration** space with task-space obstacles.
- **C-IRIS** — Dai, Amice, Werner, Zhang, Tedrake: *Certified Polyhedral Decompositions of
  Collision-Free Configuration Space*, IJRR 2024.
  https://journals.sagepub.com/doi/10.1177/02783649231201437. **Certified** collision-free
  polytopes in a rational parameterization of C-space (SOS certificates) — the only line that makes
  the downstream GCS certificate *unconditional* on sampling luck.
- **Clique covers** — Werner, Amice, Marcucci, Rus, Tedrake: *Approximating Robot Configuration
  Spaces with few Convex Sets using Clique Covers of Visibility Graphs*, ICRA 2024.
  https://arxiv.org/abs/2310.02875. Minimizes the **number** of regions — directly controls GCS
  size. For our 2D OBB world all of this is overkill-easy: per-object C-free is a 2D/3D space with
  polygonal obstacles; exact convex decompositions are computable geometrically.

### 2d. Hybrid / discrete-mode extensions (closest to TAMP)
- **Towards Tight Convex Relaxations for Contact-Rich Manipulation** — Graesdal, Chew Chia,
  Marcucci, Morozov, Amice, Parrilo, Tedrake, **RSS 2024**. https://arxiv.org/abs/2402.10312.
  GCS path = **contact-mode sequence**; per-mode SDP relaxation of nonconvex quasi-static dynamics
  (pose × contact point × force bilinearities). Planar pushing results **consistently within a few
  percent of global optimum**, no initial guess. This is the strongest evidence that *mode-sequence*
  discrete structure survives GCS relaxation tightly — but note: single object, no
  object-object placement dependencies.
  Follow-up: *Hierarchical Contact-Rich Trajectory Optimization … using Tight Convex Relaxations* —
  arXiv:2503.07963 (2025).
- **Temporal Logic Motion Planning with Convex Optimization via GCS** — Kurtz & Lin, *IEEE T-RO*
  39(5):3791–3804, 2023. https://arxiv.org/abs/2301.07773 · code
  https://github.com/vincekurtz/ltl_gcs. LTL spec × free-space regions → product automaton as a
  GCS ⇒ SPP. **This is the canonical recipe for injecting discrete task structure into GCS**:
  take a product with an automaton. Rearrangement's "which objects are where" is a (huge)
  automaton; see §3.
- **Augmented GCS for precedence (key-door) specs** — You, Luna, Shaikh, Gostin, Xiang, Koeln,
  Summers, arXiv:2510.22015 (2025/26); STL framework version arXiv:2606.00842 (2026).
  https://arxiv.org/abs/2510.22015. Exact convex partition + graph augmentation encoding
  "visit key region before door region" — literally our door puzzles, as *pure* precedence
  (the door is a region to unlock, not a movable body). Not peer-review-verified venues yet
  **[arXiv-only]**.
- **Augmented GCS and the TSP** — Luna & Summers, arXiv:2604.06406 (2026).
  https://arxiv.org/abs/2604.06406. TSP-in-GCS via augmented graph → SPP; exact Bellman–Held–Karp
  connection + a 1-tree branch-and-bound giving **certifiably optimal or near-optimal** tours.
  Relevant because rearrangement's "serve every object" structure is TSP-like, and this shows the
  augmentation cost: BHK augmentation is **exponential in the number of required visits** — the
  honest price of exactness. **[arXiv-only]**
- Adjacent, lower relevance: *Space-Time GCS for Multi-Robot Motion Planning* (arXiv:2503.00583);
  *Non-Euclidean Motion Planning with Graphs of Geodesically Convex Sets* — Cohn, Petersen,
  Simchowitz, Tedrake, IJRR 2025 (how to handle SO(2)/SO(3) factors — relevant to OBB orientation).

**Not found:** any published GCS paper that solves multi-object **rearrangement with buffers** (our
problem) end-to-end, or any GCS result certifying **minimum action count** for rearrangement. The
closest are the shortest-walks skill-chaining demo (§1, heuristic) and contact-rich GCS (single
object). This is a genuine gap in the literature as of 2026-07.

---

## 3. How GCS copes with rearrangement's discrete structure — and what blows up

GCS natively expresses exactly **one** discrete decision: *which path/walk through a fixed graph*.
Everything else must be compiled into the graph. For rearrangement the discrete choices are:
which object to grasp next, where to place it (goal vs which buffer), and in what order. Three
compilation strategies exist, with a sharp trade-off:

**(i) Full product ("arrangement-mode") graph — exact, explicit.** Vertex = (region of object 1,
…, region of object n, hand state ∈ {free, holding i}); continuous variable = the poses within
regions (+ hand pose). Edges: move-hand, grasp, place, move-held-object-between-its-regions.
Action count = constant cost on grasp (or place) edges. Solvable to **certified global optimality**
by the MICP/B&B machinery (GCSOPT). Blow-up: vertex count ≈ (r per-object regions)^n × (n+1); see
§4. This is the LTL-GCS/product-automaton recipe applied to arrangements.

**(ii) Implicit search (GCS\*) — exact-ish, lazy.** Never enumerate the product graph; expand
successors on demand with an admissible heuristic (e.g. the dependency-graph buffer lower bound
from briefing 02 §2, or SWP's SDP cost-to-go). Global optimality survives **only** with exact
domination checks, which are the expensive part; the published implementation samples. Search-tree
size is still worst-case exponential — it is A\* on the same product space, just lazily.

**(iii) Shortest walks on the factored graph — compact, heuristic.** Keep the graph small (per
object: its regions; per skill: its convex pieces — the WAFR 2026 paper's design, 22 vertices for
a multi-object sorting task) and let *walks* revisit vertices instead of unrolling the product.
The arrangement state lives implicitly in the continuous variables. Price: SWP is NP-hard and the
solver is greedy — **no certificate**. This is the only variant whose graph size does not explode,
and it is precisely the one without guarantees. That correlation is not an accident: the walk
formulation hides the exponential arrangement state, and no polynomially-sized convex relaxation
is known to see through it (nor should one be expected: min pick-and-place count is NP-hard
already for tabletop instances — Han et al., briefing 02 §2).

**Relaxation tightness under action-count costs.** The celebrated tightness of the GCS relaxation
is an *empirical* property of motion-planning instances with metric-like continuous edge costs;
there is no tightness theorem. Two structural reasons to expect it to **degrade** on our objective:
1. **Constant edge costs** make many fractional flow splittings cost-equivalent (the continuous
   part no longer breaks ties), so the LP/convex relaxation has larger optimal faces and rounding
   is less reliable — B&B then does real work.
2. Rearrangement embeds **TSP/set-cover-like** substructure ("every object must be served",
   "which obstacles to clear" is MCR/set-cover — briefing 02). The natural network-flow
   relaxations of such structures are classically loose (subtour-elimination etc.); the augmented
   GCS-TSP paper (§2d) confirms exactness requires exponential augmentation or B&B.
**Consequence**: with GCS on rearrangement, expect the certificate to come from **branch-and-bound
finishing**, not from a tight root relaxation. That is fine at small scale and hopeless at large.

**Action count vs path length — literature status.** Constant per-edge costs are fully supported
(SPP-GCS cost model; the shortest-walks paper uses cost "1 + displacement" per skill). But **no
paper proves anything special about the *count* objective** — no tightness result, no dedicated
algorithm. The count objective is exactly where our discrete lower bounds (dependency-graph cycle
counts, MRB) are *stronger* than what a generic convex relaxation recovers, which argues for
feeding those bounds into the search rather than hoping the relaxation finds them.

---

## 4. Concrete sketch: a GCS for the 2D OBB world

Setup: n movable OBBs; free bodies have C-space R² × S¹ (or R² if orientation is fixed/snapped);
jointed bodies (doors, sliders) have a 1-DoF interval. Single hand, one object moves at a time.

**Per-object region decomposition.**
- **Jointed objects**: the joint interval, split at *collision-relevant events* (angles/offsets at
  which the swept body starts/stops blocking some other object's region). Each sub-interval is
  trivially convex. Typical: 2–4 segments per door/slider.
- **Free OBBs, translation**: convex (e.g. trapezoidal) decomposition of the object's 2D
  translational C-free w.r.t. **static** geometry only, per orientation bin. Exact and cheap in 2D.
  Typical scenes: 4–12 cells.
- **Orientation**: S¹ is non-convex ⇒ either geodesic-GCS treatment (Cohn et al.) or k orientation
  bins (k = 2–4 suffices for axis-aligned-ish puzzles). Region count per free object r ≈ 8–40.

**The coupling problem (this is the crux).** Object i's cell is only *truly* free given where the
other OBBs sit. Pairwise "OBB i in cell A vs OBB j in cell B" collision, with both poses varying,
is **non-convex** (bilinear in the separating-axis terms). Options:
- (a) **Region-level conservative blocking**: precompute the Boolean r×r table "cell A of i can
  overlap cell B of j"; forbid co-occupancy at the discrete level. Sound, loses completeness in
  tight scenes (two objects that *could* share a big cell are banned from it). Refine by
  subdividing cells. With this, all continuous constraints are convex and the product-graph GCS
  (§3-i) is exact **w.r.t. the region abstraction**.
- (b) SDP-relax the pairwise separation constraints à la contact-rich GCS (Graesdal et al.) —
  principled but n²/cell-pair many SDP blocks; research project, not a build.

**Size estimate (strategy 3-i, blocking table (a)).** Vertices ≈ (n+1) · Π_i r_i.
- 2 free objects + 1 door (r = 8, 8, 3): 3 · 192 ≈ **600 vertices** — trivially solvable to
  global optimality with GCSOPT + Gurobi/Mosek (minutes at worst).
- n = 4 mixed (8·8·8·3): 5 · 1536 ≈ **7.7 k vertices** — solvable; B&B may take minutes–hours on
  hard instances with the flat count objective.
- n = 6 (r ≈ 8 each): 7 · 8⁶ ≈ **1.8 M vertices** — explicit MICP is out. Must go implicit
  (GCS\*-style with our dependency-graph lower bound as heuristic) or accept heuristic walks.
  Reachability pruning (most arrangement tuples are never reachable/relevant) can cut orders of
  magnitude, but the worst case owns you.

**What the certificate means.** Even when B&B finishes, optimality is **relative to the model**:
the region decomposition, orientation bins, and the conservative blocking table. A plan needing a
placement that straddles two cells at a banned pair is invisible. So the honest claim is
"minimum action count *over this abstraction*" plus a one-sided guarantee: the abstraction's
optimum is an **upper bound** on the true optimum, and (if the blocking table is made
*optimistic* instead of conservative) a second run yields a **lower bound** — the two-run
bracket certifies the true count whenever they meet. This bracket trick is, to our knowledge,
not in the literature — it is our Q5 (LA-RRT upper bound + cheap lower bound) transplanted into
GCS, and it is the most publishable-looking idea in this file.

**Is global optimality realistic for n ≤ 6?** For the **model**: yes for n ≤ 3–4 (explicit GCS +
B&B, hundreds-to-thousands of vertices), borderline at n = 5, no at n = 6 without implicit search.
For the **true continuous problem**: only via the optimistic/conservative bracket, or C-IRIS-style
certified decompositions plus exact pairwise handling — a research program, not an integration.
Meanwhile the *discrete* count certificate alone is cheaper without GCS: exact search/IP over the
region-blocking abstraction (it is a pebble-motion-like graph problem) gives the same count
guarantee, with GCS relegated to per-action continuous refinement — which is exactly briefing 02's
Option A, now with a principled continuous layer (per-object SPP-GCS instead of raw RRT).

**Recommended stance for LA-RRT.** (1) Build the per-object region decomposition + blocking table
anyway — it feeds the dependency-graph lower bound (Q5) at negligible cost. (2) For n ≤ 4
benchmark scenes, implement the explicit product GCS in GCSOPT as the *oracle*: certified minimum
action counts to grade LA-RRT+defrag against. (3) Do not bet the planner on shortest-walks GCS:
the WAFR 2026 paper itself is heuristic there, and our greedy defrag would be replaced by a
different non-certifying heuristic — cost without a certificate.

---

## 5. Verification summary

Confirmed (method of verification in parentheses):
- **Morozov, Marcucci, Graesdal, Amice, Parrilo, Tedrake — "Mixed Discrete and Continuous Planning
  using Shortest Walks in GCS", WAFR 2026, arXiv:2507.10878** (arXiv abstract + full HTML text
  fetched; **WAFR 2026 venue confirmed on tobiamarcucci.github.io/publications** — the arXiv page
  itself lists no venue). *This is the agenda's "WAFR 2026 convex-set pick-and-place paper"* —
  with the caveat that its pick-and-place content is the skill-chaining experiment, not the title.
- **Marcucci, Umenberger, Parrilo, Tedrake — "Shortest Paths in GCS", SIAM J. Optim. 34(1), 2024**
  (SIAM DOI 10.1137/22M1523790 + arXiv:2101.11565).
- **Marcucci, Petersen, von Wrangel, Tedrake — "Motion Planning around Obstacles with Convex
  Optimization", Science Robotics 2023** (DOI 10.1126/scirobotics.adf7843, arXiv:2205.04422, code repo).
- **Chew Chia, Jiang, Graesdal, Kaelbling, Tedrake — "GCS\*", WAFR 2024** (arXiv:2407.08848 +
  Springer proceedings chapter 10.1007/978-3-032-09967-9_5). Fine-grained claims about its
  domination-check guarantees are medium-confidence (abstract-level, not re-derived).
- **Morozov et al. — "Multi-Query Shortest-Path in GCS", WAFR 2024** (arXiv:2409.19543 +
  WAFR 2024 program PDF at parasollab).
- **Marcucci — "A Unified and Scalable Method for Optimization over GCS" + GCSOPT,
  arXiv:2510.20184, under review at Mathematical Programming** (arXiv page fetched).
- **Graesdal et al. — "Towards Tight Convex Relaxations for Contact-Rich Manipulation", RSS 2024**
  (arXiv:2402.10312 + author project page); follow-up arXiv:2503.07963 (arXiv only).
- **Kurtz & Lin — "Temporal Logic Motion Planning with Convex Optimization via GCS",
  IEEE T-RO 39(5), 2023** (T-RO listing + arXiv:2301.07773 + code repo).
- **Petersen & Tedrake — IRIS-NP** (arXiv:2303.14737); **Dai, Amice, Werner, Zhang, Tedrake —
  C-IRIS, IJRR 2024** (SAGE DOI 10.1177/02783649231201437); **Werner et al. — clique covers,
  ICRA 2024** (arXiv:2310.02875).
- **Cohn, Petersen, Simchowitz, Tedrake — geodesically convex GCS, IJRR 2025** (SAGE DOI
  10.1177/02783649241302419).

arXiv-only / not yet peer-review-verified:
- **You et al. — Augmented GCS precedence specs** (arXiv:2510.22015; STL version arXiv:2606.00842).
- **Luna & Summers — Augmented GCS + TSP** (arXiv:2604.06406).

Negative findings (searched, not found — treat as gaps, not omissions):
- No GCS paper solving multi-object rearrangement with buffers end-to-end.
- No GCS result on optimality/tightness for **action-count** objectives specifically.
- No tightness *theorem* for the SPP-GCS relaxation in any regime; tightness is empirical.
