# How Complexity Grows with the Number of Objects Requiring Intermediate (Buffer) Placements

> Literature-grounded briefing. Cited works were located via web search; unverifiable specifics are
> flagged **[uncertain]**; no references are fabricated. Produced by a research agent, 2026-07-04.

---

## 0. The running example, stated precisely

An agent of reach `h_a` must operate a lock at `h_lock > h_a`. To bridge `Δ = h_lock − h_a` it must
(1) push boxes to the door (move objects to an *intermediate* pose), (2) climb them to unlock (the
intermediate pose is *functional*), (3) climb down, (4) move the boxes away (clear the doorway),
(5) open the door and pass.

Two features make this the **hard** case:
- **Objects are moved to an intermediate pose *and back*** — the box door-stack pose is a genuine
  *buffer occupancy* that must later be vacated ⇒ **non-monotone** (some object grasped/moved >1).
- **The number of boxes is itself unknown**, `K ≈ ⌈Δ / h_box⌉` (a covering/stacking sub-problem),
  and those `K` boxes must be **simultaneously present** at the door then **simultaneously removed** —
  precisely the **Minimum Running Buffers (MRB)** question — and clearing them needs free space
  elsewhere: **buffers to vacate the buffer**, the nested-dependency structure.

So the door problem forces intermediate placements, forces several to coexist, and nests a
resource-sizing sub-problem inside the sequencing problem.

---

## 1. Combinatorial structure

Let `N` = movable objects, `K` = number needing ≥1 intermediate placement (`K ≤ N`), `B` =
candidate discrete buffer regions.

- **Move count.** Monotone ⇒ `N` pick-places. Each buffered object costs ≥1 extra ⇒ `M ≥ N + K`.
  In the door example every box is "there-and-back" ⇒ `2K` moves.
- **Sequencing (discrete).** A plan is an ordering of `M` actions under precedence constraints;
  unconstrained count `(N+K)!` — a precedence-constrained scheduling problem.
- **Placement (continuous).** Each buffered object chooses *where* to sit; discretized to `B`
  regions ⇒ `×B^K`; continuous ⇒ a choice in `SE(2)` constrained by non-overlap with all objects
  present at that instant.

**Rough discrete search-space size:**
```
|Search| ≈ (N + K)!  ·  B^K            (discrete buffers)
|Search| ≈ (N + K)!  ·  (SE(2) region)^K   (continuous, exact)
```
`(N+K)!` is the *scheduling* explosion (grows with total objects **and** with how many need
re-visiting); `B^K` is the *placement* explosion (exponent = `K`). **Increasing `K` hits the search
space on both axes at once** — a few more objects that must be buffered is dramatically worse than a
few more objects total.

**Dependency graph.** Nodes = objects; edge `i → j` = `i` cannot be placed at goal until `j` vacates
(typically `goal(i) ∩ start(j) ≠ ∅`, an OBB overlap). Then:
- **acyclic ⇒ monotone** (`K = 0`, topological order, no buffers);
- **cyclic ⇒ non-monotone**: every cycle must be broken by evicting ≥1 object to a buffer. The
  minimum set whose removal makes `G` acyclic is a **feedback vertex set (FVS)** (≈ `K`); *how many
  buffers at once* is a stronger, ordering-dependent quantity (MRB, §3). This acyclic⇔monotone /
  cyclic⇔non-monotone characterization is standard (Rutgers/ARC line).

The door example is a **cycle by construction**: the boxes must clear the doorway to pass yet must
occupy it to be climbed — start-region, functional-region, and clear-region contend for the same
space, so no single-pass order exists.

---

## 2. Complexity results (verified)

| Problem | Hardness | Source |
|---|---|---|
| Motion planning among movable obstacles, goals **specified** | **PSPACE-hard** | Wilfong 1988/1991 |
| Same, goals **unspecified** | **NP-hard** | Wilfong |
| Sokoban / block-pushing | **PSPACE-complete** (NP-hard sub-cases) | Culberson; Dor & Zwick **[note]** |
| Optimal tabletop rearrangement (min pick-place / travel) | **NP-hard** | Han, Stiffler, Krontiris, Bekris, Yu |
| Minimum Constraint Removal (fewest obstacles to delete) | **NP-hard** (even convex polygons) | Hauser; Erickson & LaValle |
| Computing MRB (min running buffers) | **NP-hard** | Gao, Feng, Yu |

- **Wilfong** — reconfiguring by moving obstacles: **PSPACE-hard (specified goals) / NP-hard
  (unspecified)**. https://link.springer.com/article/10.1007/BF01530890
- **NAMO** — Stilman & Kuffner: even restricted NAMO is PSPACE-hard; a resolution-complete planner
  exists only for the **monotone, linear** subclass (`LP1`). The door problem leaves that subclass.
  https://www.ri.cmu.edu/pub_files/pub4/stilman_michael_2005_3/stilman_michael_2005_3.pdf ; IROS 2006
  http://www.kuffner.org/james/papers/NAMO_plan_exec_iros2006.pdf
- **Sokoban** — Dor & Zwick, *Comput. Geom.* 1999. https://www.sciencedirect.com/science/article/pii/S0925772199000176
  **[note]** PSPACE-completeness usually attributed to Culberson (1997), not fetched directly here.
- **Optimal tabletop rearrangement (NP-hard)** — Han et al., IJRR 2018. Closest to our geometry;
  optimal objectives NP-hard via two-directional reductions; links optimality to the **cyclic
  structure** of the dependency graph (min-travel ~ TSP-like; min-pick-place ~ feedback/cycle
  structure). https://arxiv.org/abs/1711.07369 **[uncertain: exact source-problem names not extracted]**
- **Minimum Constraint Removal (NP-hard)** — "what is the *fewest* things I must move?" — the
  box-count question in pure form; NP-hard by reduction from SET-COVER, even for convex polygons.
  Hauser, WAFR 2012/IJRR 2014 https://motion.cs.illinois.edu/papers/wafr2012-constraintremoval.pdf ;
  Erickson & LaValle, AAAI 2013 http://msl.cs.uiuc.edu/~lericks4/papers/aaai13mcrnphard.pdf
- **Monotone vs non-monotone (practical dividing line)** — Krontiris & Bekris, RSS 2015: monotone
  instances solved completely and fast; non-monotone require fundamentally harder search. Canonical
  demonstration that "requiring intermediate placements" is the difficulty cliff.
  https://www.roboticsproceedings.org/rss11/p45.pdf ; complete monotone primitive + non-monotone
  search: Wang, Gao, Nakhimovich, Yu, Bekris, ICRA 2021 https://arxiv.org/abs/2101.12241

---

## 3. The "running buffer" / nested-dependency angle — the user's real question

The closest result to *"how many boxes at once, and does it grow?"* is **Minimum Running Buffers**.

**Definition.** A *running buffer* is temporary storage for a displaced object. As a plan executes,
some objects are "in flight" (evicted from start, not yet at goal, in a buffer). The count occupied
**simultaneously** rises and falls; **MRB = the min over all valid plans of the maximum simultaneous
buffer occupancy.** It is *peak concurrent* count (buffer "slots" you must physically own), not the
*total* number buffered (that is FVS-like).

**Complexity.** Computing MRB on a dependency graph is **NP-hard** (Gao, Feng, Yu), via an
elimination/vertex-ordering DP that minimizes the max number of partially-processed nodes —
structurally analogous to a **vertex-separation / pathwidth** measure. **[uncertain: the pathwidth
analogy is my characterization; the paper states NP-hardness + exact DP.]**

**Unbounded growth — the key scaling fact.** Even for uniform objects, **MRB can grow *unbounded* as
`N` increases** (tight for the unlabeled case). This is the rigorous version of "denser/more-entangled
instances need *more simultaneous* buffers, without limit." So "how many boxes?" answers: worst-case
*not* bounded by a constant — it scales with scene entanglement, and finding the minimum is NP-hard.

**Empirical relief.** *Random* instances have **much smaller MRB** than worst case; exact solvers
scale to 100+ objects at high density. Typical scenes are far easier than the adversarial bound.

**Mapping to the door problem.**
- The `K = ⌈Δ/h_box⌉` boxes stacked at the door are **simultaneously displaced** into a functional
  buffer ⇒ their peak-concurrent count *is* an MRB-style quantity (`K` stacking slots at once).
- After climbing, boxes must be **vacated to yet another buffer** so the door can pass ⇒ **a buffer to
  empty the buffer** (nested dependency). If the "away" space is contested you get a *second* cycle,
  raising peak occupancy.
- The "how many boxes" sub-problem is a **covering/stacking** computation nested inside sequencing —
  mirroring how MCR nests set-cover inside path planning. Two NP-hard flavors compose.

**Cited work:**
- **On Minimizing the Number of Running Buffers for Tabletop Rearrangement** — Gao, Feng, Yu, RSS
  2021. https://arxiv.org/abs/2105.06357 · code https://github.com/arc-l/running-buffer
  (IJRR 2023 journal version arXiv:2304.01764.)
- **Toward Efficient Task Planning for Dual-Arm Tabletop Object Rearrangement** — Gao & Yu, IROS
  2022. **Lazy buffer allocation**: schedule primitive actions ignoring exact buffer poses, then
  allocate poses with collision checks — a two-phase decomposition transferable to our pipeline.
  https://arxiv.org/abs/2207.08078
- **On the Utility of Buffers in Pick-n-Swap Based Lattice Rearrangement** — Gao & Yu.
  https://arxiv.org/abs/2209.05390

---

## 4. Phase transitions / empirical scaling

**Well-established:**
- **Monotone-solvable fraction collapses with density.** As density `ρ` rises, the dependency graph
  gains edges → cycles → the monotone fraction drops and required buffering rises.
- **Peak buffer demand (MRB) grows with density/`N`**, provably unbounded worst case (§3), gently on
  random instances.
- **Runtime blow-up** near high density is the practical signature of the NP-hard core.

**Not verified — flagged:** I did **not** find a paper reporting a sharp **SAT-style phase-transition
threshold** (a critical `ρ*` with a narrow easy→hard→easy window) *specifically* for rearrangement.
The behavior is best described as a **steep monotone degradation with density**, not a confirmed
critical point. Treat any precise `ρ*` claim as **[unverified]**; the analogy to random-3-SAT /
random-graph cycle-emergence is suggestive and worth *testing* (§5).

---

## 5. Synthesis for our 2D OBB pipeline: what predicts difficulty, and how to measure it

Four measurable predictors, increasing discriminating power:

**(a) Object/goal density `ρ`** = Σ area(OBB) / area(free workspace) (also `ρ_goal`). Cheap; weak
standalone.

**(b) Dependency-graph structure (primary predictor).** Build `G` (edge `i → j` iff
`goal(OBB_i) ∩ start(OBB_j) ≠ ∅`, via SAT — already in our pipeline). **Tarjan SCC**: all singletons
⇒ **acyclic ⇒ monotone ⇒ easy** (`K=0`). If cyclic: report #/size of SCCs and an approximate **FVS**
(≈ `K`).

**(c) Required simultaneous buffers (MRB) — sharpest, matches the user's question.** Run the
Gao–Feng–Yu elimination-ordering DP (or a pathwidth approximation) on `G` for **peak concurrent buffer
demand**. `MRB=0` monotone; `MRB=1` mildly non-monotone; `MRB≥2` with nesting is where solve time
explodes.

**(d) Buffer-availability margin (feasibility).** Compare **required simultaneous displaced footprint**
(Σ area of the `MRB` largest concurrently-buffered objects) vs **available free area** away from
contested regions. Ratio `< 1` ⇒ likely **infeasible without re-nesting** (buffer-to-vacate-buffer) —
captures the door problem's "is there room to set the boxes aside?", invisible to (a)–(c).

**Proposed difficulty score** (benchmark labeling / curriculum):
```
difficulty ↑ with:  ρ,  #cycles,  |FVS| (≈K),  MRB,  (1 / buffer-availability-margin)
```
with **MRB and buffer-availability weighted highest** — they, not raw density, govern the jump from
polynomial-friendly monotone instances to the NP-hard non-monotone core.

**Concrete experiment (also the honest test of §4):** sweep `ρ`, generate random OBB instances, plot
(i) fraction acyclic/monotone, (ii) mean/95th-pct MRB, (iii) solver runtime/success. A repeatable
knee at some `ρ*` would be *empirical* evidence of a rearrangement phase transition on our
distribution — a publishable observation the literature leaves open.

---

### One-paragraph answer

Adding objects that need only one placement grows the problem factorially but benignly (often
monotone, topologically solvable). Adding objects that must pass through an **intermediate/buffer**
pose is categorically worse: each (i) adds an extra pick-place (enlarging the `(N+K)!` base),
(ii) contributes a `×B` (or `SE(2)`) placement factor (`B^K`), and (iii) can create/thicken cycles,
pushing the instance from the tractable **monotone** class into the **NP-hard non-monotone** class.
The *peak number that must be buffered simultaneously* (MRB) can grow **unbounded** and is itself
**NP-hard** to compute — the rigorous form of the door agent's nested "how many boxes at once, and
where do they go afterward?"
