# Monotone vs. Non-Monotone Rearrangement Planning — literature briefing

> Scope: prehensile ("pick-and-place") and NAMO-style object rearrangement / manipulation
> planning. Every cited paper was checked against a live source (arXiv, DOI, official
> proceedings, or dblp); items that could not be fully verified are flagged **[UNCERTAIN]**.
> Produced by a research agent, 2026-07-04.

---

## 1. Precise definitions

### 1.1 The rearrangement instance
A (labeled) rearrangement instance is ⟨R, O, A_start, A_goal, W⟩:
- **O = {o₁,…,o_n}** movable objects in workspace **W** with static obstacles.
- An **arrangement** A assigns a collision-free pose to each object (objects + static obstacles
  mutually non-overlapping = a *valid* arrangement).
- **A_start**, **A_goal** are initial/target arrangements. Some objects are *movable obstacles*
  with **no goal** (any collision-free final pose is fine) — these are "reusable clutter" and are
  a common reason non-monotone reasoning is forced.
- Robot **R** manipulates **one object at a time**. A **move** (pick-and-place / *transfer*)
  removes o_i from its pose and re-places it, connected by collision-free transit + transfer.

A **plan** is a sequence of moves A_start → A_goal, alternating **TRANSIT** (robot alone) and
**TRANSFER** (robot holding one object) modes (Alami/Laumond/Siméon; central to NAMO via Stilman
& Kuffner).

### 1.2 Monotone plan (formal)
> **A plan is *monotone* iff each object is moved at most once, straight to its goal — no object
> is ever placed at a non-goal ("buffer"/intermediate) pose.**

Formally, a monotone plan is a **permutation** π of a subset of objects: move each o_{π(k)} once,
start → goal, such that at that moment its swept volume is collision-free against static
obstacles, objects already at their **goal** (π(1..k−1)), and objects still at **start**
(π(k+1..n)). Equivalently: a monotone solution exists iff there is an **ordering** where each
object can go start → goal in one shot given "future" objects at start and "past" at goal. Used by
Krontiris & Bekris (RSS 2015, *monotone Multi-object Rearrangement*) and Wang et al. (ICRA 2021,
complete `DFS_DP` primitive). In NAMO this is the **"linear"/LR-NAMO** restriction (Stilman &
Kuffner 2005).

### 1.3 Non-monotone plan
> **Non-monotone = violates monotonicity**: some object is moved more than once, and/or placed at
> a **non-goal buffer** pose. Includes the canonical case where an object **already at its goal
> must be temporarily displaced** to let another through, then returned.

Strictly more expressive: some instances are solvable **only** non-monotonically (a buffer is
provably required). The minimal extra machinery is one **buffer**; the hard question is *how many
simultaneous buffers* (the "running buffer" question — see `03_complexity_scaling.md`).

**Definition sources (verified):** Stilman & Kuffner, NAMO (Humanoids 2004 / IJHR 2005); Krontiris
& Bekris, "Dealing with Difficult Instances of Object Rearrangement" (RSS 2015); Ota, "Rearrangement
of Multiple Movable Objects" (ICRA 2004, early precedence/dependency graph).

---

## 2. Deciding a priori whether an instance is monotone-solvable

### 2.1 The dependency / constraint graph
Directed graph **G = (O, E)**; an arc = precedence "must move X before Y". Two canonical types:
- **Start-blocking (goal-vs-start):** if placing **A at its goal** overlaps **B's start** (or A's
  transfer sweep passes through B's start) ⇒ **B before A** ⇒ arc **B → A**.
- **Goal-blocking (start-vs-goal / goal-vs-goal):** if **A's start** overlaps **B's goal** ⇒
  **A before B** ⇒ arc **A → B**.

In the overhand/tabletop model (top-down grasps, vertical transfers) constraints reduce to
**pairwise start/goal footprint overlaps** — the "dependency graph" used by Gao, Feng, Huang & Yu
(IJRR 2023) to reason about buffers.

### 2.2 Acyclic ⇒ monotone; cycle ⇒ buffer needed
- **G acyclic (DAG)** ⇒ any **topological order** is a valid monotone ordering ⇒ **monotone-solvable**.
- **G has a directed cycle** (e.g. a 2-object **swap**: each goal is the other's start) ⇒ **no
  monotone ordering** ⇒ at least one object on the cycle must go to a **buffer** ⇒ **non-monotone**.

The **minimum-feedback-vertex-set / minimum-running-buffer** questions quantify "how many buffers
the cycle structure forces".

### 2.3 What is known vs. open
**Known:** Given the pairwise overlaps (i.e. assuming the simple transfer model), acyclicity is
checkable in O(n+|E|) and is a **sound-and-complete monotone test for that model**. `DFS_DP` (Wang
et al. 2021) is a **complete** decision procedure over a region-graph decomposition. General
prehensile monotone solvability is decidable by **complete backtracking over orderings**
(Krontiris & Bekris 2015; n! worst case but complete).

**Open/hard:** the dependency graph is only as correct as the **swept-volume model**. With
non-trivial grasps, arm collisions, rotations, or curved transfers, the true set of feasible
one-shot moves depends on the *continuous C-space and on which objects were already moved* — so the
static pairwise graph is an **approximation** that can be both **incomplete** (misses moves needing
a detour) and **unsound** (declares an arm-infeasible ordering). Whether a **compact certificate**
of non-monotonicity exists without C-space exploration in the general, arm-aware setting is **not
settled** — treat strong claims with caution.

---

## 3. Complexity & decidability
- **Rearrangement / NAMO is PSPACE-hard** (goals specified) / **NP-hard** (goals unspecified) —
  Wilfong, SoCG 1988 / Annals of Math & AI 1991.
- **Block-pushing / Sokoban is PSPACE-complete** (Culberson 1997; Dor & Zwick 1999); PushPush is
  **NP-hard** (Demaine, Demaine & O'Rourke).
- **Monotone tabletop rearrangement is NP-hard** — Han, Stiffler, Krontiris, Bekris, Yu (IJRR 2018,
  arXiv:1711.07369); pick-place-count objective is hard on the overlap/dependency structure,
  travel objective is TSP-like.
- **Minimum running buffers (MRB) is NP-hard and can be unbounded in n** — Gao, Feng, Huang, Yu
  (IJRR 2023, arXiv:2304.01764; RSS 2021 arXiv:2105.06357).

**Bottom line:** general rearrangement is PSPACE-hard; the "easy" monotone slice is still NP-hard;
the natural non-monotone objectives (min pick-place, min buffers, min travel) are NP-hard.

---

## 4. Key literature (verified)

**Seminal — NAMO & manipulation among movable objects**
- **Navigation Among Movable Obstacles: Real-Time Reasoning in Complex Environments** — Stilman &
  Kuffner, Humanoids 2004 / IJHR 2005. Defines NAMO, TRANSIT/TRANSFER, and the monotone ("linear")
  restriction. https://www.ri.cmu.edu/pub_files/pub4/stilman_michael_2004_1/stilman_michael_2004_1.pdf
- **Planning Among Movable Obstacles with Artificial Constraints** — Stilman & Kuffner, WAFR 2006
  (Springer STAR 47, 2008). Goal-derived "artificial constraints" that *order* obstacle moves — an
  explicit dependency-graph construction. https://link.springer.com/chapter/10.1007/978-3-540-68405-3_8
- **Rearrangement of Multiple Movable Objects** — Ota, ICRA 2004. Early precedence graph.
  https://ieeexplore.ieee.org/document/1308111/
- **Path Planning Among Movable Obstacles: A Probabilistically Complete Approach** — van den Berg,
  Stilman, Kuffner, Lin, Manocha, WAFR 2008. https://link.springer.com/chapter/10.1007/978-3-642-00312-7_37
- **Motion Planning in the Presence of Movable Obstacles** — Wilfong, 1988/1991 (the hardness
  foundation). https://link.springer.com/article/10.1007/BF01530890

**Monotone ⇄ non-monotone (core)**
- **Dealing with Difficult Instances of Object Rearrangement** — Krontiris & Bekris, RSS 2015. The
  paper that squarely tackles **non-monotone**: a monotone solver as a subroutine inside a
  higher-level search that adds intermediate/buffer placements to escape dependency cycles.
  https://www.roboticsproceedings.org/rss11/p45.pdf
- **Efficiently Solving General Rearrangement Tasks: A Fast Extension Primitive…** — Krontiris &
  Bekris, ICRA 2016. https://dblp.org/pid/98/8671.html
- **Uniform Object Rearrangement: From Complete Monotone Primitives to Efficient Non-Monotone
  Informed Search** — Wang, Gao, Nakhimovich, Yu, Bekris, ICRA 2021. Region-graph decomposition +
  complete monotone DP (`DFS_DP`), extended to single-buffer non-monotone + informed search over
  which objects/buffers. https://arxiv.org/abs/2101.12241
- **Complexity Results and Fast Methods for Optimal Tabletop Rearrangement with Overhand Grasps** —
  Han et al., IJRR 2018. https://arxiv.org/abs/1711.07369
- **Minimizing Running Buffers for Tabletop Object Rearrangement** — Gao, Feng, Huang, Yu, IJRR
  2023. https://arxiv.org/abs/2304.01764 (precursor RSS 2021 https://arxiv.org/abs/2105.06357)

**Factored / integrated TAMP**
- **PDDLStream** — Garrett, Lozano-Pérez, Kaelbling, ICAPS 2020. Declarative streams over
  continuous samplers; expresses buffers/re-grasps (non-monotone) as extra actions.
  https://arxiv.org/abs/1802.08705

**Recent GCS / convex-set planning (2024–2026)**
- **Mixed Discrete and Continuous Planning using Shortest Walks in Graphs of Convex Sets** —
  Morozov, Marcucci, Graesdal, Amice, Parrilo, Tedrake, **WAFR 2026**, arXiv:2507.10878. Shortest
  **walk** (vertices may repeat ⇒ an object can be moved twice) in a GCS. **[Most likely the WAFR
  2026 paper you recall; abstract foregrounds skill chaining / hybrid control rather than
  "pick-and-place" explicitly.]** https://arxiv.org/abs/2507.10878
- **Motion Planning with Precedence Specifications via Augmented Graphs of Convex Sets** —
  arXiv:2510.22015 (+ temporal-logic version arXiv:2606.00842). Encodes **key–door precedence** —
  structurally identical to rearrangement dependency graphs — into an augmented GCS. **[UNCERTAIN
  identity — verify authors/venue.]**
- **GCS\*: Forward Heuristic Search on Implicit Graphs of Convex Sets** — WAFR 2024.
  https://link.springer.com/chapter/10.1007/978-3-032-09967-9_5
- Rutgers ARC-L living index: https://arc-l.github.io/pub.html

---

## 5. A concrete recipe: testing monotone-solvability for a 2D OBB scene

**Input.** OBB objects, each with start s_i=(x,y,θ); *target* objects also have goal g_i; movable
obstacles have no goal. Plus static obstacles.

**Step 0 — Model the move.** Assume overhand grasp (lift/place; only the object's own OBB at start
and goal matters). This is the model under which the graph test is *sound*; deviations are the
failure modes below.

**Step 1 — Build the dependency graph G.** For every ordered pair (A,B):
- `overlap(goal(A), start(B))` (OBB–OBB via SAT, O(1)/pair) ⇒ arc **B → A**. Optionally use A's
  **transfer swept volume** (hull of A's OBB along start→goal) vs start(B) to also catch pass-over.
- `overlap(start(A), goal(B))` ⇒ arc **A → B**.

**Step 2 — Test acyclicity** (Kahn / Tarjan SCC, O(n+|E|)).
- **DAG ⇒ "monotone-plausible"**; return a topological order as the candidate one-shot ordering.
- **Cycle ⇒ "non-monotone: buffer required"**; return the SCCs as irreducible deadlocks; an FVS on
  them estimates how many objects must be buffered (cheap analogue of NP-hard MRB).

**Step 3 — (optional) estimate buffers.** Each SCC of size >1 forces ≥1 buffered object; exact
sizing is the NP-hard running-buffer problem — use only as a "how non-monotone" ranking.

**Step 4 — Movable obstacles.** No goal ⇒ only start-blocking arcs; monotone feasibility also needs
a **valid parking pose** that creates no new blocking arc (a placement-existence check).

### Why this cheap test can be WRONG
1. **False "monotone" (unsound):** OBB-only ignores the arm — a footprint-legal move can be
   kinematically infeasible.
2. **False "non-monotone" (incomplete):** a fat straight-line sweep flags a phantom cycle a curved
   transfer would avoid; too-thin sweeps make the opposite error.
3. **Rotation / non-convexity:** endpoint OBB tests miss mid-sweep collisions; hull sweeps overcount.
4. **Static graph ≠ dynamic feasibility:** relocating objects changes free space; acyclicity is
   exact only in the idealized order-independent model.
5. **Buffer existence not guaranteed:** a detected cycle says a buffer is *needed*, not that a valid
   buffer pose *fits* — in tight scenes it may not (the unbounded-buffer phenomenon).

**Practical resolution.** Use the OBB dependency-graph test as a **fast filter + heuristic**: DAG ⇒
try the topological order with a real per-move motion check, fall back to search on failure; cycle
⇒ hand the SCCs to a non-monotone planner (Krontiris–Bekris backtracking, `DFS_DP`+buffer search, or
a PDDLStream/GCS formulation) that introduces buffers/re-grasps and validates them in C-space.

---

*This directly informs our Q1 (a-priori monotonicity) and Q5 (LA-RRT as an upper bound paired with
a dependency-graph lower bound). The next concrete step for us is a `tools/monotonicity.py` that
builds G from a scene's OBBs and reports DAG/cycle + SCCs — see `SCENARIOS.md`.*
