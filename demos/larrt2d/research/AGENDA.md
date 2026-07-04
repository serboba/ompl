# Research agenda — intermediate states, monotonicity, and optimality in rearrangement

*Captured 2026-07-04. This file preserves the framing and questions so we can iterate. The
literature answers live in the numbered briefings; this file is the "why".*

---

## 1. Motivation

The **move-n-times** puzzles we care about are *non-monotone*: solving them requires putting
objects into **intermediate states** (not their start, not their goal) one or more times, and
sometimes moving the *same* object several times.

Canonical examples:
- **Door re-lock:** open a door (move it), pass an object through, then **close the door again**
  (move it a second time). The door visits an open intermediate state and returns.
- **Buffer swap:** two objects must exchange positions through a corridor that fits only one at a
  time. One object must wait in a **buffer niche** while the other passes, then leave it.
- **Stand-on-boxes-to-reach-a-lock** (the hard one): a short agent must push boxes to a door,
  climb them to reach and turn the lock, climb down, **move the boxes away again**, then open the
  door and pass. Here *several* objects (the boxes) go to intermediate poses **and back**, and
  *how many boxes are needed* is itself part of the problem.

Key requirement for the scenes we build: **intermediate states must be required not only for the
target object but for the other objects too** — multiple objects each needing a buffer visit.

---

## 2. Definitions we are working with (to be sharpened in `01_...md`)

- **Monotone** rearrangement: each object is moved **at most once**, straight to its goal. No
  object is ever parked at a non-goal pose. (Equivalently: a valid ordering of one-shot moves
  exists.)
- **Non-monotone**: objects may be relocated **multiple times**, including to intermediate
  **buffer** placements; an object already at its goal may have to be temporarily moved away.

The intuition: if the "who must move before whom" **dependency graph** is acyclic, a monotone
plan exists (topological order). A **cycle** (A must move before B *and* B before A) forces a
buffer → non-monotone.

---

## 3. Core research questions

**Q1 — A-priori monotonicity.** Given a puzzle (start + goal arrangement of OBB objects, some
free), **how do we know whether it is monotone- or non-monotone-solvable** *before* planning?
Can we decide it from a dependency/constraint graph built from swept-volume overlaps, without
exploring the full configuration space?

**Q2 — Minimal intermediate visits.** How do we know the **minimum number of intermediate
placements** (buffer visits) a solution needs, when — like RRT — we have only sampled part of the
C-space, not all of it? The graph grows, but it does not sample everywhere. Is there a
certificate of minimality?

**Q3 — Optimality of our planner.** LA-RRT + `PathDefragmenter` has **no optimality guarantee**
(defrag only greedily simplifies a found path). What would it take to **guarantee** minimal
action count / minimal buffer count? Candidate directions:
  - exact search over a discrete **mode / dependency graph**;
  - **dedicated / constraint sampling** (Toussaint — "modulo"/constraint sampling on manifolds:
    the goal set is measure-zero in the target dims, the same issue we already hit) so that
    sampling concentrates where solutions live;
  - **convex-set** formulations (GCS) with global optimality — cf. a **WAFR 2026** paper doing
    pick-and-place search over convex sets.

**Q4 — Complexity growth.** How does difficulty scale as we add **more objects that must be moved
to intermediate states**? The door/boxes example nests dependencies (buffers to enable buffers).
Relevant: "**running buffers**" / minimum-simultaneous-buffer results.

**Q5 — LA-RRT as an indicator.** LA-RRT *does* produce a plan whose action count hints at
non-monotonicity, but without optimality it's only an **upper bound**. Can we pair it with a
cheap **lower bound** (e.g. from the dependency graph) to bracket the true minimum and, when the
bounds meet, *certify* monotone/non-monotone and the buffer count?

---

## 4. Leads to chase (fed to the research agents)

- **Toussaint** — Logic-Geometric Programming; constraint / manifold ("modulo") sampling.
- **GCS** — Marcucci & Tedrake, "Shortest Paths in Graphs of Convex Sets" / "Motion Planning
  around Obstacles with Convex Optimization"; the **WAFR 2026** convex-set pick-and-place paper.
- **Monotone/non-monotone** — Stilman & Kuffner (NAMO); Krontiris & Bekris (RSS 2015, "Dealing
  with Difficult Instances of Object Rearrangement").
- **Buffers / optimality / complexity** — Han & Yu (optimal tabletop rearrangement); Gao & Yu
  (minimizing running buffers); PDDLStream (Garrett, Lozano-Pérez, Kaelbling).

---

## 5. Work plan

1. **Literature briefings** (`01`–`04`) — produced by parallel research agents; verified citations.
2. **Scenario suite** (`SCENARIOS.md` + `demos/larrt2d/scenes/*.json`) — build *non-monotone,
   multi-object* scenes where several objects require intermediate placements; verify each
   actually needs re-visits (not monotone-reducible). These are the empirical testbed.
3. **URDF→2D** (`04`) — a converter design so the existing 3D manipulation puzzles can be projected
   into this 2D pipeline and re-solved here.
4. **Longer-term (separate project thread):** an *optimality-aware* planner — e.g. dependency-graph
   lower bound + constraint sampling, or a GCS/convex-set formulation — to move from "we found a
   plan" to "we found a **provably minimal-buffer** plan". This is the Toussaint-constraint-sampling
   research direction the user wants to pursue next.

---

## 6. How this connects to the current code

- Action counting & the factored search live in `src/ompl/.../LARRT.cpp`; the greedy simplifier in
  `src/ompl/geometric/src/PathDefragmenter.cpp` (now loops to convergence, but still no global
  optimality — see Q3).
- Goal-region semantics and the measure-zero / probabilistic-completeness caveat are already
  documented in `demos/larrt2d/GOAL_REGION_AND_COMPLETENESS.md` — directly relevant to Q3's
  "sample where solutions live".
