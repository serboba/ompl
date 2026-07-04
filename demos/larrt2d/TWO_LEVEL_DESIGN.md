# Phase 2 — two-level rearrangement: LA-RRT action search + BIT*/AIT* per-object motion

Motivation (from `BASELINE_PLANNERS.md`): standard optimal planners fail as drop-in replacements
(wrong objective / invalid simultaneous motion / locked-DOF hang / measure-zero goal). But the piece
they are *good* at — a **full-dimensional, length-optimal single-object motion** — is exactly what
LA-RRT does crudely (single RRT step per extension). So keep LA-RRT's action-level search; make the
**per-object motion** optimal with BIT*/AIT*.

## The two levels

- **High level (unchanged): LA-RRT.** Searches over *which object moves next* and to *what
  intermediate placement*, minimising the number of actions (object switches), sampling the
  measure-zero goal region, with the PC-safe goal-biased projection.
- **Low level (new): a single-object optimal motion primitive.** "Move object *i* from A to B with
  all other objects frozen as obstacles." A 2-DOF (x, y) — or 3-DOF if the object rotates —
  length-optimal query solved by BIT*/AIT*.

## Why the low level is BIT*-friendly (and the full space is not)

Planning only the mover's DOF is full-dimensional with no locked axes, so BIT*/AIT*'s informed
ellipsoidal sampler is well-posed — none of the locked-DOF rejection-sampling hang that kills them
on the full factored space. Length is the native BIT* objective, so its informed machinery gives a
real speedup and near-optimal motions.

## Primitive — implemented and validated

`demo_LARRT2D_moveone <scene> <mover_index> <gx> <gy> [t] [planner]` (`LARRT2D_MoveOne.cpp`):
freezes every non-mover object at its start pose as an obstacle, builds a 2-DOF space for the mover,
runs the chosen planner (bitstar/aitstar/rrtstar/rrtconnect) to first exact solution, reports length,
dense collision-freeness, and time.

Measured (two_alcoves, move blocker B up into its alcove — a clear motion):

| planner | solved | path length | collision-free | time |
|---|---|---|---|---|
| **bitstar** | yes | **2.00 (optimal)** | yes | **0.6 ms** |
| rrtconnect | yes | 2.0096 | yes | 0.5 ms |
| rrtstar | yes | 2.00 | yes | 2.0 s (full budget) |

Blocked motions (target across the corridor with blockers frozen) correctly return **no path within
the budget** — the signal the high-level search uses to decide it must relocate an obstacle first.

## Integration plan (next)

1. **Primitive as a service** — wrap the above as a reusable function `planObjectMotion(scene, movers
   fixed, mover i, subgoal) -> path | INFEASIBLE`.
2. **LA-RRT hook** — in `growTree`, when extending group/object *g* toward a sample, replace the
   single straight RRT step with a call to the primitive that plans *g*'s full motion to the sampled
   sub-placement (others fixed at the node's pose). Each accepted edge is then an optimal single-object
   motion (still exactly one object per action, so the action-count semantics and the validator are
   unchanged). Keep a cheap feasibility budget for the primitive (short time / resolution-complete
   fallback) since blocked queries cost the whole budget otherwise.
3. **Cost/defrag** — action cost per edge stays 1 (one object); the primitive only changes the
   *geometry* of the motion, not the action structure, so `PathDefragmenter` is unaffected.
4. **Benchmark** — compare LA-RRT (RRT extension) vs LA-RRT+BIT* on the alcove scenes and on new
   narrow-passage scenes; expect the win to show up as fewer high-level iterations / faster solves
   on cluttered maps, not as a change in action count.

## Integration — IMPLEMENTED, and an honest benchmark result

Implemented as a **fallback** in `growTree` (flag `setOptimalConnector`, off by default; driver
planner name `larrtbit`): the cheap straight-line single-object step runs first, and only when it is
blocked does LA-RRT call `planFactorMotion` (BIT* over that object's DOFs, others frozen via the
full-space checker) to route the object around. Accepted as a chain of same-object edges, so the
action semantics and the validator are unchanged.

**Correctness:** regression-clean (plain `larrt` unchanged) and every `larrtbit` solution passes the
independent validator on all scenes.

**Performance — it does NOT currently pay off.** On a tight single-object slit (`bottleneck`, 0.02
clearance), success rate within a fixed budget:

| budget | larrt | larrtbit |
|---|---|---|
| 0.4 s | 5/10 | 3/10 |
| 1.0 s | 10/10 | 10/10 |
| 2.0 s | 10/10 | 8/10 |

The connector is **net-neutral-to-negative**. Two reasons, both important:
1. **Per-call overhead dominates.** Each blocked extension rebuilds a whole sub-`SpaceInformation` +
   `BITstar` + `setup()` (samplers, NN). That cost, paid on every `TRAPPED`, swamps the iterations
   saved. BIT* is 0.6 ms *in isolation* (§ primitive) but that number excludes the construction that
   an inner loop pays every call.
2. **Plain RRT is already good at single-object motion.** Range-limited RRT extension threads a slit
   fine; there is little for an optimal local planner to save on this problem class.

**What this means.** "BIT* doubles LA-RRT" is not borne out — not even at the per-object level, once
the planner is an inner loop. To have any chance it would need: (a) the sub-planner **cached/reused**
across calls (build the sub-SI + BIT* once per object, only reset start/goal + the frozen-others
vector) to kill the setup tax; and (b) scenes where per-object motion is genuinely hard for RRT
(long twisty corridors, not a single slit). Even then the upside is uncertain. The mechanism is left
in place, flag-gated off, as the substrate for that experiment; but the current evidence says the
better returns are elsewhere (e.g. smarter obstacle-clearing *order* at the action level, which is
where `three_alcoves` loses actions).

Status: primitive validated; integration implemented, correct, and benchmarked — with a negative
result that redirects the effort. Kept off by default.

## EIRM* multi-query — the positive result (why BIT* failed and this doesn't)

BIT* failed as an inner loop because it pays full construction/search cost *every* call. The
rearrangement inner loop solves "move object i through this space" hundreds of times over a largely
**shared** free space — the textbook case for a *multi-query* planner. `demo_LARRT2D_multiquery
<scene> <mover> [K] [t]` (`LARRT2D_MultiQuery.cpp`) tests it: one mover, fixed scene, K random
goals; EIRM* (roadmap reused across queries) vs BIT* (fresh per query).

| scene | EIRM* mean/query | BIT* mean/query | speedup |
|---|---|---|---|
| `bottleneck` (easy: trivial or infeasible) | 0.226 s | 0.226 s | **1.0×** |
| `maze` (hard, shared structure) | **0.006 s** | 0.825 s | **~138×** |

Reading:
- **Easy per-object motion (open scenes): no benefit.** Queries are straight-line-trivial (BIT*
  solves in 0.1 ms) or infeasible (both time out). Nothing to amortize — same as the BIT* finding.
- **Hard per-object motion with shared free space (maze): ~100×.** EIRM* builds the roadmap once
  (~0.13 s) then answers each query in 1–10 ms; BIT* re-searches from scratch (~0.8–1.0 s) each time.

So the earlier "2D is too cheap" conclusion was too broad: it holds for *open* scenes, but **cluttered
/ maze-like** per-object motion is exactly where multi-query reuse pays off massively. This is the
"complex environments" regime.

### Integrating EIRM* into LA-RRT — and the soundness caveat

The win above assumes a **fixed** collision environment across queries. In rearrangement the
per-object queries differ in the poses of the *other* movable objects, so a naively reused roadmap
would validate edges against a stale obstacle set — unsound. But the structure is favourable:
- The **static walls** (the source of maze difficulty and the bulk of validation effort) never move,
  so their roadmap structure is reusable across every query.
- The **movable objects are few** and are a small perturbation; only edges near them need
  re-validation per query.

Design for the sound integration:
1. Maintain **one EIRM* roadmap per object** over that object's DOFs, validated against the **static
   world only** (persistent, reused across the whole LA-RRT run).
2. Per query, treat the other movable objects as an **extra, cheap collision layer** re-checked
   lazily on the roadmap edges the search actually touches (a handful), rather than baking them into
   the cached validity. EIRM*'s lazy/effort-informed validation is built for exactly this
   incremental re-validation.
3. Fall back to full re-check / rebuild only if an object's roadmap becomes mostly invalid.

This is the next implementation step, and unlike the BIT* connector it has a measured 100× headroom
to justify the work — but only on cluttered maps; on open scenes it is correctly a no-op.

## EIRM* connector — IMPLEMENTED and it works on cluttered maps

Architecture (mirrors `GoalProjection`): a generic interface `ompl::geometric::FactorConnector`
(`LARRT.h`) with `connect(fromFull, targetReals, g, waypoints)`; LA-RRT holds an optional
`factorConnector_` (`setFactorConnector`) and calls it from `growTree` when an object's straight-line
extension is blocked. The geometry-aware implementation lives in the driver, so the generic planner
stays clean. (The BIT*-inside-LA-RRT code was removed.)

`EirmConnector` (`LARRT2D_Puzzle.cpp`), the sound design from above:
- **One persistent `EIRMstar` per object**, over that object's 2 free DOFs (x, y — the locked
  orientation is excluded, so there is no informed-sampler hang), with validity = **object vs static
  walls only**. Cached in a `std::map`, so the roadmap is built once and reused for the whole run.
- Each `connect`: set start/goal, `solve` (0.05 s, reusing the roadmap), then **re-check the
  wall-free path densely against the other movable objects at their current poses** (SAT); reject if
  any hit, else return the waypoints. Selected with driver planner name **`larrteirm`**.

**Result — cluttered `maze_blockers`** (serpentine maze + two movable blockers, target corner→corner):

| planner | solved (15 s budget) | best actions | validated |
|---|---|---|---|
| `larrt` (plain) | **1 / 5** | — | — |
| `larrteirm` | **3 / 6** | **4** (near-optimal) | **PASS (all 5 checks)** |

Plain LA-RRT usually **fails** this map (1/5 in 15 s, and even at 25 s) — its straight-line RRT can't
thread the maze. With the EIRM* connector, LA-RRT reuses object A's maze roadmap across the search
and solves it ~2× as often (3/6), with fully-validated plans as tight as **4 actions**. It is not
deterministic — the maze is genuinely hard and the walls-only-roadmap limitation adds variance — but
this is the first genuine planner-upgrade win in the study: on cluttered environments the EIRM*
connector lets LA-RRT solve rearrangement problems it otherwise can't, while being a no-op on open
scenes.

**Known limitation (sound, not minimal):** the walls-only roadmap returns the shortest *wall*-free
path; if a movable object sits on that path, `connect` rejects it and LA-RRT relocates the blocker,
rather than routing the object around the movable. Fixing that (per-query alternative paths, or
folding a cheap movable layer into the roadmap search) is future work.
