# Baseline planners vs. LA-RRT on the 2D rearrangement scenes

Phase-1 study: can a standard OMPL optimal planner (BIT*, AIT*, FMT*, RRT*) replace LA-RRT and be
"faster on complex environments"? Short answer: **no — not as a drop-in.** They optimise a
different objective and/or cannot handle the rearrangement problem's structure. This documents the
evidence so we don't relitigate it.

The driver exposes them for comparison: `demo_LARRT2D_puzzle <scene> <t> <planner>` with planner in
`{larrt, rrtconnect, rrtstar, bitstar, aitstar, fmt}`.

## Result (3 runs/config, 6 s budget; validated by the independent checker)

| scene | planner | solved | **valid** | best actions | time (s) |
|---|---|---|---|---|---|
| move_aside | **larrt** | 3/3 | **3/3** | 2 | 6.0 |
| move_aside | rrtconnect | 3/3 | **0/3** | 1 | 0.002 |
| move_aside | rrtstar | 3/3 | **0/3** | 1 | 6.0 |
| corridor_swap | **larrt** | 3/3 | **3/3** | 3 | 6.0 |
| corridor_swap | rrtconnect | 3/3 | **0/3** | 3 | 0.004 |
| corridor_swap | rrtstar | 3/3 | **0/3** | 2 | 6.0 |
| two_alcoves | **larrt** | 3/3 | **3/3** | 3 | 6.0 |
| two_alcoves | rrtconnect | 3/3 | **0/3** | 1 | 0.005 |
| two_alcoves | rrtstar | 3/3 | **0/3** | 1 | 6.0 |
| three_alcoves | **larrt** | 3/3 | **3/3** | 5 | 6.0 |
| three_alcoves | rrtconnect | 3/3 | **0/3** | 3 | 0.09 |
| three_alcoves | rrtstar | 3/3 | **0/3** | 1 | 6.0 |

**Every baseline is 0/3 valid on every scene.** They are fast at finding *a* path, but not a valid
*rearrangement* plan.

## Why — three distinct failure modes

1. **Wrong objective / simultaneous motion (RRTConnect, RRT\*).** These connect states with straight
   lines in the full space, moving *all* objects at once. That violates one-object-per-action, so
   the independent validator's "single object moves per segment" check fails. Their action counts
   (often 1) are meaningless — "move everything simultaneously" is not an executable pick-and-place
   sequence. RRT\* additionally optimises **path length**, not action count, so even ignoring
   validity it is not optimising our objective.

2. **Informed-sampler hang on locked DOFs (BIT\*, AIT\*).** Non-rotating objects have a locked
   orientation (a near-zero-width `theta` bound). BIT*/AIT* draw samples by **rejection inside an
   L2 informed ellipsoid**; the ellipsoid has ~zero width in the locked dimension, so almost no
   sample lands in the valid sliver and the sampler loops — the planner never returns (runs past its
   time budget without polling termination). Informed geometric planners assume a full-dimensional,
   length-metric space; a factored space with locked DOFs breaks that assumption.

3. **Batch planner can't reach a measure-zero goal (FMT\*).** FMT* samples a fixed batch up front and
   searches it once; it does not use goal sampling. The rearrangement goal is measure-zero in the
   target dimensions (see `GOAL_REGION_AND_COMPLETENESS.md`), so no batch sample ever lands in it →
   "No state inside goal region" → immediate failure.

## Takeaway

The value of LA-RRT is precisely the thing these planners lack: it searches over **which object
moves next** (minimising action switches) with **one object per edge**, and samples the measure-zero
goal region. A faster generic optimal planner does not substitute for that.

Where a smart optimal planner *does* help is one level down — as the **per-object motion planner**
(move one object, others fixed). That sub-problem is a full-dimensional, length-optimal geometric
query with **no locked-DOF pathology** (plan only the moving object's 2 DOF), exactly where BIT*/AIT*
are strong. That is the Phase-2 direction: keep LA-RRT's action-level search, swap the per-object
motion primitive to BIT*/AIT*. See the two-level design note.
