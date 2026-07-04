# Rearrangement goal region — semantics, optimality gap, and probabilistic completeness

This note documents the goal formulation used by the 2D rearrangement pipeline (and its
`RearrangementGoal` in `LARRT2D_Puzzle.cpp`), the optimality gap it currently exhibits, and the
**probabilistic-completeness (PC) implications of the proposed fix** — because the naive version
of that fix breaks PC. Read this before touching goal handling in LARRT.

## 1. The goal is a measure-zero manifold

Let the configuration space be `Q = Q_1 × … × Q_n` (one factor per object), `Q_free ⊆ Q` the
collision-free subset. Only the **target** objects `T ⊆ {1..n}` have a goal; the rest are free
movable obstacles. The goal set is

```
G = { q ∈ Q_free : q_i = g_i for all i ∈ T }        (targets pinned, others free)
```

`G` is a submanifold of `Q_free` with the target dimensions **fixed**. If there is at least one
target (there always is), `dim(G) = dim(Q) − Σ_{i∈T} dim(Q_i) < dim(Q)`, so **G has measure zero
in Q**. This single fact drives everything below: an unbiased sampler draws a state in `G` with
probability 0, so *nothing* reaches the goal "by chance" — reaching `G` must be engineered.

## 2. How LARRT reaches G today (and why it is PC)

LARRT is a **bidirectional** RRT. It handles the measure-zero goal by **sampling** it:

- `RearrangementGoal::sampleGoal` returns members of `G` directly (targets pinned to `g_i`,
  non-targets drawn at random and rejection-checked for validity). These seed/grow the **goal
  tree** `T_g` (LARRT does this via `pis_.nextGoal`, since `specs_.recognizedGoal =
  GOAL_SAMPLEABLE_REGION`).
- The **start tree** `T_s` grows from `q_start`. A solution is reported only when `T_s` connects
  to a `T_g` node (`gsc == REACHED`, `LARRT.cpp:460`).

**PC holds** by the standard RRT-Connect argument: the goal tree is seeded by concrete samples of
`G`, and the tree-connection is a positive-measure event in `Q_free`. As iterations → ∞, if a
collision-free path exists, the trees connect w.p. 1.

**But there is an optimality gap.** Every `T_g` node fixes the *non-target* objects at some
*random* sampled pose. When `T_s` connects to such a node, the free objects are forced to those
random poses — so LARRT sometimes moves a free obstacle **just to match the sample** (e.g.
`two_alcoves` reports 4 actions instead of the optimal 3; `three_alcoves` 5 instead of 4). The
"place obstacles anywhere" freedom is not exploited: the connection pins them to a sampled pose.

`LARRT.cpp:496–506` already evaluates `goal->isSatisfied(startNode)` — but only records it as an
**approximate** solution, never an exact one. That is the hook for the fix below.

## 3. Proposed fix, and the PC pitfall

Goal: exploit "obstacles anywhere" so free objects are left **wherever they already are** once the
target(s) are home — giving the true minimum action count.

### 3a. The WRONG version (breaks PC)

> "Just accept any start-tree node `q` with `isSatisfied(q)` as an exact solution."

This is **not** probabilistically complete on its own. `T_s` grows by extending toward *uniformly
random* samples of `Q`. The event "an extension lands with all targets exactly at `g_i`" has
**probability 0** (targets-at-goal is the measure-zero set of §1). So a start-tree node satisfying
the goal is produced only incidentally — today it happens *because* the start tree is being pulled
toward goal-tree nodes. Remove/short-circuit the goal tree and keep only this acceptance test, and
the planner can fail to ever reach `G` even as iterations → ∞. **Changing the acceptance criterion
alone is unsound.**

### 3b. The CORRECT version (PC-preserving): goal-biased extension onto G

Add a **goal-biased extension that projects onto the goal manifold**. With a fixed probability
`p_goal > 0` per iteration:

1. pick a start-tree node `q` (nearest to a goal sample, or random);
2. form its **projection** `π_G(q)`: set the target dims to `g_i`, **keep the non-target dims at
   their current values in `q`**;
3. attempt the straight motion `q → π_G(q)` with the normal collision checker. If valid, `π_G(q)
   ∈ G` is reached exactly — report it as an **exact** solution.

Because the non-targets are left in place, free objects are not moved to match any sample, so this
attains the minimal-motion solution. **PC is preserved** by the standard goal-bias argument: with
probability ≥ `p_goal` each iteration LARRT attempts the goal-directed connection from an expanding
tree; since `T_s` covers `Q_free` in the limit (RRT PC) and the projection motion is a
positive-probability attempt whenever it is collision-free, `G` is reached w.p. 1 if reachable.

Key requirements for the argument to hold:
- **`p_goal > 0` must remain fixed** (do not anneal it to 0). Dropping the goal bias reintroduces
  the measure-zero problem of §3a.
- **Keep a fallback that still reaches G when the direct projection is blocked.** If `q → π_G(q)`
  is never collision-free from reachable nodes (targets can't be brought home without *also*
  repositioning obstacles first), the projection alone cannot finish. Retaining the sampled goal
  tree (§2) as the fallback preserves PC in those instances. So the fix should be **additive**:
  goal-biased projection *plus* the existing goal-tree connection, not a replacement.
- **Interaction with `PathDefragmenter`.** The defragmenter must accept a path whose final state
  has free objects at arbitrary poses (it must not assume a fully-pinned goal). Verify it preserves
  the "targets at goal" invariant while it reorders/merges.

## 4. Summary of implications

| approach | reaches G? | free-obj motion | PC |
|---|---|---|---|
| goal tree sampled from G (current) | yes, by connection | forced to random sample | ✅ |
| accept start-tree `isSatisfied` only (naive fix) | not reliably | minimal | ❌ (measure-zero goal) |
| goal-biased projection onto G + goal-tree fallback (correct fix) | yes | minimal | ✅ (needs `p_goal>0` + fallback) |

**Bottom line:** the goal region is measure-zero in the target dimensions, so reaching it always
requires an explicit mechanism (goal sampling and/or goal-biased projection). The optimality fix
must *add* a goal-biased projection with a fixed positive bias and keep the sampled goal tree as a
fallback; simply loosening the acceptance test is not probabilistically complete.

## 5. Implementation status (IMPLEMENTED — the §3b correct fix)

The PC-preserving fix is implemented:

- **Interface** `ompl::geometric::GoalProjection` (`LARRT.h`): `projectToGoal(from, out)` writes the
  goal-manifold projection of `from` (targets pinned, free dims copied). Optional — LA-RRT
  `dynamic_cast`s the goal to it; ordinary goals are unaffected.
- **Planner** (`LARRT.cpp`, in the start-tree branch of `solve`): with fixed probability
  `goalBias_ = 0.05` it projects a freshly-grown start-tree node and, **only if the projection
  edge moves at most one object** (`changedGroups ≤ 1`, so it is a single valid action), accepts
  the start-tree chain (+ that one edge) as an **exact** solution. Multi-target goals (edge would
  move ≥2 objects at once) skip projection and fall back to the sampled goal tree. `goalBias_`
  stays > 0 and the goal tree is retained, so PC holds per §3b.
- **Driver** (`LARRT2D_Puzzle.cpp`): `RearrangementGoal` implements `GoalProjection` — copies the
  state and overwrites only the target objects' dims with their goal.
- **PathDefragmenter**: verified to preserve the "targets at goal, free objects wherever they are"
  end state (the independent `validate_puzzle.py` passes on every scene).

**Measured effect:** `two_alcoves` 4 → **3** (optimum reached), `move_aside` stays 2 (optimum),
`corridor_swap` (2 targets, no free objects) stays 3 via the fallback. `three_alcoves` stays **5**
(optimum is 4) — the projection removes the "free object forced to a random sample" waste, but
`three_alcoves`'s residual gap is a *different* problem: clearing three blockers minimally is an
action-search/defragmentation limitation, not a goal-matching one, and projection doesn't address
it. All plans still pass independent validation. Takeaway: the projection closes the goal-matching
gap (visible when blockers are few and easily cleared); the remaining gap on denser scenes is about
optimal obstacle-clearing order, which is the next planner-quality target.

See also: `PUZZLE_PIPELINE_SPEC.md` §1b (goal semantics), `RearrangementGoal` in
`LARRT2D_Puzzle.cpp`, and `LARRT.cpp:460` (connection) / `:496–506` (approx-only isSatisfied).
