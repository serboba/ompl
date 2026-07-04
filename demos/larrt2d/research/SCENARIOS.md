# Non-monotone / multi-object scenario suite

The empirical testbed for the research agenda (`AGENDA.md`). Each scene is designed to **require
intermediate (buffer) states** — for the target *and* for other objects — so we can study
monotone vs non-monotone behaviour and, later, optimality. Scenes live in
`demos/larrt2d/scenes/*.json`; solve with `demo_LARRT2D_puzzle`, check with
`tools/validate_puzzle.py`, render with `viz/viz_puzzle.py`.

**Non-monotonicity test (used below):** count, along the solution path, how many *separate action
runs* move each object. If any object moves in **>1** run, the plan is **non-monotone** (that object
visited a non-goal intermediate pose and moved again).

---

## Built

### `buffer_swap` — two-target swap through a buffer niche  ✅ non-monotone
- **Setup:** a single-file corridor (`y∈[2.5,3.6]`, one box tall) with a niche pocket below at
  `x∈[5.3,6.7], y∈[1.4,2.5]`. Boxes **A** (start `(2,3)` → goal `(9.7,3)`) and **B** (start
  `(9.7,3)` → goal `(2,3)`) must **swap ends**. They cannot pass in the corridor; the niche is the
  only place to let one wait.
- **Why non-monotone:** the dependency graph is a 2-cycle (A's goal = B's start and vice-versa) —
  the irreducible **swap** deadlock. A buffer is *provably* required.
- **Result:** solves, **VALIDATION: PASS**, non-monotone confirmed. Originally LA-RRT+defrag
  returned **4 actions** (`[A,B,A,B]`); the cause was a trailing-fragment off-by-one in
  `PathDefragmenter::findNextFragment` (the merge loop could never collect the path's final
  state, so a merge with the *last* fragment always failed). After the fix the planner returns
  **3 actions** (`[B, A, B]`), which **meets the endpoint-graph lower bound** from
  `tools/monotonicity.py` ⇒ **first CERTIFIED-optimal instance** (Q5 bracket closed:
  LB = UB = 3, |FVS| = 1 buffer).
- Files: `scenes/buffer_swap.json`, `out/buffer_swap.{json,gif,_strip.png}`.

### `door_relock` — pass through a door that must be re-closed  ✅ non-monotone (articulated)
- **Setup:** swing-door corridor (as `door_easy_fix`: blade length **2.85** — length 3.0 wedges the
  blade-tip corner into the upper stub, radius √(3²+0.15²)=3.0037 > gap 3.0, making the door
  *unopenable*; this same wedge is why `door_easy`/`door_dfirst`/`door_tgt` don't solve). Door is a
  **target with `goal_angle == start_angle`** (must end closed); box A crosses left→right to
  (8.9, 4.7), outside the door's swing disc.
- **Result:** solves in **8 ms**, **VALIDATION: PASS**, `[door, A, door]` = **3 actions**, door
  moves **twice** (open, re-close) ⇒ non-monotone on an *articulated* object. Matches the swept-graph
  prediction (LB_swept = 3: A mover + door forced blocker with start = goal costing 2).
- Files: `scenes/door_relock.json`, `out/door_relock.json`.

### `blocked_goal` — an object at its goal blocks another  ✅ non-monotone
- **Setup:** buffer_swap corridor; **B starts AT its goal** (6, 3) mid-corridor directly above the
  niche; A must cross (2,3)→(9.7,3). B dips into the niche and returns.
- **Result:** solves, **VALIDATION: PASS**, `[B, A, B]` = **3 actions**; **B visits an intermediate
  pose although start = goal** — the "move away and come back" motif (`01_...md` §1.3). Endpoint
  graph sees *nothing* (LB 1, no endpoint overlaps) while the swept graph predicts exactly 3 —
  a clean demonstration that this motif is invisible to endpoint-only dependency analysis.
- Files: `scenes/blocked_goal.json`, `out/blocked_goal.json`.

### `two_buffer` — two objects buffered simultaneously (MRB = 2)  ✅ non-monotone, open gap
- **Setup:** 14×6 single-file corridor, **two niches**; A (2,3)→(11.7,3) and C (11.7,3)→(2,3) swap
  ends, B (6.5,3) sits between the niches with `goal = start`. With one niche the outer pair still
  cannot pass in a pure corridor ⇒ **both B and one of A/C must be parked at the same time**:
  minimum running buffers = 2 by construction. Optimal plan is 5 actions
  (`B→niche₁, A→niche₂, C→left, A→right, B→back`).
- **Result:** solves, **VALIDATION: PASS**, but LA-RRT+defrag returns **8 actions**
  (`[C,B,A,B,C,B,C,B]`, B moved 4×) vs **LB_swept = 5** (SCC {A,B,C}, min-cost FVS = {A,B}).
  The reorder-only defragmenter cannot restructure this path ⇒ **the standing Q3 exemplar**: closing
  this gap needs buffer *re-planning* (keyframe search / `02_...md` §5, `06_...md` designs), not
  reordering.
- Files: `scenes/two_buffer.json`, `out/two_buffer.json`.

---

## Planned (to build next)

### `step_and_clear` — planar analogue of "stand on boxes to reach the lock"
A vertical `x–z` scene: a **lock** (slider/revolute) whose actuation has a **support precondition**
(a box must occupy a trigger region beneath it), after which the boxes must be **cleared** to open a
door and pass. Requires the `preconditions` metadata encoding from `04_urdf_to_2d.md` §3 (option 2),
since support is *not* a SAT collision constraint. This is the hardest planned scene and the bridge
to the URDF puzzles.

---

## Tooling to build alongside (the a-priori questions, Q1/Q4)

### `tools/monotonicity.py` — dependency-graph classifier + SOUND lower bound (v2)  ✅ built
`python3 tools/monotonicity.py <scene.json> [--solution out/x.json] [--grid h] [--no-disc] [--json]`.
Two graphs per scene:
- **sound graph** = *endpoint arcs* (goal↔start OBB overlaps) + *disconnection arcs*: freeze ONE
  pose of B as an obstacle, remove all other movables (only enlarges free space), and test whether
  A's start→goal stays connected in A's own DOF space. Box movers: over-approximated free-space
  grid — a cell is free only if the OBB **shrunk by the cell half-diagonal** is free at the cell
  centre, so any true path induces an 8-connected free chain and grid disconnection PROVES C-space
  disconnection. 1-DOF movers (door/slider/revolute): interval sweep, sound unless joint bounds
  span a full circle. Both arc kinds are necessary ordering constraints ⇒ the LB is certificate-grade.
- **swept graph** (straight-line transfer sweeps): cheap *predictor* only — not sound in principle
  (a curved transfer can dodge a straight-line blocker), though on the current suite it happens to
  agree with the sound graph everywhere.
Reports Tarjan SCCs, exact **minimum-COST feedback vertex set** (+1 buffer action for objects that
move anyway, +2 for start=goal parked objects), `LB = #movers + #forced-blockers + FVS cost`, and
with `--solution` the **Q5 bracket** ("bounds MEET ⇒ certified").

**Suite status with LB v2 (2026-07-04): 7 of 10 solvable scenes CERTIFIED optimal**
(`buffer_swap` 3, `blocked_goal` 3, `door_relock` 3, `corridor_swap` 3, `door_easy_fix` 2,
`move_aside` 2, `two_alcoves` 3). Open gaps: `two_buffer` 8 vs LB 5, `three_alcoves` 11 vs LB 4,
`my_scene` 4 vs LB 2, `door_openstart` 2 vs LB 1 — the planner-improvement target list.

**Retraction:** an earlier note here claimed `three_alcoves` had a certified 1-action optimum and
that this proved the swept graph unsound. Hand-analysis of the geometry shows the x∈(2.2, 2.4)
band is impassable for A at every height while B is parked (shelf-top and B-top constraints
contradict), so **LB 4 is correct** and the old 1-action solution file was stale/invalid (it was
never re-validated). The swept graph's unsoundness remains a theoretical caveat, but we currently
have no empirical counterexample in the suite.

### `tools/difficulty.py` — the four predictors of `03_...md` §5
Density `ρ`, #cycles, |FVS|, and (approx) MRB + buffer-availability margin — a difficulty score for
labelling/curriculum and for the phase-transition experiment (`03_...md` §4–5).
