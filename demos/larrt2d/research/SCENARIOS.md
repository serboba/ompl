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

### `tools/gen_scenes.py` — parameterized scene-family generator (T1)  ✅ built
`python3 tools/gen_scenes.py {swap|door_chain|blocked|random_room|suite} ...`. Emits
schema-correct JSON reproducing the anchor geometry exactly (box half 0.35, lane
y∈[2.5,3.6], niche opening 1.4 with a stub floor at 1.4, wall 0.3). Four families,
three with a **derivable optimum** used to cross-check the sound LB:
- **`swap_k_m`** — reverse k single-file boxes with m niches. Sound LB **2k−1** for all
  k (a valid lower bound); the LB is **met (optimum certified) only for k=2,3** — for k≥4
  the planner cannot reach 2k−1 (`swap_4_3`: bracket [7,9]) and 2k−1 is not yet proven
  achievable, so the conjecture 2k−1=optimum is **open for k≥4**. The LB is **independent of m**
  (it is a property of the reversal permutation); m only controls *feasibility*
  (a plan exists iff m ≥ MRB(k)), so sweeping m reveals **MRB(k)** as the smallest
  solving m — a clean phase-transition probe. Generalizes buffer_swap (2,1) /
  two_buffer (3,2).
- **`door_chain_d`** — d re-closing swing doors in series; A crosses all. Optimum
  **2d+1** (PROVEN: open all, cross once, close all). Generalizes door_relock.
- **`blocked_goal_chain_k`** — k parked blockers (start==goal) on A's only path,
  each with its own niche. Optimum **2k+1** (PROVEN). Generalizes blocked_goal.
- **`random_room`** — n boxes, random non-overlapping start/goal, seeded LCG. No
  closed form → exact oracle (T5b) supplies ground truth. *(deferred)*

Two **door-authoring pitfalls** the generator encodes (learned by debugging):
(1) the hinge must sit just **right of the stub edge** (X+wall+0.05), never inside
it, or the blade sweeps into the stub and the door is unopenable (cf. §3.4 wedge);
(2) A's goal must lie **beyond the last door's swing reach** (hinge+blade), else the
door cannot re-close with A parked at goal and the plan gains a spurious extra run
`[door,A,door,A]`.

### LB looseness bug — found by the T1 cross-check, FIXED  ★ (2026-07-04)
Generating the chain families and comparing the generator's *proven* optimum to the
sound LB exposed a real under-count: `blocked_goal_chain_2` LB **4** (opt 5),
`door_chain_2` LB **4** (opt 5), `door_chain_3` LB **5** (opt 7) — gap = (#blockers−1).
**Cause:** the min-COST FVS broke every 2-cycle by buffering the single shared mover A
(cost +1), dodging the +2 out-and-back each *parked* blocker must pay. **Fix
(`monotonicity.py analyse()`):** the out-and-back of a start==goal forced blocker is
mandatory (a disconnection arc proves it must leave, start==goal forces it back ⇒
≥2 runs), so it is charged as a **base cost of 2**, not an avoidable FVS surcharge;
the FVS surcharge (+1) now falls only on movers. Result: `door_chain_d` → 2d+1,
`blocked_goal_chain_k` → 2k+1, with **all 11 previously-certified suite LBs
unchanged** (no regression) and the defrag tests still green. A T4 "LB-strengthening"
win surfaced by T1, and it makes both chain families certifiable.

### `tools/bench_certify.py` — certification benchmark harness (T1)  ✅ built
`python3 tools/bench_certify.py [--runs N] [--time T] [--glob 'scenes/gen/*.json']`
(from repo root). Layers the Q5 bracket + T1 cross-check on `benchmark.run_once`:
per scene records `{family, derived_optimum, optimum_status, sound_lb, best_ub,
certified, note}` to `out/benchmark_certify.json`; `note` flags `LB!=proven_opt`
(the bug trigger) and `UB!=opt`. Heavy batch execution over the family sweep is the
step to delegate (HANDOFF §0); the bracket logic and mismatch analysis stay local.

**First sweep (18 structured scenes, larrt, best-of-10 × 8 s; 2026-07-05):
`9/18 certified`, `0` optimum↔LB mismatches** (`out/benchmark_certify.json`):
| family | certified | not yet |
|---|---|---|
| `door_chain_d` | **d=1,2,3,4** (3,5,7,9) — scales perfectly | — |
| `blocked_goal_chain_k` | **k=1,2,3** (3,5,7) | k=4 (LB 9): unsolved in 8 s → **planner-scaling gap** |
| `swap_k_m` | **swap_2_1 (3), swap_3_2 (5)** | swap_3_1, swap_4_*, swap_5_* (below) |

Two DISTINCT non-certification causes, worth separating in the paper:
- **Planner-scaling gaps** (`blocked_goal_chain_4`, all `swap_{4,5}_*`): unsolved within
  the 8 s budget; the sound LB stands, the planner just doesn't reach a solution at this
  size/time. **A 20 s best-of-12 retry did NOT close them** — `blocked_goal_chain_4` and
  `swap_4_1` stayed 0/12, and `swap_4_3` only reached **9** (never the conjectured 7), i.e.
  an open bracket **[7, 9]**. So these are *genuine* planner gaps needing better search
  (T5), not budget artifacts. **Consequence: the swap `2k−1` optimum is verified only for
  k=2,3** (brackets closed at 3, 5); for **k≥4 it is OPEN** — 7 is neither reached by the
  planner nor yet proven achievable for a 4-reversal, so `swap_4_3`'s true optimum ∈ [7, 9]
  is undetermined (planner-suboptimal vs LB-loose, same ambiguity as `swap_3_1`).
- **Buffer-scarcity LB looseness** (`swap_3_1`): SOLVED but at **7 > LB 5**. With a single
  niche the 5-action plan — which parks **two** objects simultaneously (`B→niche₁, A→niche₂,
  C→…`) — is unavailable, so the count rises. The sound LB is **niche-count-independent**
  (a property of the reversal permutation), so it does not see buffer scarcity and is
  **loose (still valid: 5 ≤ 7) when m < MRB-of-the-cheap-plan**. Whether 5 is *provably*
  unachievable with one niche (LB genuinely loose) vs merely unfound (planner suboptimal) is
  open — but the 5-plan's 2-simultaneous-buffer requirement is strong evidence for the former.
  ⟹ an **MRB-aware lower bound** is the natural T4 extension; `swap_k_m` with m<k−1 is its
  testbed. (This nuance — m sets the *cost*, not just feasibility — is a nicer result than the
  original "m only controls feasibility" guess.)

### `tools/difficulty.py` — the four predictors of `03_...md` §5
Density `ρ`, #cycles, |FVS|, and (approx) MRB + buffer-availability margin — a difficulty score for
labelling/curriculum and for the phase-transition experiment (`03_...md` §4–5).
