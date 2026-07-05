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
`9/18` certified by the *planner* bracket, `0` optimum↔LB mismatches
(`out/benchmark_certify.json`). The exact oracle (`tools/oracle.py`, 2026-07-06) then certified
the rest of the resolved cases and confirmed **the sound LB is TIGHT on every structured scene**:
| family | LB=optimum (oracle-confirmed) | certified how |
|---|---|---|
| `door_chain_d` | d=1..4 → 3,5,7,9 | planner (bracket closed) |
| `blocked_goal_chain_k` | k=1..3 → 3,5,7; **k=4 → 9** | planner (1–3); **oracle (k=4)** |
| `swap_k_m` (m=k−1) | swap_2_1 3, swap_3_2 5, **swap_4_3 7** | planner (2_1,3_2); **oracle (4_3)** |
| `swap_3_1` (m=1) | **5** (stacking plan) | **oracle** |

**Conclusion — for the CANONICAL family (`m=k−1`) + chains, there is ONE non-certification cause:
planner scaling.** On these, the sound LB equals the true optimum (oracle-confirmed: swap `2k−1`
for k=2,3,4; door `2d+1` for d=1..4; blocked `2k+1` for k=1..4). Where the planner alone did not
close the bracket (`blocked_goal_chain_4`, `swap_4_3`, `swap_3_1`), it was **planner
suboptimality/scaling**, not LB looseness — the oracle proves the true optima are 9, 7, 5 (= LB).
**The oracle closes every canonical bracket the planner misses.**

**The sub-MRB swaps (`m<k−1`) show the sound LB CAN BE LOOSE — T4 confirmed (2026-07-06).**
The full oracle is intractable at n=4 `--no-prune` (`swap_4_2` timed out at 900 s), so this was
settled by **exhaustive search over the only 7-action structure** a 4-reversal can have (A crosses
once; each of B,C,D parks once then goes to goal; all three parked simultaneously since each blocks
A). That search — validated because it **finds** the 7-plan for `swap_4_3` (3 niches, where the
oracle independently proved optimum 7) — finds **NO 7-plan for `swap_4_2`** (2 niches). Hence
**`swap_4_2`'s optimum ≥ 8 > sound LB 7: the niche-count-independent LB is genuinely LOOSE under
buffer scarcity.** Mechanism: with 2 niches you must stack a pair, and stacking imposes an
*unstack order* (top box exits first) that collides with goal placement (the top box's goal blocks
the bottom's path), forcing extra buffer visits the FVS-based LB never counts (the MRB phenomenon,
`03`). `swap_3_1` (1 niche, k=3) was the lucky case where stacking *did* rescue tightness (optimum
5 = LB); `swap_4_2` is where it cannot. **So: LB tight on the canonical family (`m=k−1`) + chains;
LB provably LOOSE on sub-MRB (`m<k−1`) ⟹ T4 (MRB / running-buffer-aware LB) is warranted with a
rigorous example.** (Exact `swap_4_2` optimum ∈ [8,12] still open — lean-oracle no-stacking bound
is 12; pinning it needs a tractable full oracle.)

**T4 IMPLEMENTED (2026-07-06): `monotonicity.py --mrb` / `bench_certify.py --mrb`.** `mrb_refine()`
soundly tightens the LB for the single-crosser box-corridor family (guard: all boxes,
`base_lb == 2·#objects−1`): it runs the exhaustive `swap_optimum_check` (all niche-slot assignments
incl. stacking × all park/unpark orders, reachability-checked); if no park-once plan of length
`base_lb` exists, no plan of that length does, so it **raises the LB by 1**. It ONLY ever raises the
LB and only on proven infeasibility ⇒ cannot be unsound. Verified: `swap_4_2` 7→**8** (certifies the
looseness), all tight scenes unchanged, non-matching scenes skipped, off by default. Open: the +1 is
not necessarily tight (swap_4_2 optimum may exceed 8), and generalizing beyond the single-crosser
corridor family is the honest remaining T4 work (use `--niche-floor` shallow scenes as the testbed).
- **~~Buffer-scarcity LB looseness~~ (`swap_3_1`) — RETRACTED by the exact oracle (2026-07-06).**
  The planner solves `swap_3_1` at **7**, so I hypothesized the LB (5) was *loose* under buffer
  scarcity (one niche can't run a 2-simultaneous-buffer 5-plan). **Wrong.** `tools/oracle.py`
  (A* over the discretized mode graph) finds an **optimal 5-action plan with one niche**, and I
  independently re-verified its critical passage at h=0.02: the plan **stacks two boxes vertically
  in the single niche column** (B at y≈1.75, C at y≈2.5) and threads A through a ~0.05-wide sliver
  over them (free A-centre y ∈ {3.21,3.23,3.25}). So the **sound LB 5 is TIGHT**, `swap_3_1` is
  **certifiable at 5**, and the planner's 7 is pure **suboptimality** (a sampling planner won't find
  the knife-edge stacking plan). ⟹ this is a *planner* gap (T5), NOT LB looseness; MRB(3-reversal)=1
  here (via stacking), and the suite currently has **no confirmed example of a loose sound LB**. Nice
  side-benefit: the oracle closes a bracket (5) the planner reliably misses.

### `tools/oracle.py` — exact small-n ground-truth oracle (T5b)  ✅ built
`python3 tools/oracle.py scenes/<scene>.json [--cand-step 0.5] [--grid 0.1]`. A* over a
**discretized mode graph**: a *configuration* assigns each object to a candidate pose (all
objects' start/goal + a collision-free buffer grid); an *action* moves one object between
candidates iff a collision-free single-object path exists with the others frozen (grid-flood
reachability for boxes — connected-components cached per (object, others-config) for speed —
interval sweep for 1-DOF). Cost = #actions; admissible heuristic = #objects off their goal. The
result is an **exact optimum over the discretization = a real, validatable UPPER bound**, so it
resolves the "planner-suboptimal vs LB-loose" question the planner alone cannot: oracle==LB ⇒
certified; oracle<planner_UB ⇒ planner gap; oracle>LB (rich candidates) ⇒ LB genuinely loose.
**Validated** against every known optimum (buffer_swap 3, blocked_goal 3, door_relock 3,
two_buffer 5, swap_3_2 5, door_chain_2 5) and used to **certify the planner's open cases**:
`swap_4_3` → **7**, `blocked_goal_chain_4` → **9**, `swap_3_1` → **5** (all = the sound LB).
**Candidate modes:** default is LEAN (buffers = off-through-path pockets, farthest-point-sampled to
`--max-buffers`) — fast, tractable to n≈5, and a valid UPPER bound. `--no-prune` keeps the full grid
(needed for near-lane **stacking** optima the lean prune removes): e.g. `swap_3_1`'s knife-edge 5-plan
(stack two boxes in one niche column + squeeze A through a 0.05 sliver) only appears with
`--no-prune --cand-step 0.7` (whose y-grid hits the stacking pose y=2.5). Workflow: run lean; if the
result exceeds the sound LB, re-run `--no-prune` (small n) to check for a stacking optimum. Certifies
via the sandwich LB ≤ optimum ≤ oracle: when oracle == LB the optimum is proven.

### `tools/difficulty.py` — the four predictors of `03_...md` §5
Density `ρ`, #cycles, |FVS|, and (approx) MRB + buffer-availability margin — a difficulty score for
labelling/curriculum and for the phase-transition experiment (`03_...md` §4–5).
