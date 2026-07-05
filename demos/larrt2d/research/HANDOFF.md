# RESEARCH HANDOFF — certified action-optimal non-monotone rearrangement (LA-RRT 2D)

*Written 2026-07-04. This is the single entry point for any (AI) agent picking up this research
track. Read this file first; it tells you what exists, what was proven, what is open, how to run
everything, and exactly how to continue each work stream — including how to write the paper.*

---

## 0. How to work on this project (read before doing anything)

1. **Read order:** this file → `AGENDA.md` (the research questions Q1–Q5) → `SCENARIOS.md`
   (scene suite + results) → the briefing you need for your task (`01`–`08`, see §4).
2. **Delegation policy (important):** the orchestrating agent should NOT burn its own context on
   mechanical subtasks. **Spawn cheaper subagents — use `claude-sonnet-4-6` subagents** (or the
   nearest available Sonnet-class model) for: authoring scene JSON variants, batch planner runs
   and result collection, plot/table generation, citation re-verification, and code chores with a
   precise spec. Keep for yourself: math/soundness arguments, experiment design, result
   interpretation, and anything that changes planner semantics. **Always verify subagent output**
   (run the validator; spot-check numbers) — do not trust reported results without re-running the
   checks in §2.4.
3. **Hard rules learned the hard way:**
   - **Never trust a stale `out/*.json`.** Every quantitative claim must come from a solution that
     was re-validated (`validate_puzzle.py` → `VALIDATION: PASS`) in the same session. A stale
     invalid file cost us a wrong "certified at 1 action" claim (see §3.6 retraction).
   - The planner is stochastic with high variance (e.g. three_alcoves: 4–12 actions across seeds).
     Report **best-of-N with N stated**, keep the best solution file, and validate it.
   - `~/rb_ws` (user's MAB-RRT workspace) is **read-only / off-limits**.
   - Run the driver **from the repo root** (it writes `demos/larrt2d/out/<scene>.json`).

---

## 1. One-paragraph project summary

We plan rearrangement puzzles (2D OBB objects: boxes, doors, sliders, revolute levers; one object
moves at a time) with **LA-RRT**, a factored RRT whose edges each move one factor (= one object),
plus a **PathDefragmenter** that merges/reorders action runs. The scientific product this track is
building: a **certification bracket** — a *sound structural lower bound* on the number of actions
(computed a-priori from the scene in ms–s) paired with the planner's *upper bound*; when they meet
the action count is **certified optimal**. Today the bracket closes on **8 of 10 solvable suite
scenes**. The paper (§5) is about making that systematic: sound LB theory, an LB-guided /
bandit-driven planner, and a benchmark of scene families nobody else covers (articulated blockers +
scarce buffer niches).

---

## 2. Repository state & how to run everything

Worktree: `/home/serboba/larrt_wt`, branch `larrt`. Nothing from today is committed yet.

### 2.1 Key code
| file | what it is |
|---|---|
| `src/ompl/geometric/planners/rrt/src/LARRT.cpp` (+`.h`) | the planner. `growTree()` ≈ line 284: isolation path (multi-factor, line ~317) vs range-limited single-factor advance (uniform factor pick at **line ~352** — the bandit hook). Env vars: `LARRT_DBG=1` (branch counters), `LARRT_NO_ISO=1` (disable isolation path). |
| `src/ompl/geometric/src/PathDefragmenter.cpp` | post-processor; **reorder-only** (merges same-object runs by commuting whole fragments; never re-plans poses). Today's fix: `findNextFragment` same-group loop bound `size()-1` → `size()` (trailing fragment could never merge; cost one extra action on every `[…,X,…,X-final]` path). Tests: `./build/tests/test_larrt_defrag` (4 cases, green). |
| `src/ompl/base/spaces/src/FactoredStateSpace.cpp` | compound space; per-group sequential interpolation; SO(2)-marked dims wrap in `dimDistance`/`interpolate`. |
| `demos/larrt2d/LARRT2D_Puzzle.cpp` | scene-JSON driver (schema in §2.3), goal region (`RearrangementGoal`: targets pinned to goal, free objects anywhere valid). |
| `demos/larrt2d/tools/monotonicity.py` | **the LB/certification tool (v2)** — see §3.1. |
| `demos/larrt2d/tools/validate_puzzle.py` | independent SAT validator of solution files. |
| `demos/larrt2d/scenes/*.json`, `out/*.json` | scene suite and (validated) solutions. |

### 2.2 Commands
```bash
# build (from repo root)
cmake --build build --target demo_LARRT2D_puzzle -j8
# solve (writes demos/larrt2d/out/<name>.json; planner ∈ {larrt, larrteirm, rrtconnect, bitstar, aitstar, fmt, rrtstar})
./build/demos/demo_LARRT2D_puzzle demos/larrt2d/scenes/<scene>.json <seconds> larrt
# validate + classify + bracket (from demos/larrt2d)
python3 tools/validate_puzzle.py out/<scene>.json
python3 tools/monotonicity.py scenes/<scene>.json --solution out/<scene>.json   # [--grid h] [--no-disc] [--json]
# defrag regression tests
./build/tests/test_larrt_defrag
```

### 2.3 Scene JSON schema (per object type)
- `box`: `hx, hy, rotates, start [x,y,θ], goal [x,y,θ] (if target), xbounds, ybounds, tbounds`
- `door`: `hinge [x,y], length, width, start_angle, goal_angle (if target), angle_bounds`
- `revolute`: `anchor, com, hx, hy, start_angle, goal_angle, angle_bounds`
- `slider`: `base, axis, hx, hy, angle, start_disp, goal_disp, disp_bounds`
- plus `world {xmin..ymax}` and axis-aligned `obstacles`. `target: false` objects have no goal
  (may end anywhere valid). Touching is NOT collision (SAT with EPS=1e-6).

### 2.4 Verification protocol (apply to EVERY new result)
1. `validate_puzzle.py` → PASS. 2. recompute the per-object run sequence (non-monotone iff any
object has >1 run). 3. `monotonicity.py --solution` → record the bracket. 4. state best-of-N and N.

---

## 3. Everything found so far (the detailed record)

### 3.1 Sound lower bound v2 = the certification engine (`tools/monotonicity.py`)
Two dependency graphs per scene; arc `u → v` means "u must move before v acts/is placed".
- **Sound graph** (certificate-grade; used for the Q5 bracket):
  - *endpoint arcs*: `goal(A) ∩ start(B)` ⇒ `B → A`; symmetric for `start(A) ∩ goal(B)`.
  - *disconnection arcs*: freeze ONE pose of B as an obstacle, delete all other movables (deletion
    only enlarges free space ⇒ relaxation is sound), test whether A's start↔goal stays connected
    in A's own DOF space. **Box movers:** over-approximated grid — a cell is free only if A's OBB
    **shrunk by the cell half-diagonal** δ=h·√2/2 is collision-free at the cell centre; any truly
    free pose then lies in a free cell and a continuous path induces an 8-connected free chain, so
    **grid disconnection PROVES C-space disconnection** (grid connection proves nothing). Default
    h=0.05. **1-DOF movers** (door/slider/revolute): interval sweep — sound because a bounded
    1-DOF path must visit every value between its endpoints, UNLESS joint bounds span ≥2π (wrap
    possible → tool refuses to certify). **Rotating boxes: not implemented** (skipped with
    warning) — open task T4.
  - `start(B)` disconnects A ⇒ `B → A`; `goal(B)` disconnects A ⇒ `A → B` (A must finish first).
- **Swept graph**: endpoint arcs + straight-line transfer sweep hits. *Predictor only* — a curved
  transfer can dodge a straight-line blocker, so NEVER certify from it. (On today's suite it
  happens to agree with the sound graph everywhere; no empirical counterexample currently exists.)
- **From graph to bound:** Tarjan SCC; cycle ⇒ non-monotone (buffer provably required). Exact
  **minimum-COST feedback vertex set** per SCC, cost(v) = extra buffer actions = **+1** if v moves
  anyway (mover or forced blocker), **+2** if v is parked with start=goal (out AND back).
  `LB = #movers + #forced_blockers + Σ FVS costs`, where forced blockers = objects with an
  outgoing arc that don't otherwise move. **Bounds meet ⇒ certified optimal** under the
  pick-place OBB model (single mover, objects move one at a time).

**§3.1a — LB looseness bug found by T1's cross-check, FIXED (2026-07-04).** The formula above
under-counted when **multiple start=goal forced blockers share one mover**: the min-COST FVS breaks
every 2-cycle by buffering that one shared mover (cost +1) and never pays the +2 out-and-back the
parked blockers owe, so `door_chain_2`/`blocked_goal_chain_2` reported LB 4 (true optimum 5), gap
growing as #blockers−1. Fix in `analyse()`: a start=goal forced blocker's out-and-back is
**mandatory** (a disconnection arc proves it must leave; start=goal forces it back ⇒ ≥2 runs), so it
is now a **base cost of 2**, not an avoidable FVS surcharge; the FVS surcharge (+1) applies only to
movers, and parked/free blockers are 0-cost FVS members. Verified: chains now certify at 2d+1 /
2k+1; **all 11 previously-certified suite LBs unchanged**; defrag tests green. (This also advances
T4 "LB strengthening.") The §3.1 formula text above describes the *original* model; the code is the
corrected one.

### 3.2 Suite status (all solutions validated 2026-07-04)
| scene | sound LB | best UB (planner) | status |
|---|---|---|---|
| buffer_swap | 3 | 3 | **CERTIFIED** (2-cycle swap, 1 buffer) |
| corridor_swap | 3 | 3 | **CERTIFIED** (needed disconnection arcs; endpoint-only saw a DAG) |
| blocked_goal | 3 | 3 | **CERTIFIED** (B leaves its own goal and returns) |
| door_relock | 3 | 3 | **CERTIFIED** (articulated: door opens + re-closes; solves in ~8 ms) |
| two_buffer | 5 | 5 | **CERTIFIED** (MRB=2: FVS={A,B}; 5-action run found with `LARRT_NO_ISO=1`, best-of-~15) |
| door_easy_fix | 2 | 2 | **CERTIFIED** |
| move_aside | 2 | 2 | **CERTIFIED** |
| two_alcoves | 3 | 3 | **CERTIFIED** |
| three_alcoves | 4 | 5 | open gap [4,5]; a 4 was observed once but not preserved/validated |
| my_scene | 2 | 3 | open gap [2,3] — LB looseness suspected (door3 statically avoidable?) |
| door_openstart | 1 | 2 | open gap [1,2] — LB looseness suspected (does A truly not need the door moved?) |
| door_easy / door_dfirst / door_tgt / door_tgt_wide / swing_door / door_fullgoal | — | unsolved | **geometrically wedged** (see §3.4); user decision pending: shorten blades to 2.85 or keep as infeasible negative examples |

### 3.3 PathDefragmenter findings
- **Fixed today:** off-by-one in `findNextFragment` (same-group branch looped `i < size()-1`,
  excluding the goal state, so merging with the path's trailing fragment always failed → exactly
  one redundant action on `[A,B,A,B-final]`-shaped paths; buffer_swap and blocked_goal both went
  4→3 after the one-line fix). Comment in code explains it; 4/4 tests green.
- **Structural limitation (paper-relevant):** defrag is *reorder-only*. It cannot reposition a
  buffer pose or split runs, so badly-structured paths (two_buffer's 8-action runs, three_alcoves)
  stay suboptimal. Closing such gaps needs buffer *re-planning* (§6 T2/T5).

### 3.4 Door-wedge geometry pitfall (affects scene authoring!)
A door blade of length L hinged at a gap of height L cannot open: the blade-tip **corner** sits at
radius √(L²+(w/2)²) > L and arcs into the wall stub before clearing (for L=3.0, w=0.3: max reach
3.0037, collision sliver θ∈(≈1.47, 1.5708)). Symptom: planner start-tree starves (RRTConnect: 9
start-tree states in 5 s). **Rule: blade length ≤ gap − 0.15** (suite uses 2.85 in a 3.0 gap).
Also: sparse angle sampling misses the sliver — validate door sweeps densely.

### 3.5 Isolation vs single-factor extension (the bandit motivation, measured)
`growTree` has two modes; 5 seeds each, 30 s, action counts:
| scene | default (isolation on) | `LARRT_NO_ISO=1` |
|---|---|---|
| two_buffer | 9 7 7 8 8 | 9 7 **5 5** 6 |
| three_alcoves | 6 8 10 9 **4** | 7 9 7 6 7 |
| my_scene | 4 4 3 3 3 | 3 5 3 4 3 |
| door_openstart | 3 2 2 3 3 | **2 2 2 2 2** |
**Neither arm dominates** — the best extension strategy is instance-(and phase-)dependent. This is
the empirical justification for bandit-based arm selection (T2). Also: UB variance is large ⇒
always best-of-N; restarts alone close some brackets.

### 3.6 RETRACTION (do not re-introduce this error)
An early note claimed `three_alcoves` was "certified optimal at 1 action" and that this proved the
swept graph unsound (LB 4 vs 1). Wrong: the 1-action file was stale/invalid and was never
re-validated. Hand-analysis: with B parked at (3,1.5), the band x∈(2.2,2.4) is impassable for A at
every height (clearing B's top needs center-y ≥ 2.3; clearing the shelf `[0.5,2]×[2.5,3]` needs
center-y ≤ 2.1 while center-x < 2.4; no under-pass: floor gap 0.6 < box 0.8). LB 4 stands.

### 3.7 Known planner/space bugs still open
- **SO(2) wrap inconsistency** (`LARRT.cpp` ~354–361): `growTree` computes factor distance and
  interpolates with UNWRAPPED angle differences, while `FactoredStateSpace::dimDistance` wraps.
  For full-circle joints (bounds spanning 2π, e.g. swing_door) extensions can go "the long way".
  Benign for bounded joints < 2π (all certified scenes). Fix together with T3 normalization.
- **Metric heterogeneity**: the range cap `maxDistance_` treats 1 rad = 1 m (see briefing 08; T3).

### 3.8 Literature map (each briefing is self-contained, with verification summaries)
- `01` monotone vs non-monotone: definitions, dependency-graph recipe (§5 = what monotonicity.py implements), complexity, Krontiris–Bekris / Stilman–Kuffner.
- `02` optimality + sampling: why RRT+shortcut has no guarantee; Han&Yu TRLB; LGP; GCS overview.
- `03` complexity scaling: PSPACE-hardness, **MRB** (min running buffers, Gao&Yu) — two_buffer is our MRB=2 instance; difficulty predictors; phase-transition experiment design.
- `04` URDF→2D converter design (not yet built).
- `05` GCS deep dive: WAFR 2026 shortest-walks paper (Morozov/Marcucci et al., arXiv:2507.10878) is NP-hard + greedy ⇒ **no certificate from GCS for rearrangement**; use explicit GCS only as a small-n oracle; the LB=UB bracket idea appears novel.
- `06` constraint sampling: MBTS keyframe bound (`Pseq`); projection/atlas/reparam taxonomy; quotient-space guides (delete-other-objects is admissible); **Design 1 = swept-corridor-complement buffer sampler** (build first); Diffusion-CCSP only if needed.
- `07` bandits: niche "bandit over factors of a factored C-space" is **open**. Anchors: Hsu 2005 adaptive hybrid sampling; Faroni & Berenson RA-L 2023; the user's own MAB-RRT (Bayraktar, Orthey, Toussaint — WAFR 2026, sliding-window bandit for sampler selection). Non-stationary theory: SW-UCB/D-UCB (Garivier–Moulines), Rexp3, EXP3.S. **Recommended design in §6 T2.**
- `08` factor spaces: no canonical SE(n) metric (Park 1995); RRT's Voronoi bias = its metric (Lindemann–LaValle); **workspace-displacement normalization** is the principled exchange rate: door step = |Δθ|·(blade reach), box = ‖Δt‖, SE(2) box = ‖Δt‖ + ρ·|Δθ| with ρ = circumradius (Schwarzer–Saha–Latombe certified bound). Composite-planner lesson (dRRT/M*): only NN-lookup and step-cap need the cross-factor exchange rate.
- `09` embodied contact manipulation (**future paper, idea only**): drop the disembodied "objects move
  themselves" abstraction — a mobile cube + a simple 2–3R stick arm must reach, contact, and push
  passive objects (doors don't open themselves). Core new difficulty: object-level feasibility ⇏
  embodied feasibility (an object's own path may be clear yet the arm can't co-move to keep contact
  during transport). Reframes factors as robot+contacted-object subsystems on a contact manifold;
  asks whether a sound reachability-aware LB/certificate survives embodiment (Q-E1..Q-E5). Separate
  problem statement, documented so the certified-rearrangement paper cites it as future work.
  Related-work citations in `09` need re-verification (delegated sweep failed on a session limit).

---

## 4. Document index
`README.md` (index) · `AGENDA.md` (Q1–Q5, the "why") · `SCENARIOS.md` (scene catalogue + results,
tool docs) · briefings `01`–`08` · this `HANDOFF.md`. Code docs: `demos/larrt2d/README.md`,
`PUZZLE_PIPELINE_SPEC.md`, `TWO_LEVEL_DESIGN.md`, `GOAL_REGION_AND_COMPLETENESS.md`,
`BASELINE_PLANNERS.md`; repo-root `LARRT_HANDOFF.md` (older, planner-focused handoff).

---

## 5. The paper

**Working claim.** *Certified action-optimal non-monotone rearrangement in geometrically
constrained continuous 2D scenes*: (i) a sound, milliseconds-to-seconds structural lower bound on
action count (dependency graph with endpoint + disconnection arcs; min-cost FVS; handles
articulated 1-DOF objects), (ii) an adaptive factored planner (LA-RRT + bandit-driven
extension/sampler selection + defrag) as the upper bound, (iii) the **bracket**: LB=UB ⇒
certificate; else a bounded optimality gap. Positioning: Han&Yu/TRLB = tabletop with free buffer
space, no articulation, no certificates for sampling planners; GCS line = no discrete-count
certificates (05); our niche = scarce buffers + articulated blockers + certification.

**Contributions list (draft):** 1. sound LB with disconnection arcs + min-cost FVS cost model
(+1/+2), incl. articulated 1-DOF soundness and the shrunk-OBB grid proof technique; 2. the
certification bracket methodology for stochastic planners (best-of-N + validation protocol);
3. bandit-driven factored extension (SW-UCB over factors × strategies with dependency-graph
rewards) — motivated by §3.5's "neither arm dominates" data; 4. benchmark: parameterized families
with derivable optima (below) + certified instances; 5. empirical: certification rates, gap
distributions, ablations (isolation on/off/bandit; normalization on/off; reward variants).

**Theory still to write (deferred by user to the end):** formal soundness proofs of (a) endpoint
arcs, (b) disconnection arcs incl. the shrunk-grid over-approximation lemma, (c) 1-DOF interval
lemma with the <2π caveat, (d) the LB formula (movers + forced + FVS cost) — all straightforward
from the arguments recorded in §3.1; write them last.

**Venues:** WAFR / ICRA / IROS / RA-L. Steps T1+T3 alone ≈ workshop paper; with T2 a full paper.

---

## 6. Continuation tasks (pick up any; each is self-contained)

### T1 — Scene generators + benchmark  ⟵ *most parallelizable; heavy sonnet-4-6 delegation*
**STATUS (2026-07-04): core built.** `tools/gen_scenes.py` emits all four families;
`tools/bench_certify.py` runs the §2.4 protocol + Q5 bracket + the step-3 cross-check into
`out/benchmark_certify.json`. The step-3 check already paid off: comparing the generator's
*proven* optimum to the sound LB exposed and fixed an **LB looseness bug** (chains of start=goal
blockers were under-counted; see §3.1a) — `door_chain_d` and `blocked_goal_chain_k` now certify at
2d+1 / 2k+1. First sweep (18 structured scenes, larrt best-of-10×8s): **9/18 certified, 0
mismatches** (`out/benchmark_certify.json`; details + the two non-cert causes in `SCENARIOS.md`).

**T1 OPEN ITEMS (2026-07-05) — prioritized, each self-contained:**
1. **CANONICAL swap/chain brackets RESOLVED (2026-07-06) by `tools/oracle.py` (T5b).** The oracle
   (A* over the discretized mode graph, validated vs every known optimum) certifies **`swap_3_1` → 5,
   `swap_4_3` → 7, `blocked_goal_chain_4` → 9 — all = the sound LB.** So on the **canonical family
   (`m=k−1`) + chains** the sound LB is TIGHT (swap `2k−1` k=2,3,4; door `2d+1` d=1..4; blocked `2k+1`
   k=1..4), and every planner non-certification there is **planner scaling/suboptimality**, not LB
   looseness. Note `swap_3_1`'s optimum-5 is a *knife-edge stacking* plan (reproduce with `oracle
   --no-prune --cand-step 0.7`; the lean default gives 7). Remaining: settle the sub-MRB swaps (item 2).
2. **MRB-aware lower bound (T4) — IMPLEMENTED + GENERALIZED (2026-07-06), sound.** `monotonicity.py
   --mrb` (and `bench_certify.py --mrb`) apply `mrb_refine()`, now backed by **`park_once_check.py`**
   which decides park-once realizability GENERALLY: a BFS over per-object run-progress states (state =
   how many of each object's base runs are done; each transition = one templated move, reachability-
   checked with the others frozen). This drops the old single-crosser restriction — **any number of
   crossers, any interleaving** — and handles **1-DOF doors/sliders** (open=buffer, close=return), so
   `door_chain` is now evaluated (was skipped). Stacking order needs no special code (a bottom box
   trapped under a top box is simply unreachable upward). Buffer poses = structural niche slots
   (per-niche free-x-interval centres × box-height levels) / open-angle samples. **Soundness guard:**
   `has_structural_buffers` — applies only to corridor/niche/articulated scenes (finite complete
   buffer-slot set), NOT open rooms (`random_room` → skipped), so 'no plan found' is never a false
   negative. If no park-once plan of length `base_lb` exists ⇒ optimum ≥ base_lb+1 ⇒ **sound +1 bump**.
   Verified across the whole suite: bumps ONLY genuinely-loose scenes (`swap_4_2`/`swap_4_1`/`swap_5_1`
   7/7/9→8/8/10, cross-validated by `swap_optimum_check`; `swap_3_1_shallow`) and known-INFEASIBLE
   scenes (wedged doors, LB≤∞ so vacuously sound); every tight-feasible scene UNCHANGED incl. all doors;
   off by default. **Remaining (T4.x):** (a) the bump is +1 only (not necessarily tight — `swap_4_2`
   optimum ∈[8,12]); iterate the search to pin it; (b) the wedged-door bumps are *infeasibility*, not
   scarcity — nicer to detect+label infeasible scenes separately; (c) `park_once_check` is O(minutes)
   on infeasible/loose scenes (exhaustive) — fast on feasible ones.
3. **Close the k≥4 planner-scaling gaps.** `blocked_goal_chain_4` (LB 9) and `swap_{4,5}_*`
   unsolved within budget; 20 s retry did **not** help → needs better search, not more time. These
   are the natural **"does the T2 bandit help?"** evaluation targets — re-run `bench_certify` after
   T2/T3 land and compare certification rate at equal budget.
4. **`random_room` ground truth — oracle now built (T5b done).** `tools/oracle.py` exists and is
   validated, so `random_room` can now be given ground-truth optima. Remaining: (a) run the
   `random_room` family through the oracle + `bench_certify`; (b) wire the generator's `--rho`
   (density) arg, currently **accepted but not implemented** (fixed 8×8 room), so the phase-transition
   sweep (`03` §4–5) is meaningful.
5. **Scale to ~500 instances + baselines (delegate).** Current sweep is 18 structured scenes,
   `larrt` only. Grow to the ~500-instance target (blocked by items 3/4) and add baseline planners
   (`bench_certify --planner`; `rrtconnect|bitstar|aitstar|...`) for the paper's comparison table.
   Heavy batch = **delegate to a sonnet-4-6 subagent**; keep mismatch analysis local.
6. **MRB(k) phase-boundary map.** Sweeping `swap_k_m` over m was meant to read off MRB(k) and the
   cost-vs-niches curve; only k=3 is solved so far (swap_3_1=7 @1 niche, swap_3_2=5 @2) → the map
   is empty for k≥4. Unblocks once item 3 does.
7. ~~**Harness nit: preserve best-of-N solutions.**~~ **DONE (2026-07-05):** `bench_certify` now
   copies the best VALIDATED run to `out/<scene>_best.json` the moment it is produced (before the next
   run overwrites `out/<scene>.json`) and records the path in `benchmark_certify.json`.

Goal: parameterized families with known/derivable optima; ~500 seeded instances.
| family | parameters | ground truth |
|---|---|---|
| `swap_k_m` corridor | k boxes single-file, m niches | closed-form optimum & MRB=f(k,m); generalizes buffer_swap (k=2,m=1) and two_buffer (k=3,m=2) |
| `door_chain_d` | d doors, each `goal_angle==start_angle` | optimum 2d+1 (open, cross, re-close each) |
| `blocked_goal_chain_k` | k parked objects (start=goal) on A's only path | optimum 2k+1 |
| `random_room` | density ρ, n objects, seed | no closed form → exact oracle for n≤4 (T5b) |
Steps: (1) write `tools/gen_scenes.py` emitting schema-§2.3 JSON (respect §3.4 door rule and the
0.7-box / corridor-height conventions of existing scenes); (2) per instance run the §2.4 protocol
+ record `{scene, seed, LB, UB_bestofN, certified?, times}` into `out/benchmark_v2.json`;
(3) sanity: generator's claimed optimum must equal the sound LB whenever both exist — investigate
every mismatch (either generator math or LB bug). Delegate to sonnet-4-6: JSON authoring, batch
runs, result tables/plots. Keep: family design, optimum derivations, mismatch analysis.

### T2 — Sliding-window UCB bandit layer in LA-RRT  ⟵ *the core algorithmic contribution*
Design (from briefing 07 + §3.5 data): two-level SW-UCB at the `growTree` decision points.
- **Arms level 1 (extension strategy):** {isolation, single-factor}; **level 2 (which factor):**
  restricted to `diff` (factors differing from the sample); optionally level 2b (which sampler:
  uniform / goal-biased / buffer-biased) once 06-Design-1 exists.
- **Reward ∈ [0,1] = max of:** (i) *dependency-arc removal* — recompute cheaply whether a blocking
  arc of the scene's sound/swept graph stopped holding at the new node (door cleared A's corridor,
  B left A's goal footprint); (ii) *per-factor cell novelty* — KPIECE-style projection grid per
  factor, reward first visits; (iii) *gated goal progress* — goal-distance progress ONLY for
  factors whose incoming blocking arcs are already cleared (this is the fix for the
  open-the-wrong-door / dead-end problem of naive goal-distance rewards).
- **Non-stationarity:** window 100–300 (door events = breakpoints; SW-UCB is the theory match).
- **Completeness:** ε-floor ≥ 0.1 uniform arm play; PC preserved by the same mixture argument as
  the existing `goalBias_` (see `GOAL_REGION_AND_COMPLETENESS.md`).
- **Hooks:** factor pick `LARRT.cpp:~352`; isolation gate `~317`; pass scene arcs in via the
  driver (it already parses the scene; expose them through a setter, or recompute overlaps in C++
  — SAT code exists in the driver).
- **Evaluate on T1 benchmark:** certification rate & mean gap vs {default, NO_ISO, uniform-random
  arm} at equal time budgets. Delegate to sonnet-4-6: batch experiments + plots. Keep: reward
  implementation and interpretation.

### T3 — Workspace-displacement normalization (+ SO(2) wrap fix)  ⟵ *small, do before/with T2*
Per briefing 08: replace the raw L1 in `growTree`'s range cap and (ideally) the NN distance with
per-dim weights so a step means "workspace displacement": door/revolute dim weight = blade
reach/anchor-max-radius; slider = 1; box translation = 1; rotating-box θ = circumradius. Fix the
unwrapped-angle bug (§3.7) in the same patch (use wrapped diffs consistent with `dimDistance`).
Ablation: certification rate / success time with vs without, especially on door scenes.

### T4 — Lower-bound strengthening
(a) 3-DOF grid (x,y,θ-slices) for rotating-box movers (currently skipped). (b) Investigate the
suspected-loose brackets: my_scene [2,3] (is a 2-action plan real? if yes find it, if no find the
missing sound arc), door_openstart [1,2] (ditto). (c) Higher-order arcs: pairs of blockers that
jointly disconnect where neither does alone (hypergraph arcs) — would tighten three_alcoves-like
scenes. (d) Optional: replace grid proof with exact interval arithmetic for the paper's rigor.

### T5 — Close remaining planner gaps
(a) three_alcoves [4,5]: reproduce a 4-action run (it happened once, default mode; run best-of-50,
validate, preserve). (b) Exact small-n oracle: A* over the discrete mode graph (object placements
at keyframes; cf. 06's `Pseq` and 05's region abstraction) for n≤4 — gives ground truth where LB
is loose and grades the planner when brackets don't close. (c) If T2's bandit still leaves gaps:
implement 06-Design-1 (swept-corridor-complement buffer sampler) as a proposal distribution.

### T6 — Paper writing (LAST, after T1–T3 results exist)
Skeleton: Intro (certification gap in rearrangement) → Related work (from briefings 01–03, 05, 07,
08 — citations are pre-verified but re-verify anything you add) → Problem def + move model →
Sound LB (§3.1 + proofs from §5) → Planner (LA-RRT + defrag + T2 bandit + T3 normalization) →
Benchmark (T1) → Results (certification rate, gaps, ablations, §3.5 table) → Limitations
(pick-place model, no arm, grid resolution, swept-predictor unsoundness). Delegate to sonnet-4-6:
citation formatting, figure generation, LaTeX mechanics. Keep: claims, proofs, analysis prose.

---

## 7. Open user decisions (ask, don't assume)
1. Wedged legacy door scenes (`door_easy`, `door_dfirst`, `door_tgt`, `door_tgt_wide`,
   `swing_door`, `door_fullgoal`): shorten blades to 2.85 or keep as infeasible negative examples?
2. Paper emphasis: certification framework (theory-led) vs bandit planner (systems-led).
3. Nothing is committed — ask before committing/pushing.
