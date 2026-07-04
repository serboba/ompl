# LA-RRT — Handoff / Continuation Brief

This document is a self-contained brief for an agent picking up the **LA-RRT** work. Read it fully before touching code. It covers what LA-RRT is, where everything lives, what is already done, the **known open bugs** (the priority work), and how to build/run/visualize.

---

## 1. What LA-RRT is

**LA-RRT (LARRT)** is the user's (Servet Bora Bayraktar) own motion planner for **solving rearrangement puzzles using FACTORED ("factored") state spaces**. It is part of an IEEE RA-L research line (related to the user's RA-L 2023 paper *"Solving Rearrangement Puzzles using Path Defragmentation"*). It is OMPL-based.

Core idea: the configuration vector is partitioned into independent **groups** (one group = one movable object / DOF set). LA-RRT is a **bidirectional RRT** (start tree + goal tree) that optimizes the **number of actions** — i.e. how many distinct groups change between consecutive states (mode-switches) — rather than geometric path length. A candidate motion that changes several groups at once is **isolated** into a chain of single-group steps; once the trees connect, the solution is run through a **PathDefragmenter** that reorders/merges consecutive same-group segments (collision-checked) to drive the action count down further.

This is NOT MAB-RRT (a different, newer planner in the same OMPL fork — do not confuse them).

---

## 2. Where everything lives

- **Repo:** `serboba/ompl` (the user's OMPL fork). Remote `git@github.com:serboba/ompl.git`.
- **Working tree for this task:** `/home/serboba/larrt_wt` — a git worktree on branch **`larrt`**, based off `origin/main` (current OMPL, Dec 2025). Work here. Do NOT touch the user's other checkout at `/home/serboba/transferompl_ws/src/ompl_iso` (it sits on a different branch with uncommitted build artifacts).
- **Canonical source commit (where LA-RRT was originally implemented):** `2b9a01c20817f98fe5c712ca5cd06553012f397c` — *"Added LA-RRT, demo LowActionsPlanning, FactoredStateSpace, MinimalActionsObjective, PathDefragmenter"* (May 2022). It is NOT a branch tip; fetch with `git fetch origin 2b9a01c2...` if you need to diff against the original.
- **Branch commits so far:**
  - `53340732` — Add LA-RRT planner, factored state space, path defragmenter; build on OMPL main (the port + production cleanup).
  - `d42e1602` — Add 2D rearrangement demos and visualization for LA-RRT.
- Nothing is pushed. Commit on `larrt`; do not push unless asked. Commit style: **short, human-looking messages, NO AI co-author trailers.**

### Algorithm files (under `src/ompl/`)
- `geometric/planners/rrt/LARRT.{h, src/LARRT.cpp}` — the planner (`og::LARRT(si, groups, useIsolation=true, goalIndex=0)`; `groups` is `vector<vector<int>>`).
- `base/spaces/FactoredStateSpace.{h, src/FactoredStateSpace.cpp}` — subclass of `RealVectorStateSpace`. `grouped_indices : vector<vector<int>>` partitions dimension indices into groups. `distance()` is L1; **`interpolate()` moves ONE GROUP AT A TIME** (sequential, proportional to each group's share of total distance) — this is why single-group ("isolated") moves are the natural primitive. Build via `addDimension(low, high)` per dimension; state type indexes `values[i]`.
- `base/objectives/MinimalActionsObjective.{h, src/MinimalActionsObjective.cpp}` — the objective LA-RRT optimizes: counts group-changes (actions/mode-switches), NOT path length.
- `geometric/PathDefragmenter.{h, src/PathDefragmenter.cpp}` — post-processing defrag (the heart of the method). **Has bugs — see §4.**

### Demos / viz (under `demos/`)
- `LowActionsPlanning.cpp` — the original reference demo (3 groups `{{0},{1},{2}}`, 3-D, obstacle slabs, start `(0,0,0)` → goal `(2,1,-1)`). Good template.
- `larrt2d/` — the new 2D explainer demos (added in `d42e1602`):
  - `demo_LARRT2D_configspace` — two 1-DOF objects, `groups={{0},{1})`, `[0,10]²`; objects must stay apart except in a passing zone; start `(1,5)` → goal `(5,1)`. Produces a 5-action axis-aligned weave. The 2D **config space** is directly plottable: each axis = one object's position; every segment is axis-aligned; "minimal actions" reads as "fewest direction changes."
  - `demo_LARRT2D_workspace` — two pucks, `groups={{0,1},{2,3}}`, 2D workspace with four corner obstacles, pucks may not overlap; A along row, B along column, crossing center. Produces a 3-action plan.
  - `viz/plot_solution.py <json>` (static PNG), `viz/animate_solution.py <json>` (GIF). matplotlib + numpy. Each demo writes a JSON (groups, bounds, obstacles as axis-aligned rects {xmin,xmax,ymin,ymax}, start, goal, full solution path) into `out/`.
  - `out/` — sample artifacts: `configspace.{png,gif,json}`, `workspace.{png,gif,json}`.
  - `README.md` — how to build/run/visualize.
- Each demo is registered with a single `add_ompl_demo(demo_NAME File.cpp)` line in `demos/CMakeLists.txt`.

---

## 3. Build / run / visualize

```bash
cd /home/serboba/larrt_wt
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target ompl -j4          # library (GLOB_RECURSE picks up the new .cpp automatically)
cmake --build build --target demo_LowActionsPlanning demo_LARRT2D_configspace demo_LARRT2D_workspace -j4
# Use -j4, NOT -j (this box ~31 GB RAM, -j can OOM).

./build/bin/demo_LARRT2D_configspace            # solves, writes demos/larrt2d/out/configspace.json
python3 demos/larrt2d/viz/plot_solution.py  demos/larrt2d/out/configspace.json
python3 demos/larrt2d/viz/animate_solution.py demos/larrt2d/out/configspace.json
```

Build is CLEAN on current OMPL main; API drift from 2022 was essentially zero (LA-RRT uses only stable OMPL APIs: `copyToReals`/`copyFromReals`/`allocState`, `checkMotion`/`isValid`/`freeState`/`copyState`, `GoalSampleableRegion`, `SelfConfig`, `OptimizationObjective`).

---

## 4. KNOWN OPEN BUGS — this is the priority work

### 4a. PathDefragmenter is buggy on 2-group problems (CRITICAL)
The `PathDefragmenter` (run inside `LARRT::solve`) is the core of the method but is **broken for small 2-group factored problems**. Depending on the path's fragment structure it will:
1. **Crash** — `std::out_of_range` underflow in `skipFragments` around `PathDefragmenter.cpp:353` (the `getFragment(fragmentIDs.at(i).end_index + 1, ...)` / index arithmetic underflows).
2. **Truncate** — the `cutOffIfGoalReached` / zero-state `temp_from` logic drops the goal-reaching tail, returning a path that doesn't reach the goal.
3. **Collapse / reintroduce collisions** — merging consecutive same-group segments straightens intra-object detours back through obstacles, yielding a colliding "solution."

The original `LowActionsPlanning` demo only avoids this because its specific 3-object path never triggers the destructive code paths. The new 2D demos currently dodge it two ways (demo-level only, algorithm untouched): (a) a coupling constraint that blocks the merge-to-2-actions case that crashes, and (b) a C++ validate-and-replan loop that re-plans until a genuinely collision-free, goal-reaching path is returned.

**Task:** fix the PathDefragmenter so it is correct and robust for general N-group problems (especially 2-group) — no crashes, never truncates the goal, never reintroduces collisions (it already collision-checks; the bug is in the fragment index/merge logic). This is the user's research code: diagnose precisely, preserve the algorithm's intent (minimize actions via same-group segment reordering/merging), and keep changes faithful. Add regression coverage. Recommended approach: build a Debug build, reproduce each of the three failure modes with a minimal 2-group instance (the demos in `larrt2d/` are good seeds — temporarily remove their coupling constraint / replan loop to surface the crash), fix the index underflow and the goal-tail truncation, and add an assertion/collision-check that the merged path still reaches the goal and is valid before accepting it.

### 4b. Cosmetic: `bestCost_` printed with `%d`
`LARRT.cpp` logs `old path cost: -2147483648` because `bestCost_` starts at `+inf` and is printed via an integer format. Harmless to the algorithm; fix the format / initial print if doing a polish pass.

---

## 5. What is already DONE (don't redo)
- Ported LA-RRT (+ FactoredStateSpace, MinimalActionsObjective, PathDefragmenter, demo) from commit `2b9a01c2` onto current OMPL `main`; builds clean, `LowActionsPlanning` demo runs (finds a minimal-action solution).
- Production cleanup on the algorithm files: OMPL BSD license headers (authored "Servet Bora Bayraktar"), accurate doc-comments (replaced copy-pasted RRT-Connect docstring with `@anchor gLARRT`; fixed the MinimalActionsObjective doc that wrongly said "path length"), removed dead/commented debug code (incl. the commented `robowflex_dart` include), `NULL`→`nullptr`, fixed include-guard names, translated German comments, silenced unused-parameter warnings. Behavior unchanged.
- Two new 2D explainer demos + matplotlib static/animation viz + README + sample artifacts.

## 6. Suggested next steps (in order)
1. **Fix the PathDefragmenter (§4a)** — the real production blocker; defragmentation is the method's centerpiece.
2. Add a small test/benchmark (a few 2-, 3-group instances) asserting: solution reaches goal, is collision-free, and action-count is reduced by defrag vs. raw path.
3. Optionally remove the demos' validate-replan workaround once the defragmenter is fixed (so the demos exercise the real defrag path).
4. Polish (§4b), tidy `demos/CMakeLists.txt`, finalize README, ensure a clean `git diff origin/main`.

## 7. Constraints
- Work only in `/home/serboba/larrt_wt`. Commit on `larrt` with short human-style messages, no AI trailers. Do not push unless asked.
- Keep `git diff origin/main` limited to the LA-RRT files + demos (verify `ProjectionFactory` under `multilevel/` stays identical to `origin/main` — it must not be clobbered by the 2022 version).
