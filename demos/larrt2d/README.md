# LA-RRT 2D rearrangement demos

## Data-driven puzzle pipeline (move-n-times / sequential unlock)

A general 2D pipeline for sequential-rearrangement puzzles where objects block each other
(move A aside → move B through → move A again). Objects are **oriented rectangles** with SAT
collision; LA-RRT minimises the number of **actions** (object switches), one group per object.
Full contract: [`PUZZLE_PIPELINE_SPEC.md`](PUZZLE_PIPELINE_SPEC.md).

```sh
# 1. build the driver (OMPL lib already built)
cmake --build build --target demo_LARRT2D_puzzle -j4

# 2. solve a scene  ->  writes demos/larrt2d/out/<name>.json
./build/demos/demo_LARRT2D_puzzle demos/larrt2d/scenes/corridor_swap.json 10 larrt

# 3. validate independently (own SAT collision code; PASS/FAIL, exit code)
python3 demos/larrt2d/tools/validate_puzzle.py demos/larrt2d/out/corridor_swap.json

# 4. visualise  ->  <name>.gif + <name>_strip.png (one panel per action)
python3 demos/larrt2d/viz/viz_puzzle.py demos/larrt2d/out/corridor_swap.json

# 5. benchmark all scenes x planners (LA-RRT is randomized -> multiple runs)
python3 demos/larrt2d/tools/benchmark.py --runs 5 --time 10
```

- **Scenes** live in `scenes/*.json` (walls + movable rectangles + per-object start/goal/DOF).
  Add a scene = drop in a JSON file; no code changes.
- **Planners**: `larrt` (action-minimizing, single object per action) or `rrtconnect` (baseline).
  RRTConnect solves fast but moves several objects at once, so its plans **fail validation** —
  a good illustration of why the factored/action structure matters.
- Everything is pure 2D (no z), so validation and visualisation are trivial to reason about.

---

# LA-RRT 2D rearrangement demos (original explainers)

Small, runnable 2-D demos of **LA-RRT (LARRT)**, a bidirectional RRT over a
*factored* (factored) state space. The state is partitioned into **groups**,
one per movable object, and the `FactoredStateSpace` moves **one group at a
time**. LA-RRT does not minimise geometric length -- it minimises the number of
**actions**, i.e. the number of times control switches between objects
(mode-switches). After a path is found, the `PathDefragmenter` reorders / merges
same-object segments to drive the action count down.

Each demo solves a problem, prints the final action count, and writes a JSON
file describing the groups, bounds, obstacles, start, goal and full solution
path. The Python scripts in `viz/` turn that JSON into a static figure and an
animated GIF.

## The two demos

### Demo A -- configuration-space explainer (`demo_LARRT2D_configspace`)
Two 1-DOF objects, `groups = {{0}, {1}}`. The full state is the 2-D
**configuration space** (x = object 0's position, y = object 1's position).
Because one object moves per action, every path segment is **axis-aligned**, so
"minimal actions" reads visually as "fewest axis-aligned segments". The two
objects share a track and may not come within a gap of each other (the forbidden
diagonal band `|x - y| < gap` while `x + y < sum_limit`), except in a passing
zone at the high end. After defragmentation, re-ordering the two objects is a
**4-action** weave up through the passing zone and back.
Output: `out/configspace.json`.

### Demo B -- workspace rearrangement explainer (`demo_LARRT2D_workspace`)
Two pucks in a shared 2-D workspace, `groups = {{0,1}, {2,3}}` (puck A =
dims 0,1; puck B = dims 2,3). Each action moves one puck while the other stays
put. Four corner obstacles leave a cross-shaped free space; puck A is relocated
along a row and puck B along a column, crossing at the centre. The pucks may not
overlap, but since only one moves per action they are never at the centre at the
same time, so after defragmentation the plan is the **2-action** optimum (move
puck B up the column, then puck A across the row). Output: `out/workspace.json`.

> **Reproducibility.** Each demo fixes the RNG seed and solves once; the returned
> path is sanity-checked in C++ (exact start/goal, dense collision check) before
> being written to JSON. No re-planning loop is needed -- the `PathDefragmenter`
> returns a valid, minimal-action path directly.

## Build

The OMPL library is already built on the `larrt` branch. From the repo root:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target demo_LARRT2D_configspace demo_LARRT2D_workspace -j4
```

## Run the demos

Run from the **repository root** (the demos write JSON to `demos/larrt2d/out/`):

```sh
./build/demos/demo_LARRT2D_configspace
./build/demos/demo_LARRT2D_workspace
```

Each prints the final action count and writes its JSON file.

## Visualise

Dependencies: `matplotlib` and `numpy` (see `viz/requirements.txt`):

```sh
pip install --user -r demos/larrt2d/viz/requirements.txt   # if needed
```

Static figure (PNG next to the JSON) and animation (GIF next to the JSON):

```sh
python3 demos/larrt2d/viz/plot_solution.py    demos/larrt2d/out/configspace.json
python3 demos/larrt2d/viz/animate_solution.py demos/larrt2d/out/configspace.json
python3 demos/larrt2d/viz/plot_solution.py    demos/larrt2d/out/workspace.json
python3 demos/larrt2d/viz/animate_solution.py demos/larrt2d/out/workspace.json
```

- `plot_solution.py` — config space: shaded obstacles / forbidden band, start &
  goal, the path drawn as connected axis-aligned segments each labelled by which
  object moves, with the total action count annotated. Workspace: both pucks'
  trajectories in distinct colours with their start/goal markers.
- `animate_solution.py` — steps through the path (one object moving at a time)
  and saves a GIF.

## Sample outputs

Committed under `out/`:

| | static | animation |
|---|---|---|
| Demo A | `out/configspace.png` | `out/configspace.gif` |
| Demo B | `out/workspace.png` | `out/workspace.gif` |

## What this demonstrates

LA-RRT solves **rearrangement** problems over a **factored state space**:
the configuration is factored into per-object groups, motions traverse one group
at a time, and the planner minimises the **number of actions** (object
switches), not distance. The `PathDefragmenter` post-processes the raw RRT
solution to reduce that action count. In Demo A this is literally the number of
axis-aligned segments in configuration space; in Demo B it is the number of
times control hands off between the two pucks.
