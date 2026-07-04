# LA-RRT 2D rearrangement puzzle pipeline — CONTRACT SPEC

This is the shared contract for a data-driven 2D pipeline for **sequential-rearrangement
puzzles** ("move n times" / lockbox style): objects block each other, so you move object A
to an intermediate spot, move B through the freed space, move A again, etc. LA-RRT minimises
the number of **actions** = the number of times control switches between objects.

Everything is purely 2D (no z). Three components are built against the two JSON schemas below
and MUST agree on the collision model. Reference implementation to imitate for the OMPL/LARRT
wiring: `demos/larrt2d/LARRT2D_Workspace.cpp`.

---

## 1. Geometry & collision model (ALL components must implement identically)

- An **object** is an *oriented rectangle* (OBB): centre `(x, y)`, half-extents `(hx, hy)`,
  orientation `theta` (radians, CCW). Its 4 corners are
  `C_k = (x,y) + R(theta) * (±hx, ±hy)`, where `R(theta)` is the 2D rotation matrix.
- An **obstacle** (static wall) is an axis-aligned rectangle `{xmin,xmax,ymin,ymax}` — treat it
  as an OBB with `theta=0`, centre `((xmin+xmax)/2,(ymin+ymax)/2)`, half-extents
  `((xmax-xmin)/2,(ymax-ymin)/2)`.
- **Overlap test = Separating Axis Theorem (SAT)** for two convex rectangles:
  - Candidate axes = the 4 edge normals (2 from each rectangle): for a rect at angle `t`,
    the normals are `(cos t, sin t)` and `(-sin t, cos t)`.
  - For each axis, project all 4 corners of both rects; the rects are **separated** on that
    axis if `maxA <= minB + EPS` or `maxB <= minA + EPS`.
  - If ANY axis separates → no collision. If none separate → collision.
  - `EPS = 1e-6`. Touching (shared edge) is NOT a collision.
- A configuration is **valid** iff: every object–obstacle pair is non-overlapping AND every
  object–object pair is non-overlapping. (World bounds are enforced by boundary-wall obstacles
  in the scene, so no separate bounds check is required, but staying in world bounds is assumed.)

## 1b. Goal semantics — targets vs. free objects (IMPORTANT)

The goal is a **region**, not a single state. Each object has a boolean `"target"`:
- **Target objects** (`"target": true`) MUST reach their `"goal"` pose. There can be one or many.
- **Non-target objects** (`"target": false`, the default) have **no goal** — they are movable
  obstacles that may end at ANY collision-free pose. Their `"goal"` is omitted / `null`.

A state is in the goal region iff every target is at its goal (others ignored). LA-RRT already
recognises a `GoalSampleableRegion`; the driver supplies one whose `sampleGoal()` pins the
target(s) at their goal and places every non-target at a random collision-free pose (rejection-
sampled). So the goal tree is seeded with many arrangements {targets fixed, others random} —
this is the intended rearrangement formulation. NOTE: because LA-RRT accepts a solution only when
the start tree connects to a sampled goal node, a free object is sometimes moved just to match the
sample's random placement (an optimality gap, not a correctness issue). The goal set is measure-zero
in the target dimensions, so closing this gap has probabilistic-completeness implications — see
[`GOAL_REGION_AND_COMPLETENESS.md`](GOAL_REGION_AND_COMPLETENESS.md) before changing goal handling.

## 2. State layout & grouping (fixed convention)

- **Every object contributes exactly 3 state dimensions: `x, y, theta`**, in object order.
  Object `i` owns dims `[3i, 3i+1, 3i+2]`. This keeps the layout uniform for all components.
- **One group per object**: `groups[i] = [3i, 3i+1, 3i+2]`. Moving any of an object's dims is
  the same action; an action ends when control switches to a different object.
- If an object does not rotate (`"rotates": false`), its theta bound is the degenerate
  `[theta0 - 1e-9, theta0 + 1e-9]` so the planner never changes it. `theta0 = start[2]`.

## 3. Scene schema — `demos/larrt2d/scenes/<name>.json` (INPUT to the driver)

```json
{
  "name": "move_once",
  "world": { "xmin": 0.0, "xmax": 10.0, "ymin": 0.0, "ymax": 10.0 },
  "obstacles": [
    { "xmin": 0.0, "xmax": 10.0, "ymin": 0.0, "ymax": 0.5 }
  ],
  "objects": [
    {
      "name": "box",
      "hx": 0.5, "hy": 0.5,
      "rotates": false,
      "start": [1.0, 1.0, 0.0],
      "goal":  [8.0, 1.0, 0.0],
      "xbounds": [0.5, 9.5],
      "ybounds": [0.5, 9.5],
      "tbounds": [-3.14159, 3.14159]
    }
  ]
}
```
- `start`/`goal` are always `[x, y, theta]`. For non-rotating objects `theta` is constant and
  equals `start[2]` (== `goal[2]`).
- `tbounds` is only used when `rotates` is true; otherwise the driver overrides it with the
  degenerate bound from `start[2]`.

## 4. Solution schema — `demos/larrt2d/out/<name>.json` (driver OUTPUT; viz + validator INPUT)

The driver echoes the scene and appends the result. This single file is everything the viz and
validator need.

```json
{
  "name": "move_once",
  "world": { "xmin":0,"xmax":10,"ymin":0,"ymax":10 },
  "obstacles": [ { "xmin":0,"xmax":10,"ymin":0,"ymax":0.5 } ],
  "objects": [
    { "name":"box","hx":0.5,"hy":0.5,"rotates":false,
      "start":[1,1,0],"goal":[8,1,0],"dims":[0,1,2] }
  ],
  "groups": [[0,1,2]],
  "planner": "larrt",
  "solved": true,
  "action_count": 3,
  "solve_time_s": 1.234,
  "path": [ [1,1,0, ...], [ ... ] ]
}
```
- `path[k]` is the FULL state vector of length `3 * num_objects` (all objects' `x,y,theta`
  concatenated in object order). `objects[i].dims` gives object i's three indices into it.
- If no solution: `"solved": false`, `"path": []`, `"action_count": -1`.

## 5. Action counting (identical in driver & validator)

```
changed_group(path[k], path[k+1]) = the group index g whose dims differ (|Δ|>1e-9); -1 if none.
action_count = number of maximal runs of the same changed_group along the path
             (i.e. count a new action every time changed_group differs from the previous one).
```
See `actionCount`/`changedGroup` in `LARRT2D_Workspace.cpp` for the exact reference logic.

## 6. Components to build

- **C++ driver** `demos/larrt2d/LARRT2D_Puzzle.cpp`, target `demo_LARRT2D_puzzle`, wired in
  `demos/CMakeLists.txt`. Usage: `demo_LARRT2D_puzzle <scene.json> [planTime=10] [planner=larrt]`.
  Reads scene, builds `FactoredStateSpace` (3 dims/object, groups per object, bounds per dim),
  SAT `StateValidityChecker`, sets `setStateValidityCheckingResolution(0.002)`, runs the planner
  (`larrt` → `og::LARRT(si, groups)`; `rrtconnect` → `og::RRTConnect`), validates the returned
  path with an independent dense SAT check, prints `solved / action_count / time`, writes the
  solution JSON. Include nlohmann json from `/home/serboba/json/single_include`.
- **Visualizer** `demos/larrt2d/viz/viz_puzzle.py <solution.json> [out.gif]`: draw obstacles as
  grey rects, each object as a coloured oriented rectangle (matplotlib polygon), start (☆) and
  goal (✕) outlines per object, animate along the path (interpolate each segment, one object
  moving at a time), title shows `action k: move <object>` and total actions. Also write a static
  keyframe strip PNG (`<name>_strip.png`) with one panel per action.
- **Validator** `demos/larrt2d/tools/validate_puzzle.py <solution.json>`: with its OWN SAT code
  (do not import the driver), check (a) start/goal of the path match the scene per object,
  (b) every waypoint is collision-free, (c) every segment is collision-free at fine resolution
  (≥200 samples), (d) recount actions and compare to `action_count`. Print a clear PASS/FAIL
  report per check; exit non-zero on any failure.
```
