# Projecting 3D URDF Manipulation Puzzles into the 2D Rearrangement Pipeline

**Design doc — no code changes proposed. Target consumers: the `demos/larrt2d` scene schema (`PUZZLE_PIPELINE_SPEC.md`), the driver `LARRT2D_Puzzle.cpp`, and the authoring model in `tools/scene_editor.html`.**

## 0. Goal and scope

We have a purely-2D sequential-rearrangement planner (LA-RRT over a `FactoredStateSpace`, one group per object, OBB+SAT collision, target/free objects, region goal). We want to take a 3D URDF manipulation puzzle — a scene of rigid links joined by revolute/prismatic/fixed joints, with a start config and a goal config — and **project it onto a plane** so the existing 2D pipeline can plan the rearrangement, count actions, and be visualized/validated unchanged.

The output of this whole exercise is a single artifact: a `scenes/<name>.json` file that already conforms to the pipeline contract in `PUZZLE_PIPELINE_SPEC.md` §3, plus a set of **flags** describing what the projection could not represent faithfully. Nothing downstream changes.

The key constraint we must respect: the 2D contract gives **every object exactly one movable "kind"** — `box` (2–3 DOF free body), `door` (1-DOF revolute, hinged rect), `revolute` (1-DOF general hinge, anchor+com), `slider` (1-DOF prismatic) — and **one group per object**. So the projection is fundamentally a *dimensionality-reduction and DOF-classification* problem: each movable URDF link must collapse to one of those four kinds or be dropped/flagged.

---

## 1. URDF elements vs. our model

### 1.1 The URDF elements we consume

| URDF element | Meaning | What we use it for |
|---|---|---|
| `<link>` with `<collision><geometry>` | rigid body geometry (box/cylinder/mesh) in the link frame | source of the OBB footprint after projection |
| `<joint type="…">` | connects parent→child link | decides the object *kind* and its 1 DOF |
| `<joint><origin xyz rpy>` | pose of the joint frame in the parent frame | locates the hinge/rail/anchor in the world after FK |
| `<joint><axis xyz>` | rotation/translation axis in the joint frame | must be checked against the projection plane normal |
| `<joint><limit lower upper>` | joint range (rad or m) | carried over verbatim to `angle_bounds`/`disp_bounds`/`tbounds` |
| start config, goal config (external, per-joint values) | the puzzle's initial and target joint vector | source of `start*`/`goal*` and `target:true/false` |

We do **not** consume `<inertial>`, `<dynamics>`, `<mimic>` semantics (except to flag them), transmissions, or Gazebo tags.

### 1.2 The mapping table (clean cases)

Let **n** be the chosen projection-plane normal (world **+z** for top-down tabletop; world **+y** for a vertical x–z "door/lock/climb" scene). Let a joint's world-frame axis (after composing all parent origins via forward kinematics at the *start* config) be **a**.

| URDF joint (world-frame condition) | Our kind | 1D DOF mapping | Notes |
|---|---|---|---|
| `revolute`, **a ∥ n** (axis pierces the plane) | `revolute` (or `door`) | joint angle `q` → `start_angle`/`goal_angle`, `angle_bounds = [lower, upper]` | This is the clean planar hinge. In-plane the link *rotates*, exactly our model. |
| `continuous`, **a ∥ n** | `revolute` | same, with `angle_bounds = [-π, π]` (mark as SO(2)) | URDF `continuous` = unbounded `revolute`; the driver already `markAngleDim`s door/revolute DOF. |
| `prismatic`, **a ⟂ n** (axis lies in the plane) | `slider` | displacement `q` → `start_disp`/`goal_disp`, `disp_bounds = [lower, upper]`; `axis = proj(a)`, `base = projected start centre` | Clean planar rail. |
| `floating`, or `planar` with plane ∥ our plane | `box` (`rotates:true`, 3 DOF) | (x,y) = projected centre, θ = in-plane yaw; `xbounds/ybounds` from world, `tbounds` from limits or `[-π,π]` | A free planar body — our box is exactly this. |
| `fixed` | fold into an **obstacle** (static wall) or into the parent's geometry | none | If its subtree never moves in either config, it is scenery, not an object. |
| the "grasped/target" link, movable by a free joint | `box` (`rotates:false`, 2 DOF) if it only translates | (x,y) only, θ pinned to start | The classic "move the cube to its goal" target. |

### 1.3 What maps cleanly, restated

- **Planar revolute about a world-vertical axis → our `revolute`/`door`.** A cabinet door, a lever, a swinging arm whose hinge axis is the table normal projects with zero loss: the hinge point, the swept rectangle, and the angle limits all survive. If the body is a rectangle hinged at one short edge (`com = (L/2, 0)`, `hx=L/2`, `hy=W/2`) the editor/driver will even round-trip it as the friendlier `door` form (see `toScene()`'s `doorForm` test).
- **Prismatic along a horizontal (in-plane) axis → our `slider`.** A drawer, a sliding bolt/latch, a translating block. `base = projected rail start`, `axis = unit(projected axis)`, `disp_bounds` copied directly.
- **A free-floating planar body → our `box`.** A puck/cube on a table that can be repositioned. 2 DOF if orientation is irrelevant to the puzzle, 3 DOF (`rotates:true`) if yaw matters.

These three are the "happy path" and cover the bulk of tabletop and 2D-lockbox puzzles.

---

## 2. The projection

### 2.1 Choosing the plane

Two canonical choices, selectable per scene:

- **Top-down x–y (normal = +z)** — the natural tabletop projection. Objects slide/rotate on a table; gravity is out-of-plane and is simply ignored (bodies rest on the table by assumption). Revolute joints whose axis is vertical (the table normal) become in-plane hinges. This is the default.
- **Vertical x–z (normal = +y)** — for the "door / lock / climb-on-boxes" style puzzle where the interesting motion is in a vertical wall plane. Here gravity is *in-plane* (−z) and becomes semantically important (support/height), which §3 addresses. A door on a vertical wall hinged about a vertical axis is **not** in-plane here (its axis ∥ z is in the plane, so it swings out of the wall) — pick the plane so that the puzzle's dominant motions are in-plane.

The chooser is a per-URDF assumption the converter must emit and the modeller must confirm; a wrong plane silently turns motion into "nothing moves."

### 2.2 Per-movable-link projection procedure

For each movable link (child of a non-fixed joint):

1. **Forward-kinematics the joint frame** at the start config: compose parent `<origin>` transforms from the root to get the joint origin **O** and the world-frame axis **a**.
2. **Classify the DOF** vs. the plane normal **n** (§1.2). If a joint's axis is neither ∥ **n** (revolute) nor ⟂ **n** (prismatic) — e.g. a hinge tilted 30° out of plane — the motion is *partly* out of plane: **flag** it, project onto the nearest clean case (use the in-plane component of the swept motion), and record the residual angle as a fidelity loss.
3. **Project the collision geometry to an OBB.** Take the link's `<collision>` geometry, express its 8 corners (box) / bounding box (cylinder/mesh) in world coords at the start config, drop the out-of-plane coordinate, and compute the **minimum-area bounding rectangle** of the resulting 2D point set. That rectangle's centre offset, half-extents (`hx`,`hy`) and in-plane orientation define the object's fixed geometry.
   - For a `revolute`/`door`, the OBB must be expressed **relative to the hinge** as `com`/`length`/`width` so the driver's `obb(v)` (which places `centre = anchor + R(a)·com`) reproduces the sweep. Concretely `com = R(−q₀)·(projected_centre − O_proj)`, matching the editor's own anchor-replacement math.
   - For a `slider`, express the OBB centre at zero displacement as `base`, and the block's fixed `angle`.
4. **Map the joint to the 2D DOF** and **carry over limits verbatim** — radians for revolute/continuous, metres for prismatic. `continuous` → `[-π,π]` marked SO(2).
5. **Derive start/goal.** Read this link's joint value from the URDF start config → `start_angle`/`start_disp`/`start`. If the link's value differs between start and goal config → it is a **target** (`target:true`) and its goal-config value → `goal_*`. If it is identical in both configs but is still movable, it is a **free / movable-obstacle** object (`target:false`) — the planner may reposition it to clear a path (exactly the rearrangement formulation in spec §1b).

### 2.3 What is lost, and how to approximate or flag it

| Lost quantity | Why | Approximation / flag |
|---|---|---|
| **Out-of-plane translation/rotation** | a 3D joint may move the link partly along **n** | Project onto in-plane component; emit `flag: "out_of_plane_motion"` with the residual magnitude. Large residual → the puzzle isn't really 2D; reject. |
| **Non-box collision geometry** (cylinders, meshes) | our collision model is OBB-only | Replace with the min-area bounding rectangle of the projected footprint. Conservative (grows the object); emit `flag: "geometry_bbox_approx"` per link. |
| **The manipulator arm itself** | the arm is a kinematic chain doing the grasping; our model has no agent, objects "teleport" along collision-free 1-object paths | Drop the arm entirely; the abstraction is "which object moves, in what order" — LA-RRT's action = object-switch is the arm's pick/place. Emit `flag: "manipulator_omitted"`. Reachability of the arm is **not** modelled (see §3). |
| **Gravity / support** (the "climb on boxes" case) | 2D collision is planar overlap; there is no notion of resting-on / height | Cannot be a collision constraint. Encode as an artificial dependency (see §3). Emit `flag: "support_constraint"`. |
| **3D stacking / z-overlap** | two links at the same (x,y) but different z don't collide in 3D but *do* overlap in the top-down projection | Either (a) separate them into distinct planar regions, or (b) if they are genuinely on different levels, drop the lower/occluded one from collision and flag `z_layer_conflict`. Top-down projection is only sound when the scene is roughly single-layer. |
| **Joint coupling** (`<mimic>`, closed loops) | our objects are independent 1-DOF groups | Not representable; pick the driving joint, flag `mimic_dropped`. |

---

## 3. What cannot be captured — and the modelling decision it forces

The pipeline's only notion of "hard" is **planar non-overlap** (SAT). Anything that is not "rectangle A must not intersect rectangle B" is outside the model. The paradigm case is:

> **"Climb on a box to reach a lock."** The lock is only *actuable* once a box has been moved under it and the agent has gained height. In 3D this is a support + reachability precondition. In our 2D world there is no height and no agent, so this is invisible to SAT — the planner would happily open the lock with nothing underneath.

This is genuinely not a collision constraint, so it must become a **precondition / artificial dependency** that we bolt onto the 2D formulation. Options, in increasing order of pipeline change:

1. **Encode the precondition as a geometric gate (no code change).** Add a static **obstacle** that physically blocks the lock's DOF (e.g. a wall across the lock's swing) and make it *removable only by* placing the "step" box into a trigger region. Practically: model the lock as a `door`/`slider` whose free range is pinned (the editor's own "pinned joint" detector, `pinSweep`, surfaces exactly this) until the step box occupies the spot beneath it — but SAT alone can't express "occupying spot X *unlocks* joint Y," so pure geometry can only *block*, not *condition*, and this option degrades to "the box must be moved out of the way," losing the support semantics. Use it only when the real constraint is actually blocking, not support.

2. **Ordering / dependency precondition carried as metadata (small, honest change).** Emit into the scene JSON a non-schema `"preconditions"` block, e.g. `{"actuate": "lock", "requires_placed": {"object": "step_box", "region": [x0,y0,x1,y1]}}`. The 2D planner ignores it, but the **converter records it and the modeller (or a thin post-check) enforces** that in the solution path the `step_box` enters `region` in an action *before* any action that changes `lock`. This keeps the support constraint explicit and testable (the validator could grow one extra check) without pretending it is geometry. This is the recommended encoding: it names the dependency as a *precondition on the action order*, which is exactly what LA-RRT is optimizing over.

3. **Full precondition-aware planning (out of scope).** Making LA-RRT natively refuse to actuate a joint until a predicate holds is a real planner extension. Flag it as future work; the projection's job is only to *emit the predicate*, not to solve it.

**Decision rule for the converter:** if a joint's actuation in the URDF puzzle depends on the *configuration of another link* (support, key-in-lock, gating), it cannot be a clean object. Emit it as an object **plus** a `preconditions` entry (option 2) and a `flag: "support_constraint"` / `"actuation_precondition"`. Do not silently produce a scene that the 2D planner will "solve" by ignoring the constraint.

---

## 4. Concrete converter plan — `urdf_to_scene2d.py`

### 4.1 Inputs / output

```
urdf_to_scene2d.py \
    --urdf scene.urdf \
    --start start.yaml \        # { joint_name: value, ... }  (rad / m)
    --goal  goal.yaml \         # subset of joints that must reach a target value
    --plane xy|xz \             # projection plane (normal +z or +y); default xy
    --world auto|xmin,xmax,ymin,ymax \
    --out scenes/<name>.json
```

Output: a `scenes/<name>.json` conforming to `PUZZLE_PIPELINE_SPEC.md` §3, plus a sibling `<name>.flags.json` listing every fidelity flag and every `preconditions` entry.

### 4.2 Libraries

- **`yourdfpy`** (preferred) or **`urdfpy`** for parsing + forward kinematics. `yourdfpy` is actively maintained, gives `robot.link_map`, `robot.joint_map`, and `scene`/FK via `trimesh`; `urdfpy` gives `robot.link_fk(cfg=…)` returning a 4×4 per link which is exactly what we need for step 1/3. Fall back to raw `xml.etree.ElementTree` only if neither is available (then we implement FK by composing `<origin>` matrices ourselves).
- **`numpy`** for the transforms, projection, min-area rectangle (rotating-calipers on the 2D convex hull, via `scipy.spatial.ConvexHull` if available, else a coarse angle sweep).
- **`trimesh`** (comes with `yourdfpy`) to get collision-geometry vertices for mesh/cylinder links before projecting.

### 4.3 Per-joint mapping (the decision function)

```
classify(joint, axis_world, n):
    if joint.type == "fixed":                      -> ("obstacle" or fold into parent)
    elif joint.type in ("revolute","continuous"):
        if parallel(axis_world, n):                -> "revolute"/"door"
        else:                                       -> flag out_of_plane, best-effort "revolute"
    elif joint.type == "prismatic":
        if perpendicular(axis_world, n):           -> "slider"
        else:                                       -> flag out_of_plane, best-effort "slider"
    elif joint.type in ("floating","planar"):      -> "box" (rotates=True)
    else:                                            -> flag unsupported, skip
```

`door` vs. `revolute`: emit the `door` form iff the projected OBB is a rectangle hinged at one short edge (mirror `scene_editor.html`'s `doorForm` test: `|com.y|≈0 && |com.x − hx|≈0`), else the general `revolute` form.

### 4.4 Skeleton

```python
#!/usr/bin/env python3
"""urdf_to_scene2d.py — project a 3D URDF manipulation puzzle onto a plane
   and emit a demos/larrt2d scene JSON. DESIGN SKELETON — not a full impl."""
import argparse, json, numpy as np
# import yourdfpy   (or urdfpy)

PLANE_NORMAL = {"xy": np.array([0,0,1.]), "xz": np.array([0,1,0.])}
PARALLEL_TOL = 0.05          # rad between axis and normal to count as "in/out of plane"

def load(urdf_path): ...                      # -> robot (yourdfpy.URDF)
def fk_joint_frame(robot, joint, cfg): ...    # -> (O_world 3x1, axis_world 3x1)
def projected_footprint(robot, link, cfg, plane):
    """world-space collision vertices -> 2D points on the plane."""
    ...
def min_area_rect(pts2d):                     # -> (cx,cy, hx,hy, theta)
    ...                                       # rotating calipers on convex hull

def to_2d(p3, plane):                         # drop out-of-plane coord
    return (p3[0], p3[1]) if plane=="xy" else (p3[0], p3[2])

def classify(joint, axis_w, n):               # -> kind + flags   (§4.3)
    ...

def build_object(robot, joint, start_cfg, goal_cfg, plane, n, flags, preconds):
    kind, jflags = classify(joint, axis_world(...), n)
    O2 = to_2d(O_world, plane)
    cx,cy,hx,hy,th = min_area_rect(projected_footprint(...))
    q0 = start_cfg[joint.name]
    is_target = (joint.name in goal_cfg) and \
                abs(goal_cfg[joint.name]-q0) > 1e-6
    if kind == "box":
        obj = {"name":joint.child, "type":"box", "hx":hx,"hy":hy,
               "rotates": bool(free_yaw), "target":is_target,
               "start":[cx,cy,th],
               "xbounds":world_x, "ybounds":world_y,
               "tbounds":[lo,hi] if free_yaw else [th,th]}
        if is_target: obj["goal"] = [gx,gy,gth]
    elif kind in ("door","revolute"):
        com = rot(-q0) @ (np.array([cx,cy]) - np.array(O2))   # anchor-relative
        obj = {"name":..., "type":kind,
               "hinge"/"anchor":[O2[0],O2[1]],
               ("length","width") or ("com","hx","hy"):...,
               "start_angle":q0, "angle_bounds":[joint.limit.lower, joint.limit.upper]}
        if is_target: obj["goal_angle"] = goal_cfg[joint.name]
    elif kind == "slider":
        axis2 = unit(to_2d(axis_world, plane))
        obj = {"name":..., "type":"slider",
               "base":[cx0,cy0], "axis":[axis2[0],axis2[1]],
               "hx":hx,"hy":hy,"angle":th,
               "start_disp":q0, "disp_bounds":[joint.limit.lower, joint.limit.upper]}
        if is_target: obj["goal_disp"] = goal_cfg[joint.name]
    flags += jflags
    # support/precondition detection is a separate pass over the puzzle spec
    return obj

def main():
    args = parse_args()
    robot = load(args.urdf)
    n = PLANE_NORMAL[args.plane]
    obstacles, objects, flags, preconds = [], [], [], []
    for joint in robot.actuated_joints:           # skip fixed unless scenery
        if joint.type == "fixed":
            obstacles += fixed_link_to_obstacle(robot, joint, start_cfg, args.plane)
            continue
        objects.append(build_object(robot, joint, start_cfg, goal_cfg,
                                     args.plane, n, flags, preconds))
    scene = {"name":args.name,
             "world":world_bounds(robot, args),
             "obstacles":obstacles, "objects":objects}
    json.dump(scene, open(args.out,"w"), indent=2)
    json.dump({"flags":flags, "preconditions":preconds},
              open(args.out.replace(".json",".flags.json"),"w"), indent=2)
    print(f"wrote {args.out}: {len(objects)} objects, "
          f"{len(flags)} flags, {len(preconds)} preconditions")
```

### 4.5 Assumptions & flags the converter must emit

Emit **assumptions** (things it *decided*, that the modeller must confirm):

- `plane`, and which world axis it dropped.
- `world` bounds (auto = projected AABB of all static geometry + margin, mirroring the editor's border walls).
- For each `box`: whether yaw was treated as free (`rotates`) or pinned.
- Any `fixed` link folded into an obstacle vs. into its parent's geometry.
- Unit conventions carried through (rad for revolute/continuous, m for prismatic).

Emit **fidelity flags** (things it *could not* represent faithfully), one per affected link, each with a magnitude where meaningful:

- `out_of_plane_motion` (residual angle/length lost to projection).
- `geometry_bbox_approx` (non-box collision replaced by bounding rect; report area inflation).
- `z_layer_conflict` (projected footprints overlap but are on different z levels).
- `manipulator_omitted` (the arm chain dropped; reachability not modelled).
- `support_constraint` / `actuation_precondition` (paired with a `preconditions` entry per §3 option 2).
- `mimic_dropped`, `unsupported_joint_type` (planar/floating collapsed, or joint skipped).

### 4.6 Round-trip sanity

Because the driver, validator, and `scene_editor.html` already agree on the exact OBB/anchor/com math (`obb(v)` in `LARRT2D_Puzzle.cpp`, `bodyOBB` in the editor), the converter only has to emit numbers in those conventions. A cheap self-check: load the emitted scene in `scene_editor.html`, run its "Check scene (SAT + pinned joints)" — a `pinned` warning on a joint that should move is a red flag that the projection plane or a limit was wrong, before ever invoking the planner.

---

## 5. Summary of the mapping decisions

- **Four clean cases:** vertical-axis revolute→`revolute`/`door`; horizontal-axis prismatic→`slider`; free planar body→`box`; fixed link→obstacle. Limits and start/goal configs carry over verbatim; a joint that differs between start and goal config becomes a `target`, otherwise a free movable-obstacle.
- **Geometry** collapses to the min-area bounding rectangle of the projected collision footprint (OBB, conservative).
- **The arm, gravity, out-of-plane motion, and non-box shape** are the four systematic losses; the first three are dropped-and-flagged, the last is bounded-and-flagged.
- **Support / reachability preconditions** (the "climb on a box" family) are not geometry and must be emitted as an explicit `preconditions` ordering constraint (option 2) rather than faked with SAT.
- **The converter is the only new artifact**; it produces a spec-conformant `scenes/*.json` plus a `*.flags.json`, and changes nothing downstream in the pipeline.

---

*Source: research agent briefing, 2026-07-04. The "support precondition as ordering metadata" (§3 option 2) is the recommended path for the door/lock/climb family and connects directly to the monotone/non-monotone dependency-graph work in [`01_monotone_vs_nonmonotone.md`](01_monotone_vs_nonmonotone.md).*
