# 09b — Implementation blueprint: minimal embodied contact-manipulation prototype

**Companion to `09_embodied_contact_manipulation.md` (the idea).** This is a *design/blueprint only*
(no code changes) for the §8 minimal experiment. Produced by a delegated Sonnet sub-agent that read
the actual codebase, then **chair-reviewed**: every load-bearing `file:line` reference below was
independently verified to exist and say what is claimed —
`LARRT::getChangedIndex` (`src/ompl/geometric/planners/rrt/src/LARRT.cpp:235`, single-`int` return),
`Motion::index_changed` (`LARRT.h:183`), `FactoredStateSpace::interpolate` sequential-per-group
(`src/ompl/base/spaces/src/FactoredStateSpace.cpp:85`, `findIndex`/`getDistances` 30/44), the
isolation branch `diff.size() > 1 && useIsolation_` (`LARRT.cpp:317`), and
`validate_puzzle.py:check_single_object` (`:287`, `MOVE_TOL=1e-9`). The central architectural claim
(a pervasive **single-group-per-segment invariant**) is real.

**Status: a pure-Python REFERENCE PROTOTYPE of this route-1 design now exists and is chair-validated —
`tools/embodied_prototype.py`** (RobotSpec FK, closed-form 2R IK, `try_contact_pose`/reachability,
`push_transfer` explicit-waypoint primitive, a basic `transit_rrt`, and a `mode_sequencer`). It passes
both acceptance tests: SUCCESS with contact maintained to machine precision on `scenes/embodied_smoke.json`,
and ABORT (box stalled ≈6.05–6.11, short of goal) on `scenes/embodied_ce.json` — the accepted poses were
independently SAT-audited by the chair. The **C++/OMPL integration described below remains unimplemented**;
the Python prototype is the reference the C++ version should follow.

---

## Architecture decision (the load-bearing fact)

**Every piece of the current pipeline that counts or defragments "actions" hard-assumes exactly one
group/factor changes per path segment**, but a contact push moves *two* groups in lockstep (the robot
and the object it pushes). Confirmed by reading, not assuming:
- `LARRT::getChangedIndex` (`LARRT.cpp:235`) returns the *first* differing group — a single `int`
  (`Motion::index_changed`, `LARRT.h:183`), set once per edge (`createNewMotion`, `LARRT.cpp:209`).
- `MinimalActionsObjective.h` and `PathDefragmenter::isolateChecked` build on the same premise, and
  the defragmenter actively **splits/reorders** multi-group edges into single-group ones — which would
  tear apart a physically-coupled push.
- `FactoredStateSpace::interpolate` (`FactoredStateSpace.cpp:85`) fully finishes one group before
  touching the next — it has **no notion of two groups advancing in lockstep**, exactly what a push
  needs (robot dims and object DOF must move together, sample-by-sample).
- `validate_puzzle.py:check_single_object` (`:287`) independently enforces the same invariant.

**Decision:** do NOT teach LA-RRT / `FactoredStateSpace` / `PathDefragmenter` to understand contact
coupling. Follow `09` §6 **route 1** literally:
- **Transit** (robot moves alone, object DOF frozen) → ordinary *unmodified* LA-RRT over a state space
  whose only group is the robot's DOF block (objects frozen as static obstacles). Direct reuse of the
  existing "single mover, others frozen" pattern (`EirmConnector`, `LARRT2D_Puzzle.cpp:218-330`;
  `TWO_LEVEL_DESIGN.md`).
- **Transfer** (coupled robot+object push) → a dedicated primitive that builds full-state waypoints
  directly (robot IK per sample, object DOF advanced in lockstep), **never** fed through `growTree`'s
  diff/isolation logic or `FactoredStateSpace::interpolate`.
- Outer sequencer concatenates transit+transfer segments; "action count" is redefined as **number of
  mode segments**, sidestepping the single-group invariant rather than patching five call sites.

This keeps the disembodied pipeline (`LARRT2D_Puzzle.cpp`, `validate_puzzle.py`, checked-in
`out/*.json` baselines) **completely untouched**, matching `09`'s framing that embodiment is a
different problem statement, not an ablation.

## 1. Scene-schema + driver changes
New driver `demos/larrt2d/LARRT2D_Embodied.cpp` (copy — don't edit — the proven geometry primitives
`OBB`/`obbCorners`/`projectRange`/`obbOverlap` at `LARRT2D_Puzzle.cpp:56-91` and the `ObjSpec`/`Kind`
pattern at `:97-137`). Add `Kind::Robot` + a `RobotSpec` struct owning **3 rigid bodies** (base cube +
2 links), DOF `[xb, yb, θb, q1, q2]`:
```cpp
struct RobotSpec {
    int off, ndof = 5;
    double baseHx, baseHy, shoulderOffX, shoulderOffY, L1, W1, L2, W2;
    void obbs(const double *v, OBB &base, OBB &link1, OBB &link2) const; // FK -> 3 OBBs
};
```
`RobotSpec::obbs` chains the existing revolute `anchor + R(a)*com` mapping (`LARRT2D_Puzzle.cpp:126-132`)
twice: `θ1=θb+q1`, link1 at `shoulder + R(θ1)*(L1/2,0)`; elbow at `shoulder + R(θ1)*(L1,0)`; `θ2=θ1+q2`,
link2 at `elbow + R(θ2)*(L2/2,0)`. Extend the object-parsing loop (`:411-471`) with a `type=="robot"`
branch; extend `configValid` (`:149-160`) to emit a *list* of OBBs per body (base+link1+link2) and SAT
them against walls + each other. New per-object JSON field `pushable_faces` (id, `normal_local`,
`contact_offset_local`, `friction_half_angle`) in the object's body frame; world contact point/normal
= `R(θ_obj)` applied, θ_obj from the object's existing `.obb()` mapping.

## 2. Contact/push TRANSFER primitive
`TransferResult pushTransfer(scene, robot, obj, faceId, fromFull, targetObjDof, nSteps=200)` sweeping a
scalar `u`:
- **Box:** `center(u) = center(0) + u·n_world`, `n_world = R(θ_box)·normal_local`.
- **Door/Revolute:** object DOF `a = u`; `contactPoint(u) = hinge + R(u)·contact_offset_local`.
Per-sample constraint: robot end-link tip coincident with `contactPoint(u)` **and**
`|angle(endLinkAxis, −n_world(u))| ≤ friction_half_angle` (friction cone). Algorithm: for each `u`,
compute contact point/normal → `tryContactPose` (reuse previous base pose, else local base search) →
build full state (robot IK dims + `obj.off=u`, rest fixed) → SAT-check 3 robot OBBs + moving object vs
walls + other objects at their frozen poses (same "re-check vs other movables" as
`EirmConnector::connect`, `:260-269`) → **abort whole transfer on any step failure** (this all-or-
nothing is exactly what should trip on the Q-E2 counterexample). Plugs in **nowhere** inside `growTree`
/ `interpolate` — called directly by the sequencer, like `EirmConnector::connect` but changing two
groups per step (which is why it must bypass the single-`g` `FactorConnector` contract).

## 3. Reachability test (planar 2R IK)
`solve2RIK(shoulderX,Y, tipX,Y, L1,L2)` → closed-form (`d=hypot`; infeasible if `d>L1+L2` or
`d<|L1−L2|`; `cos q2=(d²−L1²−L2²)/(2L1L2)`, `q2=±acos`; `q1=atan2(dy,dx)−atan2(L2 sin q2, L1+L2 cos q2)`),
both elbow up/down. `tryContactPose(scene, robot, baseX,Y,θ, tipX,Y, pushNormalAngle, frictionHalfAngle,
robotDofOut)` solves IK, builds the 3 OBBs, SAT vs walls + target (touching-allowed, `SAT_EPS`) + other
objects, and checks the friction cone via `θ2`. Called (a) per-sample inside `pushTransfer`; (b) once
standalone by the sequencer to pick the approach pose the preceding transit must reach.

## 4. Mode structure
Mode = `{contactedObject:int(−1=free), faceId:int}`. **Transit** (`−1`): stock `og::LARRT(si, {robotDims})`
— a single group, so `growTree`'s `diff.size()>1` branch (`:317`) never fires; robot's 5-DOF block is the
mover, objects frozen (baked into the validity checker like `EirmConnector`'s frozen-others closures,
`:308-319`). **Transfer** (`k,f`): entirely `pushTransfer`. Outer sequencer (small, not general TAMP for
v1): for each mode in a fixed order → reach test → transit sub-LARRT to approach ball → `pushTransfer`
sweeps object DOF to goal → on any failure report INFEASIBLE (the Q-E2 outcome) and stop; concatenate,
tag each waypoint with its mode. **Certification/LB (`09` §4):** v1 does NOT extend
`MinimalActionsObjective`; cost = #mode-segments; reachability-aware LB arcs (Q-E1) are out of scope —
the only certification-relevant deliverable is the **counterexample scene** (Q-E2 demonstration).

## 5. Files, build/test, risks
**Create:** `LARRT2D_Embodied.cpp` (`demo_LARRT2D_embodied <scene> [planTime=20]`); optional
`robot_ik.h`; `scenes/embodied_door_box.json` (smoke: door + box + wall-with-pocket + one robot start
that reaches the door but not initially the box); `tools/validate_embodied.py` (sibling that duplicates
— per that file's stated no-import policy — the SAT/FK code + per-sample contact-coincidence + friction-
cone checks). **Modify:** `demos/CMakeLists.txt` — add `add_ompl_demo(demo_LARRT2D_embodied ...)` next to
the existing `larrt2d/LARRT2D_*` entries (`:65-69`). **Not modified:** `LARRT2D_Puzzle.cpp`, `LARRT.*`,
`FactoredStateSpace.*`, `PathDefragmenter.*`, `MinimalActionsObjective.h`, `validate_puzzle.py`.
**Build/test:** build target → run smoke (expect `[transit→door, transfer→door, transit→box,
transfer→box]`) → `validate_embodied.py` → extend `viz/viz_puzzle.py` to draw the 3-body robot →
build the **Q-E2 counterexample** (corridor exactly box-width but too narrow for base+box side-by-side:
box-only sub-scene solves trivially in `demo_LARRT2D_puzzle`, while `pushTransfer` fails mid-sweep — the
paper's key figure).
**Biggest risks:** (1) single-group invariant everywhere else — avoided by keeping transfers outside
LA-RRT; a future version that wants LA-RRT to search transit/transfer *interleavings* needs invasive
changes. (2) `interpolate` can't express coupled motion — the reason `pushTransfer` self-builds
waypoints. (3) IK/contact-tracking heuristic can give **false negatives** — a failed counterexample must
be confirmed to fail for the *right* reason (widen base-search radius, confirm still infeasible).
(4) No robot self-collision in v1 (justified by short stick links; spot-check). (5) Doors couple reach
to their own state — the door leg is the likely first failure; be ready to let the base nudge every few
`u` steps. (6) Schema/driver duplication is deliberate (protect the checked-in disembodied baselines).

---

*Chair note:* the one design point to watch when implementing is **risk 3** — the counterexample's
scientific value (Q-E2) depends on its infeasibility being *geometric*, not an artifact of the greedy
base-reuse IK. The acceptance test for the counterexample must include widening the base search until
either it solves (counterexample invalid) or infeasibility is robust (counterexample sound). This is
the same "prove it fails for the right reason" discipline the disembodied track used for the door-wedge
and buffer-scarcity findings.