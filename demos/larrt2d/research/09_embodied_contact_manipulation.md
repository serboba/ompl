# 09 — Embodied contact-based rearrangement: objects don't move themselves

**Status: research idea / parking lot (documented, NOT to be solved now).** This is a
future, harder paper distinct from the current certified-rearrangement track. Captured
2026-07-04 at the user's request. Read `08_factor_spaces.md` (heterogeneous factors) and
`06_constraint_sampling_deep_dive.md` (manifold/constraint sampling) first — this briefing
sits directly on top of both.

---

## 1. The idea in one paragraph

Everywhere in the current pipeline an object is its **own factor** and moves **autonomously**:
a box slides to its goal, a door swings open, a slider extends — each along its own DOFs, as if
by an invisible hand. That is the **pick-place / disembodied abstraction** (`01`, `02`). The idea
here is to **remove the invisible hand**: objects are **passive** and cannot move on their own.
The only actuated thing in the scene is a **robot** — model it as a mobile base (a cube) carrying
a very simple kinematic chain, a **stick of 2–3 revolute joints**. To move anything, the robot
must **drive to it, reach out, make contact, and push**. A door will not open by itself; the arm
has to reach the blade and rotate it. This turns a graph-of-object-motions problem into a
**physically grounded manipulation problem**, and it introduces a coupling that the current
factored model never sees: *whether an object can be moved at all now depends on whether the
robot can reach it and keep contact while co-moving with it*.

## 2. The model (proposed, minimal)

- **Robot** = mobile base `(x_b, y_b, θ_b)` ∈ SE(2) + a serial revolute arm `(q_1, …, q_n)`,
  n ∈ {2, 3}, links short and stick-like. Total robot config `r = (base, q)` ∈ SE(2) × T^n.
  Keep it 2D (planar) to stay inside the existing scene schema and geometry (OBBs, SAT).
- **Objects** = the existing passive movables (boxes, swing doors, sliders). They have **no
  self-actuation**: an object's DOF changes **only** while a robot link is in contact and
  applying motion consistent with the contact.
- **Interaction = contact/push.** Start with the simplest sound model: **quasi-static rigid
  contact** — the arm's end link touches an object face; while contact holds, the object moves
  as driven by the contact (a pushed box translates/rotates within the pushing friction cone; a
  touched door rotates about its hinge as the contact point sweeps). No grasping needed for v1;
  pure nonprehensile pushing is enough to express "open the door / shove the box aside."
- **Goal** unchanged in spirit: target objects to target poses, free objects anywhere valid — but
  now every object-pose change must be **realized by a robot contact trajectory**.

This is deliberately the *smallest* embodiment that breaks the disembodied abstraction: no gripper,
no dynamics, no force control — just reach + touch + quasi-static push in the plane.

## 3. Why this is fundamentally harder — the coupling

In the current model, "object A can go from pose p to pose q" is a **property of A alone**
(A's own C-space connectivity with everything else frozen — exactly what the sound-LB
disconnection arcs test, `SCENARIOS.md`/`monotonicity.py`). In the embodied model that property is
**necessary but no longer sufficient**. Moving A now requires a **coupled trajectory** of
`(base, arm, A)` such that, at every instant:

1. **Reachability** — the arm can touch A's current contact face from a collision-free base pose
   (the base+arm must be placeable next to A without hitting walls, other objects, or A's own body).
2. **Contact-maintenance under transport** — as A advances toward its goal, the contact point and
   the whole robot must **co-move** so the push stays valid (contact kept, inside the friction
   cone, arm within joint limits, base collision-free) for the *entire* transport, not just at the
   endpoints.
3. **Simultaneous collision-freedom** — base + all arm links + the moving object are mutually and
   environmentally collision-free throughout.

The user's key observation, stated precisely:

> **Object-level feasibility ⇏ embodied feasibility.** A box may need to travel right into a pocket,
> and the box's *own* path there may be clear — yet the robot pushing it may be unable to travel
> right alongside it (a wall, a narrow slot, its own arm's joint limits, or another object blocks
> the base/arm), so contact cannot be maintained the whole way. The object *could* reach the goal;
> the *contact* cannot escort it there.

So the embodied problem adds an **existential quantifier over robot trajectories** inside what used
to be a simple per-object reachability check. The "can A move p→q?" oracle becomes "does there
exist a coupled base+arm push trajectory that carries A p→q while keeping contact, reach, limits,
and collisions all satisfied?" — a constrained motion-planning query in a higher-dimensional,
**contact-manifold-constrained** space.

A second, subtler difficulty: **doors couple reach to their own state.** To open a door you must
reach its blade; but the blade's reachable region *changes as the door opens*, and the opening door
may itself sweep into the base's parking spot. Reachability and the manipulated DOF are entangled.

## 4. Relation to the current factored / certification framework

- **Factors stop being independent.** LA-RRT's defining move — "one edge moves exactly one factor"
  (`TWO_LEVEL_DESIGN.md`) — becomes "one edge moves the **robot subsystem plus the single object it
  is currently in contact with**." The natural embodied factor is the *robot*, transiently
  coupled to one object at a time. Transfer edges are constrained (contact manifold); transit edges
  (robot moving with no object) are free-space.
- **Modes.** The problem acquires a discrete **mode** structure: {which object, if any, the robot
  is touching; which contact face}. This is the classic transit/transfer mode graph of manipulation
  planning (§6 lit). Rearrangement order (our whole non-monotone story) becomes the *outer* discrete
  layer; reach+push feasibility becomes the *inner* continuous-constraint layer.
- **The certification story must be re-examined.** Our sound lower bound counts **object actions**
  (`§3.1`). Under embodiment the natural cost changes: a single "object action" may need several
  robot sub-motions (approach, contact, push, retract), and — more importantly — the **disconnection
  arcs are no longer the right necessary conditions**. An arc "B blocks A" could now also be a
  *reachability* block ("B blocks the robot from reaching A's push face") even when B does not block
  A's own C-space. The LB engine would need **reachability-aware arcs**. Conversely, some object-level
  arcs may weaken (the robot can push A along a curved contact-respecting path a straight sweep can't
  see). Whether a *sound, cheap* a-priori LB survives embodiment is itself an open question — and a
  strong paper hook, because it is exactly where our certification niche would extend.
- **The MAB idea (`07`) transfers and grows.** Arms to select over now include *contact-face choice*
  and *base-approach side*, with reward candidates like "reachable free-space opened by this push."

## 5. What kind of problem / planning is this?

It is **constraint / manifold-constrained manipulation planning with contact**, wrapped in a
**TAMP-style mode structure**. The "maintain contact" requirement is an **equality constraint**
defining a lower-dimensional manifold in the composite `(robot, object)` space (a closed-chain-like
constraint: the end link and the object face are coincident along the contact). Planning transfer
motions = planning **on that manifold** (projection/atlas/CBiRRT-style), while the friction cone and
joint/collision limits are **inequality constraints** carving out the feasible region. The user's
"constraint sampling / constraint planning" instinct is right: this is the constrained-sampling
regime of `06`, but with the constraint now generated by *physical contact* rather than a task frame.

## 6. Candidate attack routes (sketches, NOT commitments)

1. **Mode-based TAMP + constrained transfer sampling.** Outer search over the transit/transfer mode
   graph (which object, which face, in what order — reuse the non-monotone dependency machinery for
   ordering); inner constrained planner (CBiRRT/atlas, `06`) for each transfer on the contact
   manifold, with the friction cone + reach + collisions as feasibility filters. Cleanest first cut.
2. **Contact-implicit trajectory optimization** for the local push (`§6.4` lit): let the optimizer
   discover contact timing/forces for a single object transport, used as the transfer primitive
   inside route 1. Powerful but heavier; likely overkill for planar quasi-static v1.
3. **Reachability-map precomputation.** Precompute, per object pose, the set of base poses from which
   the arm can contact each face (a planar reachability map, `§6.3`). Turns the inner reach test into
   a table lookup and lets the *outer* planner reason about "can the robot get into a pushing
   pose here at all" cheaply — and feeds **reachability-aware LB arcs** (§4).
4. **Quotient / decomposition guide.** Plan the object rearrangement first in the disembodied model
   (our current planner) to get a *candidate object-action sequence + buffer poses*, then try to
   **realize** each object action with a reach+push motion; on failure, feed the infeasibility back
   as a new constraint/arc and replan. A "disembodied plan as admissible guide, embodiment as
   feasibility certifier" loop — echoes the delete-other-objects quotient guide of `05`/`06`.

## 7. Open questions (the paper's spine)

- **Q-E1.** Does a *sound, cheap a-priori lower bound* on robot effort (or object actions) survive
  embodiment? What are the **reachability-aware necessary arcs** (§4), and are they still
  certificate-grade?
- **Q-E2.** Characterize the **object-feasible-but-contact-infeasible** gap: when does object-level
  connectivity fail to imply embodied feasibility? Can we test it a-priori (a reachability-corridor
  analogue of the swept-corridor complement, `06`-Design-1)?
- **Q-E3.** Right **contact model** granularity for planar rearrangement: rigid quasi-static
  point/face contact vs full friction-cone push mechanics — how much fidelity is needed before the
  *combinatorial* structure (ordering, buffers) stops changing?
- **Q-E4.** Does the **non-monotone dependency structure change** under embodiment (new deadlocks
  from reach conflicts; the robot's own body as a transient movable obstacle)?
- **Q-E5.** Complexity: embodiment surely doesn't lower it (already PSPACE-hard disembodied, `03`);
  does the contact-manifold constraint change the *practical* phase transition / difficulty
  predictors (`difficulty.py`, `03` §5)?

## 8. Minimal first experiment (if/when picked up)

Stay 2D, reuse the schema. Add one robot object = SE(2) base + 2R planar arm. One target box, one
swing door, a wall with a pocket. Implement only: (a) planar reachability test (arm IK to a contact
face from a base pose, collision-checked); (b) a quasi-static push transfer primitive (box translates
along contact normal; door rotates as contact sweeps); (c) route-1 mode search. Success metric: solve
"open the door, push the box into the pocket" where the *disembodied* planner solves it trivially but
the embodied one must sequence reach→open→re-approach→push — and construct the **counterexample scene**
of §3 (object path clear, contact escort impossible) to demonstrate Q-E2 concretely. That single
counterexample scene is the figure that motivates the whole paper.

## 8.5 Formal core + a validated counterexample (accepted contribution, chair-verified)

*Contributed by a delegated sub-agent and then chair-reviewed; the counterexample geometry was
**independently re-validated** (a second SAT check over 352,824 base×orientation candidates, not the
author's script) — reproducible via `tools/embodied_ce_check.py` on `scenes/embodied_ce.json`.*

**Formal problem definition.** Composite space
`C = SE(2)_base × T^n_arm × O_1 × … × O_m` (each `O_i` = the object's own DOF space:
`R²×S¹` box, `S¹` door/revolute, `R` slider — the `obb_from_dof` cases). Write `r=(b,q)` for the
robot sub-config and `FK_i(r)` for link poses; the **end link's** distal edge is the only surface
allowed to contact objects. A **contact-maintenance equality constraint** for object `k`, face `f`
holds when the end-link edge is flush against face `f`:
`g_{k,f}(r,o_k)=0` (edge points coincide at some contact parameter `s∈[0,1]`) **and** the edge normals
are antiparallel (flush, not a corner graze). This carves the **transfer manifold**
`M_{k,f} = { c∈C : g_{k,f}=0 }`. On `M_{k,f}` object `k`'s DOF are **driven** by the robot (pushed box
translates along `−n_f`; touched door rotates as the contact point sweeps about the hinge); **off** any
manifold, all object DOF are frozen — this *is* the formal statement of "objects don't self-actuate."
**Inequalities:** joint limits; SAT/OBB collision-freedom of base + every link vs obstacles and vs all
objects (the sole licensed touch = end link on its active contact face, EPS-touching allowed); and a
**friction-cone / non-separation** condition on the contact-point velocity (`|v_c·t_f| ≤ μ (v_c·(−n_f))`,
`v_c·n_f ≤ 0`) — for a face-normal on-centroid push this reduces to pure translation along `−n_f`.
**Modes:** transit (`μ=⊥`, robot free, objects frozen) vs transfer (`μ=(k,f)`, motion on `M_{k,f}`).
**Cost:** robot path length over transit+transfer + a per-mode-switch overhead — generalizes the
disembodied "object-action count" by charging for the robot's own reach/re-approach motions.

**The counterexample (`scenes/embodied_ce.json`).** World 12×6. A wall with a horizontal **slot**:
solid pieces `[4.5,7.5]×[0,2.64]` and `[4.5,7.5]×[3.36,6.0]` leave an open slot `x∈[4.5,7.5]`,
`y∈[2.64,3.36]` — **height 0.72, length 3.0**. Target **box** `0.7×0.7`, start `(2,3)`, goal
`(6.35,3)` (deep in the slot; rear push-face at `x=6.0`). **Robot**: square base `0.8×0.8`, 2R arm
`L1=L2=0.6` (reach **1.2**), shoulder front-mounted (the most favorable mount), joint limits left
*unconstrained* so the result is a pure reach impossibility.

**Why it proves Q-E2 (object-feasible ⇏ embodied-feasible):**
- *(a,b) the box alone can get there.* Its y-extent `[2.65,3.35]` stays inside the slot band
  `[2.64,3.36]` (0.01 clearance) along the whole straight path — start, goal, and 6001 interior
  samples all collision-free.
- *(c) but no contact can escort it.* For **any** orientation the square base's y-extent is
  `2·0.4·(|sinθ|+|cosθ|) ≥ 0.8 > 0.72`, so the base **cannot follow the box through the slot** (it
  cannot fit); at best a *tilted* base noses one corner into the open slot mouth, pushing its shoulder
  to `x≈4.55`. The **airtight** claim is at the goal: the required goal contact point is `(6.0,3)`, and
  the closest a collision-free base's shoulder can get is **1.46** (independent grid), `> 1.2` reach —
  over 352,824 base×orientation candidates, **zero** are simultaneously collision-free and within arm
  reach, so the contact-support set at the goal is provably empty. In practice the box stalls at center
  **≈6.05–6.11** (contact just past `x≈5.76` becomes unreachable; the exact point is mildly dependent
  on how the base tilts into the slot mouth), well short of goal 6.35. The box *could* sit at its goal;
  no push trajectory can *deliver* it there. ∎

This is the concrete realization of §3's phenomenon and the figure §8 calls for. **The reference
prototype `tools/embodied_prototype.py` dynamically reproduces it**: `push_transfer` on this scene
aborts with the box stalled at center ≈6.05–6.11 (accepted poses independently SAT-audited: tip on
contact, no collision, reach `< 1.2`), while it *succeeds* on a wide-slot smoke scene
(`scenes/embodied_smoke.json`) with contact maintained to machine precision every sample. The
goal-unreachability margin (0.26, ~22% of reach) is robust to grid resolution and orientation
sampling. *(Chair note: an initial "prototype accepts an invalid pose" suspicion was traced to a
too-conservative hand bound — the base can tilt a corner into the slot mouth; the prototype's poses
are genuinely valid, confirmed by independent SAT audit of the actual accepted states.)*

## 9. Related work

> **Provenance.** Compiled by a delegated Sonnet sub-agent from live web search (all 26 refs checked
> against ≥1 authoritative source; none cited from memory), then **chair-reviewed**: the recent arXiv
> entries most likely to be wrong were independently re-verified — R-LGP (arXiv:2310.02791, Ly et al.,
> ICRA 2024) and Predictive Reachability (arXiv:2410.21059, Feng et al., RA-L 2024/25) both confirmed
> exact. The remaining refs are canonical works. Re-confirm DOIs at camera-ready.

**A. Navigation / Manipulation Among Movable Obstacles (NAMO / MAMO).**
Wilfong, "Motion planning in the presence of movable obstacles," *Annals of Math. & AI* 1991 (orig.
SoCG 1988) — NP-hard (unspecified final positions), PSPACE-hard (specified). Stilman & Kuffner,
"Navigation Among Movable Obstacles," *Humanoids* 2004 / *IJHR* 2005 (DOI 10.1142/S0219843605000545).
Stilman, Schamburek, Kuffner & Asfour, "Manipulation Planning Among Movable Obstacles," *ICRA* 2007
(ResolveSpatialConstraints). Van den Berg, Stilman, Kuffner, Overmars & Manocha, "Path Planning among
Movable Obstacles: A Probabilistically Complete Approach," *WAFR* 2008. Learning-based NAMO with a
mobile manipulator: "Efficient NAMO via Hierarchical Policy Learning," arXiv:2506.15380, 2025.
*Takeaway:* NAMO/MAMO establishes that *which* obstacles move and in what order is already
combinatorially hard under a pick/rigid-attach abstraction of the mover; our problem inherits that
combinatorics but **denies the rigid-attach shortcut** — base+arm must stay kinematically coupled to
the object throughout the push, not just at endpoints.

**B. Nonprehensile manipulation & planar push planning.**
Mason, "Mechanics and Planning of Manipulator Pushing Operations," *IJRR* 5(3):53–71, 1986 (friction
cones, voting theorem). Lynch & Mason, "Stable Pushing: Mechanics, Controllability, and Planning,"
*IJRR* 15(6):533–556, 1996. Dogar & Srinivasa, "A Framework for Push-Grasping in Clutter," *RSS*
2011. Bauza & Rodriguez, "A Probabilistic Data-Driven Model for Planar Pushing," *ICRA* 2017
(arXiv:1704.03033); Hogan, Grau & Rodriguez, "Reactive Planar Manipulation with Convex Hybrid MPC,"
*ICRA* 2018. *Takeaway:* gives the local mechanics/control of a stable, contact-consistent push, but
assumes the pusher is already reachably positioned; it does **not** address a mobile base needing to
*co-navigate* to stay within reach as the pushed object's pose — and thus the required contact point —
changes during transport. (Supplies our transfer primitive, §6 routes 1–2.)

**C. Reachability-constrained / whole-body mobile manipulation.**
Zacharias, Borst & Hirzinger, "Capturing Robot Workspace Structure: Representing Robot Capabilities,"
*IROS* 2007 (capability maps). Vahrenkamp, Asfour & Dillmann, "Robot Placement Based on Reachability
Inversion," *ICRA* 2013. Chitta, Cohen & Likhachev, "Planning for Autonomous Door Opening with a
Mobile Manipulator," *ICRA* 2010. Ly, Semenov, Risiglione, Merkt & Havoutis, "R-LGP: A
Reachability-guided Logic-geometric Programming Framework…," *ICRA* 2024 (arXiv:2310.02791). Feng,
Horii & Nagai, "Predictive Reachability for Embodiment Selection in Mobile Manipulation Behaviors,"
*RA-L* 2024 (arXiv:2410.21059). *Takeaway:* capability maps + base-placement give **one-shot /
keyframe** "can the arm reach from here" (→ §6 route 3 lookup, and seeds reachability-aware LB arcs
§4), but almost all treat reach as a static query — not a constraint enforced **continuously** along
a coupled base+arm+object trajectory while the target reach point itself moves.

**D. Contact-implicit & mode-based / hybrid planning.**
Posa, Cantu & Tedrake, "A Direct Method for Trajectory Optimization of Rigid Bodies Through Contact,"
*IJRR* 33(1):69–81, 2014. Mordatch, Todorov & Popović, "Discovery of Complex Behaviors through
Contact-Invariant Optimization," *ACM ToG (SIGGRAPH)* 31(4), 2012. Cheng, Huang, Chavan-Dafle &
Rodriguez, "Contact Mode Guided … Quasistatic Dexterous Manipulation in 2D," *IROS* 2021
(arXiv:2011.01454) and 3D, *ICRA* 2022 (arXiv:2105.14431). *Takeaway:* solves *when/how* contact is
made or broken for the manipulated object, but is posed with a fixed-base / already-reachable
manipulator; jointly reasoning over a **mobile base's changing reachability envelope during a push**
is the missing ingredient. (Heavy-machinery option for the local push, §6 route 2.)

**E. Manifold / constraint-based sampling (the "maintain contact" constraint).**
Berenson, Srinivasa & Kuffner, "Task Space Regions," *IJRR* 30(12):1435–1460, 2011 (CBiRRT2).
Kingston, Moll & Kavraki, "Sampling-Based Methods for Motion Planning with Constraints," *Annu. Rev.
Control Robot. Auton. Syst.* 2018 (DOI 10.1146/annurev-control-060117-105226). Jaillet & Porta,
atlas-based RRT / "Asymptotically-Optimal Path Planning on Manifolds," *IJRR/RSS* 2012–13. Stilman,
"Global Manipulation Planning in Robot Joint Space with Task Constraints," *T-RO* 26(3):576–584,
2010. *Takeaway:* these sample the manifold of configs satisfying a **fixed** task constraint; embodied
transport instead needs the feasible manifold (valid base+arm configs maintaining contact) **re-derived
at every point along the object's trajectory** — a manifold that moves with the plan. (Inner transfer
planner, §6 route 1; ties to `06`.)

**F. TAMP for rearrangement gated by grasp/contact reachability.**
Garrett, Chitnis, Holladay, Kim, Silver, Kaelbling & Lozano-Pérez, "Integrated Task and Motion
Planning," *Annu. Rev. Control Robot. Auton. Syst.* 4:265–293, 2021. Garrett, Lozano-Pérez &
Kaelbling, "PDDLStream," *ICAPS* 2020. Toussaint, "Logic-Geometric Programming," *IJCAI* 2015.
*Takeaway:* handles discrete grasp/contact-mode switching gated by geometric feasibility streams, but
the push-action feasibility stream is typically checked at **isolated keyframes** (pre-/post-push), not
as a continuous, trajectory-length reach+contact-maintenance predicate — which is the crux here. (Outer
ordering layer, §4; our non-monotone dependency machinery is the heuristic inside it.)

**Closest prior formulations & what stays open.** The nearest precedents:
1. **Chitta, Cohen & Likhachev, "Planning for Autonomous Door Opening with a Mobile Manipulator,"
   ICRA 2010** — the most direct: coordinated base+arm motion that *keeps grasping* the handle while
   the door swings, i.e. a moving-target reach/contact constraint over a transport arc — but a single
   1-DOF articulated object with a fixed grasp, not general passive objects under a slip-prone push.
2. **R-LGP (Ly et al., ICRA 2024)** — embeds a sampling-based reachability graph into LGP so discrete
   actions are pruned by whole-body reachability — but reachability is checked at action *keyframes*,
   not enforced as a continuous invariant across a transport segment.
3. **Predictive Reachability (Feng et al., RA-L 2024)** — trajectory-aware reachability for the robot's
   *own* embodiment/mode selection — closest to "reachability that must hold along a horizon," but not
   applied to whether a *passive pushed object* stays transportable under the coupling.

**Open / novel** for the mobile-cube + simple 2–3R stick + 2D passive-object formulation: (i) the
binding constraint is a **deliberately weak, near-non-redundant stick arm** (little freedom to
reshuffle contact without moving the base), not an assumed-capable manipulator; (ii) it is embedded in
**non-monotone multi-object rearrangement with buffers** (our whole track), not single-object
relocation; (iii) contact geometry/direction changes **continuously and nontrivially with the object's
own state** (a swinging door rotates the required contact normal); (iv) feasibility is a **global
coupling** — an object's goal may be feasible in isolation yet infeasible once contact-maintenance is
required throughout transport, possibly via interactions with *other* objects constraining the base's
admissible path (§3, Q-E2); and (v) it asks whether a **sound, cheap, reachability-aware lower
bound / certificate** survives embodiment (§4, Q-E1) — which NAMO/MAMO and push-planning do not pursue.
This intersection of NAMO combinatorics, planar-push mechanics, and trajectory-length reachability
constraints is, per verifiable literature, unaddressed.

---

**Why this is a separate paper.** The current track's thesis is *certified action-optimal
disembodied rearrangement*. Embodiment changes the object model, the feasibility oracle, the cost
metric, and possibly the soundness of the lower bound — it is not an ablation of the current system
but a different problem statement. Documented here so the certified-rearrangement paper can cite it
as future work and a follow-up can pick it up cleanly.
