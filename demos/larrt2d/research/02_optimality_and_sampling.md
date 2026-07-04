# Optimality Guarantees for Rearrangement & Constraint-Based Sampling — literature briefing

> Context: LA-RRT is a bidirectional RRT over a *factored* state space minimizing the **number of
> actions** (object mode-switches), followed by `PathDefragmenter` (reorder/merge same-object
> segments). This gives **no guarantee** on (i) action count or (ii) number of intermediate
> placements/buffers. Every citation was checked against arXiv/DOI/publisher; unverifiable claims
> are flagged. Produced by a research agent, 2026-07-04.

---

## 1. Why RRT + shortcut/defrag lacks optimality

**Feasibility vs. asymptotic optimality.** Plain RRT / RRT-Connect / bidirectional RRT is
*probabilistically complete* but **not** asymptotically optimal; the returned cost converges to a
suboptimal value. Asymptotic optimality is the contribution of **RRT\***.
- **Sampling-based Algorithms for Optimal Motion Planning** — Karaman & Frazzoli, IJRR 30(7), 2011.
  https://arxiv.org/abs/1105.1186

**Why RRT\*'s guarantee does not transfer to our objective.** RRT\* assumes an **additive,
continuous line-integral cost over a single continuous C-space**; its `Near`-ball rewiring relies on
that metric structure. Our objective differs fundamentally:
1. It is a **discrete combinatorial count** (object mode-switches). Piecewise-constant, integer,
   flat almost everywhere, jumps at mode boundaries — no traction for metric rewiring.
2. It lives on a **multi-modal / foliated** space: each grasp/placement is a lower-dimensional mode
   manifold; action count ≈ number of manifold transitions. Optimizing the *number of transitions*
   is a graph/search question *between* modes, orthogonal to what RRT\* optimizes *within* a mode.

**Shortcutting/defrag is only local.** Path shortcutting — and by direct analogy
`PathDefragmenter`'s reorder-and-merge — is a *local* post-process: it removes redundancy already
in the found path but cannot discover that a *different mode sequence* (different buffers) needs
fewer actions. The merged result is a local optimum of a greedy improver, with no global minimality
argument.

**Takeaway.** To bound action/placement count you must reason about the **discrete combinatorial
structure** (§2) or embed it in a **globally-optimizing formulation** (§§3–4). RRT\*-style continuous
optimality does not deliver it; defrag is strictly local.

---

## 2. Optimality for the discrete structure (min pick-places / buffers)

The min-action / min-buffer question is **combinatorial optimization**, and provably hard — which is
exactly why heuristic RRT+defrag cannot certify optimality without an exact search / IP layer.

- **Complexity Results and Fast Methods for Optimal Tabletop Rearrangement with Overhand Grasps** —
  Han, Stiffler, Krontiris, Bekris, Yu, WAFR 2016 / IJRR. Minimizing **pick-and-place count** (with
  start/goal overlaps forcing buffers) and end-effector travel are **NP-hard**; gives IP + fast
  heuristics; ties buffers to the **feedback/cycle structure** of the dependency graph.
  https://arxiv.org/abs/1711.07369
- **Toward Optimal Tabletop Rearrangement with Multiple Manipulation Primitives** — Rutgers ARC-L,
  2023. Optimal solvers mixing pick-place + pushing/toppling. https://arxiv.org/abs/2310.00167
- **On the Utility of Buffers in Pick-n-Swap Based Lattice Rearrangement** — Yu group, 2022.
  Bounds/optimization for minimizing temporary buffer slots. https://arxiv.org/abs/2209.05390
- **Toward Efficient Task Planning for Dual-Arm Tabletop Object Rearrangement (TRLB)** — Gao & Yu,
  IROS 2022. Handles **non-monotone** instances (objects moved more than once due to dependency
  cycles); schedules across two arms; reasons explicitly over the dependency structure that dictates
  the minimum re-grasps/buffers. https://arxiv.org/abs/2207.08078 · code https://github.com/arc-l/TRLB
- **Minimum Constraint Removal (MCR)** — Hauser, WAFR 2012 / IJRR 2014. Fewest constraints to remove
  to connect start–goal; **NP-hard (reduction from SET-COVER)**, even for convex polygons in the
  plane. "Which objects must be moved" is MCR-flavored. https://link.springer.com/chapter/10.1007/978-3-642-36279-8_1
  Related: **Minimum Constraint Displacement** — Hauser, RSS 2013. https://www.roboticsproceedings.org/rss09/p17.pdf

**Structural implication for LA-RRT.** The discrete objective is inherently a **search/IP problem
over a dependency (or mode) graph**. An optimality certificate on action/buffer count comes from
**solving that graph problem exactly (or with a proven bound)** — not from continuous rewiring or
segment merging.

---

## 3. Optimization-based TAMP & constraint-manifold sampling

### 3a. Toussaint's LGP (globally nonconvex, locally optimal)
- **Logic-Geometric Programming: An Optimization-Based Approach to Combined Task and Motion
  Planning** — Toussaint, IJCAI 2015. Casts TAMP as an FOL-parameterized NLP: a discrete symbolic
  skeleton selects the constraints of one NLP over the whole trajectory; finds **locally optimal**
  motions given a skeleton and searches skeletons (so **not** a global action-count certificate by
  itself). https://www.ijcai.org/Abstract/15/274
- **Differentiable Physics and Stable Modes for Tool-Use and Manipulation Planning** — Toussaint,
  Allen, Smith, Tenenbaum, RSS 2018. Adds differentiable dynamics / stable modes.
  http://www.mit.edu/~k2smith/publication/stable_modes/

### 3b. The "modulo sampling" question — resolved
**"Modulo sampling" is NOT a robotics/TAMP method** — in the literature it is a signal-processing /
unlimited-sampling ADC technique, unrelated to planning. The user's term is **almost certainly a
misremembering** of Toussaint-group constraint sampling:
- **NLP Sampling: Combining MCMC and NLP Methods for Diverse Constrained Sampling** — Toussaint,
  Braun, Ortiz-Haro, 2024. Sample **diverse** points satisfying hard constraints (MCMC + Gauss-Newton
  projection + hit-and-run). https://arxiv.org/abs/2407.03035
- **Learning Efficient Constraint Graph Sampling for Robotic Sequential Manipulation** —
  Ortiz-Haro, Hartmann, Oguz, Toussaint, ICRA 2021. https://arxiv.org/abs/2011.04828
- **Learning Feasibility of Factored Nonlinear Programs…** — Ortiz-Haro et al., ICRA 2023.
  https://arxiv.org/abs/2210.12386

> If the user has a specific source, "modulo sampling" may be a private label for NLP-Sampling /
> constraint-graph sampling, or a confusion with "random-MMP". **Flag: unconfirmed / likely misnomer.**

### 3c. Sampling *on* constraint manifolds — the exact LA-RRT issue (measure-zero goal)
Our goal placements are **measure-zero in the target dims**, so blind sampling almost never lands on
them. Canonical taxonomy:
- **Sampling-Based Methods for Motion Planning with Constraints** — Kingston, Moll, Kavraki, Annu.
  Rev. 2018. Organizes methods by two primitives (sample constraint-satisfying config; generate
  constraint-satisfying motion): **rejection** (hopeless for measure-zero), **projection**
  (Newton/Gauss-Newton onto the manifold), **tangent-space/atlas**, **reparameterization**.
  https://doi.org/10.1146/annurev-control-060117-105226
- **Manipulation Planning on Constraint Manifolds (CBiRRT)** — Berenson, Srinivasa, Ferguson,
  Kuffner, ICRA 2009 — bidirectional RRT that **projects** samples onto constraint manifolds; and
  **Task Space Regions (TSR)** — Berenson et al., IJRR 2011 — represents a measure-zero pose
  constraint as a **sampleable set with slack**. This is the standard fix for exactly our
  "goal is measure-zero in target dims" problem: sample the TSR, then project. (Gives
  feasibility/completeness on the manifold, **not** action-count optimality.)
  https://www.ri.cmu.edu/pub_files/2009/5/berenson_dmitry_2009_2.pdf
- **Sampling-Based Motion Planning on Sequenced Manifolds (PSM\*)** — Englert, Rayas Fernández,
  Ramachandran, Sukhatme, 2020. Plans across a fixed sequence of intersecting manifolds; RRT\* inner
  loop places **optimal intersection points** (optimizes *where* transitions happen given a fixed
  mode sequence). https://arxiv.org/abs/2006.02027

---

## 4. Convex-set methods (GCS): global optimality of the relaxation
- **Shortest Paths in Graphs of Convex Sets** — Marcucci, Umenberger, Parrilo, Tedrake, SIAM J.
  Optim. 2024. Each vertex carries a continuous variable in a **convex set**; NP-hard, but a tight
  mixed-integer-convex formulation whose **convex relaxation is empirically very tight** ⇒
  branch-and-bound/rounding returns solutions with **certified optimality bounds**.
  https://doi.org/10.1137/22M1523790 · https://arxiv.org/abs/2101.11565
- **Motion Planning around Obstacles with Convex Optimization (GCS Traj-Opt)** — Marcucci, Petersen,
  von Wrangel, Tedrake, Science Robotics 2023. Free space → convex regions (GCS vertices); trajectory
  as shortest path with Bézier edges; **automatic optimality bounds**; beat sampling planners on a
  7-DoF arm. https://doi.org/10.1126/scirobotics.adf7843 · https://arxiv.org/abs/2205.04422 ·
  code https://github.com/RobotLocomotion/gcs-science-robotics

**What GCS guarantees, precisely.** Global optimality **of the convex/MIC formulation** (to within a
usually-tiny relaxation gap), **conditional on the modeling**: you must (a) decompose space into
convex sets and (b) express costs/constraints convexly per set. The optimum is only as good as the
decomposition. Fundamental tradeoff: **global optimality of the surrogate vs. fidelity of the convex
surrogate to the true problem.**

**GCS for mixed discrete–continuous / manipulation (the WAFR-era work):**
- **Mixed Discrete and Continuous Planning using Shortest Walks in Graphs of Convex Sets** —
  Morozov, Marcucci, Graesdal, Amice, Parrilo, Tedrake, 2025 (WAFR 2026). Generalizes shortest
  **path** to shortest **walk** — vertices may be **revisited**, crucial for rearrangement where an
  object/mode recurs; SDP piecewise-quadratic lower bound guides search; returns an **approximate**
  shortest walk. https://arxiv.org/abs/2507.10878
  **Flags:** abstract lists collision-free motion / **skill chaining** / hybrid control — not
  "pick-and-place" explicitly; "WAFR 2026" venue reported but treat as to-confirm. Best candidate for
  the paper the user recalls.
- Adjacent: **Multi-Query Shortest-Path in GCS** (WAFR 2024, arXiv:2409.19543); **GCS\*: Forward
  Heuristic Search on Implicit GCS** (arXiv:2407.08848).

---

## 5. Synthesis — what it would take to give LA-RRT an optimality guarantee

Three routes, increasing modeling investment:

**Option A — Exact search over a dependency / mode graph (most direct; §2).** Extract the
object-dependency graph; solve the discrete problem *exactly* (IP / branch-and-bound over
pick-place orderings + buffer assignments), following Han et al. and TRLB. LA-RRT/CBiRRT then fills
in feasible motions for a **provably minimum-action skeleton**. Guarantee: **optimal action/buffer
count** (given the primitive set); motions merely feasible. Lowest-risk path to a real guarantee.
(Count is NP-hard ⇒ "optimal" = optimal small instance / possibly-slow solver.)

**Option B — GCS over placement/mode regions (strongest joint certificate; §4).** Model each
object's admissible **placement regions** and each grasp/mode as **convex sets**; build a GCS whose
**walks** are action sequences (revisitable vertices ⇒ move an object twice); action count =
**per-edge unit cost**. Solve via shortest-walk (arXiv:2507.10878) / shortest-path GCS. Guarantee:
**global optimality of the convex model + certified relaxation gap.** Cost: build the decomposition;
guarantee is relative to the model. Most principled route to *jointly* optimal actions + placements.

**Option C — Keep the RRT, add an admissible lower bound (cheapest add-on; §§1,3).** Wrap LA-RRT in
a branch-and-bound / anytime loop guided by an **admissible lower bound on remaining action count**
(from the dependency graph: #overlap-cycles ⇒ min buffers/re-grasps, à la MCR/running-buffer). For
the **measure-zero goal**, replace direct goal sampling with **TSR + projection** (CBiRRT) or
**NLP/constraint-graph sampling** (Toussaint). Guarantee: **only if** the bound is admissible and you
prune to it — then a returned plan whose cost meets the bound is *certified* minimum-action. Keeps
the existing architecture; `PathDefragmenter` stays a *local* polish, never the guarantee source.

**Recommendation.** For a *provable* bound with least re-engineering: **A** (exact discrete solver
feeding LA-RRT). For a single framework *jointly* optimal over placements + action count: invest in
**B** (GCS shortest-walk) — after confirming the exact WAFR reference. Use **C**'s TSR/NLP-sampling
fix regardless, since it addresses the measure-zero-goal bug independently of the optimality route.

---

### Verification summary
- **Confirmed:** RRT\* (Karaman–Frazzoli); Han et al. tabletop hardness; TRLB; Hauser MCR & MCD;
  Toussaint LGP (IJCAI 2015) & RSS 2018; NLP Sampling; Kingston–Moll–Kavraki; CBiRRT & TSR; PSM\*;
  GCS SIAM 2024; GCS Traj-Opt (Science Robotics 2023); Shortest-Walk GCS (arXiv:2507.10878).
- **Flagged:** "modulo sampling" is **not** a verified TAMP method (signal-processing term); intended
  method is almost certainly **NLP Sampling / constraint-graph sampling**. The "WAFR 2026
  pick-and-place convex-sets" paper is best matched by **Morozov et al. arXiv:2507.10878**, but its
  explicit pick-and-place framing is unconfirmed — verify the exact reference.
