# Constraint-aware sampling deep dive — LGP, manifold sampling, quotient spaces → a recipe for LA-RRT

> Context: LA-RRT is a factored bidirectional RRT over per-object configurations (2D OBBs, some on
> prismatic/revolute joints), one object moving per action. The pain point (documented in
> `GOAL_REGION_AND_COMPLETENESS.md` and briefing `02` §3): goal sets and *useful intermediate
> placements* are measure-zero or low-measure in the joint space, so uniform sampling almost never
> hits them. This briefing digs into the three literatures that attack exactly this — Toussaint's
> LGP line, constraint-manifold sampling, and quotient-space planning — and ends with concrete
> sampler designs for LA-RRT. Extends (does not repeat) `02_optimality_and_sampling.md` §3.
> Every citation was checked against arXiv/DOI/publisher pages; unverified items are flagged.
> Produced by a research agent, 2026-07-04.

---

## 1. Where we stand (baseline from existing notes)

Already established and **implemented**:
- The goal manifold `G` (targets pinned, free objects anywhere) is measure-zero; LA-RRT reaches it
  via goal-tree sampling **plus** the PC-preserving goal-biased projection `π_G(q)` with fixed
  `goalBias_ > 0` (`GOAL_REGION_AND_COMPLETENESS.md` §3b, §5).
- Briefing `02` §3 identified TSR/CBiRRT projection and Toussaint NLP-sampling as the generic fixes,
  and resolved "modulo sampling" as a misnomer for constraint sampling.

What is **not yet addressed**: the same measure problem for *intermediate* states. A useful buffer
placement — e.g. "object B parked out of the swept corridor A will need" — is a *positive-measure
but tiny* subset of B's factor, and near-measure-zero as a *joint* event across several objects.
Uniform per-factor sampling finds such placements only by luck; this is why `three_alcoves` stays
at 5 actions (optimum 4) and why non-monotone scenes need many iterations. This briefing is about
engineering that luck away.

A useful classification for what follows (three different "hard set" types, needing different tools):

| set | geometry | right primitive |
|---|---|---|
| goal (`q_i = g_i`) | coordinate pinning, exactly measure-zero | projection (done) / per-factor goal sampling |
| joint-constrained objects (prismatic/revolute) | equality manifold | **reparameterization** — our factors already do this natively (sample the joint variable); no Newton projection needed |
| useful buffers | semi-algebraic, positive but tiny measure | **biased/conditional region sampling**, not equality projection |

---

## 2. Toussaint's LGP line — what is sampled vs. what is optimized

### 2a. LGP (IJCAI 2015): nothing is sampled — keyframes and paths are *optimized*

- **Logic-Geometric Programming: An Optimization-Based Approach to Combined Task and Motion
  Planning** — Toussaint, IJCAI 2015. https://www.ijcai.org/Abstract/15/274
  A discrete symbolic **skeleton** `a_1..a_K` (found by tree search over a STRIPS-like logic)
  *parameterizes the constraints* of one nonlinear program (KOMO) over the whole trajectory.
  **Mode-switch constraints enter as equality/inequality constraints active on time intervals**:
  each symbolic action toggles which kinematic-switch constraints hold (e.g. `touch`, `stable(obj,
  gripper)` — object pose rigidly attached to a new parent frame after the switch). The continuous
  side has **no uniform C-space sampling at all** — every continuous decision (grasp pose,
  placement, path) is a variable of an NLP whose constraint set is dictated by the skeleton.
  Randomness enters only through NLP initialization (random restarts).

### 2b. Multi-Bound Tree Search (ICRA 2017): the keyframe hierarchy — the exportable idea

- **Multi-Bound Tree Search for Logic-Geometric Programming in Cooperative Manipulation Domains** —
  Toussaint & Lopes, ICRA 2017. https://ieeexplore.ieee.org/document/7989464/ ·
  PDF https://argmin.lis.tu-berlin.de/papers/17-toussaint-ICRA.pdf
  Solving the full-path NLP for every candidate skeleton is unaffordable, so MBTS evaluates a
  **hierarchy of relaxed NLPs, each a lower bound / necessary feasibility condition** for the next:
  - **P1 ("pose bound")**: optimize only the *single* configuration at the current mode switch
    (plus its predecessor), with effective kinematics — no sequence, no path.
  - **P2 / `Pseq` ("sequence bound")**: jointly optimize **only the mode-switch keyframes** (coarse
    time resolution, ~2 steps per symbolic decision) — i.e. the tuple of configurations *at the
    action boundaries*, under all switch constraints, ignoring the fine motion between them.
  - **P3 / `Ppath` ("path bound")**: the full trajectory NLP.
  Branch-and-bound/MCTS over skeletons expands a node only if the cheaper bounds are feasible.
  Later LGP variants confirm the same architecture (R-LGP https://arxiv.org/abs/2310.02791,
  D-LGP https://arxiv.org/abs/2312.02731 — both verified to exist; they even drop `Ppath` online).

  **Takeaway for LA-RRT (the single most transferable idea):** the continuous variables that matter
  for the *combinatorial* quality of a plan are exactly the **keyframes — the object placements at
  action boundaries**. LGP never asks a sampler to stumble onto them; it *solves for them* at
  `Pseq` level before committing to any motion. Our factored analog is small: for a candidate
  action skeleton, the keyframe problem is one 3-DoF (or 1-DoF jointed) block per moved object per
  action, with pairwise-disjointness + swept-corridor constraints — a tiny NLP or CSP (see §5,
  Design 2).

### 2c. Sequence-of-Constraints MPC (IROS 2022): mode switches as constraint phases

- **Sequence-of-Constraints MPC: Reactive Timing-Optimal Control of Sequential Manipulation** —
  Toussaint, Harris, Ha, Driess, Hönig, IROS 2022. https://arxiv.org/abs/2203.05390 ·
  https://ieeexplore.ieee.org/document/9982236/
  Takes the *discrete decisions as given* (a single sequence of constraint sets = the TAMP plan) and
  decomposes execution into three online NLPs: (i) **sequential waypoints** (again: the keyframes),
  (ii) their **timing**, (iii) a short receding-horizon path; with phase backtracking on constraint
  violation. Reinforces the same layering: *keyframes first, timing/path later*. Not directly a
  sampler, but the cleanest statement of "a plan **is** a sequence of constraint activations".

### 2d. When LGP *does* sample: the Ortiz-Haro sub-line (2021–2024)

Where NLP restarts fail (multimodal constraint sets — exactly our buffer case), the Toussaint group
moved from pure optimization to **structured sampling of the keyframe problem**:
- **Learning Efficient Constraint Graph Sampling for Robotic Sequential Manipulation** — Ortiz-Haro,
  Hartmann, Oguz, Toussaint, ICRA 2021. https://arxiv.org/abs/2011.04828
  The keyframe problem is a **constraint graph**: variables = object/robot poses at mode switches,
  factors = constraints (grasp, placement, reachability). Solutions are found by choosing a
  *computational assembly order* — sample/solve variables one at a time, conditioned on already-fixed
  neighbors — and the paper *learns* which assembly order minimizes wasted samples. This is
  factored conditional sampling of keyframes, i.e. structurally identical to what LA-RRT needs.
- **Structured Deep Generative Models for Sampling on Constraint Manifolds in Sequential
  Manipulation (DGCS)** — Ortiz-Haro, Ha, Driess, Toussaint, CoRL 2021 (PMLR v164).
  https://proceedings.mlr.press/v164/ortiz-haro22a.html
  A deep generative model conditioned on a scene image proposes samples *near* the constraint
  manifold; **Gauss-Newton projection finishes the job**. Exploits the factored structure of the
  constraint graph for precision/diversity. Pattern to copy: *cheap approximate proposal + exact
  local projection*.
- **NLP Sampling: Combining MCMC and NLP Methods for Diverse Constrained Sampling** — Toussaint,
  Braun, Ortiz-Haro, 2024. https://arxiv.org/abs/2407.03035 (verified in briefing 02). The
  no-learning fallback: MCMC (hit-and-run) interleaved with Gauss-Newton projection for *diverse*
  constraint-satisfying samples.
- **Factored Task and Motion Planning with Combined Optimization, Sampling and Learning** —
  Ortiz-Haro, PhD thesis, 2024. https://arxiv.org/abs/2404.03567 — synthesis of the above three;
  the best single reading on "sample vs. optimize" trade-offs in LGP-style keyframe problems.

**Answer to the headline question (what is sampled vs. optimized in the LGP line):** the *skeleton*
is searched discretely; *keyframes* are optimized (MBTS `Pseq`) or, when multimodal, sampled via
constraint-graph / generative / MCMC methods with projection; *paths* are always optimized last and
only for skeletons whose keyframe problem is feasible. Uniform C-space sampling appears nowhere.

---

## 3. Sampling ON constraint manifolds — beyond what briefing 02 covered

Briefing 02 §3c already verified the taxonomy (Kingston–Moll–Kavraki Annu. Rev. 2018), CBiRRT
(ICRA 2009) and TSR (IJRR 2011). New material:

### 3a. IMACS and the projection/continuation/atlas trichotomy
- **Exploring Implicit Spaces for Constrained Sampling-Based Planning (IMACS)** — Kingston, Moll,
  Kavraki, IJRR 38(10–11), 2019. https://journals.sagepub.com/doi/10.1177/0278364919868530
  Reframes constrained planning so the **planner** and the **constraint-adherence method** are
  orthogonal choices: any sampling-based planner works if two primitives respect the implicit
  manifold — (i) constrained *sampling*, (ii) constrained *local motion* (geodesic interpolation
  substitute). Crucially for us, IMACS **preserves probabilistic completeness and asymptotic
  optimality** through these primitives. Methods compared:
  - **Projection** (Newton/Gauss-Newton to the manifold) — robust default, per-sample cost.
  - **Atlas / continuation** — build local charts (tangent polytopes) incrementally:
    **AtlasRRT** — Jaillet & Porta, ISRR 2011 ("Path Planning with Loop Closure Constraints Using
    an Atlas-Based RRT", https://www.semanticscholar.org/paper/05b2564b68a9b2add79ca15c7b0e29abc0088961)
    and asymptotically-optimal follow-up — Jaillet & Porta, RSS 2012,
    https://www.roboticsproceedings.org/rss08/p19.pdf. Cheaper per-sample after warm-up, but chart
    bookkeeping only pays on smooth equality manifolds of substantial codimension.
  - **Reparameterization**: when the manifold has an explicit parameterization, sample the
    parameters directly — strictly dominant when available. **Our jointed objects (prismatic /
    revolute) are already in this regime** — the factor's coordinate *is* the manifold chart. No
    projection machinery needed for them, ever.
- Follow-ups on the *multi-modal* (mode-switching) side of the Kavraki line — directly on-topic
  because rearrangement is a multi-modal problem, one mode per (object, placement):
  - **Using Experience to Improve Constrained Planning on Foliations for Multi-Modal Problems** —
    Kingston, Chamzas, Kavraki, IROS 2021.
    https://www.kavrakilab.org/publications/kingston2021experience-foliations.pdf
    Each constraint defines a *foliation* (family of parallel manifolds — for us: "object i fixed
    at pose p" foliates the joint space); experience from one leaf transfers to sibling leaves.
  - **Scaling Multimodal Planning: Using Experience and Informing Discrete Search** — Kingston &
    Kavraki, IEEE T-RO 39(1):128–146, 2023.
    https://www.kavrakilab.rice.edu/publications/kingston2022-scaling-mmp.html
    Two mechanisms: (1) **ALEF** — reuse sampling experience across similar modes; (2) a **discrete
    lead** — weights over *mode transitions*, updated by planning successes/failures, bias which
    transition to attempt next. This is a ready-made template for LA-RRT's "which object do I move
    next, and to which region" decision (see §5, cross-cutting).

### 3b. Learned samplers for constraint manifolds & rearrangement placements (2023–2026)
- **Diffusion-CCSP: Compositional Diffusion-Based Continuous Constraint Solvers** — Yang, Mao, Du,
  Wu, Tenenbaum, Lozano-Pérez, Kaelbling, CoRL 2023. https://arxiv.org/abs/2309.00966 ·
  https://diffusion-ccsp.github.io/
  **The closest published system to our buffer-sampling problem.** Represents a continuous CSP as a
  **factor graph over object poses** (constraints: collision-free, containment, qualitative
  relations); trains one diffusion model *per constraint type*; composes their energies to sample
  *joint* placements satisfying novel constraint combinations; integrates into a TAMP planner.
  Generalizes to more objects than trained on. 2D packing/placement tasks included.
- **DiMSam: Diffusion Models as Samplers for Task and Motion Planning under Partial Observability**
  — Fang, Garrett, Eppner, Lozano-Pérez, Kaelbling, Fox, IROS 2024 (Best-Paper finalist).
  https://arxiv.org/abs/2306.13196
  Diffusion models play the role of *individual TAMP samplers/constraints* (on learned latent
  object states); the symbolic TAMP solver still does the multi-step reasoning. Confirms the
  architecture "classical planner outside, learned conditional samplers inside" — the planner keeps
  its completeness story, the sampler only supplies proposals.
- **Adaptive Diffusion Constrained Sampling** for bimanual manipulation, 2025.
  https://arxiv.org/abs/2505.13667 — *existence verified from arXiv listing only; details not
  independently checked.* Diffusion sampling with manifold-projection steps interleaved.
- **Zero-Shot Constrained Motion Planning Transformers Using Learned Sampling Dictionaries**, 2023.
  https://arxiv.org/abs/2309.15272 — *existence verified from listing only.* Learned sampling
  dictionaries for constrained planning without per-constraint retraining.
- Non-learned but directly on "where to park obstacles":
  **Where to Relocate? Object Rearrangement Inside Cluttered and Confined Environments** — Cheong,
  Cho, Lee, Kim, Nam, ICRA 2020. https://arxiv.org/abs/2003.10863 ·
  https://ieeexplore.ieee.org/document/9197485/
  Explicitly plans *placements of removed objects* (not just which to remove) to minimize
  pick-and-place count in confined spaces — an engineered, search-based version of our Design 1;
  reports up to 23% fewer actions vs. baselines. Related recent: **Dynamic Buffers** (tabletop
  rearrangement with stacking, 2025) https://arxiv.org/abs/2509.22828 — *listing-verified only.*

**Reading of this literature for LA-RRT:** for equality manifolds use projection/reparameterization
(largely already have it); for the *buffer* problem the field converged on **factored conditional
sampling over a constraint graph of object poses** — engineered (Cheong, constraint-graph ordering)
or learned (Diffusion-CCSP, DGCS) — always mixed with a classical planner that owns completeness.

---

## 4. Quotient-space planning (Orthey & Toussaint) — the "modulo the other objects" idea

- **Rapidly-Exploring Quotient-Space Trees (QRRT)** — Orthey & Toussaint, ISRR 2019.
  https://arxiv.org/abs/1906.01350 · Springer https://link.springer.com/chapter/10.1007/978-3-030-95459-8_4
  Input: a nested sequence of **admissible projections** onto lower-dimensional quotient spaces
  (e.g. project a manipulator to its base, a rigid body to a point). Trees are grown on *all*
  levels; lower-level (quotient) trees bias sampling of higher levels. Guarantees: QRRT is
  **probabilistically complete** (admissibility is what makes the guidance sound: any feasible
  full-space path must project to a feasible quotient path — the quotient is a *relaxation*, so
  quotient-infeasibility certifies infeasibility, and quotient trees never mislead completeness).
  Reported speedups: **≥ one order of magnitude**, growing with how narrow the environment is.
- **Multilevel Motion Planning: A Fiber Bundle Formulation** — Orthey, Akbar, Toussaint, IJRR
  (published online 2023, print 2024). https://journals.sagepub.com/doi/full/10.1177/02783649231209337
  Formalizes the levels as fiber bundles (local product spaces base × fiber); derives **QRRT\***
  (almost-surely asymptotically optimal) and bundle-restriction/section machinery. Implemented in
  OMPL as the `ompl::multilevel` framework — https://ompl.kavrakilab.org/multiLevelPlanning.html
  (we already build against OMPL; the scaffolding is in our tree).
- **Sparse Multilevel Roadmaps for High-Dimensional Robot Motion Planning** — Orthey & Toussaint,
  ICRA 2021. https://arxiv.org/abs/2011.00832 — roadmap (multi-query) variant.

### 4a. Does the quotient idea apply to a *factored rearrangement* space?

Our space is a product `Q = Q_1 × … × Q_n`. Two candidate quotient structures:

1. **Delete-other-objects quotient** `Q → Q_i` ("plan object i as if alone"). This projection **is
   admissible**: removing movable obstacles only enlarges the free space, so any feasible joint
   path projects to a feasible single-object path in the relaxed world. Consequences:
   - a single-object tree/roadmap per factor (cheap: 3-DoF or 1-DoF) is a **sound relaxation** —
     if object i cannot reach `g_i` even alone, the instance is infeasible (a certificate!);
   - its solution paths give the **swept corridors** that other objects must vacate — exactly the
     input Design 1 in §5 needs;
   - QRRT's mechanism — sample the quotient tree, lift by sampling the fiber (= other objects'
     poses) — is precisely "move object i along its solo corridor while choosing where the rest
     stand". LA-RRT's one-object-at-a-time extension is a *degenerate lift* (fiber held constant).
   - **Caveat:** admissibility makes quotient *guidance* sound, but the quotient solo path may be
     jointly infeasible until blockers move — the lift step is where all the rearrangement
     difficulty lives. QRRT's speedups (narrow passages) transfer; its machinery does not solve the
     ordering/buffer combinatorics for us.
2. **Fix-the-rest foliation** (`Q → Q_i` with the other objects *frozen at current poses*): not a
   single quotient but a **foliation**, one leaf per arrangement of the rest — this is exactly the
   Kingston 2021/2023 multi-modal picture (§3a), and LA-RRT already walks leaf-to-leaf. The
   experience-reuse + weighted-mode-transition ("discrete lead") results apply verbatim.

**Bottom line for Q3 of the task:** quotient-space results say the per-object relaxations are sound,
probabilistically-complete-compatible guidance with order-of-magnitude potential in narrow scenes —
use them as *guides, corridors, and infeasibility certificates*, not as a replacement search.

---

## 5. Recipe: constraint-aware sampling for LA-RRT

Cross-cutting principle (every source in §§2–4 agrees): **keep the uniform sampler and mix biased
proposals in with fixed probability** — a mixture `p·(biased) + (1−p)·(uniform)`, `p` bounded away
from 0 and 1, never anneals away the base measure, so PC is preserved by the standard argument (the
same one already used for `goalBias_` in `GOAL_REGION_AND_COMPLETENESS.md` §3b). All designs below
are proposal distributions inside that mixture; none touches the acceptance logic.

### (a) Goal configurations per object
Already solved for pinned goals (projection `π_G`). Two cheap upgrades:
- **TSR-ify the goal**: allow per-object goal *regions* (pose intervals — e.g. "door angle ≥ 80°",
  "box anywhere in alcove") à la Berenson TSR. Sampling is direct (regions are boxes in the factor's
  chart — reparameterization, §3a); projection `π_G` becomes clamp-to-interval. Strictly enlarges
  the goal measure, removes fake precision that currently forces extra actions.
- **Conditional goal completion**: when sampling the goal tree, instead of drawing non-target poses
  uniformly and rejection-checking, draw them *conditioned on the pinned targets* factor-by-factor
  in a constraint-graph order (Ortiz-Haro ICRA 2021 idea, §2d): sample large/most-constrained
  objects first, each conditioned on already-placed ones. Cost: same per-sample order as now with
  far fewer rejections in cluttered scenes. Completeness: unchanged (goal-tree seeding argument
  needs only that the sampler has support on all of `G` — conditional factor sampling with
  full-support per-factor proposals satisfies it).

### (b) Useful buffer / intermediate placements — three candidate designs

**Design 1 — Swept-corridor-complement buffer sampler (engineered; build this first).**
*Proposal:* maintain, per object `i`, the solo corridor `S_i` = inflated swept volume of object i's
current best relaxed path (from the delete-other-objects quotient guide of §4a, or from the current
goal-tree/start-tree best partial path). When LA-RRT decides to extend by moving object `j`, with
probability `p_buf` sample `q_j` uniformly from `free(Q_j) \ ⋃_{i pending} S_i` (grid-rasterize the
corridors once per replan; rejection inside cells is exact) — i.e. *park j out of everyone's way* —
optionally Gauss-Newton-slide out of residual overlap (DGCS pattern: coarse proposal + local
projection).
*Cost:* corridor rasterization O(grid) per guide-path update; per-sample cost ≈ uniform sampling.
No learning, no new solver.
*Completeness:* preserved (mixture). *Expected effect:* directly attacks the buffer-measure problem
— the "useful" set is sampled at rate `p_buf` instead of its natural (tiny) measure; this is the
sampling-level version of Cheong et al.'s placement search and of "minimize running buffers"
heuristics (briefing 03). Also gives better goal-tree non-target poses for free (use it in (a)).

**Design 2 — Keyframe constraint-graph solver, MBTS-`Pseq` style (optimization; medium effort).**
*Proposal:* for a candidate skeleton (ordering of moves from the dependency graph of briefings
01/03, or from LA-RRT's current best plan during defrag), pose the **keyframe problem**: variables =
placements of each moved object at each action boundary; constraints = pairwise disjointness, each
placement outside the swept corridors of *later* moves, containment in workspace, joint limits.
Solve it as a small NLP with random restarts, or with NLP-Sampling (hit-and-run + projection,
arXiv:2407.03035) when multimodal. Feed the resulting keyframe states into LA-RRT as (i) sampling
targets and (ii) seed states for the goal/start trees; the RRT connects them (LGP's division of
labor with the roles of optimizer and planner adapted: keyframes solved, motion sampled).
*Cost:* per-skeleton NLP over ~(#moves)·3 variables — milliseconds at our scale; the skeleton
enumeration is the expensive part and is *already* on the roadmap (Option A of briefing 02 §5).
*Completeness:* preserved (seeds/bias only). *Extra payoff:* an infeasible keyframe problem for
*every* ≤k-move skeleton is evidence (and with exact CSP solving, a certificate) that more than k
actions are needed — the lower-bound half of AGENDA Q5.

**Design 3 — Learned factored placement sampler, Diffusion-CCSP style (only at scale).**
*Proposal:* train per-constraint-type diffusion (or simpler CVAE) models — `collision-free(i,j)`,
`out-of-corridor(i, S)`, `in-region(i, R)` — over our procedurally generated 2D scenes; compose on
the placement factor graph to propose joint buffer assignments conditioned on scene occupancy;
always finish with the Design-1 projection/slide and a hard validity check.
*Cost:* dataset generation (we have the pipeline), training, and maintenance — days, not hours;
inference ~ms.
*Completeness:* preserved (mixture + hard check). *When justified:* only if Designs 1–2 plateau
*and* we run large scene distributions where amortization pays; the literature (DiMSam,
Diffusion-CCSP) is consistent that the learned part must stay a proposal inside a classical
planner, which is exactly our mixture architecture.

**Cross-cutting: quotient guides + discrete lead (cheap, orthogonal, do alongside Design 1).**
Maintain the per-object solo trees/paths of §4a permanently: they (i) supply Design 1's corridors,
(ii) certify per-object infeasibility, (iii) provide a lower bound on per-object path existence for
AGENDA Q5, and (iv) drive a Kingston-style **discrete lead**: keep weights over "(move object j)
next" transitions, boosted when extensions succeed / decayed on failure, and use them to pick the
extension factor instead of uniform factor choice. T-RO 2023 reports this lead + experience reuse
as the difference-maker at scale in multi-modal planning.

### Recommended order
1. Design 1 + quotient guides + discrete lead (no new dependencies; directly targets the measured
   `three_alcoves`-type gap).
2. Design 2 once the dependency-graph/skeleton layer (briefing 02 Option A) exists — it doubles as
   the lower-bound machinery.
3. Design 3 only if 1–2 plateau on large scene suites.

---

## 6. Verification summary

**Verified (publisher/arXiv page confirmed, title+authors+venue match):**
- Toussaint, *LGP*, IJCAI 2015 (re-verified; also in briefing 02).
- Toussaint & Lopes, *Multi-Bound Tree Search for LGP*, ICRA 2017 — IEEE 7989464; bound hierarchy
  (`P1`/pose, `Pseq`, `Ppath`) cross-confirmed via R-LGP (arXiv:2310.02791) and D-LGP
  (arXiv:2312.02731), both of which exist.
- Toussaint, Harris, Ha, Driess, Hönig, *Sequence-of-Constraints MPC*, IROS 2022 — arXiv:2203.05390,
  IEEE 9982236.
- Ortiz-Haro et al., *Constraint Graph Sampling*, ICRA 2021 (arXiv:2011.04828); *DGCS*, CoRL 2021
  (PMLR v164); *NLP Sampling* 2024 (arXiv:2407.03035); Ortiz-Haro PhD thesis (arXiv:2404.03567).
- Kingston, Moll, Kavraki, *IMACS*, IJRR 2019 — SAGE DOI 10.1177/0278364919868530; PC and AO
  preservation claims confirmed from abstract/lab page.
- Kingston, Chamzas, Kavraki, *Experience on Foliations*, IROS 2021; Kingston & Kavraki, *Scaling
  Multimodal Planning* (ALEF + discrete lead), IEEE T-RO 39(1):128–146, 2023.
- Jaillet & Porta, *Atlas-based RRT*, ISRR 2011 and *Asymptotically-Optimal Path Planning on
  Manifolds*, RSS 2012 (roboticsproceedings p19).
- Orthey & Toussaint, *QRRT*, ISRR 2019 (arXiv:1906.01350; Springer chapter) — PC + ≥10× speedup
  claims taken from the abstract. Orthey, Akbar, Toussaint, *Multilevel Motion Planning: A Fiber
  Bundle Formulation*, IJRR 2023/24 (SAGE 02783649231209337). Orthey & Toussaint, *Sparse
  Multilevel Roadmaps*, ICRA 2021 (arXiv:2011.00832). OMPL `multilevel` framework page confirmed.
- Yang et al., *Diffusion-CCSP*, CoRL 2023 (arXiv:2309.00966, PMLR v229, project page).
- Fang et al., *DiMSam*, IROS 2024 (arXiv:2306.13196; NVIDIA SRL page; best-paper-finalist claim
  from the NVIDIA page).
- Cheong et al., *Where to Relocate?*, ICRA 2020 (arXiv:2003.10863, IEEE 9197485).
- Berenson CBiRRT/TSR, Kingston-Moll-Kavraki 2018 review — verified previously in briefing 02.

**Flagged / partially verified:**
- *Adaptive Diffusion Constrained Sampling* (arXiv:2505.13667) and *Zero-Shot Constrained MPT
  Sampling Dictionaries* (arXiv:2309.15272): existence confirmed from arXiv listings only; venue
  and details not independently checked.
- *Dynamic Buffers* (arXiv:2509.22828): listing-verified only.
- MBTS bound-level details (`P1`/`Pseq`/`Ppath` semantics): the ICRA 2017 PDF itself did not parse
  in fetch; semantics reconstructed from search snippets of the paper plus two independent
  secondary sources (R-LGP, D-LGP). Treat exact notation as paraphrase, substance as confirmed.
- The admissibility argument for the "delete-other-objects" quotient in §4a is **our own
  reasoning** applied to Orthey's definitions, not a claim made in the QRRT papers — flagged so it
  is not cited as literature.
