# Bandit selection deep dive — adaptive factor & sampler choice for LA-RRT

> Context: LA-RRT extends its trees by moving exactly ONE factor (object) per action; the factor is
> currently chosen **uniformly at random** among the factors that differ from the sample
> (`LARRT.cpp:347-352`), and the sample itself comes from a uniform sampler plus the fixed
> `goalBias_` projection (`LARRT.cpp:575-578`, `GOAL_REGION_AND_COMPLETENESS.md` §3b). Two decision
> points are natural bandit arms: **(a) which factor to extend**, **(b) which proposal/sampler to
> draw the target from** (uniform, goal-biased, buffer-region-biased à la briefing 06 Design 1).
> This briefing maps the bandit-in-motion-planning literature, the MCTS/bandit layer of TAMP and
> rearrangement, goal-distance-free progress signals usable as rewards, and non-stationary bandit
> theory — then proposes candidate reward functions and one default design. Extends briefing 06
> (whose §5 "discrete lead" cross-cutting item is exactly decision (a)); does not repeat it.
> The user's own MAB-RRT line (sliding-window bandit over samplers; WAFR 2026 paper below) is the
> starting point — this briefing situates it and builds on it. Citations checked against
> arXiv/DOI/publisher pages; unverified items flagged. Produced by a research agent, 2026-07-04.

---

## 1. The two arms LA-RRT exposes today

From the code:
- **Factor choice.** `growTree` computes `diff` (factors differing between nearest node and
  sample) and picks `g = diff[uniformInt(...)]` — a uniform bandit with no learning
  (`src/ompl/geometric/planners/rrt/src/LARRT.cpp:347-352`). Every extension already produces a
  binary outcome (motion valid / trimmed / rejected) and a moved-factor id — i.e. a **free reward
  stream per factor**, currently discarded.
- **Sampler choice.** One uniform sampler + goal projection with fixed `goalBias_`. Briefing 06 §5
  adds buffer-region proposals (Design 1) and conditional goal completion — once those exist there
  are ≥3 proposal distributions whose usefulness varies over the run: a textbook sampler-selection
  bandit, and precisely the user's MAB-RRT formulation.

The design constraint carried over from briefing 06 §5: whatever the bandit does, keep an
**ε-floor of uniform choice** on both decisions (mixture with fixed ε > 0) so probabilistic
completeness is preserved by the same argument as `goalBias_`. The bandit only reallocates the
remaining (1−ε) mass; it never owns correctness.

---

## 2. Bandits for sampler / expansion selection in sampling-based planning

### 2a. The precursor: adaptive hybrid sampling (2005–2008)

- **Hybrid PRM Sampling with a Cost-Sensitive Adaptive Strategy** — Hsu, Sánchez-Ante, Sun,
  ICRA 2005, pp. 3885–3891. https://ieeexplore.ieee.org/document/1570712/ ·
  https://motion.comp.nus.edu.sg/adaptive-hybrid-sampling/
  The canonical ancestor of sampler-selection bandits. A portfolio of PRM samplers (uniform,
  Gaussian, bridge, …) is combined by **multiplicative weight updates with a cost-sensitive
  reward** (reward for samples that improve roadmap connectivity, discounted by the sampler's
  compute cost) — structurally an adversarial-bandit / expert-weighting (EXP3-family) scheme,
  though the paper predates that framing. Result: the hybrid is **provably competitive against
  every individual strategy** and performs consistently across environments up to 12 DoF. Lesson
  that transfers verbatim: reward = *connectivity contribution per unit cost*, not raw success.
- **Adaptive Workspace Biasing for Sampling-Based Planners** — Zucker, Kuffner, Bagnell,
  ICRA 2008. https://www.ri.cmu.edu/pub_files/pub4/zucker_matthew_2008_1/zucker_matthew_2008_1.pdf
  Learns a weighting over **workspace features** (a continuous-armed cousin of sampler selection)
  with REINFORCE policy gradient; unifies earlier workspace-biasing heuristics. Relevant less for
  the algorithm (offline, gradient) than for the arm design: arms indexed by *workspace regions*
  work — our factors are workspace regions with identities.
- **Burns & Brock** — *Single-Query Entropy-Guided Path Planning*, ICRA 2005, and *Single-Query
  Motion Planning with Utility-Guided Random Trees*, ICRA 2007
  (https://www.semanticscholar.org/paper/8d1710a3357f94c936c88cbf40bffce1bdc25f7c). Choose the
  next expansion to maximize **expected information gain / utility** under an incrementally
  learned C-space model — the decision-theoretic (Bayesian, non-bandit) end of the same idea:
  score expansions by what they *reveal*, not by goal distance. Their utility signal is a direct
  ancestor of the novelty rewards in §4.

### 2b. Bandit-formulated planners (2023–2026) — the field the user's MAB-RRT sits in

- **Motion Planning as Online Learning: A Multi-Armed Bandit Approach to Kinodynamic
  Sampling-Based Planning** — Faroni & Berenson, IEEE RA-L 8(10):6651–6658, 2023.
  https://arxiv.org/abs/2308.13949 · https://ieeexplore.ieee.org/document/10238731/
  The closest published relative. Sampling bias is formulated **explicitly as a non-stationary
  MAB**: transitions from previous kinodynamic-RRT runs are clustered (HDBSCAN); each cluster is
  an arm; a bandit picks the cluster to draw the next transition from, trading exploitation of
  high-reward regions against under-explored ones. Rewards come from solution quality of past
  runs. Finds better solutions faster with higher execution success. Note the granularity: arms =
  *state-space regions*, not samplers — complementary to sampler-arms.
- **Online Adaptation of Sampling-Based Motion Planning with Inaccurate Models** — Faroni &
  Berenson, 2024. https://arxiv.org/abs/2403.07638 — same group; bandit-style adaptation of the
  sampling distribution when the dynamics model is wrong. *(arXiv listing verified; venue not
  independently checked.)*
- **Scale-Invariant Sampling in Multi-Arm Bandit Motion Planning for Object Extraction** —
  **Bayraktar, Orthey, Toussaint, WAFR 2026** (accepted; arXiv:2604.14026,
  https://arxiv.org/pdf/2604.14026) — *the user's own paper*. Geometric **MAB-RRT**: each
  iteration a bandit (UCB-family) selects which sampler generates the next sample, rewards
  computed from sample validity/usefulness; the paper adds a scale-invariant grow–shrink sampler
  as a new arm and shows order-of-magnitude success-rate gains on eight 3D disassembly scenes.
  For LA-RRT this is the direct template for decision (b); what it does *not* yet address is
  decision (a) (factor choice) and rearrangement-aware rewards — the gap this briefing targets.
- Adjacent, for completeness: **Chamzas, Shrivastava, Kavraki, "Using Local Experiences for
  Global Motion Planning", ICRA 2019** (pp. 8606–8612,
  https://www.kavrakilab.org/publications/chamzas2019using-local-experiences-for-global-motion-planning.pdf)
  — a *database* of local samplers selected by workspace-primitive matching (retrieval, not
  bandit); and OMPL's experience planners **Lightning** (Berenson, Abbeel, Goldberg, ICRA 2012)
  and **Thunder** (Coleman et al., 2014, https://arxiv.org/abs/1410.1950), which run
  retrieve-and-repair *in parallel with* plan-from-scratch rather than selecting between them —
  explicitly **not** bandit-formulated; a bandit over {retrieve, scratch} is an obvious untaken
  extension. Kingston & Kavraki's **discrete lead** (T-RO 2023, verified in briefing 06 §3a) is
  the multi-modal-planning instance of decision (a): weights over mode transitions updated by
  success/failure — a bandit in all but name (no confidence bounds, no regret analysis).

**Landscape summary:** sampler/region-selection bandits in SBMP are a small but real line —
multiplicative-weights hybrids (2005), one non-stationary-MAB formulation with regret framing
(Faroni–Berenson 2023), and the user's sampler-arm MAB-RRT (WAFR 2026). Nobody has published a
bandit over **factors of a factored C-space** with rearrangement-aware rewards. That combination
appears to be open.

---

## 3. Bandit / MCTS selection in TAMP and rearrangement ("which object next")

- **MCTS for Efficient Visually Guided Rearrangement Planning** — Labbé, Zagoruyko, Kalevatykh,
  Laptev, Carpentier, Aubry, Sivic, IEEE RA-L 5(2):3715–3722, 2020.
  https://arxiv.org/abs/1904.10348 · https://ylabbe.github.io/rearrangement-planning/
  Tree nodes = arrangements, edges = single-object moves; **UCT decides which object to move
  next**. Scales to 25 objects at ~60 ms/plan and finds shorter move sequences than baselines.
  Their reward is task-level (arrangement progress after rollout) — works because their moves are
  abstract pick-and-places; our low-level tree cannot afford rollouts per extension, hence the
  cheap per-extension rewards of §5.
- **Multi-Stage MCTS for Non-Monotone Object Rearrangement in Narrow Confined Environments** —
  Ren & Qureshi, 2023 (v3 2024). https://arxiv.org/abs/2305.17175
  UCT over object moves in *confined, non-monotone* scenes (our regime). Key mechanism: an
  **object stage topology** decomposes the problem, and tree expansion is *subgoal-focused* —
  buffer placements are first-class actions, so "move away from goal" is rewarded via stage
  progress rather than goal distance. Closest published answer to our reward-design problem at
  the task level.
- **Multi-Agent MCTS for Makespan-Efficient Object Rearrangement (CAM-MCTS)**, 2026.
  https://arxiv.org/abs/2602.02411 — *listing-verified only.* Confirms UCT-over-object-moves is
  the current default at the task layer.
- **Extended Tree Search for Robot Task and Motion Planning** — Ren, Chalvatzaki, Peters
  (arXiv:2103.05456; IEEE conference publication 2024, https://arxiv.org/abs/2103.05456).
  Top-k task planning generates a **skeleton space**; skeleton selection is "a bandit as an
  extended root" of a single UCT tree that also binds continuous motion variables — i.e. UCB at
  the skeleton level, exactly the "UCT over task skeletons" pattern asked about.
- **Learning to Guide TAMP Using Score-Space Representation** — Kim, Wang, Kaelbling,
  Lozano-Pérez, IJRR 38(7), 2019. https://journals.sagepub.com/doi/10.1177/0278364919848837
  Represents a problem instance by the **vector of scores of solutions attempted so far** and
  uses it to transfer constraints from past instances — a contextual-bandit view of "which plan
  fragment to try next" (black-box function optimization with UCB-style acquisition in the
  original conference version).
- **PDDLStream** — Garrett, Lozano-Pérez, Kaelbling, ICAPS 2020.
  https://ojs.aaai.org/index.php/ICAPS/article/view/6739 — its **Adaptive** algorithm explicitly
  "dynamically balances exploring new candidate plans and exploiting existing ones" (search vs.
  sampling budget split) — the same exploration/exploitation dial, solved with a fixed ratio
  rather than a bandit. **Khodeir, Agro, Shkurti, "Learning to Search in TAMP with Streams",
  RA-L 2023** (https://arxiv.org/abs/2111.13144) replaces the breadth-first object/fact expansion
  with best-first ordering by a learned GNN — learned priors where a bandit would use online
  rewards.
- Object-selection policies without bandits (context for arm design): Stilman-style NAMO
  backward-chaining picks the objects intersecting the target's swept volume (briefing 01);
  **ClutterNav** (2025, https://arxiv.org/abs/2511.12479, *listing-verified only*) learns a
  removability critic for "next best object to remove". Both effectively rank objects by
  **de-blocking value** — the quantity our reward R3/R4 below estimates online for free.

**Takeaway:** at the task layer, UCT/UCB over "which object to move next" is established and works
in exactly our non-monotone confined setting (MS-MCTS). Inside a *sampling-based tree planner*,
the same decision is only handled by Kingston's heuristic discrete lead. Porting the UCT-style
selection down into LA-RRT's extension loop, with per-extension (not per-rollout) rewards, is the
novel, simple thing to build.

---

## 4. Progress signals that do not use goal distance

Rearrangement rewards must credit actions that *increase future options* (open door, vacate
corridor, park in buffer) even when they increase goal distance. The classical exploration
literature supplies exactly such signals:

- **Coverage / cell novelty (KPIECE).** Şucan & Kavraki, *Kinodynamic Motion Planning by
  Interior-Exterior Cell Exploration*, WAFR 2008
  (https://www.kavrakilab.org/publications/sucan-kavraki2009kinodynamic-motion-planning.html) and
  the T-RO version (*A Sampling-Based Tree Planner for Systems with Complex Dynamics*,
  https://files.sucan.ro/ioan/files/pubs/kpiece_preprint.pdf). Progress = **discovering new cells
  or expanding the exterior (boundary) cells** of a grid over a low-dimensional *projection* of
  the state space; cell "importance" combines discovery iteration, visit count, and neighborhood
  saturation. No goal term anywhere — the planner is driven purely by coverage deficit. Directly
  liftable per factor: each factor already lives in a 1–3-D chart, which *is* the projection.
- **Expansiveness / visibility.** Hsu, Latombe, Motwani, *Path Planning in Expansive
  Configuration Spaces*, ICRA 1997; journal version IJCGA 9(4–5):495–512, 1999
  (https://ai.stanford.edu/~latombe/pub.htm). EST weights nodes inversely to local sample
  density; the theory measures progress as **growth of the visibility/lookout set** of the
  explored region. Same family: reward = "this extension enlarged what the tree can see".
- **Information gain / model entropy.** Burns & Brock (§2a): reward = reduction in uncertainty of
  a learned free-space model. Elegant but needs a C-space model we don't maintain — skip.
- **Frontier growth (workspace version).** Yamauchi, *A Frontier-Based Approach for Autonomous
  Exploration*, IEEE CIRA 1997, pp. 146–151 (https://dl.acm.org/doi/10.5555/523996.793157).
  Progress = pushing the boundary between known-open and unknown. Our flood-fill reward R3 is a
  frontier measure on the *target object's reachable free space* rather than on a robot's map.
- **Width / novelty pruning (classical planning).** Lipovetzky & Geffner, *Width and
  Serialization of Classical Planning Problems*, ECAI 2012
  (https://ebooks.iospress.nl/volumearticle/7029). A state is novel if it makes some small tuple
  of atoms true for the first time; IW(k) explores only novel states and solves most benchmarks
  at k ≤ 2. The discrete twin of KPIECE-cell novelty, and the right way to think about factored
  novelty: **novelty of small factor-value tuples** (e.g. "door open ∧ box-A in corridor" seen
  for the first time) captures *joint* progress that per-factor cells miss, at O(#pairs) cost.
- RL-style count-based/pseudo-count intrinsic rewards are the same idea in function-approximation
  clothing; nothing there beats the KPIECE/IW forms for our discrete-grid setting, so we do not
  import that literature.

**Common structure:** all of these are *diminishing* signals — each arm's novelty payoff decays as
its region saturates. That is a feature (it automatically rotates attention across factors) but it
means the reward process is non-stationary **by construction**, independent of world events. Which
is why §5 matters.

---

## 5. Non-stationarity: why rearrangement breaks stationary bandits, and what handles it

Why our reward processes are non-stationary, concretely: (i) **world events reprice every arm** —
while the door is shut, extensions of the trapped box's factor almost always fail (reward ≈ 0) and
door-factor extensions earn novelty/unblocking reward; the moment some branch opens the door, the
box factor's success and connectivity-gain rewards jump while the door factor's collapse to zero.
One extension can flip the identity of the best arm. (ii) **Saturation** — every novelty-type
reward decays as a factor's chart fills (§4). (iii) **Tree growth** — success probability of an
arm depends on the current tree, which the bandit itself changes (rewards are neither i.i.d. nor
exogenous). Stationary UCB1 provably over-commits here: its confidence intervals shrink
permanently, so after the door opens it keeps pulling the stale door arm for Ω(log T) rounds per
unit of accumulated advantage — in practice for a long time. The three standard remedies:

**SW-UCB (sliding-window UCB).** Garivier & Moulines, *On Upper-Confidence Bound Policies for
Switching Bandit Problems*, ALT 2011 (Springer LNCS,
https://link.springer.com/chapter/10.1007/978-3-642-24412-4_16; arXiv:0805.3415). UCB computed
only over the last τ pulls of each arm. For abruptly-changing environments with Υ_T breakpoints,
SW-UCB with window τ ≈ 2√(T log T / Υ_T) achieves expected regret **O(√(Υ_T · T) · log T)**,
matching the Ω(√(Υ_T · T)) lower bound up to log factors. Behavior: after a breakpoint the stale
arm's window empties of wins within ~τ pulls and the bandit re-explores. Cost: a deque of the last
τ (arm, reward) pairs — trivial. This is what the user's MAB-RRT line already uses; the theory
says it is the right default when changes are *abrupt and few* (door opens: yes).

**D-UCB (discounted UCB).** Same paper (D-UCB is originally due to Kocsis & Szepesvári's
discounted-UCB proposal, analyzed by Garivier–Moulines). Empirical means and counts decay
geometrically with factor γ; with γ = 1 − (1/4)√(Υ_T/T) the regret is **O(√(Υ_T · T) · log T)**
(a √log factor worse than SW-UCB in their bounds). Exponential forgetting reacts *smoothly* —
better matched to gradual drift (our saturation effect (ii)) than to jumps; slightly cheaper than
a window (two floats per arm, no deque). SW-UCB and D-UCB are near-interchangeable in practice;
pick by memory taste.

**Rexp3 / variation-budget theory.** Besbes, Gur, Zeevi, *Stochastic Multi-Armed-Bandit Problem
with Non-Stationary Rewards*, NeurIPS 2014
(https://papers.nips.cc/paper/5378-stochastic-multi-armed-bandit-problem-with-non-stationary-rewards);
journal version in Stochastic Systems 2019 (https://pubsonline.informs.org/doi/10.1287/stsy.2019.0033).
Instead of counting breakpoints, allow total drift ≤ V_T (variation budget). Minimax regret is
fully characterized as **Θ((K·V_T)^{1/3} · T^{2/3})**, achieved by **Rexp3** = EXP3 restarted from
scratch every Δ_T = ⌈(K log K)^{1/3}(T/V_T)^{2/3}⌉ rounds. Note the T^{2/3}: continuous drift is
fundamentally harder than a few switches — a reason to *prefer* reward designs whose changes are
event-like (R3/R4 below) over smoothly drifting ones.

**EXP3 / EXP3.S (adversarial).** Auer, Cesa-Bianchi, Freund, Schapire, *The Nonstochastic
Multiarmed Bandit Problem*, SIAM J. Computing 32(1):48–77, 2002
(https://cesa-bianchi.di.unimi.it/Pubblicazioni/J18.pdf). EXP3: regret **O(√(T·K·log K))** against
the best *fixed* arm with no stochastic assumptions at all — immune to the "rewards depend on the
bandit's own past actions" objection (iii) above. **EXP3.S** tracks the best *sequence* of arms
with S switches at regret **O(√(S·K·T·log(KT)))**. The safe choice if we distrust our reward's
statistical behavior; the price is more exploration noise than SW-UCB at our small K (K = #factors
≤ ~6, or #samplers ≤ ~4), where SW-UCB's sharper exploitation wins.

**Verdict for LA-RRT:** K is tiny, changes are mostly abrupt and event-like, rewards are bounded
in [0,1] by construction (§6) → **SW-UCB** is theory-matched and matches the user's prior work;
keep EXP3.S in the back pocket if reward hacking (§6 failure modes) ever manifests as an
adversarial-looking reward stream.

---

## 6. Candidate reward functions for LA-RRT

All rewards below are per-extension, bounded in [0,1], and credited to the (factor-arm, sampler-arm)
pair that produced the extension. `1{·}` is the indicator.

**R1 — Extension success rate.** r = 1{motion valid and node added} (optionally scaled by
advance distance / maxDistance_).
*Cost:* zero — the outcome already exists at `LARRT.cpp:386-401`. *Failure modes:* rewards
free-space wiggling: a factor with a huge empty chart earns forever while contributing nothing;
says nothing about usefulness. *Stationarity:* mildly drifting (success decays as the local region
fills; jumps when blockers move). *Role:* necessary hygiene term (kills arms that only collide),
never sufficient alone.

**R2 — Per-factor cell novelty (KPIECE-style).** Maintain per factor i a coarse grid over its own
chart (2-D for prismatic boxes, 3-D coarse for SE(2), 1-D for door/slider — 16–64 cells per dim).
r = 1{the new node lands in a cell never before visited *by this tree*}; optionally the softer
KPIECE score 1/(1+visits(cell)).
*Cost:* one hash per extension; ~KB memory. *Failure modes:* novelty ≠ usefulness — happily parks
objects in cells that block future corridors; big factors have more cells and thus a structural
advantage (normalize by factor cell count); exhausts (reward → 0) once a chart is covered — which
is *correct* behavior (stop hammering a saturated door axis) but means late-run rewards go silent.
*Stationarity:* monotonically decaying per arm → exactly what SW/D-forgetting handles. *Note:* an
IW(2)-style upgrade — novelty of (factor-i-cell, factor-j-cell) *pairs* — captures joint progress
("door open ∧ box in corridor") for O(#pairs) memory; keep as a v2 option.

**R3 — Connectivity gain of the target's free space (flood-fill delta).** Precompute a coarse
occupancy grid of the workspace (we already rasterize scenes for the corridors of briefing 06
Design 1). After an extension that moves factor j, re-rasterize only j's footprint, erode by the
target object's inradius, and flood-fill from the target's current cell: r = clip(Δ(reachable area
of the target's component)/A_norm, 0, 1), or the sharper binary r = 1{target's cell and its goal
cell become connected}.
*Cost:* O(grid) flood fill (64×64 → microseconds); compute lazily only when the moved factor's
footprint intersects a corridor mask, else r = 0 at O(1). *Failure modes:* resolution — a legal
narrow passage can read as closed (erosion by inradius is conservative for non-circular boxes; use
the OBB's *width* along the corridor if this bites); rewards opening *any* connection, including
ones the final plan won't use; gives no credit for the second half of non-monotone plans (closing
the door back earns 0 or negative — do **not** penalize reconnection loss, clip at 0). *Stationarity:*
event-like jumps — the friendliest possible shape for SW-UCB. This is the one reward that
*directly* pays for moving away from the goal.

**R4 — Dependency-graph progress.** Maintain the blocking graph of briefings 01/03: arc (j → i) if
object j's footprint intersects corridor S_i (the rasterized solo-path corridor of factor i from
briefing 06's quotient guides). r = 1{this extension *removes* an arc} (j's new pose exits some
S_i it previously occupied), evaluated by O(1) lookups in the precomputed corridor masks.
*Cost:* O(#pending corridors) mask lookups per extension; corridors already exist in the Design 1
plan. *Failure modes:* corridors come from *relaxed solo paths* — wrong homotopy class ⇒ the graph
rewards clearing a corridor the true solution never uses (mitigate: refresh corridors from the
current best partial path); in non-monotone instances some arcs **must** be re-violated (buffer
swap), so an agent maximizing R4 greedily can stall — the bandit only *biases* extensions, and the
window forgets, so this degrades gracefully rather than deadlocking; reward hacking: oscillating in
and out of a corridor farms r — credit each arc removal **once per (arc, tree) pair**.
*Stationarity:* piecewise-constant with a small number of jumps — ideal for SW-UCB.

**R5 — Gated goal progress.** r = clip(Δ(−goal distance of factor j)/maxDistance_, 0, 1), but
**only** counted when all in-arcs of j in the current blocking graph are cleared (its corridor is
free); r = 0 otherwise.
*Cost:* O(1) given R4's bookkeeping. *Failure modes:* inherits the graph's errors (a wrongly
"cleared" factor gets goal reward while actually blocked — wasted pulls until the window forgets);
re-lock tasks work naturally (closing the door is goal progress *for the door factor* once its own
path is clear), but an object that must *leave* its goal temporarily earns 0 for doing the right
thing — R3/R4 must carry that case. *Stationarity:* gate openings are jumps; within a regime,
near-stationary. This is the *only* place goal distance should ever appear in the reward.

### Recommended default

**Two-level SW-UCB, one simple composite reward, ε-floors everywhere:**

- **Arms.** Level 1: which factor to extend — restricted to `diff` (the factors differing from the
  sample), i.e. run SW-UCB over all factors but argmax only over `diff`, replacing the uniform pick
  at `LARRT.cpp:352`. Level 2 (independent bandit, shared reward): which proposal generated the
  sample — {uniform, goal-projected, buffer-region (Design 1)} — the direct port of the user's
  MAB-RRT sampler arms.
- **Reward.** r = 0 if the extension failed; else **r = max(R2_binary, R4, R5)** (all already in
  {0,1} / [0,1]). Rationale: `max` keeps r in [0,1] with no weight tuning; R2 keeps early
  exploration alive when no corridor events fire; R4 pays for de-blocking (the move-away-from-goal
  actions); R5 pays for cashing in once unblocked; R1 is subsumed (failed ⇒ 0). Defer R3's flood
  fill to v2 — R4 approximates it at O(1) using masks we already build, and R3 is the fallback if
  the blocking graph proves too wrong in practice.
- **Bandit.** SW-UCB, window τ ∈ [100, 300] extensions (our runs are 10⁴–10⁵ iterations with a
  handful of regime changes; τ ≈ √(T/Υ_T) lands in this range), UCB bonus c·√(log min(t,τ)/N_τ(a))
  with c = √2; ε-floor 0.1 uniform on both levels (PC preserved by the mixture argument,
  briefing 06 §5). Per-tree bandit state (start and goal trees see different regimes).
- **Why this and not something smarter:** every quantity is O(1) at extension time given artifacts
  briefing 06 already schedules (corridor masks, per-factor charts); K ≤ ~6 arms makes regret
  constants irrelevant — engineering simplicity and non-stationarity handling dominate, which is
  exactly SW-UCB; the composite is monotone in "did something objectively new or de-blocking
  happen", which is hard to hack given the once-per-arc crediting; and it degrades to today's
  planner (uniform everything) as ε → 1, so it can ship behind a flag and be A/B-benchmarked on
  the §SCENARIOS suite (buffer_swap, door re-lock, three_alcoves are the discriminating cases —
  uniform factor choice currently wastes most pulls there on already-saturated factors).

---

## 7. Verification summary

**Verified (publisher/arXiv/DOI page confirmed; title+authors+venue match):**
- Hsu, Sánchez-Ante, Sun, *Hybrid PRM Sampling with a Cost-Sensitive Adaptive Strategy*, ICRA 2005
  — IEEE 1570712 + NUS AdaComp page (pp. 3885–3891, competitiveness claim from abstract).
- Zucker, Kuffner, Bagnell, *Adaptive Workspace Biasing for Sampling-Based Planners*, ICRA 2008 —
  CMU RI page (REINFORCE-based, feature weighting confirmed).
- Burns & Brock, *Single-Query Entropy-Guided Path Planning* (ICRA 2005) and *Utility-Guided
  Random Trees* (ICRA 2007) — Semantic Scholar records; entropy/utility framing confirmed.
- Faroni & Berenson, *Motion Planning as Online Learning: A MAB Approach to Kinodynamic
  Sampling-Based Planning*, IEEE RA-L 8(10):6651–6658, 2023 — arXiv:2308.13949 + IEEE 10238731;
  non-stationary MAB formulation, HDBSCAN cluster arms confirmed.
- Bayraktar, Orthey, Toussaint, *Scale-Invariant Sampling in Multi-Arm Bandit Motion Planning for
  Object Extraction*, WAFR 2026 (accepted) — arXiv:2604.14026 abstract fetched (the user's paper;
  sampler-as-arm MAB-RRT and order-of-magnitude claim from abstract).
- Chamzas, Shrivastava, Kavraki, *Using Local Experiences for Global Motion Planning*, ICRA 2019,
  pp. 8606–8612 — Kavraki Lab PDF.
- Berenson, Abbeel, Goldberg, *A Robot Path Planning Framework that Learns from Experience*
  (Lightning), ICRA 2012; Coleman et al., *Experience-Based Planning with Sparse Roadmap Spanners*
  (Thunder), arXiv:1410.1950 — both confirmed; "not bandit-formulated" is our reading of their
  parallel-planning architecture, not their claim.
- Labbé et al., *MCTS for Efficient Visually Guided Rearrangement Planning*, RA-L 5(2):3715–3722,
  2020 — arXiv:1904.10348, HAL, project page.
- Ren & Qureshi, *Multi-Stage MCTS for Non-Monotone Object Rearrangement in Narrow Confined
  Environments* — arXiv:2305.17175 (authors confirmed; final venue not confirmed — arXiv only).
- Ren, Chalvatzaki, Peters, *Extended Tree Search for Robot TAMP* — arXiv:2103.05456 + IEEE
  conference record (2024); skeleton-bandit-as-extended-root confirmed from abstract material.
- Kim, Wang, Kaelbling, Lozano-Pérez, *Learning to Guide TAMP Using Score-Space Representation*,
  IJRR 2019 — SAGE DOI 10.1177/0278364919848837.
- Garrett, Lozano-Pérez, Kaelbling, *PDDLStream*, ICAPS 2020 — ICAPS proceedings + dblp; the
  Adaptive algorithm's explore/exploit balancing from the ICAPS page.
- Khodeir, Agro, Shkurti, *Learning to Search in TAMP with Streams*, RA-L 2023 — arXiv:2111.13144.
- Şucan & Kavraki, KPIECE (WAFR 2008 + T-RO preprint) — Kavraki Lab pages; interior/exterior cell
  and coverage-grid mechanics confirmed.
- Hsu, Latombe, Motwani, *Path Planning in Expansive Configuration Spaces*, ICRA 1997 / IJCGA
  9(4–5):495–512, 1999 — Latombe publication list.
- Yamauchi, *A Frontier-Based Approach for Autonomous Exploration*, IEEE CIRA 1997, pp. 146–151 —
  ACM DL record.
- Lipovetzky & Geffner, *Width and Serialization of Classical Planning Problems*, ECAI 2012 — IOS
  Press ebook page; IW(k) novelty definition cross-confirmed.
- Garivier & Moulines, *On Upper-Confidence Bound Policies for Switching Bandit Problems*,
  ALT 2011 — Springer LNCS chapter + arXiv:0805.3415; SW-UCB/D-UCB regret orders and
  "match lower bound up to log factor" from abstract/records.
- Besbes, Gur, Zeevi, *Stochastic MAB with Non-Stationary Rewards*, NeurIPS 2014 + Stochastic
  Systems 2019 — NeurIPS page + INFORMS DOI; variation budget and Θ((K V_T)^{1/3} T^{2/3})
  characterization confirmed; Rexp3 batch length from the paper.
- Auer, Cesa-Bianchi, Freund, Schapire, *The Nonstochastic Multiarmed Bandit Problem*, SIAM J.
  Comput. 32(1):48–77, 2002 — author-hosted PDFs; EXP3/EXP3.S bounds confirmed.
- Kingston & Kavraki discrete lead (T-RO 2023) — verified previously in briefing 06 §3a.

**Flagged / partially verified:**
- Faroni & Berenson, *Online Adaptation of SBMP with Inaccurate Models* (arXiv:2403.07638) —
  listing-verified; venue unchecked.
- *CAM-MCTS* (arXiv:2602.02411, 2026) and *ClutterNav* (arXiv:2511.12479, 2025) —
  listing-verified only.
- Exact SW-UCB/D-UCB window/discount constants (τ ≈ 2√(T log T/Υ_T), γ = 1 − (1/4)√(Υ_T/T)) are
  quoted from memory of Garivier–Moulines and standard surveys, not re-derived from the PDF —
  treat the *orders* as verified, the constants as paraphrase.
- The claim that Hsu 2005's update is "EXP3-family" is our structural reading, not the paper's
  terminology; likewise "bandit in all but name" for Kingston's discrete lead.
- The WAFR 2026 paper's reward function and bandit variant details were not extractable from the
  abstract page — the user knows them first-hand; §6 assumes UCB-family with validity-based
  rewards as summarized there.
