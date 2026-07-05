# Research track: monotone vs non-monotone rearrangement, optimality, and intermediate states

This directory collects the research agenda and literature briefings for the LA-RRT 2D
rearrangement project. It is a *thinking/notes* space, separate from the runnable pipeline in
`demos/larrt2d/`. Pick it up any time — each file is self-contained.

> **Picking this project up (human or AI agent)? Start with [`HANDOFF.md`](HANDOFF.md)** — it
> holds the complete state (findings, certified results, open gaps, bugs), the paper plan, and
> step-by-step continuation instructions for every work stream (benchmark generators, SW-UCB
> bandit layer, normalization, LB strengthening), including the subagent-delegation policy.

## Index

| File | What it is | Status |
|------|-----------|--------|
| [`AGENDA.md`](AGENDA.md) | The research questions, framing, and work plan (source of truth for *why*) | authored |
| [`01_monotone_vs_nonmonotone.md`](01_monotone_vs_nonmonotone.md) | When does a puzzle *require* intermediate states? Definitions, a-priori detection, complexity, literature | from research agent |
| [`02_optimality_and_sampling.md`](02_optimality_and_sampling.md) | Optimality guarantees; constraint/manifold ("modulo") sampling; GCS / convex-set methods | from research agent |
| [`03_complexity_scaling.md`](03_complexity_scaling.md) | How difficulty grows with #objects needing buffers; running-buffer results | from research agent |
| [`04_urdf_to_2d.md`](04_urdf_to_2d.md) | Projecting 3D URDF manipulation puzzles into the 2D scene schema | from research agent |
| [`05_gcs_deep_dive.md`](05_gcs_deep_dive.md) | GCS deep dive: WAFR 2026 shortest-walks paper, certifiability limits, small-n oracle plan | from research agent |
| [`06_constraint_sampling_deep_dive.md`](06_constraint_sampling_deep_dive.md) | LGP/MBTS keyframe bounds, manifold & quotient-space sampling, 3 concrete sampler designs for LA-RRT | from research agent |
| [`07_bandit_selection.md`](07_bandit_selection.md) | Multi-armed bandits for factor/sampler selection: landscape, non-stationary theory (SW-UCB), 5 reward candidates + recommended design | from research agent |
| [`08_factor_spaces.md`](08_factor_spaces.md) | Heterogeneous factors (revolute vs prismatic vs SE(2)): metric/normalization theory, workspace-displacement metrics, LA-RRT recommendations | from research agent |
| [`09_embodied_contact_manipulation.md`](09_embodied_contact_manipulation.md) | **Future paper (idea/parking-lot):** objects don't move themselves — a mobile cube + simple revolute-stick arm must reach, contact, and push them; reachability + contact-maintenance-during-transport coupling; open questions Q-E1..5 | authored; related work verified (subagent + chair) |
| [`09b_embodied_prototype_blueprint.md`](09b_embodied_prototype_blueprint.md) | Companion **implementation blueprint** for the `09` §8 minimal experiment (robot=SE(2)+2R, push transfer primitive, 2R IK reach test, transit/transfer modes); grounds the route-1 "bypass the single-group core" decision in verified `file:line` references | design accepted; **Python reference prototype built & validated** (`tools/embodied_prototype.py`); C++/OMPL version not yet |
| [`SCENARIOS.md`](SCENARIOS.md) | Catalogue of the non-monotone / multi-object test scenes we build | authored as we build |

**Tooling:** `../tools/monotonicity.py` — a-priori monotone/non-monotone classifier + Q5 bracket.
v2 sound lower bound: endpoint arcs + **disconnection arcs** (shrunk-OBB grid proof for box movers,
interval sweep for 1-DOF movers) → SCCs → min-cost FVS → certificate when LB = planner count.
**7/10 solvable scenes certified optimal**; open planner gaps: `two_buffer` (8 vs 5),
`three_alcoves` (11 vs 4), `my_scene` (4 vs 2), `door_openstart` (2 vs 1).

## One-paragraph orientation

Our current planner (LA-RRT + `PathDefragmenter`) *finds and simplifies* action sequences but
gives **no guarantee** that the number of actions — or the number of times an object is parked
in an intermediate ("buffer") pose — is minimal. The scientific thread here is: **given a puzzle,
can we say in advance whether it needs intermediate placements at all (non-monotone), how many,
and can a planner certify optimality of that count?** See `AGENDA.md`.
