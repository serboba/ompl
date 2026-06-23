# MAB-RRT planner (this OMPL fork)

This branch (`MAB-RRT`) adds the **MAB-RRT** planner to OMPL: a multi-armed-bandit RRT that chooses
its sampler online (a uniform sampler + scale-invariant cylinder samplers). It is used by the
multi-part robot disassembly system in *"Parallel Batch Protocol: Long-Horizon Multi-Part
Disassembly Planning"* (RA-L).

## What it adds (purely additive to upstream OMPL)
- `src/ompl/geometric/planners/rrt/MAB_RRT.{h,cpp}` — the planner
- `src/ompl/datastructures/MultiArmedBandits.h` — the bandit
- `src/ompl/base/samplers/sphere/*` — adaptive sphere / cylinder / Fibonacci / uniform samplers
- `src/ompl/datastructures/geometry/*` — `Point3D` / `Cylinder3D` / `GeometryUtils`
- one additive method, `RealVectorStateSpace::sampleSelectedIndices()`

No existing OMPL files are otherwise modified. The planner is auto-compiled by OMPL's source glob,
and uses `yaml-cpp` (already an OMPL dependency) to read its config.

## How to build & run it

You almost certainly want the **meta-repo**, which builds this in-tree together with the disassembly
pipeline and provides ready-to-run demos:

### → **https://github.com/serboba/pbp-disassembly**

As a standalone OMPL library it builds with the normal OMPL instructions in `README.md`.
Instantiate it like any geometric planner: `ompl::geometric::MAB_RRT(si, yamlConfigPath)`.

The branch is kept clean and additive for a possible future contribution to upstream OMPL.
