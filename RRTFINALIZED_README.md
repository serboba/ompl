# RRTFINALIZED Standalone OMPL Integration

This document describes how to build and run the custom RRTFINALIZED planners in the standalone OMPL environment (ompl_iso), without RobowFlex or ROS dependencies.

## Overview

The following custom planners have been transferred from the RobowFlex-integrated workspace (rb_ws) to this standalone OMPL workspace:

| Planner | Description |
|---------|-------------|
| **RRTFINALIZED** | Nested 2-layer MAB (UNIFORM/CYLINDER → UP/DOWN) |
| **RRTFINALIZED_SINGLELAYER** | Single flat 3-arm MAB (UNIFORM, CYLINDER_UP, CYLINDER_DOWN) |
| **RRTDISASSEMBLEDONLY** | Disassembly detection via local rank-of-motion (no explicit goal) |

## Directory Structure

```
ompl_iso/
├── src/ompl/
│   ├── geometric/planners/disassemblyrrt/    # Custom RRT planners
│   ├── base/samplers/sphere/                  # Sphere samplers (Fibonacci, VMF, etc.)
│   └── datastructures/                        # MAB, geometry, statistics
├── config/
│   └── benchmark_baseline.yaml                # Configuration file
├── demos/disassembly/
│   └── RRTFINALIZEDDemo.cpp                   # Standalone demo
├── run_rrtfinalized_demo.sh                   # Convenience script
└── RRTFINALIZED_README.md                     # This file
```

## Prerequisites

- CMake 3.10+
- C++17 compatible compiler
- Boost (serialization, program_options)
- Eigen3
- yaml-cpp

## Building

```bash
# Navigate to the ompl_iso directory
cd /home/serboba/transferompl_ws/src/ompl_iso

# Create build directory
mkdir -p build && cd build

# Configure (enable demos)
cmake .. -DOMPL_BUILD_DEMOS=ON

# Build the demo
make demo_RRTFinalizedDemo -j4
```

## Running the Demo

### Option 1: Using the Wrapper Script (Recommended)

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso

# Run with defaults (RRTFINALIZED, 10 second timeout)
./run_rrtfinalized_demo.sh

# Run with single-layer variant
./run_rrtfinalized_demo.sh --planner singlelayer --timeout 15

# Show help
./run_rrtfinalized_demo.sh --help
```

### Option 2: Running Directly

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso/build/demos

# RRTFINALIZED (nested MAB)
./demo_RRTFinalizedDemo --config ./benchmark_baseline.yaml --timeout 10 --planner rrtfinalized

# RRTFINALIZED_SINGLELAYER (flat MAB)
./demo_RRTFinalizedDemo --config ./benchmark_baseline.yaml --timeout 10 --planner singlelayer
```

## Demo Output

A successful run will show:

```
==================================================
    RRTFINALIZED Standalone Demo
==================================================
[INFO] State space: 6D RealVector (rx, ry, rz, x, y, z)
[INFO] Bounds: rotations [-π, π], translations [-1, 1]
[INFO] Start: (0, 0, 0, 0, 0, 0)
[INFO] Goal:  (0.5, 0.3, 0.2, 0.8, 0.7, 0.6)
...
[RESULT] Planning status: Exact solution
[RESULT] Solution found!
[RESULT] Path length: 4.55981
[RESULT] Path states: 4
...
```

## Configuration

The `benchmark_baseline.yaml` file contains all MAB and sampling parameters. Key settings:

```yaml
# MAB Parameters
mab_uniform_sphere_window_size: 256    # Window size for MAB arm selection
mab_cylinder_direction_window_size: 512  # For nested planners only

# Adaptive Sphere Sampling
adaptive_quasirandom_sample_size: 128
adaptive_start_radius: 1.0
adaptive_min_radius: 0.00001

# Goal Bias
uniform_goal_bias: 0.75
sphere_goal_bias: 0.0000001
```

## Using the Planners in Your Code

```cpp
#include <ompl/geometric/planners/disassemblyrrt/RRTFINALIZED.h>
#include <ompl/geometric/planners/disassemblyrrt/RRTFINALIZED_SINGLELAYER.h>

// Setup space information (si) and problem definition (pdef) as usual...

// Create planner with config path
auto planner = std::make_shared<og::RRTFINALIZED>(si, "/path/to/config.yaml");
planner->setProblemDefinition(pdef);
planner->setup();

// Solve
ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(10.0));
```

## Notes

1. **DEBUGRRT** is not included as it has WebSocket dependencies not available in standalone OMPL.

2. The `sampleSelectedIndices` method was added to `RealVectorStateSampler` to support the custom sampling strategies.

3. All planners use yaml-cpp for configuration loading, which is now a public dependency of the OMPL library.

## Transferred Components

- **Planners**: RRTFINALIZED, RRTFINALIZED_SINGLELAYER, RRTDISASSEMBLEDONLY
- **Samplers**: AdaptiveSphereSampler, CylinderSampler, FibonacciSphereSampler, VMFSampler, UniformSphereSampler
- **Datastructures**: MultiArmedBandits, HammersleySphere, geometry utilities, statistics (VMF, CylinderFitter)

