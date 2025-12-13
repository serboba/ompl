# MAB-SSRRT Demo

This directory contains demos for the **MAB-SSRRT** (Multi-Armed Bandit Sphere-Sampled RRT) planner.

## Overview

The MAB-SSRRT planner uses a Multi-Armed Bandit (MAB) approach to adaptively select between different sampling strategies:
- **Uniform sampling**: Random samples across the entire state space
- **Cylinder sampling**: Directed samples along constraint axes (UP/DOWN directions)

The planner is particularly effective for:
- Disassembly planning where objects have limited motion directions
- Narrow passage problems with known constraint structure
- Environments where directed exploration outperforms uniform sampling

## Files

- **MAB_SSRRT_Demo.cpp**: Main demo executable
- **visualize_bugtrap.py**: Python script for visualizing bug trap results
- **benchmark_baseline.yaml**: Default configuration file for the planner

## Building

The demo is automatically built when you build OMPL with demos enabled:

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso
mkdir -p build && cd build
cmake .. -DOMPL_BUILD_DEMOS=ON
make demo_MAB_SSRRT
```

The executable will be located at: `build/demos/demo_MAB_SSRRT`

## Running the Demo

### Basic Usage

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso/build
export LD_LIBRARY_PATH=$PWD/src/ompl:$LD_LIBRARY_PATH

# Run bug trap scenario (using config from build directory)
./demos/demo_MAB_SSRRT \
    --config ./demos/benchmark_baseline.yaml \
    --planner bugtrap \
    --timeout 10
```

### Command Line Options

```
Usage: demo_MAB_SSRRT [options]

Options:
  --config <path>     Path to YAML config file (required)
  --timeout <seconds> Planning timeout in seconds (default: 10.0)
  --planner <name>    Planner scenario to use (default: mab-ssrrt)
  --debug             Enable debug mode (extensive logging + sample tracking)
  --help              Show this help message

Available scenarios:
  mab-ssrrt   Standard 6D Assembly Demo
  bugtrap     2D Bug Trap (Start inside U-shape, Goal outside)
```

### Examples

**Run bug trap scenario:**
```bash
./demos/demo_MAB_SSRRT \
    --config ./demos/benchmark_baseline.yaml \
    --planner bugtrap \
    --timeout 10
```

**Run bug trap with debug mode (for visualization):**
```bash
./demos/demo_MAB_SSRRT \
    --config ./demos/benchmark_baseline.yaml \
    --planner bugtrap \
    --timeout 10 \
    --debug
```

**Run 6D assembly scenario:**
```bash
./demos/demo_MAB_SSRRT \
    --config ./demos/benchmark_baseline.yaml \
    --planner mab-ssrrt \
    --timeout 15
```

## Scenarios

### 1. Bug Trap (2D)

A classic local minimum problem where:
- **Start**: (0, 0) - inside a U-shaped obstacle
- **Goal**: (10, 0) - outside the obstacle
- **Challenge**: Direct path is blocked; must explore backwards (negative X) to escape

This scenario tests the planner's ability to escape local minima using adaptive sampling.

**Visualization**: After running with `--debug`, use `visualize_bugtrap.py` to see:
- RRT tree with all sampled states
- Valid (green) vs invalid (red) samples
- Tree edges (connections)
- Planned path from start to goal
- Sampler type information

### 2. 6D Assembly

A constrained assembly planning scenario:
- **State space**: 6D (RPY rotation + XYZ translation)
- **Obstacles**: Spheres and walls
- **Challenge**: Find collision-free path in constrained space

## Visualization

### Bug Trap Visualization

After running the bug trap demo with `--debug`, visualize the results:

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso/build/demos

# Generate visualization
python3 ../../demos/disassembly/visualize_bugtrap.py \
    bugtrap_path.csv \
    bugtrap_visualization_debug.pdf \
    bugtrap_samples_debug.csv
```

**Output files:**
- `bugtrap_visualization_debug.pdf` - High-resolution PDF visualization
- `bugtrap_visualization_debug.png` - PNG version

**What the visualization shows:**
- **Left plot**: RRT tree with valid/invalid samples and edges
- **Right plot**: Samples colored by sampler type (inner) and validity (border)

See `visualize_bugtrap.py` for more details.

## Configuration

The demo requires a YAML configuration file. A default configuration is provided at `benchmark_baseline.yaml` in this directory.

Key parameters:
- `adaptive_quasirandom_sample_size`: Number of samples during burn-in
- `adaptive_start_radius`: Initial sampling radius
- `mab_uniform_sphere_window_size`: MAB sliding window size
- `uniform_goal_bias`: Goal bias for uniform sampling
- `sphere_goal_bias`: Goal bias for cylinder sampling

## Output Files

When running with `--debug`, the demo generates:
- `bugtrap_path.csv`: Solution path with sampler information
- `bugtrap_samples_debug.csv`: All sampled states with debug information

These files can be used for visualization and analysis.

## Troubleshooting

**Error: "Error parsing YAML configuration file"**
- Verify the config file path is correct
- Check that the file exists and is valid YAML

**Error: "No solution found within timeout"**
- Try increasing the timeout: `--timeout 30`
- Check that start and goal states are valid
- Verify configuration parameters are appropriate

**Visualization not working**
- Make sure you ran with `--debug` flag
- Check that CSV files were generated
- Verify Python and matplotlib are installed: `pip3 install matplotlib numpy`

## Quick Start

```bash
# 1. Build
cd /home/serboba/transferompl_ws/src/ompl_iso
mkdir -p build && cd build
cmake .. -DOMPL_BUILD_DEMOS=ON
make demo_MAB_SSRRT

# 2. Run bug trap with debug
export LD_LIBRARY_PATH=$PWD/src/ompl:$LD_LIBRARY_PATH
./demos/demo_MAB_SSRRT \
    --config ./demos/benchmark_baseline.yaml \
    --planner bugtrap \
    --timeout 10 \
    --debug

# 3. Visualize
cd demos
python3 ../../demos/disassembly/visualize_bugtrap.py
```
