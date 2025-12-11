#!/bin/bash
# =============================================================================
# MAB-SSRRT Planner Demo Runner
# =============================================================================
# 
# This script runs the MAB-SSRRT planners in an isolated OMPL environment.
#
# Available planners:
#   - mab-ssrrt-1l : Single-layer flat MAB (3 arms: UNIFORM, CYL_UP, CYL_DOWN) - 6D
#   - mab-ssrrt-1l-2d : Single-layer flat MAB in 2D state space (XY only)
#
# Usage:
#   ./run_rrtfinalized_demo.sh [planner] [timeout]
#
# Examples:
#   ./run_rrtfinalized_demo.sh                    # Run 1L (6D) with 3s timeout
#   ./run_rrtfinalized_demo.sh mab-ssrrt-1l 10    # Run 1L (6D) with 10s timeout
#   ./run_rrtfinalized_demo.sh mab-ssrrt-1l-2d 5  # Run 1L in 2D with 5s timeout
#
# =============================================================================

PLANNER_TYPE=${1:-mab-ssrrt-1l}  # Default to 1L variant
TIMEOUT=${2:-3}                   # Default timeout to 3 seconds
CONFIG_FILE="${PWD}/build/demos/benchmark_baseline.yaml"

echo "=============================================="
echo "  MAB-SSRRT Planner Demo"
echo "=============================================="
echo "  Planner: ${PLANNER_TYPE}"
echo "  Timeout: ${TIMEOUT} seconds"
echo "  Config:  ${CONFIG_FILE}"
echo "=============================================="
echo ""

# Ensure the config file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found at ${CONFIG_FILE}"
    echo ""
    echo "Make sure you have copied benchmark_baseline.yaml to build/demos/"
    exit 1
fi

# Check if the demo executable exists
DEMO_EXEC="./build/demos/demo_RRTFinalizedDemo"
if [ ! -f "$DEMO_EXEC" ]; then
    echo "Error: Demo executable not found at ${DEMO_EXEC}"
    echo ""
    echo "Make sure you have built the project:"
    echo "  cd build && make -j4"
    exit 1
fi

# Run the demo executable
$DEMO_EXEC --config "${CONFIG_FILE}" --planner "${PLANNER_TYPE}" --timeout "${TIMEOUT}" ${DEBUG_FLAG}
