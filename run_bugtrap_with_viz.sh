#!/bin/bash
# =============================================================================
# Bug Trap Demo with Visualization
# =============================================================================
# 
# This script runs the bug trap demo and automatically generates a visualization.
#
# Usage:
#   ./run_bugtrap_with_viz.sh [timeout]
#
# Example:
#   ./run_bugtrap_with_viz.sh 10
#
# =============================================================================

TIMEOUT=${1:-10}  # Default timeout to 10 seconds

echo "=============================================="
echo "  Bug Trap Demo with Visualization"
echo "=============================================="
echo "  Timeout: ${TIMEOUT} seconds"
echo "=============================================="
echo ""

# Step 1: Run the bug trap demo
echo "[1/2] Running bug trap demo..."
./run_rrtfinalized_demo.sh bugtrap ${TIMEOUT}

# Check if path file was created
if [ ! -f "bugtrap_path.csv" ]; then
    echo ""
    echo "Error: Path file 'bugtrap_path.csv' was not created."
    echo "The demo may not have found a solution."
    exit 1
fi

echo ""
echo "[2/2] Generating visualization..."
python3 demos/disassembly/visualize_bugtrap.py bugtrap_path.csv bugtrap_visualization.pdf

echo ""
echo "=============================================="
echo "  Complete!"
echo "=============================================="
echo "  Path file: bugtrap_path.csv"
echo "  Visualization: bugtrap_visualization.pdf"
echo "  Visualization: bugtrap_visualization.png"
echo "=============================================="
