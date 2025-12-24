#!/bin/bash

# Shell script to run occupancy grid benchmarking
# Compares MAB-SSRRT, RRT, RRT-Gaussian, and RRT-Bridge planners

set -e

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Build directory is at src/ompl_iso/build (3 levels up from scripts/)
BUILD_DIR="${SCRIPT_DIR}/../../../build"

# Default values
GRID_FILE="${SCRIPT_DIR}/../grids/occupancy_grid_1.csv"
CONFIG_FILE="${SCRIPT_DIR}/../benchmark_baseline.yaml"
NUM_RUNS=10
TIMEOUT=10.0
OUTPUT_FILE="${SCRIPT_DIR}/../benchmark_results.csv"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --grid)
            GRID_FILE="$2"
            shift 2
            ;;
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --runs)
            NUM_RUNS="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        --output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --grid <file>        Occupancy grid CSV file (default: ../grids/occupancy_grid_1.csv)"
            echo "                       Can be relative to scripts/ or absolute path"
            echo "  --config <file>      YAML config file (default: ../benchmark_baseline.yaml)"
            echo "  --runs <N>           Number of runs per planner (default: 10)"
            echo "  --timeout <T>        Planning timeout in seconds (default: 10.0)"
            echo "  --output <file>      Output CSV file (default: benchmark_results.csv)"
            echo "  --build-dir <dir>    Build directory (default: ../../../build)"
            echo "  --help, -h           Show this help message"
            echo ""
            echo "Example:"
            echo "  $0 --grid ../grids/occupancy_grid_1.csv --runs 20 --timeout 15"
            echo "  $0 --grid occupancy_grid_2.csv --runs 20 --timeout 15"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    echo "Error: Build directory not found: $BUILD_DIR"
    echo "Please build the project first:"
    echo "  cd ${SCRIPT_DIR}/../.."
    echo "  mkdir -p build && cd build"
    echo "  cmake .. -DOMPL_BUILD_DEMOS=ON"
    echo "  make demo_benchmark_occupancy_grid"
    exit 1
fi

# Check if benchmark executable exists
BENCHMARK_EXE="${BUILD_DIR}/demos/demo_benchmark_occupancy_grid"
if [ ! -f "$BENCHMARK_EXE" ]; then
    echo "Error: Benchmark executable not found: $BENCHMARK_EXE"
    echo "Please build the benchmark first:"
    echo "  cd ${BUILD_DIR}"
    echo "  make demo_benchmark_occupancy_grid"
    exit 1
fi

# Make paths absolute if they're relative
if [[ "$CONFIG_FILE" != /* ]]; then
    CONFIG_FILE="${SCRIPT_DIR}/../${CONFIG_FILE}"
fi
if [[ "$GRID_FILE" != /* ]]; then
    # If it's a relative path, try grids/ first, then parent directory
    if [ -f "${SCRIPT_DIR}/../grids/${GRID_FILE}" ]; then
        GRID_FILE="${SCRIPT_DIR}/../grids/${GRID_FILE}"
    else
        GRID_FILE="${SCRIPT_DIR}/../${GRID_FILE}"
    fi
fi
if [[ "$OUTPUT_FILE" != /* ]]; then
    OUTPUT_FILE="${SCRIPT_DIR}/../${OUTPUT_FILE}"
fi

# Check if files exist
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found: $CONFIG_FILE"
    exit 1
fi

if [ ! -f "$GRID_FILE" ]; then
    echo "Error: Grid file not found: $GRID_FILE"
    exit 1
fi

# Set library path
export LD_LIBRARY_PATH="${BUILD_DIR}/src/ompl:${LD_LIBRARY_PATH}"

# Build command
CMD="${BENCHMARK_EXE} ${GRID_FILE} ${CONFIG_FILE} --runs ${NUM_RUNS} --timeout ${TIMEOUT} --output ${OUTPUT_FILE}"

echo "=================================================="
echo "Running Occupancy Grid Benchmark"
echo "=================================================="
echo "Grid file: ${GRID_FILE}"
echo "Config file: ${CONFIG_FILE}"
echo "Runs per planner: ${NUM_RUNS}"
echo "Timeout per run: ${TIMEOUT}s"
echo "Output file: ${OUTPUT_FILE}"
echo "=================================================="
echo ""

# Run the benchmark
cd "${BUILD_DIR}"
$CMD

# Check if benchmark was successful
if [ $? -ne 0 ]; then
    echo ""
    echo "Error: Benchmark failed."
    exit 1
fi

echo ""
echo "Benchmark complete! Results saved to: ${OUTPUT_FILE}"

