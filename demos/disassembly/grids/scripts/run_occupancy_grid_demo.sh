#!/bin/bash
# Shell script to run MAB-SSRRT demo with occupancy grid and generate visualization

set -e  # Exit on error

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Build directory is at src/ompl_iso/build (3 levels up from scripts/)
BUILD_DIR="${SCRIPT_DIR}/../../../build"

# Default values
CONFIG_FILE="${SCRIPT_DIR}/../benchmark_baseline.yaml"
GRID_FILE="${SCRIPT_DIR}/../grids/occupancy_grid_1.csv"
TIMEOUT=10
DEBUG=false
VISUALIZE=true
SHOW_INVALID=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --grid)
            GRID_FILE="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        --no-debug)
            DEBUG=false
            shift
            ;;
        --debug)
            DEBUG=true
            shift
            ;;
        --no-visualize)
            VISUALIZE=false
            shift
            ;;
        --show-invalid)
            SHOW_INVALID=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --config <file>      Path to YAML config file (default: ../benchmark_baseline.yaml)"
            echo "  --grid <file>        Path to occupancy grid CSV file (default: ../grids/occupancy_grid_1.csv)"
            echo "                       Can be relative to scripts/ or absolute path"
            echo "  --timeout <seconds>  Planning timeout in seconds (default: 10)"
            echo "  --debug              Enable debug mode (default: false)"
            echo "  --no-debug           Disable debug mode"
            echo "  --no-visualize       Skip visualization generation"
            echo "  --show-invalid       Show invalid samples in visualization (default: hidden)"
            echo "  --help, -h           Show this help message"
            echo ""
            echo "Example:"
            echo "  $0 --grid ../grids/occupancy_grid_1.csv --timeout 15 --debug"
            echo "  $0 --grid occupancy_grid_2.csv --timeout 15 --debug"
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
    echo "  make demo_MAB_SSRRT"
    exit 1
fi

# Check if demo executable exists
DEMO_EXE="${BUILD_DIR}/demos/demo_MAB_SSRRT"
if [ ! -f "$DEMO_EXE" ]; then
    echo "Error: Demo executable not found: $DEMO_EXE"
    echo "Please build the demo first:"
    echo "  cd ${BUILD_DIR}"
    echo "  make demo_MAB_SSRRT"
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

# Check if files exist
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found: $CONFIG_FILE"
    exit 1
fi

if [ ! -f "$GRID_FILE" ]; then
    echo "Error: Grid file not found: $GRID_FILE"
    exit 1
fi

# Extract grid filename (without path and extension) for output file naming
GRID_BASENAME=$(basename "$GRID_FILE")
GRID_NAME="${GRID_BASENAME%.*}"  # Remove extension

# Set library path
export LD_LIBRARY_PATH="${BUILD_DIR}/src/ompl:${LD_LIBRARY_PATH}"

# Build command
CMD="${DEMO_EXE} --config ${CONFIG_FILE} --planner occupancy-grid --occupancy-grid ${GRID_FILE} --timeout ${TIMEOUT}"

if [ "$DEBUG" = true ]; then
    CMD="${CMD} --debug"
fi

echo "=================================================="
echo "Running MAB-SSRRT Occupancy Grid Demo"
echo "=================================================="
echo "Config: ${CONFIG_FILE}"
echo "Grid: ${GRID_FILE}"
echo "Timeout: ${TIMEOUT}s"
echo "Debug: ${DEBUG}"
echo "=================================================="
echo ""

# Run the demo
cd "${BUILD_DIR}"
$CMD

# Check if demo was successful
if [ $? -ne 0 ]; then
    echo ""
    echo "Error: Demo failed. Visualization will not be generated."
    exit 1
fi

# Output files go to parent directory (disassembly folder)
OUTPUT_DIR="${SCRIPT_DIR}/.."
PATH_FILE="${OUTPUT_DIR}/${GRID_NAME}_path.csv"
SAMPLES_FILE="${OUTPUT_DIR}/${GRID_NAME}_samples_debug.csv"
OUTPUT_FILE="${OUTPUT_DIR}/${GRID_NAME}_visualization.pdf"

# Find and move/rename output files to output directory with grid name
# The C++ code may output to different locations:
# - Debug mode: build directory (bugtrap_path.csv, bugtrap_samples_debug.csv)
# - Non-debug mode: grid file directory (bugtrap_path.csv)
# - Debug planner: build directory (bugtrap_path.csv, bugtrap_samples_debug.csv)

# Check build directory first (for debug mode)
BUILD_PATH_FILE="${BUILD_DIR}/bugtrap_path.csv"
BUILD_SAMPLES_FILE="${BUILD_DIR}/bugtrap_samples_debug.csv"

# Check grid file directory (for non-debug mode)
GRID_DIR="$(dirname "$(readlink -f "$GRID_FILE")")"
GRID_DIR_PATH_FILE="${GRID_DIR}/bugtrap_path.csv"

# Also check output directory (in case files were already moved)
OUTPUT_PATH_FILE="${OUTPUT_DIR}/bugtrap_path.csv"
OUTPUT_SAMPLES_FILE="${OUTPUT_DIR}/bugtrap_samples_debug.csv"

# Find and move path file
if [ -f "$BUILD_PATH_FILE" ]; then
    mv "$BUILD_PATH_FILE" "$PATH_FILE"
    echo "[INFO] Path file saved as: ${PATH_FILE}"
elif [ -f "$GRID_DIR_PATH_FILE" ]; then
    mv "$GRID_DIR_PATH_FILE" "$PATH_FILE"
    echo "[INFO] Path file saved as: ${PATH_FILE}"
elif [ -f "$OUTPUT_PATH_FILE" ]; then
    mv "$OUTPUT_PATH_FILE" "$PATH_FILE"
    echo "[INFO] Path file saved as: ${PATH_FILE}"
fi

# Find and move samples file (debug mode only)
if [ "$DEBUG" = true ]; then
    if [ -f "$BUILD_SAMPLES_FILE" ]; then
        mv "$BUILD_SAMPLES_FILE" "$SAMPLES_FILE"
        echo "[INFO] Samples file saved as: ${SAMPLES_FILE}"
    elif [ -f "$OUTPUT_SAMPLES_FILE" ]; then
        mv "$OUTPUT_SAMPLES_FILE" "$SAMPLES_FILE"
        echo "[INFO] Samples file saved as: ${SAMPLES_FILE}"
    fi
fi

# Generate visualization if requested
if [ "$VISUALIZE" = true ]; then
    echo ""
    echo "=================================================="
    echo "Generating Visualization"
    echo "=================================================="
    
    # Check if visualization script exists
    VIZ_SCRIPT="${SCRIPT_DIR}/../visualize_bugtrap.py"
    if [ ! -f "$VIZ_SCRIPT" ]; then
        echo "Error: Visualization script not found: $VIZ_SCRIPT"
        exit 1
    fi
    
    # Run visualization (using bugtrap_path.csv for compatibility)
    # The script will use the grid file if provided as 4th argument
    # Invalid samples are hidden by default (use --show-invalid to show them)
    VIZ_CMD="python3 \"${VIZ_SCRIPT}\" \"${PATH_FILE}\" \"${OUTPUT_FILE}\" \"${SAMPLES_FILE}\" \"${GRID_FILE}\""
    if [ "$SHOW_INVALID" = true ]; then
        VIZ_CMD="${VIZ_CMD} --show-invalid"
    fi
    eval $VIZ_CMD
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "=================================================="
        echo "Visualization Complete!"
        echo "=================================================="
        echo "Output files:"
        echo "  - ${PATH_FILE}"
        if [ "$DEBUG" = true ] && [ -f "$SAMPLES_FILE" ]; then
            echo "  - ${SAMPLES_FILE}"
        fi
        echo "  - ${OUTPUT_FILE}"
        echo "  - ${OUTPUT_FILE%.pdf}.png"
        echo ""
        echo "All output files are prefixed with: ${GRID_NAME}_"
    else
        echo ""
        echo "Warning: Visualization generation failed."
        exit 1
    fi
    
    # Generate burn-in phase visualization if debug mode is enabled and samples file exists
    if [ "$DEBUG" = true ] && [ -f "$SAMPLES_FILE" ]; then
        echo ""
        echo "=================================================="
        echo "Generating Burn-In Phase Visualization"
        echo "=================================================="
        
        BURNIN_VIZ_SCRIPT="${SCRIPT_DIR}/../visualize_burnin.py"
        if [ ! -f "$BURNIN_VIZ_SCRIPT" ]; then
            echo "Warning: Burn-in visualization script not found: $BURNIN_VIZ_SCRIPT"
        else
            BURNIN_OUTPUT_FILE="${OUTPUT_DIR}/${GRID_NAME}_burnin_visualization.pdf"
            BURNIN_CMD="python3 \"${BURNIN_VIZ_SCRIPT}\" \"${SAMPLES_FILE}\" \"${GRID_FILE}\" \"${BURNIN_OUTPUT_FILE}\""
            eval $BURNIN_CMD
            
            if [ $? -eq 0 ]; then
                echo "Burn-in visualization saved to: ${BURNIN_OUTPUT_FILE}"
                echo "  - ${BURNIN_OUTPUT_FILE}"
                echo "  - ${BURNIN_OUTPUT_FILE%.pdf}.png"
            else
                echo "Warning: Burn-in visualization generation failed."
            fi
        fi
    fi
fi

echo ""
echo "Done!"

