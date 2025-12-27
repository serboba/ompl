#!/bin/bash
# Clean build script for OMPL
set -e

echo "=========================================="
echo "Cleaning OMPL build directory"
echo "=========================================="

cd "$(dirname "$0")"
BUILD_DIR="build"

if [ -d "$BUILD_DIR" ]; then
    echo "Removing existing build directory..."
    rm -rf "$BUILD_DIR"
fi

echo "Creating fresh build directory..."
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

echo "Running CMake configuration..."
cmake ..

echo "Building OMPL..."
make -j$(nproc)

echo ""
echo "=========================================="
echo "Clean build complete!"
echo "=========================================="
echo "Demo executable: $BUILD_DIR/demos/demo_MAB_RRT"
