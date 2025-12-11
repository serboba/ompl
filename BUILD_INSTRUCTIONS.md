# Building OMPL_ISO

## Prerequisites

Required dependencies:
- **CMake** (version 3.12 or higher)
- **Boost** (version 1.68 or higher) - with components: `serialization`, `program_options`
- **Eigen3** (version 3.3 or higher)
- **yaml-cpp** - Required for MAB-SSRRT-1L planner configuration files
- **C++ compiler** with C++14 support

Optional dependencies:
- **VAMP** - For high-performance collision checking (enabled by default)
- **Python** - For Python bindings
- **Doxygen** - For documentation

## Quick Build Instructions

### 1. Install Dependencies (Ubuntu/Debian)

```bash
sudo apt-get update
sudo apt-get install -y \
    cmake \
    libboost-all-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    build-essential
```

### 2. Build OMPL_ISO

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso

# Create build directory (if it doesn't exist)
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build (replace 4 with number of CPU cores)
make -j4

# Optional: Install (if you want to install system-wide)
# sudo make install
```

### 3. Build Specific Targets

To build only the demos:
```bash
cd build
make demo_RRTFinalizedDemo test_2d_sampling test_burnin_2d -j4
```

To build only the library:
```bash
cd build
make ompl -j4
```

## Build Options

You can configure build options using CMake:

```bash
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DOMPL_BUILD_DEMOS=ON \
    -DOMPL_BUILD_PYBINDINGS=OFF \
    -DOMPL_BUILD_TESTS=ON
```

Common CMake variables:
- `CMAKE_BUILD_TYPE`: `Release`, `Debug`, `RelWithDebInfo` (default: `Release`)
- `OMPL_BUILD_DEMOS`: Build demo programs (default: `ON`)
- `OMPL_BUILD_PYBINDINGS`: Build Python bindings (default: `OFF`)
- `OMPL_BUILD_TESTS`: Build test programs (default: `ON`)

## Running the Demos

After building, you can run the demos:

```bash
# Run the main demo
cd /home/serboba/transferompl_ws/src/ompl_iso
./run_rrtfinalized_demo.sh mab-ssrrt-1l 10

# Or run directly from build directory
cd build
./demos/demo_RRTFinalizedDemo --config ../config/benchmark_baseline.yaml --timeout 10 --planner mab-ssrrt-1l

# Run 2D bug trap demo
./demos/demo_RRTFinalizedDemo --config ../config/benchmark_baseline.yaml --timeout 10 --planner bugtrap

# Run test programs
./demos/test_2d_sampling
./demos/test_burnin_2d
```

## Troubleshooting

### Missing Dependencies

If CMake fails to find dependencies:

1. **Boost**: Make sure `libboost-dev` and `libboost-program-options-dev` are installed
2. **Eigen3**: Install `libeigen3-dev` or set `EIGEN3_INCLUDE_DIR` manually
3. **yaml-cpp**: Install `libyaml-cpp-dev` or set `yaml-cpp_DIR` manually

### Build Errors

- If you get compilation errors, try a clean build:
  ```bash
  cd build
  rm -rf *
  cmake ..
  make -j4
  ```

- If linking fails, check that all required libraries are installed and in your library path.

## Development Build

For development with debugging symbols:

```bash
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j4
```
