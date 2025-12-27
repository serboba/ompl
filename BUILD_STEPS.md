# OMPL Clean Build Steps

## Quick Clean Build (Recommended)

Use the provided clean build script:

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso
./CLEAN_BUILD.sh
```

This script will:
1. Remove the existing `build/` directory
2. Create a fresh `build/` directory
3. Run CMake configuration
4. Build OMPL with all available CPU cores

## Manual Clean Build Steps

If you prefer to do it manually:

### Step 1: Clean the build directory
```bash
cd /home/serboba/transferompl_ws/src/ompl_iso
rm -rf build
```

### Step 2: Create and enter build directory
```bash
mkdir build
cd build
```

### Step 3: Configure with CMake
```bash
cmake ..
```

### Step 4: Build OMPL
```bash
# Build everything (including tests)
make -j$(nproc)

# OR build only demos (skip tests if they fail)
make demos -j$(nproc)
```

### Step 5: Verify build
```bash
# Check if demo executable exists
ls -lh demos/demo_MAB_RRT
```

## Partial Rebuild (After Code Changes)

If you only modified source files and want to rebuild:

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso/build
make -j$(nproc)
```

## Build Only Demos (Skip Tests)

If tests are failing but you only need the demos:

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso/build
make demos -j$(nproc)
```

## Clean Build for Specific Target

To rebuild only a specific component:

```bash
cd /home/serboba/transferompl_ws/src/ompl_iso/build

# Rebuild only MAB_RRT demo
make demo_MAB_RRT -j$(nproc)

# Rebuild only the library
make ompl -j$(nproc)
```

## Troubleshooting

### If CMake cache is corrupted:
```bash
cd /home/serboba/transferompl_ws/src/ompl_iso/build
rm -rf CMakeCache.txt CMakeFiles/
cmake ..
make -j$(nproc)
```

### If you get "No rule to make target" errors:
Run a clean build (Step 1-4 above)

### If tests fail but demos work:
This is normal - you can ignore test failures and just build demos:
```bash
make demos -j$(nproc)
```

