# Occupancy Grid Coordinate System Analysis

## Issues Found in MAB_SSRRT_Demo.cpp

### Issue 1: Goal Y Coordinate Negation (CRITICAL BUG)

**Location:** Line 327 in `OccupancyGridValidityChecker::loadOccupancyGrid()`

**Problem:**
```cpp
goalY_ = -std::stod(yStr);  // Line 327
// Comment says: "CSV coordinates are in display coordinates (positive Y is below)"
// Comment says: "Convert to world coordinates (positive Y is above) by negating Y"
```

**Contradiction:**
- Line 737 comment says: "Goal coordinates from CSV are already in world coordinates (centered at origin)"
- Python visualization script (`visualize_grids.py`) does NOT negate goal Y coordinates
- The goal is used directly at line 739: `goal[1] = goalY;` without any conversion

**Expected Behavior:**
Based on the coordinate system documentation and Python visualization:
- CSV goal coordinates are already in world coordinates (centered at origin)
- No negation should be applied
- The visualization uses `extent=[-width/2, width/2, -height/2, height/2]` which means:
  - World coordinates: Y increases upward (mathematical convention)
  - Visualization: Y increases downward (image convention) but coordinates are the same

**Fix Required:**
Remove the negation on line 327:
```cpp
goalY_ = std::stod(yStr);  // Remove the negation
```

### Issue 2: Unused invertY_ Flag

**Location:** Throughout `OccupancyGridValidityChecker` class

**Problem:**
- `invertY_` flag is set to `true` by default (line 182)
- Flag is only used in verification code (lines 394-396)
- NOT used in the actual `isValid()` function (lines 188-237)
- Comments say Y-inversion is needed, but implementation doesn't use it

**Current State:**
- According to `COORDINATE_SYSTEM_FIX.md`, Y-inversion was intentionally removed
- The coordinate mapping formula already produces correct grid indices
- The flag should either be removed or the comments should be updated

**Recommendation:**
- Remove `invertY_` flag entirely OR
- Update comments to clarify that Y-inversion is not needed with current formula

### Issue 3: Inconsistent Comments

**Location:** Multiple locations

**Problems:**
1. Line 325-326: Says CSV coordinates are in "display coordinates"
2. Line 737: Says CSV coordinates are "already in world coordinates"
3. Line 172-174: Says Y-inversion is needed, but it's not implemented

**Fix Required:**
- Standardize comments to reflect actual implementation
- CSV coordinates are in world coordinates (centered at origin)
- No coordinate conversion needed for goal coordinates

## Coordinate System Summary

### CSV Format
- Header: `# start,x,y` and `# goal,x,y`
- Coordinates are in **world coordinates** (centered at origin)
- Grid is centered at (0, 0)
- X: -width/2 to +width/2 (left to right)
- Y: -height/2 to +height/2 (bottom to top in mathematical coordinates)

### Grid Mapping
- `gridOffsetX = -width/2`
- `gridOffsetY = -height/2`
- `gridX = floor((x - gridOffsetX) / resolution)`
- `gridY = floor((y - gridOffsetY) / resolution)`
- **No Y-inversion needed** - formula already correct

### Visualization
- Uses `extent=[-width/2, width/2, -height/2, height/2]` with `origin='upper'`
- World (-width/2, -height/2) → grid[0][0] (top-left in image)
- World (0, 0) → grid[height/2][width/2] (center)
- World (+width/2, +height/2) → grid[height-1][width-1] (bottom-right in image)


