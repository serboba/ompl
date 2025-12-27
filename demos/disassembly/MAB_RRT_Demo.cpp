/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Willow Garage nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Servet Bora Bayraktar - MAB-RRT Standalone Demo */

/**
 * @file MAB_RRT_Demo.cpp
 * @brief Standalone demo for MAB-RRT planner
 * 
 * This demo tests the MAB-RRT planner in an isolated OMPL environment:
 * - MAB-RRT: Multi-Armed Bandit RRT planner
 * 
 * Supports:
 * - 6D: Constrained Assembly (RPY + XYZ)
 * - 2D: Bug Trap Scenario (XY) - Hard local minimum
 */

#include <iostream>
#include <cmath>
#include <memory>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

// OMPL includes
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
 
// MAB-RRT planner
#include <ompl/geometric/planners/disassemblyrrt/MAB_RRT.h>
#include <ompl/geometric/planners/disassemblyrrt/MAB_RRT_DEBUG.h>
 
namespace ob = ompl::base;
namespace og = ompl::geometric;
 
/**
 * @brief Simple validity checker for a 6D state space (Assembly)
 */
class AssemblyValidityChecker : public ob::StateValidityChecker
{
public:
    AssemblyValidityChecker(const ob::SpaceInformationPtr& si, bool verbose = false)
        : ob::StateValidityChecker(si), verbose_(verbose)
    {
    }

    bool isValid(const ob::State* state) const override
    {
        const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();
        
        double x = realState->values[3];
        double y = realState->values[4];
        double z = realState->values[5];

        if (!si_->satisfiesBounds(state)) return false;

        // Obstacle 1: Sphere
        double dist1 = std::sqrt((x - 0.5)*(x - 0.5) + (y - 0.5)*(y - 0.5) + (z - 0.5)*(z - 0.5));
        if (dist1 < 0.2) return false;

        // Obstacle 2: Sphere
        double dist2 = std::sqrt((x + 0.5)*(x + 0.5) + (y - 0.3)*(y - 0.3) + (z - 0.0)*(z - 0.0));
        if (dist2 < 0.15) return false;

        // Obstacle 3: Wall constraint
        if (std::abs(x) < 0.1 && std::abs(y) < 0.1 && z > 0.05 && z < 0.3) return false;

        return true;
    }

private:
    bool verbose_;
};

/**
 * @brief Validity checker for 2D Bug Trap
 * 
 * Defines a "U" shaped trap containing the start state.
 * The opening is to the negative X direction.
 */
class BugTrapValidityChecker : public ob::StateValidityChecker
{
public:
    BugTrapValidityChecker(const ob::SpaceInformationPtr& si) : ob::StateValidityChecker(si) {}

    bool isValid(const ob::State* state) const override
    {
        const double* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
        double x = values[0];
        double y = values[1];

        if (!si_->satisfiesBounds(state)) return false;

        // --- BUG TRAP OBSTACLES ---
        // The trap is roughly a box from x[-2, 5] around y=0.
        // It is closed on Top, Bottom, and Right. Open on Left.
        
        // 1. Top Wall: x in [-2, 5], y in [1, 2]
        if (x >= -2.0 && x <= 5.0 && y >= 1.0 && y <= 2.0) return false;

        // 2. Bottom Wall: x in [-2, 5], y in [-2, -1]
        if (x >= -2.0 && x <= 5.0 && y >= -2.0 && y <= -1.0) return false;

        // 3. Back Wall (Blocking the goal): x in [5, 6], y in [-2, 2]
        if (x >= 5.0 && x <= 6.0 && y >= -2.0 && y <= 2.0) return false;

        return true;
    }
};

/**
 * @brief Validity checker for 2D Occupancy Grid
 * 
 * Reads occupancy grid from CSV file where:
 * - 0 = free space
 * - 100 = occupied/obstacle
 * - Grid is stored row by row
 * - Grid is centered at world origin (0, 0)
 * - Start is always at (0, 0) in world coordinates
 * 
 * Coordinate System:
 * - Grid is centered at (0, 0) in world coordinates
 * - For a WxH grid: bounds are X[-W/2, +W/2], Y[-H/2, +H/2]
 * - In world coordinates: Y increases upward (mathematical coordinates)
 *   - Negative Y is below center, positive Y is above center
 * 
 * Grid Storage (CSV):
 * - CSV row 0 is at the TOP (image coordinates, Y increases downward)
 * - grid[0][0] in CSV maps to world (-W/2, +H/2) [top-left in image, but bottom-left in world]
 * - grid[H/2][W/2] maps to world (0, 0) [center, where start is]
 * - grid[H-1][W-1] maps to world (W/2-1, -H/2+1) [bottom-right in image, but top-right in world]
 * 
 * Y-Axis Inversion:
 * - The coordinate mapping formula already produces correct grid indices
 * - No Y-inversion is needed in isValid() - the formula matches visualization
 * - The invertY_ flag exists for backward compatibility but is not used in collision checking
 */
class OccupancyGridValidityChecker : public ob::StateValidityChecker
{
public:
    OccupancyGridValidityChecker(const ob::SpaceInformationPtr& si, 
                                  const std::string& gridFile,
                                  double resolution = 1.0,
                                  bool invertY = true)
        : ob::StateValidityChecker(si), resolution_(resolution), invertY_(invertY)
    {
        loadOccupancyGrid(gridFile);
    }

    bool isValid(const ob::State* state) const override
    {
        const double* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
        double x = values[0];
        double y = values[1];

        if (!si_->satisfiesBounds(state)) return false;

        // Convert world coordinates to grid indices
        // Grid is centered at (0, 0) in world coordinates
        // Visualization uses extent=[-W/2, W/2, H/2, -H/2] with origin='upper'
        // This means:
        //   - World (-W/2, -H/2) maps to grid[0][0] (top-left of CSV)
        //   - World (0, 0) maps to grid[H/2][W/2] (center)
        //   - World (W/2, H/2) maps to grid[H-1][W-1] (bottom-right of CSV)
        // 
        // To match visualization exactly:
        //   - World x: maps from [-W/2, W/2] to [0, W-1]
        //     gridX = floor((x - (-W/2)) / resolution) = floor((x + W/2) / resolution)
        //   - World y: maps from [-H/2, H/2] to [0, H-1]
        //     gridY = floor((y - (-H/2)) / resolution) = floor((y + H/2) / resolution)
        //     world(-H/2) → gridY = 0, world(+H/2) → gridY = H (clamped to H-1)
        //
        // With gridOffsetX = -W/2 and gridOffsetY = -H/2:
        // Use floor() for proper cell mapping (cell [i] covers [i*res, (i+1)*res))
        // Add small epsilon to handle boundary cases where coordinate is exactly at cell boundary
        double epsilon = 1e-10;
        int gridX = static_cast<int>(std::floor((x - gridOffsetX_ + epsilon) / resolution_));
        int gridY = static_cast<int>(std::floor((y - gridOffsetY_ + epsilon) / resolution_));

        // Clamp to valid range
        if (gridX < 0) gridX = 0;
        if (gridX >= gridWidth_) gridX = gridWidth_ - 1;
        if (gridY < 0) gridY = 0;
        if (gridY >= gridHeight_) gridY = gridHeight_ - 1;

        // NO Y-axis inversion needed - the formula already matches visualization
        // Visualization: world(-H/2) → grid[0], world(+H/2) → grid[H-1]
        // Our formula: world(-H/2) → gridY = 0, world(+H/2) → gridY = H-1 (after clamp)
        // This matches perfectly, so we don't need to invert

        // Check bounds
        if (gridX < 0 || gridX >= gridWidth_ || gridY < 0 || gridY >= gridHeight_) {
            return false;  // Out of bounds is invalid
        }

        // Check if cell is occupied (100 = occupied, 0 = free)
        // Grid is stored row by row: grid[y][x] = grid_[y * width + x]
        int cellValue = grid_[gridY * gridWidth_ + gridX];
        
        // Return true if cell is free (value < 50), false if occupied (value >= 50)
        return (cellValue < 50);
    }

    void getStartGoal(double& startX, double& startY, double& goalX, double& goalY) const
    {
        startX = startX_;
        startY = startY_;
        goalX = goalX_;
        goalY = goalY_;
    }

    void getGridDimensions(int& width, int& height) const
    {
        width = gridWidth_;
        height = gridHeight_;
    }

    void getGridOffset(double& offsetX, double& offsetY) const
    {
        offsetX = gridOffsetX_;
        offsetY = gridOffsetY_;
    }

private:
    void loadOccupancyGrid(const std::string& gridFile)
    {
        std::ifstream file(gridFile);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open occupancy grid file: " + gridFile);
        }

        std::string line;
        startX_ = 0.0;
        startY_ = 0.0;
        goalX_ = 0.0;
        goalY_ = 0.0;
        gridWidth_ = 0;
        gridHeight_ = 0;
        // Initialize invertY_ (default false: CSV row 0 is at bottom, math coordinates)

        // Read header lines for start and goal
        while (std::getline(file, line)) {
            // Skip empty lines
            if (line.empty() || line[0] == '\n' || line[0] == '\r') continue;

            // Check for start/goal comments
            if (line[0] == '#') {
                // Parse line like: # start,0,0 or # goal,6,-15
                std::string content = line.substr(1);
                // Remove leading/trailing whitespace
                while (!content.empty() && (content[0] == ' ' || content[0] == '\t' || content[0] == '\r' || content[0] == '\n')) {
                    content = content.substr(1);
                }
                while (!content.empty() && (content.back() == ' ' || content.back() == '\t' || content.back() == '\r' || content.back() == '\n')) {
                    content.pop_back();
                }
                
                // Split by comma
                std::vector<std::string> tokens;
                std::istringstream iss(content);
                std::string token;
                while (std::getline(iss, token, ',')) {
                    // Trim each token
                    while (!token.empty() && (token[0] == ' ' || token[0] == '\t')) {
                        token = token.substr(1);
                    }
                    while (!token.empty() && (token.back() == ' ' || token.back() == '\t' || token.back() == '\r' || token.back() == '\n')) {
                        token.pop_back();
                    }
                    if (!token.empty()) {
                        tokens.push_back(token);
                    }
                }
                
                if (tokens.size() >= 3) {
                    std::string keyword = tokens[0];
                    std::string xStr = tokens[1];
                    std::string yStr = tokens[2];
                    
                    if (keyword == "start") {
                        try {
                            startX_ = std::stod(xStr);
                            startY_ = std::stod(yStr);
                        } catch (const std::exception& e) {
                            std::cerr << "[WARNING] Failed to parse start coordinates from '" << line << "': " << e.what() << std::endl;
                        }
                    } else if (keyword == "goal") {
                        try {
                            goalX_ = std::stod(xStr);
                            // CSV coordinates are already in world coordinates (centered at origin)
                            // No conversion needed - use coordinates directly
                            goalY_ = std::stod(yStr);
                        } catch (const std::exception& e) {
                            std::cerr << "[WARNING] Failed to parse goal coordinates from '" << line << "': " << e.what() << std::endl;
                        }
                    }
                }
                continue;
            }

            // Parse grid row
            std::vector<int> row;
            std::istringstream iss(line);
            std::string cell;
            
            while (std::getline(iss, cell, ',')) {
                try {
                    row.push_back(std::stoi(cell));
                } catch (...) {
                    // Skip invalid cells
                }
            }

            if (!row.empty()) {
                if (gridWidth_ == 0) {
                    gridWidth_ = row.size();
                }
                grid_.insert(grid_.end(), row.begin(), row.end());
                gridHeight_++;
            }
        }

        file.close();

        if (gridWidth_ == 0 || gridHeight_ == 0) {
            throw std::runtime_error("Failed to load occupancy grid: empty grid");
        }

        // Grid is centered at world origin (0, 0)
        // For a grid of size WxH, the bounds are:
        //   X: [-W/2, +W/2]
        //   Y: [-H/2, +H/2] in world coordinates (mathematical: positive Y is above)
        // CSV row 0 is at TOP (image coordinates)
        // Coordinate mapping (no Y-inversion needed):
        //   - World (-W/2, -H/2) → Grid[0][0] (top-left of CSV)
        //   - World (0, 0) → Grid[H/2][W/2] (center)
        //   - World (+W/2, +H/2) → Grid[H-1][W-1] (bottom-right of CSV)
        // The gridOffsetY = -H/2 correctly maps world (-H/2) to grid row 0
        gridOffsetX_ = -static_cast<double>(gridWidth_) / 2.0;
        gridOffsetY_ = -static_cast<double>(gridHeight_) / 2.0;

        std::cout << "[INFO] Loaded occupancy grid: " << gridWidth_ << "x" << gridHeight_ << std::endl;
        std::cout << "[INFO] Start: (" << startX_ << ", " << startY_ << ")" << std::endl;
        std::cout << "[INFO] Goal: (" << goalX_ << ", " << goalY_ << ")" << std::endl;
        std::cout << "[INFO] Grid offset: (" << gridOffsetX_ << ", " << gridOffsetY_ << ")" << std::endl;
        
        // Verify start and goal are valid
        // NOTE: Start is ALWAYS (0, 0) in world coordinates (grid center), regardless of CSV header
        double actualStartX = 0.0;
        double actualStartY = 0.0;
        
        int startGridX = static_cast<int>(std::floor((actualStartX - gridOffsetX_) / resolution_));
        int startGridY = static_cast<int>(std::floor((actualStartY - gridOffsetY_) / resolution_));
        int goalGridX = static_cast<int>(std::floor((goalX_ - gridOffsetX_) / resolution_));
        int goalGridY = static_cast<int>(std::floor((goalY_ - gridOffsetY_) / resolution_));
        
        // Apply Y-inversion if needed for verification
        if (invertY_) {
            startGridY = gridHeight_ - 1 - startGridY;
            goalGridY = gridHeight_ - 1 - goalGridY;
        }
        
        std::cout << "[VERIFY] Coordinate mapping verification:" << std::endl;
        std::cout << "[VERIFY]   Grid dimensions: " << gridWidth_ << "x" << gridHeight_ << std::endl;
        std::cout << "[VERIFY]   Grid offset: (" << gridOffsetX_ << ", " << gridOffsetY_ << ")" << std::endl;
        std::cout << "[VERIFY]   Grid[0][0] maps to world: (" << gridOffsetX_ << ", " << gridOffsetY_ << ")" << std::endl;
        std::cout << "[VERIFY]   Grid[" << (gridHeight_/2) << "][" << (gridWidth_/2) << "] maps to world: (0, 0) [center]" << std::endl;
        
        if (startGridX >= 0 && startGridX < gridWidth_ && startGridY >= 0 && startGridY < gridHeight_) {
            int startVal = grid_[startGridY * gridWidth_ + startGridX];
            std::cout << "[VERIFY] Start: world (" << actualStartX << ", " << actualStartY << ") -> grid [" 
                      << startGridY << ", " << startGridX << "] = " << startVal 
                      << (startVal < 50 ? " (FREE)" : " (OBSTACLE)") << std::endl;
            if (startVal >= 50) {
                std::cerr << "[ERROR] Start position is in an OBSTACLE! This will cause planning to fail." << std::endl;
            }
        } else {
            std::cerr << "[ERROR] Start grid coordinates [" << startGridY << ", " << startGridX 
                      << "] are out of bounds!" << std::endl;
        }
        
        if (goalGridX >= 0 && goalGridX < gridWidth_ && goalGridY >= 0 && goalGridY < gridHeight_) {
            int goalVal = grid_[goalGridY * gridWidth_ + goalGridX];
            std::cout << "[VERIFY] Goal: world (" << goalX_ << ", " << goalY_ << ") -> grid [" 
                      << goalGridY << ", " << goalGridX << "] = " << goalVal 
                      << (goalVal < 50 ? " (FREE)" : " (OBSTACLE)") << std::endl;
            if (goalVal >= 50) {
                std::cerr << "[ERROR] Goal position is in an OBSTACLE! This will cause planning to fail." << std::endl;
            }
        } else {
            std::cerr << "[ERROR] Goal grid coordinates [" << goalGridY << ", " << goalGridX 
                      << "] are out of bounds!" << std::endl;
        }
        
        // Additional verification: check array indexing
        std::cout << "[VERIFY] Array indexing: grid_[gridY * gridWidth_ + gridX]" << std::endl;
        std::cout << "[VERIFY]   Example: grid_[0 * " << gridWidth_ << " + 0] = grid_[0] = " 
                  << (grid_.size() > 0 ? std::to_string(grid_[0]) : "N/A") << " (first cell)" << std::endl;
        std::cout << "[VERIFY]   Example: grid_[" << startGridY << " * " << gridWidth_ << " + " << startGridX 
                  << "] = grid_[" << (startGridY * gridWidth_ + startGridX) << "] = " 
                  << (startGridY * gridWidth_ + startGridX < static_cast<int>(grid_.size()) ? 
                      std::to_string(grid_[startGridY * gridWidth_ + startGridX]) : "OUT_OF_BOUNDS") << std::endl;
    }

    std::vector<int> grid_;
    int gridWidth_;
    int gridHeight_;
    double resolution_;
    bool invertY_;  // If true, CSV row 0 is at TOP (image coords), need Y-inversion
    double gridOffsetX_, gridOffsetY_;  // World coordinate of grid[0][0] when not inverted
    double startX_, startY_;
    double goalX_, goalY_;
};

/**
 * @brief Runs the MAB-RRT (6D Assembly) demo
 */
void runMAB_RRT_Demo(const std::string& configPath, double timeout = 10.0)
{
    std::cout << "\n==================================================" << std::endl;
    std::cout << "    MAB-RRT (6D Assembly) Demo" << std::endl;
    std::cout << "==================================================" << std::endl;

    auto space = std::make_shared<ob::RealVectorStateSpace>(6);
    ob::RealVectorBounds bounds(6);
    bounds.setLow(0, -M_PI);  bounds.setHigh(0, M_PI);
    bounds.setLow(1, -M_PI);  bounds.setHigh(1, M_PI);
    bounds.setLow(2, -M_PI);  bounds.setHigh(2, M_PI);
    bounds.setLow(3, -1.0);   bounds.setHigh(3, 1.0);
    bounds.setLow(4, -1.0);   bounds.setHigh(4, 1.0);
    bounds.setLow(5, -1.0);   bounds.setHigh(5, 1.0);
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<AssemblyValidityChecker>(si, false));
    si->setStateValidityCheckingResolution(0.01);
    si->setup();

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    ob::ScopedState<> start(space);
    start[0] = 0.0; start[1] = 0.0; start[2] = 0.0;
    start[3] = 0.0; start[4] = 0.0; start[5] = 0.0;

    ob::ScopedState<> goal(space);
    goal[0] = 0.5; goal[1] = 0.3; goal[2] = 0.2;
    goal[3] = 0.8; goal[4] = 0.7; goal[5] = 0.6;

    pdef->setStartAndGoalStates(start, goal, 0.1);

    std::cout << "\n[INFO] Creating MAB-RRT planner..." << std::endl;
    auto planner = std::make_shared<og::MAB_RRT>(si, configPath);
    planner->setProblemDefinition(pdef);
    planner->setup();

    std::cout << "[INFO] Starting planning with timeout: " << timeout << "s..." << std::endl;
    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeout));
    std::cout << "[RESULT] Planning status: " << status.asString() << std::endl;

    if (status) {
        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (path) std::cout << "[RESULT] Path length: " << path->length() << std::endl;
    }

    ob::PlannerData pdata(si);
    planner->getPlannerData(pdata);
    std::cout << "[STATS] Tree vertices: " << pdata.numVertices() << std::endl;
}

/**
 * @brief Runs the MAB-RRT planner demo in a 2D Bug Trap
 * 
 * Scenario:
 * - Start is at (0,0) inside a U-shaped obstacle.
 * - Goal is at (10,0) outside the obstacle.
 * - Direct path is blocked.
 * - Planner must explore "backwards" (negative x) to escape.
 */
void runBugTrapDemo(const std::string& configPath, double timeout = 10.0, bool debug = false)
{
    std::cout << "\n==================================================" << std::endl;
    std::cout << "    MAB-RRT 2D Bug Trap Demo" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "[INFO] Algorithm: Multi-Armed Bandit Sphere-Sampled RRT" << std::endl;
    std::cout << "[INFO] Scenario: Bug Trap (Escape Local Minima)" << std::endl;

    // 2D state space (XY only)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    
    // Bounds must be large enough to go around the trap
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -5.0);   bounds.setHigh(0, 15.0);  // X: -5 to allow escape left, 15 for goal
    bounds.setLow(1, -5.0);   bounds.setHigh(1, 5.0);   // Y: -5 to 5
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    
    // Use the Bug Trap Validity Checker
    si->setStateValidityChecker(std::make_shared<BugTrapValidityChecker>(si));
    si->setStateValidityCheckingResolution(0.005); // High res for thin walls
    si->setup();

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);

    // Start inside the trap
    ob::ScopedState<> start(space);
    start[0] = 0.0;  // X
    start[1] = 0.0;  // Y

    // Goal outside the trap
    ob::ScopedState<> goal(space);
    goal[0] = 10.0;   // X
    goal[1] = 0.0;    // Y

    pdef->setStartAndGoalStates(start, goal, 0.2);

    std::cout << "\n[INFO] Creating MAB-RRT planner for Bug Trap..." << std::endl;
    if (debug) {
        std::cout << "[INFO] DEBUG MODE ENABLED" << std::endl;
    }
    std::cout << "[INFO] Config file: " << configPath << std::endl;
    
    std::shared_ptr<og::MAB_RRT> planner;
    std::shared_ptr<og::MAB_RRT_DEBUG> debugPlanner;
    
    if (debug) {
        debugPlanner = std::make_shared<og::MAB_RRT_DEBUG>(si, configPath);
        planner = debugPlanner;  // MAB_RRT_DEBUG inherits from MAB_RRT (note: class names not yet renamed)
    } else {
        planner = std::make_shared<og::MAB_RRT>(si, configPath);
    }
    
    planner->setProblemDefinition(pdef);
    planner->setup();

    std::cout << "[INFO] Starting planning with timeout: " << timeout << " seconds..." << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;

    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeout));

    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "\n[RESULT] Planning status: " << status.asString() << std::endl;

    if (status == ob::PlannerStatus::EXACT_SOLUTION || 
        status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout << "[RESULT] Solution found!" << std::endl;
        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (path)
        {
            std::cout << "[RESULT] Path length: " << path->length() << std::endl;
            std::cout << "[RESULT] Path states: " << path->getStateCount() << std::endl;
            
            // In debug mode, path with samplers is already exported by the planner
            // Only save basic path if not in debug mode
            if (!debug) {
                // Save path to demos/disassembly/ directory (absolute path from config)
                std::filesystem::path configFilePath(configPath);
                if (!configFilePath.is_absolute()) {
                    configFilePath = std::filesystem::absolute(configFilePath);
                }
                
                // Find demos/disassembly/ directory by looking for it in the config path
                std::filesystem::path currentPath = configFilePath.parent_path();
                std::filesystem::path targetDir;
                
                // Search up the directory tree for demos/disassembly/
                while (!currentPath.empty() && currentPath != currentPath.root_path()) {
                    std::filesystem::path testPath = currentPath / "demos" / "disassembly";
                    if (std::filesystem::exists(testPath) && std::filesystem::is_directory(testPath)) {
                        targetDir = testPath;
                        break;
                    }
                    currentPath = currentPath.parent_path();
                }
                
                // If not found, use the config file's directory (assuming config is in demos/disassembly/)
                if (targetDir.empty()) {
                    targetDir = configFilePath.parent_path();
                }
                
                std::string pathFile = targetDir.string() + "/bugtrap_path.csv";
                
                std::ofstream outFile(pathFile);
                if (outFile.is_open())
                {
                    outFile << std::fixed << std::setprecision(6);
                    outFile << "x,y\n";
                    for (std::size_t i = 0; i < path->getStateCount(); ++i)
                    {
                        const auto* state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
                        outFile << state->values[0] << "," << state->values[1] << "\n";
                    }
                    outFile.close();
                    std::cout << "[INFO] Path saved to: " << pathFile << std::endl;
                }
                else
                {
                    std::cerr << "[WARNING] Could not save path to file: " << pathFile << std::endl;
                }
            } else {
                std::cout << "[INFO] Path with sampler info exported by debug planner" << std::endl;
            }
        }
    }
    else
    {
        std::cout << "[RESULT] No solution found within timeout." << std::endl;
        std::cout << "[HINT] If stuck in trap, verify parameter 'epsilon_greedy' or 'cylinder_radius'." << std::endl;
    }

    ob::PlannerData pdata(si);
    planner->getPlannerData(pdata);
    std::cout << "\n[STATS] Tree vertices: " << pdata.numVertices() << std::endl;
    std::cout << "[STATS] Tree edges: " << pdata.numEdges() << std::endl;
    
    // Export debug data if in debug mode (always to demos/disassembly/)
    if (debug && debugPlanner) {
        debugPlanner->exportSampleData("bugtrap_samples_debug.csv");
        std::cout << "[DEBUG] Sample data exported to: demos/disassembly/bugtrap_samples_debug.csv" << std::endl;
    }

    std::cout << "\n==================================================" << std::endl;
    std::cout << "    Bug Trap Demo Complete!" << std::endl;
    std::cout << "==================================================" << std::endl;
}

/**
 * @brief Runs the MAB-RRT planner demo with occupancy grid
 * 
 * Loads occupancy grid from CSV file and uses it for collision checking.
 * Grid format:
 * - Header lines: # start,x,y and # goal,x,y
 * - Grid data: 0 = free, 100 = occupied
 * - Supports negative coordinates
 */
void runOccupancyGridDemo(const std::string& configPath, 
                          const std::string& gridFile, 
                          double timeout = 10.0, 
                          bool debug = false)
{
    std::cout << "\n==================================================" << std::endl;
    std::cout << "    MAB-RRT 2D Occupancy Grid Demo" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "[INFO] Algorithm: Multi-Armed Bandit Sphere-Sampled RRT" << std::endl;
    std::cout << "[INFO] Scenario: 2D Occupancy Grid" << std::endl;
    std::cout << "[INFO] Grid file: " << gridFile << std::endl;

    // 2D state space (XY only)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    
    // Create a temporary space info to load grid and get dimensions/start/goal
    auto tempSi = std::make_shared<ob::SpaceInformation>(space);
    auto tempChecker = std::make_shared<OccupancyGridValidityChecker>(tempSi, gridFile, 1.0);
    
    int gridWidth, gridHeight;
    double startX, startY, goalX, goalY;
    double gridOffsetX, gridOffsetY;
    tempChecker->getGridDimensions(gridWidth, gridHeight);
    tempChecker->getStartGoal(startX, startY, goalX, goalY);
    tempChecker->getGridOffset(gridOffsetX, gridOffsetY);
    
    // Set bounds based on centered grid
    // Grid is centered at (0, 0), so bounds are [-width/2, +width/2] for X and [-height/2, +height/2] for Y
    ob::RealVectorBounds bounds(2);
    double halfWidth = static_cast<double>(gridWidth) / 2.0;
    double halfHeight = static_cast<double>(gridHeight) / 2.0;
    bounds.setLow(0, -halfWidth);
    bounds.setHigh(0, halfWidth);
    bounds.setLow(1, -halfHeight);
    bounds.setHigh(1, halfHeight);
    space->setBounds(bounds);
    
    std::cout << "[INFO] Grid dimensions: " << gridWidth << "x" << gridHeight << std::endl;
    std::cout << "[INFO] Grid centered at (0, 0)" << std::endl;
    std::cout << "[INFO] State space bounds: X[" << -halfWidth << ", " << halfWidth 
              << "], Y[" << -halfHeight << ", " << halfHeight << "]" << std::endl;
    std::cout << "[INFO] Grid offset (grid[0][0] maps to): (" << gridOffsetX << ", " << gridOffsetY << ")" << std::endl;
    std::cout << "[INFO] Start from grid file: (" << startX << ", " << startY << ")" << std::endl;
    std::cout << "[INFO] Goal from grid file: (" << goalX << ", " << goalY << ")" << std::endl;

    auto si = std::make_shared<ob::SpaceInformation>(space);
    
    // Create the checker (it will reload the grid, but that's okay)
    // CSV row 0 is at TOP (image coordinates), so we need to invert Y
    auto checker = std::make_shared<OccupancyGridValidityChecker>(si, gridFile, 1.0, true);
    si->setStateValidityChecker(checker);
    // Use a very fine resolution to check paths - check every 0.01 units along edges
    // For large grids (100x100), use even finer resolution to ensure all cells are checked
    // Resolution should be at least 10x smaller than grid cell size (1.0 / 10 = 0.1, but use 0.01 for safety)
    si->setStateValidityCheckingResolution(0.005); // Finer resolution for large grids
    si->setup();

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);

    // Set start and goal from grid file
    // IMPORTANT: Start is always at (0, 0) in world coordinates (grid center)
    // Goal coordinates in CSV header are in world coordinates (relative to center)
    ob::ScopedState<> start(space);
    start[0] = 0.0;  // Start is always at origin
    start[1] = 0.0;

    ob::ScopedState<> goal(space);
    // Goal coordinates from CSV are already in world coordinates (centered at origin)
    goal[0] = goalX;
    goal[1] = goalY;

    std::cout << "[INFO] Start state: (0, 0) [always at grid center]" << std::endl;
    std::cout << "[INFO] Goal state: (" << goalX << ", " << goalY << ") [world coordinates]" << std::endl;

    pdef->setStartAndGoalStates(start, goal, 0.5);

    std::cout << "\n[INFO] Creating MAB-RRT planner for Occupancy Grid..." << std::endl;
    if (debug) {
        std::cout << "[INFO] DEBUG MODE ENABLED" << std::endl;
    }
    std::cout << "[INFO] Config file: " << configPath << std::endl;
    
    std::shared_ptr<og::MAB_RRT> planner;
    std::shared_ptr<og::MAB_RRT_DEBUG> debugPlanner;
    
    if (debug) {
        std::cout << "[DEBUG] Creating MAB_RRT_DEBUG planner instance..." << std::endl;
        debugPlanner = std::make_shared<og::MAB_RRT_DEBUG>(si, configPath);
        planner = debugPlanner;
        std::cout << "[DEBUG] MAB_RRT_DEBUG planner created successfully" << std::endl;
    } else {
        std::cout << "[INFO] Creating standard MAB_RRT planner (non-debug)" << std::endl;
        planner = std::make_shared<og::MAB_RRT>(si, configPath);
    }
    
    planner->setProblemDefinition(pdef);
    planner->setup();

    std::cout << "[INFO] Starting planning with timeout: " << timeout << " seconds..." << std::endl;
    std::cout << "--------------------------------------------------" << std::endl;

    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeout));

    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "\n[RESULT] Planning status: " << status.asString() << std::endl;

    if (status == ob::PlannerStatus::EXACT_SOLUTION || 
        status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout << "[RESULT] Solution found!" << std::endl;
        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (path)
        {
            std::cout << "[RESULT] Path length: " << path->length() << std::endl;
            std::cout << "[RESULT] Path states: " << path->getStateCount() << std::endl;
            
            // In debug mode, the debug planner already exports path with sampler info
            // Only save basic path if not in debug mode
            if (!debug) {
                // Save path to CSV file
                std::filesystem::path gridFilePath(gridFile);
                if (!gridFilePath.is_absolute()) {
                    gridFilePath = std::filesystem::absolute(gridFilePath);
                }
                
                std::filesystem::path targetDir = gridFilePath.parent_path();
                std::string pathFile = targetDir.string() + "/bugtrap_path.csv";
                
                std::ofstream outFile(pathFile);
                if (outFile.is_open())
                {
                    outFile << std::fixed << std::setprecision(6);
                    outFile << "x,y\n";
                    for (std::size_t i = 0; i < path->getStateCount(); ++i)
                    {
                        const auto* state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
                        outFile << state->values[0] << "," << state->values[1] << "\n";
                    }
                    outFile.close();
                    std::cout << "[INFO] Path saved to: " << pathFile << std::endl;
                }
                else
                {
                    std::cerr << "[WARNING] Could not save path to file: " << pathFile << std::endl;
                }
            } else {
                std::cout << "[INFO] Path with sampler info exported by debug planner" << std::endl;
            }
        }
    }
    else
    {
        std::cout << "[RESULT] No solution found within timeout." << std::endl;
    }

    ob::PlannerData pdata(si);
    planner->getPlannerData(pdata);
    std::cout << "\n[STATS] Tree vertices: " << pdata.numVertices() << std::endl;
    std::cout << "[STATS] Tree edges: " << pdata.numEdges() << std::endl;
    
    // Export debug data if in debug mode
    if (debug && debugPlanner) {
        std::string samplesFile = "bugtrap_samples_debug.csv";
        debugPlanner->exportSampleData(samplesFile);
        std::cout << "[DEBUG] Sample data exported to: " << samplesFile << std::endl;
    }

    std::cout << "\n==================================================" << std::endl;
    std::cout << "    Occupancy Grid Demo Complete!" << std::endl;
    std::cout << "==================================================" << std::endl;
}

void printUsage(const char* programName)
{
    std::cout << "\nMAB-RRT Planner Demo" << std::endl;
    std::cout << "======================\n" << std::endl;
    std::cout << "Usage: " << programName << " [options]" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  --config <path>        Path to YAML config file (required)" << std::endl;
    std::cout << "  --timeout <seconds>    Planning timeout in seconds (default: 10.0)" << std::endl;
    std::cout << "  --planner <name>       Planner scenario to use (default: mab-ssrrt)" << std::endl;
    std::cout << "  --occupancy-grid <file> Path to occupancy grid CSV file (for 2D grid scenario)" << std::endl;
    std::cout << "  --debug                Enable debug mode (extensive logging + sample tracking)" << std::endl;
    std::cout << "  --help                 Show this help message" << std::endl;
    std::cout << "\nAvailable scenarios:" << std::endl;
    std::cout << "  mab-ssrrt      Standard 6D Assembly Demo" << std::endl;
    std::cout << "  bugtrap        2D Bug Trap (Start inside U-shape, Goal outside)" << std::endl;
    std::cout << "  occupancy-grid 2D Occupancy Grid (requires --occupancy-grid)" << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  " << programName << " --config ./cfg.yaml --timeout 15 --planner bugtrap" << std::endl;
    std::cout << "  " << programName << " --config ./cfg.yaml --planner bugtrap --debug" << std::endl;
    std::cout << "  " << programName << " --config ./cfg.yaml --planner occupancy-grid --occupancy-grid ./grid.csv" << std::endl;
}

int main(int argc, char** argv)
{
    std::string configPath;
    double timeout = 10.0;
    std::string plannerName = "mab-ssrrt";
    std::string occupancyGridFile;
    bool debug = false;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        
        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "--config" && i + 1 < argc)
        {
            configPath = argv[++i];
        }
        else if (arg == "--timeout" && i + 1 < argc)
        {
            timeout = std::stod(argv[++i]);
        }
        else if (arg == "--planner" && i + 1 < argc)
        {
            plannerName = argv[++i];
        }
        else if (arg == "--occupancy-grid" && i + 1 < argc)
        {
            occupancyGridFile = argv[++i];
        }
        else if (arg == "--debug")
        {
            debug = true;
        }
    }

    // Default to demos/disassembly/benchmark_baseline.yaml if not specified
    if (configPath.empty())
    {
        configPath = "demos/disassembly/benchmark_baseline.yaml";
    }
    
    // Note: We use the config path as-is. The planner will handle path resolution.
    // No need to convert to absolute path here - it can cause buffer overflow issues.

    try
    {
        if (plannerName == "occupancy-grid" || plannerName == "grid")
        {
            if (occupancyGridFile.empty())
            {
                std::cerr << "Error: --occupancy-grid option is required for occupancy-grid scenario" << std::endl;
                printUsage(argv[0]);
                return 1;
            }
            runOccupancyGridDemo(configPath, occupancyGridFile, timeout, debug);
        }
        else if (plannerName == "bugtrap" || plannerName == "2d")
        {
            runBugTrapDemo(configPath, timeout, debug);
        }
        else
        {
            runMAB_RRT_Demo(configPath, timeout);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
