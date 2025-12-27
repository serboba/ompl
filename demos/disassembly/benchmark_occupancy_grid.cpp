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

/* Author: Servet Bora Bayraktar - Occupancy Grid Benchmarking */

/**
 * @file benchmark_occupancy_grid.cpp
 * @brief Benchmark script for comparing planners on occupancy grid environments
 * 
 * Benchmarks:
 * - MAB-SSRRT: Multi-Armed Bandit Sphere-Sampled RRT
 * - RRT: Standard RRT with uniform sampling
 * - RRT-Gaussian: RRT with Gaussian valid state sampler
 * - RRT-Bridge: RRT with Bridge test valid state sampler
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
#include <chrono>

// OMPL includes
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/BridgeTestValidStateSampler.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRT.h>

// MAB-SSRRT planner
#include <ompl/geometric/planners/disassemblyrrt/MAB_RRT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Forward declaration - reuse OccupancyGridValidityChecker from MAB_RRT_Demo
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
        const auto* realState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = realState->values[0];
        double y = realState->values[1];

        if (!si_->satisfiesBounds(state)) return false;

        // Convert world coordinates to grid indices
        // Grid is centered at (0, 0) in world coordinates
        // Visualization uses extent=[-W/2, W/2, H/2, -H/2] with origin='upper'
        // To match visualization:
        //   - World x: maps from [-W/2, W/2] to [0, W-1]
        //   - World y: maps from [-H/2, H/2] to [0, H-1]
        //     world(-H/2) → gridY = 0, world(+H/2) → gridY = H-1
        int gridX = static_cast<int>(std::floor((x - gridOffsetX_) / resolution_));
        int gridY = static_cast<int>(std::floor((y - gridOffsetY_) / resolution_));

        // Clamp to valid range
        if (gridX < 0) gridX = 0;
        if (gridX >= gridWidth_) gridX = gridWidth_ - 1;
        if (gridY < 0) gridY = 0;
        if (gridY >= gridHeight_) gridY = gridHeight_ - 1;

        // NO Y-axis inversion needed - the formula already matches visualization

        // Check bounds
        if (gridX < 0 || gridX >= gridWidth_ || gridY < 0 || gridY >= gridHeight_) {
            return false;
        }

        // Check if cell is occupied (100 = occupied, 0 = free)
        int cellValue = grid_[gridY * gridWidth_ + gridX];
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

        // Read header lines for start and goal
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '\n' || line[0] == '\r') continue;

            if (line[0] == '#') {
                std::string content = line.substr(1);
                while (!content.empty() && (content[0] == ' ' || content[0] == '\t' || content[0] == '\r' || content[0] == '\n')) {
                    content = content.substr(1);
                }
                while (!content.empty() && (content.back() == ' ' || content.back() == '\t' || content.back() == '\r' || content.back() == '\n')) {
                    content.pop_back();
                }
                
                std::vector<std::string> tokens;
                std::istringstream iss(content);
                std::string token;
                while (std::getline(iss, token, ',')) {
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
                            std::cerr << "[WARNING] Failed to parse start coordinates: " << e.what() << std::endl;
                        }
                    } else if (keyword == "goal") {
                        try {
                            goalX_ = std::stod(xStr);
                            // CSV coordinates are already in world coordinates (centered at origin)
                            // No conversion needed - use coordinates directly
                            goalY_ = std::stod(yStr);
                        } catch (const std::exception& e) {
                            std::cerr << "[WARNING] Failed to parse goal coordinates: " << e.what() << std::endl;
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
        // CSV row 0 is at TOP (image coordinates)
        // With invertY=true, we need:
        //   - World (+H/2) → Grid row 0 (top of CSV)
        //   - World (0) → Grid row H/2 (center)
        //   - World (-H/2) → Grid row H-1 (bottom of CSV)
        // The gridOffsetY = -H/2 maps world (-H/2) to grid 0 (before invert)
        // After invertY, world (-H/2) → grid H-1 and world (+H/2) → grid 0
        gridOffsetX_ = -static_cast<double>(gridWidth_) / 2.0;
        gridOffsetY_ = -static_cast<double>(gridHeight_) / 2.0;
    }

    std::vector<int> grid_;
    int gridWidth_;
    int gridHeight_;
    double resolution_;
    double gridOffsetX_;
    double gridOffsetY_;
    double startX_, startY_, goalX_, goalY_;
    bool invertY_;
};

/**
 * @brief Benchmark result structure
 */
struct BenchmarkResult {
    std::string plannerName;
    int runNumber;
    bool solved;
    double planningTime;      // seconds
    double pathLength;
    int pathStates;
    int treeVertices;
    int treeEdges;
    std::string status;
};

/**
 * @brief Run a single benchmark with a specific planner
 */
BenchmarkResult runBenchmark(const std::string& plannerName,
                            ob::SpaceInformationPtr si,
                            const ob::ProblemDefinitionPtr& pdef,
                            const std::string& configPath,
                            double timeout,
                            int runNumber)
{
    BenchmarkResult result;
    result.plannerName = plannerName;
    result.runNumber = runNumber;
    result.solved = false;
    result.planningTime = 0.0;
    result.pathLength = 0.0;
    result.pathStates = 0;
    result.treeVertices = 0;
    result.treeEdges = 0;
    result.status = "UNKNOWN";

    ob::PlannerPtr planner;

    if (plannerName == "MAB-SSRRT") {
        planner = std::make_shared<og::MAB_RRT>(si, configPath);
    } else if (plannerName == "RRT" || plannerName == "RRT-Gaussian" || plannerName == "RRT-Bridge") {
        planner = std::make_shared<og::RRT>(si);
    } else {
        std::cerr << "[ERROR] Unknown planner: " << plannerName << std::endl;
        return result;
    }

    planner->setProblemDefinition(pdef);
    planner->setup();

    // Measure planning time
    auto startTime = std::chrono::high_resolution_clock::now();
    ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeout));
    auto endTime = std::chrono::high_resolution_clock::now();
    
    result.planningTime = std::chrono::duration<double>(endTime - startTime).count();
    result.status = status.asString();

    // Only consider exact solutions as successful
    if (status == ob::PlannerStatus::EXACT_SOLUTION) {
        result.solved = true;
        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (path) {
            result.pathLength = path->length();
            result.pathStates = path->getStateCount();
        }
    }

    // Get planner statistics
    ob::PlannerData pdata(si);
    planner->getPlannerData(pdata);
    result.treeVertices = pdata.numVertices();
    result.treeEdges = pdata.numEdges();

    return result;
}

/**
 * @brief Get grid information (dimensions, start, goal) from grid file
 */
void getGridInfo(const std::string& gridFile,
                 int& gridWidth, int& gridHeight,
                 double& startX, double& startY, double& goalX, double& goalY)
{
    // 2D state space (XY only) - temporary for loading grid info
    auto tempSpace = std::make_shared<ob::RealVectorStateSpace>(2);
    auto tempSi = std::make_shared<ob::SpaceInformation>(tempSpace);
    auto tempChecker = std::make_shared<OccupancyGridValidityChecker>(tempSi, gridFile, 1.0, true);
    
    tempChecker->getGridDimensions(gridWidth, gridHeight);
    tempChecker->getStartGoal(startX, startY, goalX, goalY);
}

/**
 * @brief Create a fresh environment for a specific planner type
 */
void createEnvironmentForPlanner(const std::string& gridFile,
                                 const std::string& plannerName,
                                 ob::SpaceInformationPtr& si,
                                 ob::ProblemDefinitionPtr& pdef)
{
    // Get grid info
    int gridWidth, gridHeight;
    double startX, startY, goalX, goalY;
    getGridInfo(gridFile, gridWidth, gridHeight, startX, startY, goalX, goalY);
    
    // 2D state space (XY only)
    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    
    // Set bounds based on centered grid
    ob::RealVectorBounds bounds(2);
    double halfWidth = static_cast<double>(gridWidth) / 2.0;
    double halfHeight = static_cast<double>(gridHeight) / 2.0;
    bounds.setLow(0, -halfWidth);
    bounds.setHigh(0, halfWidth);
    bounds.setLow(1, -halfHeight);
    bounds.setHigh(1, halfHeight);
    space->setBounds(bounds);
    
    // Create fresh SpaceInformation
    si = std::make_shared<ob::SpaceInformation>(space);
    
    // Create the checker
    auto checker = std::make_shared<OccupancyGridValidityChecker>(si, gridFile, 1.0, true);
    si->setStateValidityChecker(checker);
    // Use finer resolution for large grids (100x100) to ensure all cells are checked
    // Resolution should be at least 10x smaller than grid cell size
    si->setStateValidityCheckingResolution(0.005); // Finer resolution for large grids
    
    // Set sampler allocator based on planner type
    if (plannerName == "RRT-Gaussian") {
        si->setValidStateSamplerAllocator([](const ob::SpaceInformation* si) {
            return std::make_shared<ob::GaussianValidStateSampler>(si);
        });
    } else if (plannerName == "RRT-Bridge") {
        si->setValidStateSamplerAllocator([](const ob::SpaceInformation* si) {
            return std::make_shared<ob::BridgeTestValidStateSampler>(si);
        });
    }
    // For MAB-SSRRT and RRT, use default (no allocator set = uniform)
    
    si->setup();

    // Create fresh ProblemDefinition
    pdef = std::make_shared<ob::ProblemDefinition>(si);

    // Set start and goal from grid file
    ob::ScopedState<> start(space);
    start[0] = 0.0;  // Start is always at origin
    start[1] = 0.0;

    ob::ScopedState<> goal(space);
    goal[0] = goalX;
    goal[1] = goalY;

    // Debug: Check if start/goal are valid before setting
    bool startValid = si->isValid(start.get());
    bool goalValid = si->isValid(goal.get());
    std::cout << "[DEBUG] Start state (0, 0) validity: " << (startValid ? "VALID" : "INVALID") << std::endl;
    std::cout << "[DEBUG] Goal state (" << goalX << ", " << goalY << ") validity: " << (goalValid ? "VALID" : "INVALID") << std::endl;

    pdef->setStartAndGoalStates(start, goal, 0.5);
}

/**
 * @brief Save benchmark results to CSV
 */
void saveResults(const std::vector<BenchmarkResult>& results, const std::string& outputFile)
{
    std::ofstream outFile(outputFile);
    if (!outFile.is_open()) {
        std::cerr << "[ERROR] Failed to open output file: " << outputFile << std::endl;
        return;
    }

    // Write header
    outFile << "planner,run,solved,time_sec,path_length,path_states,tree_vertices,tree_edges,status\n";

    // Write results
    for (const auto& result : results) {
        outFile << std::fixed << std::setprecision(6);
        outFile << result.plannerName << ","
                << result.runNumber << ","
                << (result.solved ? 1 : 0) << ","
                << result.planningTime << ","
                << result.pathLength << ","
                << result.pathStates << ","
                << result.treeVertices << ","
                << result.treeEdges << ","
                << result.status << "\n";
    }

    outFile.close();
    std::cout << "[INFO] Results saved to: " << outputFile << std::endl;
}

/**
 * @brief Print summary statistics
 */
void printSummary(const std::vector<BenchmarkResult>& results)
{
    std::vector<std::string> planners = {"MAB-SSRRT", "RRT", "RRT-Gaussian", "RRT-Bridge"};
    
    std::cout << "\n==================================================" << std::endl;
    std::cout << "           BENCHMARK SUMMARY" << std::endl;
    std::cout << "==================================================" << std::endl;
    
    for (const auto& plannerName : planners) {
        std::vector<BenchmarkResult> plannerResults;
        for (const auto& r : results) {
            if (r.plannerName == plannerName) {
                plannerResults.push_back(r);
            }
        }
        
        if (plannerResults.empty()) continue;
        
        int solvedCount = 0;
        double totalTime = 0.0;
        double totalPathLength = 0.0;
        int totalVertices = 0;
        
        for (const auto& r : plannerResults) {
            if (r.solved) {
                solvedCount++;
                totalTime += r.planningTime;
                totalPathLength += r.pathLength;
            }
            totalVertices += r.treeVertices;
        }
        
        int totalRuns = plannerResults.size();
        double successRate = 100.0 * solvedCount / totalRuns;
        double avgTime = solvedCount > 0 ? totalTime / solvedCount : 0.0;
        double avgPathLength = solvedCount > 0 ? totalPathLength / solvedCount : 0.0;
        double avgVertices = static_cast<double>(totalVertices) / totalRuns;
        
        std::cout << "\n" << plannerName << ":" << std::endl;
        std::cout << "  Success Rate: " << std::fixed << std::setprecision(1) 
                  << successRate << "% (" << solvedCount << "/" << totalRuns << ")" << std::endl;
        if (solvedCount > 0) {
            std::cout << "  Avg Planning Time: " << std::setprecision(3) 
                      << avgTime << " sec" << std::endl;
            std::cout << "  Avg Path Length: " << std::setprecision(2) 
                      << avgPathLength << std::endl;
        }
        std::cout << "  Avg Tree Vertices: " << std::setprecision(0) 
                  << avgVertices << std::endl;
    }
    
    std::cout << "\n==================================================" << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <grid_file> <config_file> [options]" << std::endl;
        std::cout << "\nOptions:" << std::endl;
        std::cout << "  --runs <N>          Number of runs per planner (default: 10)" << std::endl;
        std::cout << "  --timeout <T>       Planning timeout in seconds (default: 10.0)" << std::endl;
        std::cout << "  --output <file>     Output CSV file (default: benchmark_results.csv)" << std::endl;
        return 1;
    }

    std::string gridFile = argv[1];
    std::string configFile = argv[2];
    int numRuns = 10;
    double timeout = 10.0;
    std::string outputFile = "benchmark_results.csv";

    // Parse optional arguments
    for (int i = 3; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--runs" && i + 1 < argc) {
            numRuns = std::stoi(argv[++i]);
        } else if (arg == "--timeout" && i + 1 < argc) {
            timeout = std::stod(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            outputFile = argv[++i];
        }
    }

    std::cout << "\n==================================================" << std::endl;
    std::cout << "    OCCUPANCY GRID PLANNER BENCHMARK" << std::endl;
    std::cout << "==================================================" << std::endl;
    std::cout << "[INFO] Grid file: " << gridFile << std::endl;
    std::cout << "[INFO] Config file: " << configFile << std::endl;
    std::cout << "[INFO] Runs per planner: " << numRuns << std::endl;
    std::cout << "[INFO] Timeout per run: " << timeout << " seconds" << std::endl;
    std::cout << "[INFO] Output file: " << outputFile << std::endl;

    // Run benchmarks
    std::vector<std::string> planners = {"MAB-SSRRT", "RRT", "RRT-Gaussian", "RRT-Bridge"};
    std::vector<BenchmarkResult> allResults;

    for (const auto& plannerName : planners) {
        std::cout << "\n[INFO] Benchmarking " << plannerName << "..." << std::endl;
        
        // Create fresh environment for this planner type
        ob::SpaceInformationPtr si;
        ob::ProblemDefinitionPtr pdef;
        try {
            createEnvironmentForPlanner(gridFile, plannerName, si, pdef);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to setup environment for " << plannerName << ": " << e.what() << std::endl;
            continue;
        }
        
        for (int run = 1; run <= numRuns; run++) {
            std::cout << "  Run " << run << "/" << numRuns << "... ";
            std::cout.flush();
            
            // Reset problem definition for each run
            pdef->clearSolutionPaths();
            
            BenchmarkResult result = runBenchmark(plannerName, si, pdef, configFile, timeout, run);
            allResults.push_back(result);
            
            if (result.solved) {
                std::cout << "SOLVED (time: " << std::fixed << std::setprecision(2) 
                          << result.planningTime << "s, length: " << result.pathLength << ")" << std::endl;
            } else {
                std::cout << "FAILED" << std::endl;
            }
        }
    }

    // Save results
    saveResults(allResults, outputFile);

    // Print summary
    printSummary(allResults);

    std::cout << "\nBenchmark complete!" << std::endl;
    return 0;
}

