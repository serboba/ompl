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

/* Author: Servet Bora Bayraktar - MAB-SSRRT Standalone Demo */

/**
 * @file RRTFINALIZEDDemo.cpp
 * @brief Standalone demo for MAB-SSRRT-1L planner
 * * This demo tests the MAB-SSRRT-1L planner in an isolated OMPL environment:
 * - MAB-SSRRT-1L: Single-layer flat MAB variant (3 arms competing directly)
 * * Supports:
 * - 6D: Constrained Assembly (RPY + XYZ)
 * - 2D: Bug Trap Scenario (XY) - Hard local minimum
 */

#include <iostream>
#include <cmath>
#include <memory>
#include <fstream>
#include <iomanip>

// OMPL includes
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
 
// MAB-SSRRT planner
#include <ompl/geometric/planners/disassemblyrrt/MAB_SSRRT_1L.h>
#include <ompl/geometric/planners/disassemblyrrt/MAB_SSRRT_1L_DEBUG.h>
 
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
  * * Defines a "U" shaped trap containing the start state.
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
  * @brief Runs the MAB-SSRRT-1L (6D Assembly)
  */
 void runMAB_SSRRT_1L_Demo(const std::string& configPath, double timeout = 10.0)
 {
     std::cout << "\n==================================================" << std::endl;
     std::cout << "    MAB-SSRRT-1L (6D Assembly) Demo" << std::endl;
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
 
     std::cout << "\n[INFO] Creating MAB-SSRRT-1L planner..." << std::endl;
     auto planner = std::make_shared<og::MAB_SSRRT_1L>(si, configPath);
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
  * @brief Runs the MAB-SSRRT-1L planner demo in a 2D Bug Trap
  * * Scenario:
  * - Start is at (0,0) inside a U-shaped obstacle.
  * - Goal is at (10,0) outside the obstacle.
  * - Direct path is blocked.
  * - Planner must explore "backwards" (negative x) to escape.
  */
 void runBugTrapDemo(const std::string& configPath, double timeout = 10.0, bool debug = false)
 {
     std::cout << "\n==================================================" << std::endl;
     std::cout << "    MAB-SSRRT-1L 2D Bug Trap Demo" << std::endl;
     std::cout << "==================================================" << std::endl;
     std::cout << "[INFO] Algorithm: Flat single-layer MAB" << std::endl;
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
 
    std::cout << "\n[INFO] Creating MAB-SSRRT-1L planner for Bug Trap..." << std::endl;
    if (debug) {
        std::cout << "[INFO] DEBUG MODE ENABLED" << std::endl;
    }
    std::cout << "[INFO] Config file: " << configPath << std::endl;
    
    std::shared_ptr<og::MAB_SSRRT_1L> planner;
    std::shared_ptr<og::MAB_SSRRT_1L_DEBUG> debugPlanner;
    
    if (debug) {
        debugPlanner = std::make_shared<og::MAB_SSRRT_1L_DEBUG>(si, configPath);
        planner = debugPlanner;  // MAB_SSRRT_1L_DEBUG inherits from MAB_SSRRT_1L
    } else {
        planner = std::make_shared<og::MAB_SSRRT_1L>(si, configPath);
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
                // Save path to file for visualization
                // Try multiple locations
                std::vector<std::string> possiblePaths = {
                    "bugtrap_path.csv",
                    "../bugtrap_path.csv",
                    "../../bugtrap_path.csv",
                    "/tmp/bugtrap_path.csv"
                };
                
                bool saved = false;
                for (const auto& pathFile : possiblePaths)
                {
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
                        saved = true;
                        break;
                    }
                }
                
                if (!saved)
                {
                    std::cerr << "[WARNING] Could not save path to file!" << std::endl;
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
    
    // Export debug data if in debug mode
    if (debug && debugPlanner) {
        debugPlanner->exportSampleData("bugtrap_samples_debug.csv");
        std::cout << "[DEBUG] Sample data exported to: bugtrap_samples_debug.csv" << std::endl;
    }

    std::cout << "\n==================================================" << std::endl;
    std::cout << "    Bug Trap Demo Complete!" << std::endl;
    std::cout << "==================================================" << std::endl;
}
 
 void printUsage(const char* programName)
 {
     std::cout << "\nMAB-SSRRT Planner Demo" << std::endl;
     std::cout << "======================\n" << std::endl;
     std::cout << "Usage: " << programName << " [options]" << std::endl;
     std::cout << "\nOptions:" << std::endl;
    std::cout << "  --config <path>     Path to YAML config file (required)" << std::endl;
    std::cout << "  --timeout <seconds> Planning timeout in seconds (default: 10.0)" << std::endl;
    std::cout << "  --planner <name>    Planner to use (default: mab-ssrrt-1l)" << std::endl;
    std::cout << "  --debug             Enable debug mode (extensive logging + sample tracking)" << std::endl;
    std::cout << "  --help              Show this help message" << std::endl;
     std::cout << "\nAvailable planners:" << std::endl;
     std::cout << "  mab-ssrrt-1l   Standard 6D Assembly Demo" << std::endl;
     std::cout << "  bugtrap        2D Bug Trap (Start inside U-shape, Goal outside)" << std::endl;
     std::cout << "\nExample:" << std::endl;
     std::cout << "  " << programName << " --config ./cfg.yaml --timeout 15 --planner bugtrap" << std::endl;
 }
 
int main(int argc, char** argv)
{
    std::string configPath;
    double timeout = 10.0;
    std::string plannerName = "mab-ssrrt-1l";
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
        else if (arg == "--debug")
        {
            debug = true;
        }
    }
 
     if (configPath.empty())
     {
         std::cerr << "Error: --config <path> is required" << std::endl;
         printUsage(argv[0]);
         return 1;
     }
 
    try
    {
        if (plannerName == "bugtrap" || plannerName == "2d")
        {
            runBugTrapDemo(configPath, timeout, debug);
        }
        else
        {
            runMAB_SSRRT_1L_Demo(configPath, timeout);
        }
    }
     catch (const std::exception& e)
     {
         std::cerr << "Error: " << e.what() << std::endl;
         return 1;
     }
 
     return 0;
 }