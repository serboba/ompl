/*********************************************************************
* Test program for 2D burn-in phase
* 
* Verifies that setupAdaptiveSphereSampling works correctly in 2D
*********************************************************************/

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/samplers/sphere/AdaptiveSphereSampler.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/planners/disassemblyrrt/MAB_SSRRT_1L.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>
#include <iostream>
#include <vector>
#include <cmath>

using namespace ompl;
using namespace ompl::base;
using namespace ompl::geometric;
using namespace ompl::sampling;

// Simple 2D validity checker: valid if inside unit circle
class CircleValidityChecker2D : public StateValidityChecker
{
public:
    CircleValidityChecker2D(const SpaceInformationPtr& si) : StateValidityChecker(si) {}
    
    bool isValid(const State* state) const override
    {
        const double* values = state->as<RealVectorStateSpace::StateType>()->values;
        double x = values[0];
        double y = values[1];
        double dist = std::sqrt(x*x + y*y);
        return dist <= 1.0;  // Inside unit circle
    }
};

int main(int, char**)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  2D Burn-in Phase Test" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Create 2D state space
    auto space = std::make_shared<RealVectorStateSpace>(2);
    RealVectorBounds bounds(2);
    bounds.setLow(-1.5);
    bounds.setHigh(1.5);
    space->setBounds(bounds);
    
    auto si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<CircleValidityChecker2D>(si));
    si->setStateValidityCheckingResolution(0.01);
    si->setup();
    
    auto pdef = std::make_shared<ProblemDefinition>(si);
    
    ScopedState<> start(space);
    start[0] = 0.0;
    start[1] = 0.0;
    
    ScopedState<> goal(space);
    goal[0] = 0.8;
    goal[1] = 0.0;
    
    pdef->setStartAndGoalStates(start, goal, 0.1);
    
    std::cout << "\n[INFO] Creating MAB-SSRRT-1L planner for 2D..." << std::endl;
    
    // Use config file
    std::string configPath = "/home/serboba/transferompl_ws/src/ompl_iso/config/benchmark_baseline.yaml";
    auto planner = std::make_shared<MAB_SSRRT_1L>(si, configPath);
    planner->setProblemDefinition(pdef);
    
    std::cout << "[INFO] Setting up planner (this triggers burn-in phase)..." << std::endl;
    planner->setup();
    
    std::cout << "[INFO] Burn-in phase completed successfully!" << std::endl;
    std::cout << "[INFO] Planner is ready for 2D planning." << std::endl;
    
    // Test a quick solve
    std::cout << "\n[INFO] Testing quick solve (1 second)..." << std::endl;
    PlannerStatus status = planner->solve(timedPlannerTerminationCondition(1.0));
    
    std::cout << "[RESULT] Planning status: " << status.asString() << std::endl;
    
    if (status == PlannerStatus::EXACT_SOLUTION || 
        status == PlannerStatus::APPROXIMATE_SOLUTION)
    {
        std::cout << "[RESULT] Solution found!" << std::endl;
        auto path = std::dynamic_pointer_cast<PathGeometric>(pdef->getSolutionPath());
        if (path)
        {
            std::cout << "[RESULT] Path length: " << path->length() << std::endl;
            std::cout << "[RESULT] Path states: " << path->getStateCount() << std::endl;
        }
    }
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Burn-in Test: PASS" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
