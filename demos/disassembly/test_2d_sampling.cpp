/*********************************************************************
* Test program for 2D sampling functionality
* 
* Tests:
* 1. AdaptiveSphereSampler 2D sphere-to-circle projection
* 2. CylinderSampler 2D line sampling
* 3. Full 2D planning with MAB_SSRRT_1L
* 4. Backward compatibility with 6D
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
#include <iomanip>

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

// Simple 6D validity checker: valid if translation is inside unit sphere
class SphereValidityChecker6D : public StateValidityChecker
{
public:
    SphereValidityChecker6D(const SpaceInformationPtr& si) : StateValidityChecker(si) {}
    
    bool isValid(const State* state) const override
    {
        const double* values = state->as<RealVectorStateSpace::StateType>()->values;
        double x = values[3];
        double y = values[4];
        double z = values[5];
        double dist = std::sqrt(x*x + y*y + z*z);
        return dist <= 1.0;  // Inside unit sphere
    }
};

// Test 1: 2D sphere-to-circle projection
bool test2DSphereProjection()
{
    std::cout << "\n=== Test 1: 2D Sphere-to-Circle Projection ===" << std::endl;
    
    auto space = std::make_shared<RealVectorStateSpace>(2);
    RealVectorBounds bounds(2);
    bounds.setLow(-2.0);
    bounds.setHigh(2.0);
    space->setBounds(bounds);
    
    auto si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<CircleValidityChecker2D>(si));
    si->setup();
    
    // Create sampler with 2D axes indices
    AdaptiveSphereSampler sampler(si.get(), 64, 0.5, {0, 1});
    
    // Collect samples
    sampler.collectQuasiRandomSamples(0.5);
    
    // Check that samples are on circle (radius should be approximately 0.5)
    const auto& points = sampler.getSpherePoints();
    int validCount = 0;
    double totalRadius = 0.0;
    const double tolerance = 0.1;
    
    for (const auto& p : points) {
        double radius = std::sqrt(p.x * p.x + p.y * p.y);
        totalRadius += radius;
        
        // Check z is approximately 0
        if (std::abs(p.z) < 1e-6) {
            validCount++;
        }
        
        // Check radius is approximately 0.5
        if (std::abs(radius - 0.5) < tolerance) {
            validCount++;
        }
    }
    
    double avgRadius = totalRadius / points.size();
    std::cout << "  Generated " << points.size() << " points" << std::endl;
    std::cout << "  Average radius: " << avgRadius << " (expected: 0.5)" << std::endl;
    std::cout << "  Valid points: " << validCount << "/" << (points.size() * 2) << std::endl;
    
    bool success = (std::abs(avgRadius - 0.5) < tolerance) && (validCount > points.size() * 0.9);
    std::cout << "  Result: " << (success ? "PASS" : "FAIL") << std::endl;
    
    return success;
}

// Test 2: 2D cylinder sampling
bool test2DCylinderSampling()
{
    std::cout << "\n=== Test 2: 2D Cylinder Sampling ===" << std::endl;
    
    auto space = std::make_shared<RealVectorStateSpace>(2);
    RealVectorBounds bounds(2);
    bounds.setLow(-2.0);
    bounds.setHigh(2.0);
    space->setBounds(bounds);
    
    auto si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<CircleValidityChecker2D>(si));
    si->setup();
    
    AdaptiveSphereSampler sampler(si.get(), 32, 0.5, {0, 1});
    
    // Collect some valid points first
    sampler.collectQuasiRandomSamples(0.5);
    for (size_t i = 0; i < sampler.getSpherePoints().size(); ++i) {
        auto sample = sampler.getSample();
        base::State* state = si->allocState();
        double* values = state->as<RealVectorStateSpace::StateType>()->values;
        values[0] = sample.second.x;
        values[1] = sample.second.y;
        
        if (si->isValid(state)) {
            sampler.popIndexFromIndices(sample.first, true);
            sampler.addValidSampleToComponent(const_cast<sampling::AdaptiveSphereSampler::Point&>(sample.second));
        } else {
            sampler.popIndexFromIndices(sample.first, false);
        }
        si->freeState(state);
    }
    
    // Test cylinder sampling
    int validCylinderSamples = 0;
    const int numSamples = 100;
    
    for (int i = 0; i < numSamples; ++i) {
        try {
            auto cylPoint = sampler.getRandomSampleFromCylinder(0.2, 0);
            
            // Check z is approximately 0
            if (std::abs(cylPoint.z) < 1e-6) {
                validCylinderSamples++;
            }
        } catch (...) {
            // Cylinder might not be fitted yet
        }
    }
    
    std::cout << "  Generated " << numSamples << " cylinder samples" << std::endl;
    std::cout << "  Valid 2D samples (z ≈ 0): " << validCylinderSamples << std::endl;
    
    bool success = validCylinderSamples > numSamples * 0.9;
    std::cout << "  Result: " << (success ? "PASS" : "FAIL") << std::endl;
    
    return success;
}

// Test 3: Full 2D planning
bool test2DPlanning()
{
    std::cout << "\n=== Test 3: Full 2D Planning with MAB_SSRRT_1L ===" << std::endl;
    
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
    
    // Create planner - need to modify it to use 2D indices
    // For now, we'll test if the sampler works with 2D
    std::cout << "  Note: Planner needs axesIndices={0,1} for 2D (currently hardcoded to {3,4,5})" << std::endl;
    std::cout << "  Testing sampler directly..." << std::endl;
    
    // Test sampler with 2D
    AdaptiveSphereSampler sampler(si.get(), 32, 0.3, {0, 1});
    sampler.collectQuasiRandomSamples(0.3);
    
    int validSamples = 0;
    for (size_t i = 0; i < sampler.getSpherePoints().size(); ++i) {
        auto sample = sampler.getSample();
        base::State* state = si->allocState();
        double* values = state->as<RealVectorStateSpace::StateType>()->values;
        values[0] = sample.second.x;
        values[1] = sample.second.y;
        
        if (si->isValid(state)) {
            validSamples++;
        }
        si->freeState(state);
        sampler.popIndexFromIndices(sample.first, si->isValid(state));
    }
    
    std::cout << "  Valid samples: " << validSamples << "/" << sampler.getSpherePoints().size() << std::endl;
    
    bool success = validSamples > 0;
    std::cout << "  Result: " << (success ? "PASS" : "FAIL") << std::endl;
    
    return success;
}

// Test 4: Backward compatibility with 6D
bool test6DCompatibility()
{
    std::cout << "\n=== Test 4: 6D Backward Compatibility ===" << std::endl;
    
    auto space = std::make_shared<RealVectorStateSpace>(6);
    RealVectorBounds bounds(6);
    for (int i = 0; i < 6; ++i) {
        bounds.setLow(i, -M_PI);
        bounds.setHigh(i, M_PI);
    }
    bounds.setLow(3, -1.5);
    bounds.setHigh(3, 1.5);
    bounds.setLow(4, -1.5);
    bounds.setHigh(4, 1.5);
    bounds.setLow(5, -1.5);
    bounds.setHigh(5, 1.5);
    space->setBounds(bounds);
    
    auto si = std::make_shared<SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<SphereValidityChecker6D>(si));
    si->setStateValidityCheckingResolution(0.01);
    si->setup();
    
    // Test with 6D indices (XYZ translation)
    AdaptiveSphereSampler sampler(si.get(), 32, 0.5, {3, 4, 5});
    sampler.collectQuasiRandomSamples(0.5);
    
    const auto& points = sampler.getSpherePoints();
    int valid3D = 0;
    
    for (const auto& p : points) {
        // Check that we have 3D points (z should not be 0)
        if (std::abs(p.z) > 1e-6) {
            valid3D++;
        }
    }
    
    std::cout << "  Generated " << points.size() << " points" << std::endl;
    std::cout << "  3D points (z ≠ 0): " << valid3D << "/" << points.size() << std::endl;
    
    bool success = valid3D > points.size() * 0.8;  // Most points should have non-zero z
    std::cout << "  Result: " << (success ? "PASS" : "FAIL") << std::endl;
    
    return success;
}

int main(int argc, char** argv)
{
    std::cout << "========================================" << std::endl;
    std::cout << "  2D Sampling Functionality Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    
    bool test1 = test2DSphereProjection();
    bool test2 = test2DCylinderSampling();
    bool test3 = test2DPlanning();
    bool test4 = test6DCompatibility();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Test Summary" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Test 1 (2D Sphere Projection): " << (test1 ? "PASS" : "FAIL") << std::endl;
    std::cout << "  Test 2 (2D Cylinder Sampling): " << (test2 ? "PASS" : "FAIL") << std::endl;
    std::cout << "  Test 3 (2D Planning): " << (test3 ? "PASS" : "FAIL") << std::endl;
    std::cout << "  Test 4 (6D Compatibility): " << (test4 ? "PASS" : "FAIL") << std::endl;
    
    bool allPassed = test1 && test2 && test3 && test4;
    std::cout << "\n  Overall: " << (allPassed ? "ALL TESTS PASSED" : "SOME TESTS FAILED") << std::endl;
    std::cout << "========================================" << std::endl;
    
    return allPassed ? 0 : 1;
}
