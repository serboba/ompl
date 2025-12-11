/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/base/samplers/sphere/AdaptiveSphereSampler.h>
#include <ompl/datastructures/geometry/GeometryUtils.h>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <cmath>
#include <ompl/util/Exception.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
using namespace ompl::sampling;
using namespace ompl::geometry;
using namespace ompl::statistics;

AdaptiveSphereSampler::AdaptiveSphereSampler(const base::SpaceInformation *si, int sampleSize, double radius, const std::vector<int>& axesIndices)
    : ValidStateSampler(si),
      sampleSize_(sampleSize),
      radius_(radius),
      rng_(std::random_device{}()),
      axesIndices_(axesIndices)
{
    name_ = "AdaptiveSphereSampler";
}

bool AdaptiveSphereSampler::sample(base::State *state)
{
    // Basic implementation: sample uniformly from sphere
    auto points = uniSampler_.sample(1, radius_, rng_);
    if (points.empty()) return false;

    double* values = state->as<base::RealVectorStateSpace::StateType>()->values;
    
    if (axesIndices_.size() == 2) {
        // 2D case: Project 3D sphere point to 2D circle
        // Project (x, y, z) -> (x/r, y/r) * radius where r = sqrt(x² + y²)
        double x = points[0].x;
        double y = points[0].y;
        double r_xy = std::sqrt(x * x + y * y);
        
        if (r_xy > 1e-10) {  // Avoid division by zero
            // Normalize to unit circle, then scale by radius
            values[axesIndices_[0]] = (x / r_xy) * radius_;
            values[axesIndices_[1]] = (y / r_xy) * radius_;
        } else {
            // If r_xy is too small, sample uniformly on circle
            std::uniform_real_distribution<double> angleDist(0.0, 2.0 * M_PI);
            double theta = angleDist(rng_);
            values[axesIndices_[0]] = radius_ * std::cos(theta);
            values[axesIndices_[1]] = radius_ * std::sin(theta);
        }
        return true;
    }
    else if (axesIndices_.size() >= 3) {
        // 3D case: Use all three coordinates
        values[axesIndices_[0]] = points[0].x;
        values[axesIndices_[1]] = points[0].y;
        values[axesIndices_[2]] = points[0].z;
        return true;
    }
    return false;
}

bool AdaptiveSphereSampler::sampleNear(base::State *state, const base::State *near, const double distance)
{
    throw ompl::Exception("AdaptiveSphereSampler::sampleNear", "not implemented");
    return false;
}

void AdaptiveSphereSampler::collectQuasiRandomSamples(double radius)
{
    indices_.clear();
    spherePoints_.clear();

    // Use sub-samplers
    auto fibPoints = fibSampler_.sample(sampleSize_, radius, rng_);
    auto uniPoints = uniSampler_.sample(sampleSize_, radius, rng_);

    // Project points to 2D circle if needed
    if (axesIndices_.size() == 2) {
        // Project 3D sphere points to 2D circle
        for (auto& p : fibPoints) {
            projectSphereToCircle(p, radius);
        }
        for (auto& p : uniPoints) {
            projectSphereToCircle(p, radius);
        }
    }

    spherePoints_.insert(spherePoints_.end(), fibPoints.begin(), fibPoints.end());
    spherePoints_.insert(spherePoints_.end(), uniPoints.begin(), uniPoints.end());

    indices_.resize(spherePoints_.size());
    std::iota(indices_.begin(), indices_.end(), 0);

    if (spherePoints_.empty() || spherePoints_.size() > 2 * sampleSize_)
        throw std::out_of_range(
            "spherePoints size(" + std::to_string(spherePoints_.size()) +
            ") is not correct after quasi-random sampling.");

    std::shuffle(indices_.begin(), indices_.end(), rng_);
}

std::pair<int, AdaptiveSphereSampler::Point> AdaptiveSphereSampler::getSample()
{
    if (indices_.empty()) {
        throw std::out_of_range("No indices available for sampling");
    }
    
    std::uniform_int_distribution<> randomIndex(0, indices_.size() - 1);
    int index = randomIndex(rng_);
    
    if (index < 0 || index >= static_cast<int>(indices_.size()))
        throw std::out_of_range("Sample index out of range");

    return {index, spherePoints_[indices_[index]]};
}

void AdaptiveSphereSampler::popIndexFromIndices(int index, bool valid)
{
    if (valid)
    {
        allPoints_cached_.push_back({spherePoints_[indices_[index]]});
    }

    if (index >= 0 && index < indices_.size())
        indices_.erase(indices_.begin() + index);
    else
        throw std::out_of_range("Index is out of range for erasing.");
}

void AdaptiveSphereSampler::addValidSampleToComponent(Point& p)
{
    if (p.sampledByCylinder != true)
    {
        auto& components = vmfSampler_.getComponents();
        if (p.componentID >= 0 && p.componentID < components.size()) {
            components.at(p.componentID).points.push_back(p);
            updateComponentCenter(p.componentID);
            
            if (p.sampledAtKappa > 0.0 && p.sampledAtKappa > components.at(p.componentID).kappa)
            {
                components.at(p.componentID).kappa = p.sampledAtKappa;
            }
        }
    }
    allValidPoints_.push_back(p);
    
    // Mark cache as dirty since we added a new point
    validPointsDirty_ = true;
    
    if (p.sampledAtRadius > bestRadius_) {
        bestRadius_ = p.sampledAtRadius;
        bestRadius = bestRadius_; // Update public member
    }
}

void AdaptiveSphereSampler::appendCachedValidSamples()
{
    for (auto& sample_point : allPoints_cached_)
    {
        allValidPoints_.push_back(sample_point);
    }
    allPoints_cached_.clear();
}

void AdaptiveSphereSampler::clearSamples()
{
    spherePoints_.clear();
    indices_.clear();
    allValidPoints_.clear();
    allPoints_cached_.clear();
    vmfSampler_.getComponents().clear();
    bestRadius_ = 0.0;
    bestRadius = 0.0;
}

void AdaptiveSphereSampler::clearCachedValidSamples()
{
    allPoints_cached_.clear();
}

void AdaptiveSphereSampler::clearValidPoints()
{
    allValidPoints_.clear();
    validPointsDirty_ = true;
}

double AdaptiveSphereSampler::getValidSampleRate() const
{
    if (spherePoints_.empty()) return 0.0;
    return static_cast<double>(allPoints_cached_.size()) / static_cast<double>(spherePoints_.size());
}

AdaptiveSphereSampler::Point AdaptiveSphereSampler::sampleFromVMF(int componentID, double newRadius, double kappaMultiplier)
{
    return vmfSampler_.sampleFromComponent(componentID, newRadius, kappaMultiplier, rng_);
}

AdaptiveSphereSampler::Point AdaptiveSphereSampler::getRandomSampleFromCylinder(double extensionHeight, int chosenDirection)
{
    // Use cached cylinder if available and valid
    if (hasCylinderAxis_)
    {
         cylinderSampler_.fitWithAxis(allValidPoints_, cylinderAxis_, cylinderSampler_.getRadiusOffsetMultiplier());
    }
    else if (validPointsDirty_)
    {
        cylinderSampler_.fit(allValidPoints_, pcaFilterTopPercent_);
        validPointsDirty_ = false;
    }
    
    Point sample = cylinderSampler_.sampleCylinder(extensionHeight, chosenDirection, cylinderSampler_.getSamplingRadiusMultiplier(), rng_);
    
    // Project to 2D circle if needed
    if (axesIndices_.size() == 2) {
        projectSphereToCircle(sample, sample.sampledAtRadius);
    }
    
    return sample;
}

AdaptiveSphereSampler::Point AdaptiveSphereSampler::getRandomSampleFromCylinder(double extensionHeight, int chosenDirection, const Eigen::Vector3d &axis)
{
    cylinderSampler_.fitWithAxis(allValidPoints_, axis, cylinderSampler_.getRadiusOffsetMultiplier());
    Point sample = cylinderSampler_.sampleCylinder(extensionHeight, chosenDirection, cylinderSampler_.getSamplingRadiusMultiplier(), rng_);
    
    // Project to 2D circle if needed
    if (axesIndices_.size() == 2) {
        projectSphereToCircle(sample, sample.sampledAtRadius);
    }
    
    return sample;
}

void AdaptiveSphereSampler::updateCylinderAxis()
{
    if (allValidPoints_.size() < 3) {
        return; // Not enough points for PCA
    }

    // 1. Compute PCA direction from current valid points
    Eigen::Vector3d newAxisRaw = CylinderSampler::computePCAAxis(allValidPoints_);

    // If this is the first axis, accept it as is
    if (!hasCylinderAxis_) {
        cylinderAxis_ = newAxisRaw;
        hasCylinderAxis_ = true;
        return;
    }

    // 2. Enforce sign consistency with previous axis
    double dotVal = cylinderAxis_.dot(newAxisRaw);

    // If new axis points in the opposite direction, flip it
    if (dotVal < 0.0) {
        newAxisRaw = -newAxisRaw;
    }

    // 3. Store the stabilized axis
    cylinderAxis_ = newAxisRaw;
}

std::vector<AdaptiveSphereSampler::VMFComponent> AdaptiveSphereSampler::fitVMFMixtureModel(int K, int maxIter, double tol)
{
    vmfSampler_.fit(allValidPoints_, K, maxIter, tol, rng_);
    return vmfSampler_.getComponents();
}

int AdaptiveSphereSampler::selectComponent()
{
    return vmfSampler_.selectComponent(rng_);
}

void AdaptiveSphereSampler::updateComponentCenter(int componentIndex)
{
    vmfSampler_.updateComponentCenter(componentIndex);
}

void AdaptiveSphereSampler::appendFibonacciSamples(double customRadius)
{
    double r = (customRadius == -1.0) ? radius_ : customRadius;
    auto points = fibSampler_.sample(sampleSize_, r, rng_);
    
    // Project points to 2D circle if needed
    if (axesIndices_.size() == 2) {
        for (auto& p : points) {
            projectSphereToCircle(p, r);
        }
    }
    
    spherePoints_.insert(spherePoints_.end(), points.begin(), points.end());
}

void AdaptiveSphereSampler::appendUniformSamples(double customRadius)
{
    double r = (customRadius == -1.0) ? radius_ : customRadius;
    auto points = uniSampler_.sample(sampleSize_, r, rng_);
    
    // Project points to 2D circle if needed
    if (axesIndices_.size() == 2) {
        for (auto& p : points) {
            projectSphereToCircle(p, r);
        }
    }
    
    spherePoints_.insert(spherePoints_.end(), points.begin(), points.end());
}

void AdaptiveSphereSampler::projectSphereToCircle(Point& p, double radius)
{
    // Project 3D sphere point (x, y, z) to 2D circle: (x/r, y/r) * radius
    // where r = sqrt(x² + y²)
    double x = p.x;
    double y = p.y;
    double r_xy = std::sqrt(x * x + y * y);
    
    if (r_xy > 1e-10) {  // Avoid division by zero
        // Normalize to unit circle, then scale by radius
        p.x = (x / r_xy) * radius;
        p.y = (y / r_xy) * radius;
        p.z = 0.0;  // Set z to 0 for 2D
    } else {
        // If r_xy is too small, sample uniformly on circle
        std::uniform_real_distribution<double> angleDist(0.0, 2.0 * M_PI);
        double theta = angleDist(rng_);
        p.x = radius * std::cos(theta);
        p.y = radius * std::sin(theta);
        p.z = 0.0;
    }
    // Preserve the sampledAtRadius
    p.sampledAtRadius = radius;
}
