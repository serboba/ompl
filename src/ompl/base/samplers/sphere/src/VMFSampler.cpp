/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/base/samplers/sphere/VMFSampler.h>
#include <ompl/datastructures/statistics/VMFMixtureModel.h>

using namespace ompl::sampling;
using namespace ompl::statistics;

VMFSampler::VMFSampler()
{
}

std::vector<VMFSampler::Point> VMFSampler::sample(int numSamples, double radius, std::mt19937& rng)
{
    std::vector<Point> samples;
    samples.reserve(numSamples);

    if (components_.empty())
        return samples;

    for (int i = 0; i < numSamples; ++i)
    {
        int compID = selectComponent(rng);
        // Default sampling without kappa multiplier
        samples.push_back(sampleFromComponent(compID, radius, 1.0, rng));
    }

    return samples;
}

VMFSampler::Point VMFSampler::sampleFromComponent(int componentID, double radius, double kappaMultiplier, std::mt19937& rng)
{
    if (componentID < 0 || componentID >= static_cast<int>(components_.size()))
        throw std::out_of_range("Component ID out of range");

    auto& component = components_[componentID];
    VMFComponent tempComp = component;
    tempComp.kappa *= kappaMultiplier;

    Point sample = VMFDistribution::sample(tempComp, rng);

    // Scale to radius
    sample.x *= radius;
    sample.y *= radius;
    sample.z *= radius;
    
    // Metadata
    sample.sampledAtRadius = radius;
    sample.sampledAtKappa = tempComp.kappa;
    sample.componentID = componentID;

    return sample;
}

void VMFSampler::fit(const std::vector<Point>& points, int K, int maxIter, double tol, std::mt19937& rng)
{
    // VMFMixtureModel expects non-const reference, so we need to copy
    std::vector<geometry::Point3D> pointsCopy = points;
    components_ = VMFMixtureModel::fitVMFMixtureModel(pointsCopy, K, maxIter, tol, rng);
}

int VMFSampler::selectComponent(std::mt19937& rng) const
{
    return VMFMixtureModel::selectComponent(components_, rng);
}

void VMFSampler::updateComponentCenter(int componentID)
{
    if (componentID < 0 || componentID >= static_cast<int>(components_.size()))
        return;

    auto& comp = components_[componentID];
    if (comp.points.empty()) return;

    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (const auto& p : comp.points)
    {
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
    }

    double norm = std::sqrt(sum_x * sum_x + sum_y * sum_y + sum_z * sum_z);
    if (norm > 1e-6)
    {
        comp.mu_x = sum_x / norm;
        comp.mu_y = sum_y / norm;
        comp.mu_z = sum_z / norm;
    }
}
