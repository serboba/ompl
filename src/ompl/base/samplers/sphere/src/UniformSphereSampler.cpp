/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/base/samplers/sphere/UniformSphereSampler.h>
#include <cmath>

using namespace ompl::sampling;
using namespace ompl::geometry;

std::vector<Point3D> UniformSphereSampler::sample(int numSamples, double radius, std::mt19937& rng)
{
    std::vector<Point3D> points;
    points.reserve(numSamples);

    std::uniform_real_distribution<double> uniform_0_1(0.0, 1.0);

    for (int i = 0; i < numSamples; ++i)
    {
        double z = 2.0 * uniform_0_1(rng) - 1.0; // Uniform in [-1, 1]
        double theta = 2.0 * M_PI * uniform_0_1(rng); // Uniform in [0, 2*pi]

        // Compute spherical to Cartesian coordinates
        double r = std::sqrt(1.0 - z * z); // Radius in x-y plane
        double x = radius * r * std::cos(theta);
        double y = radius * r * std::sin(theta);
        double z_scaled = radius * z; // Scale z by radius

        Point3D p{x, y, z_scaled};
        p.sampledAtRadius = radius;
        points.push_back(p);
    }
    return points;
}
