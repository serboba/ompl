/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/base/samplers/sphere/FibonacciSphereSampler.h>
#include <cmath>

using namespace ompl::sampling;
using namespace ompl::geometry;

FibonacciSphereSampler::FibonacciSphereSampler(double jitterRadius)
    : jitterRadius_(jitterRadius)
{
}

std::vector<Point3D> FibonacciSphereSampler::sample(int numSamples, double radius, std::mt19937& rng)
{
    std::vector<Point3D> points;
    points.reserve(numSamples);

    std::uniform_real_distribution<double> uniform_0_1(0.0, 1.0);

    for (int i = 0; i < numSamples; i++)
    {
        // Compute latitude
        double t = (double)i / (numSamples - 1); // t ranges from 0 to 1
        double latitude = std::acos(1 - 2 * t) - M_PI_2; // Latitude in radians (-π/2 to π/2)
        latitude += (uniform_0_1(rng) - 0.5) * jitterRadius_; // Jitter in radians

        // Compute longitude using golden angle increment
        double golden_angle = M_PI * (3.0 - std::sqrt(5.0)); // Approximately 2.399963
        double longitude = fmod(golden_angle * i, 2.0 * M_PI); // Longitude in [0, 2π]
        longitude += (uniform_0_1(rng) - 0.5) * jitterRadius_; // Jitter in radians

        // Convert spherical coordinates to Cartesian coordinates
        double x = radius * std::cos(latitude) * std::cos(longitude);
        double y = radius * std::cos(latitude) * std::sin(longitude);
        double z = radius * std::sin(latitude);

        // Store the point
        Point3D p{x, y, z};
        p.sampledAtRadius = radius;
        points.push_back(p);
    }
    return points;
}

void FibonacciSphereSampler::setJitterRadius(double jitter)
{
    jitterRadius_ = jitter;
}

double FibonacciSphereSampler::getJitterRadius() const
{
    return jitterRadius_;
}
