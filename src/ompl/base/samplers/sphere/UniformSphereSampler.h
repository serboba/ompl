/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_SAMPLING_UNIFORM_SPHERE_SAMPLER_H
#define OMPL_DATASTRUCTURES_SAMPLING_UNIFORM_SPHERE_SAMPLER_H

#include <ompl/base/samplers/sphere/SphereSampler.h>

namespace ompl
{
    namespace sampling
    {
        /** @brief Uniform random sphere sampling */
        class UniformSphereSampler : public SphereSampler
        {
        public:
            std::vector<geometry::Point3D> sample(int numSamples, double radius, std::mt19937& rng) override;
        };
    }
}

#endif // OMPL_DATASTRUCTURES_SAMPLING_UNIFORM_SPHERE_SAMPLER_H
