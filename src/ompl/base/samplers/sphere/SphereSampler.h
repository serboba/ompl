/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_SAMPLING_SPHERE_SAMPLER_H
#define OMPL_DATASTRUCTURES_SAMPLING_SPHERE_SAMPLER_H

#include <ompl/datastructures/geometry/Point3D.h>
#include <vector>
#include <random>

namespace ompl
{
    namespace sampling
    {
        /** @brief Base interface for sphere sampling strategies */
        class SphereSampler
        {
        public:
            virtual ~SphereSampler() = default;

            /** Generate samples on a sphere */
            virtual std::vector<geometry::Point3D> sample(int numSamples, double radius, std::mt19937& rng) = 0;
        };
    }
}

#endif // OMPL_DATASTRUCTURES_SAMPLING_SPHERE_SAMPLER_H
