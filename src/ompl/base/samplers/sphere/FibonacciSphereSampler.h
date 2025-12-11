/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_SAMPLING_FIBONACCI_SPHERE_SAMPLER_H
#define OMPL_DATASTRUCTURES_SAMPLING_FIBONACCI_SPHERE_SAMPLER_H

#include <ompl/base/samplers/sphere/SphereSampler.h>

namespace ompl
{
    namespace sampling
    {
        /** @brief Fibonacci lattice sphere sampling with jitter */
        class FibonacciSphereSampler : public SphereSampler
        {
        public:
            FibonacciSphereSampler(double jitterRadius = 0.05);

            std::vector<geometry::Point3D> sample(int numSamples, double radius, std::mt19937& rng) override;

            void setJitterRadius(double jitter);
            double getJitterRadius() const;

        private:
            double jitterRadius_;
        };
    }
}

#endif // OMPL_DATASTRUCTURES_SAMPLING_FIBONACCI_SPHERE_SAMPLER_H
