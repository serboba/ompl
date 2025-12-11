/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_SAMPLING_VMF_SAMPLER_H
#define OMPL_DATASTRUCTURES_SAMPLING_VMF_SAMPLER_H

#include <ompl/base/samplers/sphere/SphereSampler.h>
#include <ompl/datastructures/statistics/VMFDistribution.h>
#include <vector>
#include <memory>

namespace ompl
{
    namespace sampling
    {
        /** @brief Von Mises-Fisher distribution sampler */
        class VMFSampler : public SphereSampler
        {
        public:
            using VMFComponent = statistics::VMFComponent;
            using Point = geometry::Point3D;

            VMFSampler();

            /** @brief Sample multiple points from the mixture model */
            std::vector<Point> sample(int numSamples, double radius, std::mt19937& rng) override;

            /** @brief Sample a single point from a specific component */
            Point sampleFromComponent(int componentID, double radius, double kappaMultiplier, std::mt19937& rng);

            /** @brief Fit the mixture model to the given points */
            void fit(const std::vector<Point>& points, int K, int maxIter, double tol, std::mt19937& rng);

            /** @brief Select a component based on weights */
            int selectComponent(std::mt19937& rng) const;

            /** @brief Update a component's center based on new points */
            void updateComponentCenter(int componentID);

            // Accessors
            const std::vector<VMFComponent>& getComponents() const { return components_; }
            std::vector<VMFComponent>& getComponents() { return components_; }

        private:
            std::vector<VMFComponent> components_;
        };
    }
}

#endif // OMPL_DATASTRUCTURES_SAMPLING_VMF_SAMPLER_H
