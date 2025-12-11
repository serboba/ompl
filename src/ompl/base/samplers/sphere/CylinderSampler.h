/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_SAMPLING_CYLINDER_SAMPLER_H
#define OMPL_DATASTRUCTURES_SAMPLING_CYLINDER_SAMPLER_H

#include <ompl/base/samplers/sphere/SphereSampler.h>
#include <ompl/datastructures/geometry/Cylinder3D.h>
#include <Eigen/Dense>

namespace ompl
{
    namespace sampling
    {
        /** @brief Cylinder-based sampler */
        class CylinderSampler : public SphereSampler
        {
        public:
            using Cylinder = geometry::Cylinder3D;
            using Point = geometry::Point3D;

            CylinderSampler();

            /** @brief Sample multiple points (not typically used for cylinder but implemented for interface) */
            std::vector<Point> sample(int numSamples, double radius, std::mt19937& rng) override;

            /** @brief Sample a single point from the cylinder */
            Point sampleCylinder(double extensionHeight, int direction, double radiusMultiplier, std::mt19937& rng);

            /** @brief Fit cylinder to points */
            void fit(const std::vector<Point>& points, double pcaFilterTopPercent = 1.0);

            /** @brief Fit cylinder with explicit axis */
            void fitWithAxis(const std::vector<Point>& points, const Eigen::Vector3d& axis, double radiusOffsetMultiplier);

            /** @brief Compute PCA axis from points */
            static Eigen::Vector3d computePCAAxis(const std::vector<Point>& points);

            // Configuration
            void setRadiusOffsetMultiplier(double val) { radiusOffsetMultiplier_ = val; }
            void setSamplingRadiusMultiplier(double val) { samplingRadiusMultiplier_ = val; }
            double getSamplingRadiusMultiplier() const { return samplingRadiusMultiplier_; }
            double getRadiusOffsetMultiplier() const { return radiusOffsetMultiplier_; }
            
            // Accessors
            const Cylinder& getCylinder() const { return cylinder_; }
            bool hasValidCylinder() const { return hasValidCylinder_; }

        private:
            Cylinder cylinder_;
            bool hasValidCylinder_ = false;
            double radiusOffsetMultiplier_ = 2.0;
            double samplingRadiusMultiplier_ = 4.0;
            
            // Helper to sample a point
            Point samplePointInternal(const Cylinder& cyl, double extensionHeight, int direction, double maxRadius, std::mt19937& rng);
        };
    }
}

#endif // OMPL_DATASTRUCTURES_SAMPLING_CYLINDER_SAMPLER_H
