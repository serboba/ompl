#ifndef OMPL_DATASTRUCTURES_SAMPLING_ADAPTIVE_SPHERE_SAMPLER_H
#define OMPL_DATASTRUCTURES_SAMPLING_ADAPTIVE_SPHERE_SAMPLER_H

#include <ompl/datastructures/geometry/Point3D.h>
#include <ompl/datastructures/geometry/Cylinder3D.h>
#include <ompl/datastructures/statistics/VMFDistribution.h>
#include <ompl/base/samplers/sphere/FibonacciSphereSampler.h>
#include <ompl/base/samplers/sphere/UniformSphereSampler.h>
#include <ompl/base/samplers/sphere/VMFSampler.h>
#include <ompl/base/samplers/sphere/CylinderSampler.h>
#include <ompl/base/ValidStateSampler.h>
#include <vector>
#include <random>
#include <memory>
#include <Eigen/Dense>

namespace ompl
{
    namespace sampling
    {
        /** @brief Adaptive sphere sampling manager */
        class AdaptiveSphereSampler : public base::ValidStateSampler
        {
        public:
            using Point = geometry::Point3D;
            using Cylinder = geometry::Cylinder3D;
            using VMFComponent = statistics::VMFComponent;

            AdaptiveSphereSampler(const base::SpaceInformation *si, int sampleSize, double radius, const std::vector<int>& axesIndices = {0, 1, 2});

            bool sample(base::State *state) override;
            bool sampleNear(base::State *state, const base::State *near, const double distance) override;

            /** Collect quasi-random samples (Fibonacci + Uniform) */
            void collectQuasiRandomSamples(double radius);

            /** Get a sample from the collected set */
            std::pair<int, Point> getSample();

            /** Remove index from available indices */
            void popIndexFromIndices(int index, bool valid);

            /** Add a valid sample to the cache and update components */
            void addValidSampleToComponent(Point& p);

            /** Append cached samples to the main valid list */
            void appendCachedValidSamples();

            /** Clear all samples */
            void clearSamples();

            /** Clear only cached valid samples */
            void clearCachedValidSamples();

            /** Clear all valid points (used by RRTFINALIZED) */
            void clearValidPoints();

            /** Get the rate of valid samples */
            double getValidSampleRate() const;

            /** Sample from a specific VMF component */
            Point sampleFromVMF(int componentID, double newRadius, double kappaMultiplier);

            /** Sample from a cylinder fitted to valid points */
            Point getRandomSampleFromCylinder(double extensionHeight, int chosenDirection);
            Point getRandomSampleFromCylinder(double extensionHeight, int chosenDirection, const Eigen::Vector3d &axis);

            /** @brief Update cylinder axis using PCA on valid points */
            void updateCylinderAxis();

            /** Fit VMF mixture model to valid points */
            std::vector<VMFComponent> fitVMFMixtureModel(int K, int maxIter, double tol);

            /** Select a component based on weights */
            int selectComponent();

            // Accessors
            const std::vector<Point>& getSpherePoints() const { return spherePoints_; }
            const std::vector<int>& getIndices() const { return indices_; }
            const std::vector<Point>& getAllValidPoints() const { return allValidPoints_; }
            const std::vector<Point>& getAllPointsCached() const { return allPoints_cached_; }
            const std::vector<VMFComponent>& getSphereComponents() const { return vmfSampler_.getComponents(); }
            double getBestRadius() const { return bestRadius_; }
            
            // Configuration
            void setCylinderSamplingRadiusMultiplier(double val) { cylinderSampler_.setSamplingRadiusMultiplier(val); }
            void setCylinderRadiusOffsetMultiplier(double val) { cylinderSampler_.setRadiusOffsetMultiplier(val); }
            void setFibonacciJitterRadius(double val) { fibSampler_.setJitterRadius(val); }
            void setPCAFilterTopPercent(double val) { pcaFilterTopPercent_ = val; }

            void updateComponentCenter(int componentIndex);
            void appendFibonacciSamples(double customRadius = -1.0);
            void appendUniformSamples(double customRadius = -1.0);

            // Public members for easy access (legacy support / direct access)
            double bestRadius = 0.0; // Exposed for RRTFINALIZED

        private:
            int sampleSize_;
            double radius_;
            std::mt19937 rng_;
            
            // Samplers
            FibonacciSphereSampler fibSampler_;
            UniformSphereSampler uniSampler_;
            VMFSampler vmfSampler_;
            CylinderSampler cylinderSampler_;

            // Data
            std::vector<Point> spherePoints_;
            std::vector<int> indices_;
            std::vector<int> axesIndices_;
            std::vector<Point> allValidPoints_;
            std::vector<Point> allPoints_cached_;
            // sphereComponents_ managed by vmfSampler_

            // Parameters
            double bestRadius_ = 0.0;
            double cylinderRadiusOffsetMultiplier_ = 2.0;
            double pcaFilterTopPercent_ = 1.0; 

            // Caching for PCA
            bool validPointsDirty_ = true;
            Eigen::Vector3d cylinderAxis_ = Eigen::Vector3d::UnitZ();
            bool hasCylinderAxis_ = false;
            
            /** @brief Project 3D sphere point to 2D circle (in-place modification) */
            void projectSphereToCircle(Point& p, double radius);
        };
    }
}

#endif // OMPL_DATASTRUCTURES_SAMPLING_ADAPTIVE_SPHERE_SAMPLER_H
