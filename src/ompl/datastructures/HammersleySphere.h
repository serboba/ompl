/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef HAMMERSLEYSPHERE_H
#define HAMMERSLEYSPHERE_H

// Backward compatibility header - maintains original include path
// Now uses organized geometry and statistics components

#include <ompl/datastructures/geometry/Point3D.h>
#include <ompl/datastructures/geometry/Cylinder3D.h>
#include <ompl/datastructures/geometry/GeometryUtils.h>
#include <ompl/datastructures/statistics/VMFDistribution.h>
#include <ompl/base/samplers/sphere/AdaptiveSphereSampler.h>
#include <ompl/datastructures/MultiArmedBandits.h>

#include <cmath>
#include <random>
#include <vector>
#include <Eigen/Dense>
#include <iostream>

namespace ompl
{
    class HammersleySphere
    {
    public:
        // Type aliases for backward compatibility
        using Point = geometry::Point3D;
        using Cylinder = geometry::Cylinder3D;
        using VMFComponent = statistics::VMFComponent;

        HammersleySphere(int sampleSize_, double radius_);
        ~HammersleySphere() = default;

        // Geometry utilities (inline implementations using new components)
        void normalizePoint(Point& point) { geometry::GeometryUtils::normalizePoint(point); }
        void normalizeAllPoints(std::vector<Point>& allPoints);
        void normalizeVector(double& x, double& y, double& z) { geometry::GeometryUtils::normalizeVector(x, y, z); }
        Point randomUnitVector(std::mt19937& rng) { return geometry::GeometryUtils::randomUnitVector(rng); }
        Point orthogonalize(const Point& v, const Point& ref) { return geometry::GeometryUtils::orthogonalize(v, ref); }
        
        // Dot product overloads (inline implementations)
        double dotProduct(const Point& a, const Point& b) { return geometry::GeometryUtils::dotProduct(a, b); }
        double dotProduct(double px1, double py1, double pz1, double px2, double py2, double pz2) { 
            return geometry::GeometryUtils::dotProduct(px1, py1, pz1, px2, py2, pz2); 
        }
        double dotProduct(const Point& p, double mu_x, double mu_y, double mu_z) { 
            return geometry::GeometryUtils::dotProduct(p, mu_x, mu_y, mu_z); 
        }

        // VMF operations
        std::vector<VMFComponent> initializeParameters(std::vector<Point>& allPoints, int K);
        void testInitializeVMF(int k_, int max_iter, double tol_);
        double computeNormalizationConstant(double kappa);
        std::vector<double> computeResponsibilities(std::vector<Point>& allPoints, std::vector<VMFComponent>& components);
        void updateParameters(std::vector<Point>& allPoints, const std::vector<double>& responsibilities, std::vector<VMFComponent>& components);
        double computeLogLikelihood(std::vector<Point>& allPoints, const std::vector<VMFComponent>& components);
        std::vector<VMFComponent> fitVMFMixtureModel(int K, int maxIter, double tol);
        int selectComponent();
        
        // Cylinder operations
        void computeCovariance(const std::vector<Point>& points, Eigen::Matrix3d& cov);
        void normalize(double& x, double& y, double& z);
        void largestEigenVector(double C[3][3], double& ex, double& ey, double& ez);
        Cylinder fitCylinder(const std::vector<Point>& points);
        
        // Sphere sampling
        void createSamplesFibonacciLattice(double customRadius = -1.0);
        void createSamplesUniform(double customRadius = -1.0);
        void collectQuasiRandomSamples(double radius_);
        std::pair<int, Point> getSample();
        Point sampleFromVMF(int componentID, double newRadius_, double kappaMultiplier_);
        
        // Sample management (inline implementations)
        void clearCachedValidSamples() { sampler_->clearCachedValidSamples(); allPoints_cached.clear(); }
        void clearSamples() { spherePoints.clear(); }
        int getAllValidPointSize() { return allValidPoints.size(); }
        
        // Sample management (need implementations from .cpp)
        void popIndexFromIndices(int index, bool valid);
        void appendCachedValidSamples();
        double getValidSampleRate();
        void updateComponentCenter(int componentIndex);
        void addValidSampleToComponent(Point& p);
        
        // Cylinder sampling
        Point getRandomSampleFromCylinder(double extensionHeight, int chosenDirection);
        Point getRandomSampleFromCylinder(double extensionHeight, int chosenDirection, const Eigen::Vector3d& axis);
        
        // Configuration setters (inline implementations)
        void setCylinderRadiusOffsetMultiplier(double val) { this->cylinderRadiusOffsetMultiplier = val; }
        void setFibonacciJitterRadius(double val) { this->fibonacciJitterRadius = val; }
        void setCylinderSamplingRadiusMultiplier(double multiplier) { cylinderSamplingRadiusMultiplier = multiplier; }
        void setPCAFilterTopPercent(double val);

        // Public members
        std::mt19937 rng;
        std::uniform_real_distribution<> uniform_0_1_distribution;
        std::uniform_real_distribution<> uniform_neg_pos_1_distribution;

        double radius;
        int sampleSize;
        int exhaustionCounter;
        double timeVMFInitialization;

        // Sample management (need implementations from .cpp)
        void appendFibonacciSamples(double customRadius = -1.0);
        void appendUniformSamples(double customRadius = -1.0);

        // Public members for easy access (legacy support)
        std::vector<Point> spherePoints;
        std::vector<int> indices;
        std::vector<Point> allValidPoints;
        std::vector<Point> allPoints_cached;
        std::vector<VMFComponent> sphereComponents;
        double bestRadius = 0.0;
        double cylinderSamplingRadiusMultiplier = 4.0;
        double cylinderRadiusOffsetMultiplier = 2.0;
        double fibonacciJitterRadius = 0.0;
        double pcaFilterTopPercent = 1.0;

    private:
        std::unique_ptr<sampling::AdaptiveSphereSampler> sampler_;
    };
}

#endif // HAMMERSLEYSPHERE_H
