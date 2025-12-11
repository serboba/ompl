/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/base/samplers/sphere/CylinderSampler.h>
#include <ompl/datastructures/statistics/CylinderFitter.h>
#include <ompl/datastructures/geometry/GeometryUtils.h>
#include <algorithm>
#include <cmath>

using namespace ompl::sampling;
using namespace ompl::statistics;
using namespace ompl::geometry;

CylinderSampler::CylinderSampler()
{
}

std::vector<CylinderSampler::Point> CylinderSampler::sample(int numSamples, double radius, std::mt19937& rng)
{
    // Not the primary use case, but we can sample from the current cylinder
    std::vector<Point> samples;
    if (!hasValidCylinder_) return samples;

    for (int i = 0; i < numSamples; ++i)
    {
        // Default params for generic sampling
        samples.push_back(sampleCylinder(0.0, 0, 1.0, rng)); 
    }
    return samples;
}

void CylinderSampler::fit(const std::vector<Point>& points, double pcaFilterTopPercent)
{
    if (points.empty()) return;

    std::vector<Point> pointsForFit = points;

    if (pcaFilterTopPercent < 1.0 && !points.empty())
    {
        size_t keepCount = std::max((size_t)3, (size_t)(points.size() * pcaFilterTopPercent));
        if (keepCount < points.size()) {
            std::nth_element(pointsForFit.begin(), pointsForFit.begin() + keepCount, pointsForFit.end(),
                [](const Point& a, const Point& b) {
                    return a.sampledAtRadius > b.sampledAtRadius;
                });
            pointsForFit.resize(keepCount);
        }
    }

    cylinder_ = CylinderFitter::fitCylinder(pointsForFit);
    hasValidCylinder_ = true;
    hasValidCylinder_ = true;
}

Eigen::Vector3d CylinderSampler::computePCAAxis(const std::vector<Point>& points)
{
    if (points.size() < 2) {
        return Eigen::Vector3d::UnitZ();
    }

    // Check if points are 2D (all z values are approximately 0)
    bool is2D = true;
    const double zTolerance = 1e-6;
    for (const auto &p : points) {
        if (std::fabs(p.z) > zTolerance) {
            is2D = false;
            break;
        }
    }

    if (is2D) {
        // 2D case: PCA on x, y only
        Eigen::Vector2d mean2D = Eigen::Vector2d::Zero();
        for (const auto &p : points) {
            mean2D += Eigen::Vector2d(p.x, p.y);
        }
        mean2D /= double(points.size());

        // Compute 2x2 covariance matrix
        Eigen::Matrix2d cov2D = Eigen::Matrix2d::Zero();
        for (const auto &p : points) {
            Eigen::Vector2d vec(p.x, p.y);
            Eigen::Vector2d d = vec - mean2D;
            cov2D += d * d.transpose();
        }
        cov2D /= double(points.size());

        // Eigen decomposition for 2D
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2D(cov2D);
        
        // The principal axis in 2D (largest eigenvalue, last column)
        Eigen::Vector2d axis2D = es2D.eigenvectors().col(1).normalized();
        
        // Return as 3D vector with z=0
        return Eigen::Vector3d(axis2D(0), axis2D(1), 0.0).normalized();
    }
    else {
        // 3D case: Original implementation
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto &p : points) {
            mean += Eigen::Vector3d(p.x, p.y, p.z);
        }
        mean /= double(points.size());

        // Compute covariance
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto &p : points) {
            Eigen::Vector3d vec(p.x, p.y, p.z);
            Eigen::Vector3d d = vec - mean;
            cov += d * d.transpose();
        }
        cov /= double(points.size());

        // Eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
        
        // The eigenvectors are stored in increasing order of eigenvalues
        // The principal axis corresponds to the largest eigenvalue (last column)
        return es.eigenvectors().col(2).normalized();
    }
}

void CylinderSampler::fitWithAxis(const std::vector<Point>& points, const Eigen::Vector3d& axis, double radiusOffsetMultiplier)
{
    if (points.empty()) return;

    cylinder_.ax = axis(0);
    cylinder_.ay = axis(1);
    cylinder_.az = axis(2);
    cylinder_.direction = {axis(0), axis(1), axis(2)};

    double maxHeight = 0.0;
    double maxRadius = 0.0;
    for (const auto& p : points)
    {
        double h = p.x * cylinder_.ax + p.y * cylinder_.ay + p.z * cylinder_.az;
        maxHeight = std::max(maxHeight, h);
        Eigen::Vector3d point(p.x, p.y, p.z);
        Eigen::Vector3d proj = h * axis;
        double dist = (point - proj).norm();
        maxRadius = std::max(maxRadius, dist);
    }

    double offset = radiusOffsetMultiplier * maxRadius;
    cylinder_.height = maxHeight;
    cylinder_.radius = maxRadius + offset;
    hasValidCylinder_ = true;
}

CylinderSampler::Point CylinderSampler::sampleCylinder(double extensionHeight, int direction, double radiusMultiplier, std::mt19937& rng)
{
    if (!hasValidCylinder_) throw std::runtime_error("Cylinder not initialized");
    return samplePointInternal(cylinder_, extensionHeight, direction, cylinder_.radius * radiusMultiplier, rng);
}

CylinderSampler::Point CylinderSampler::samplePointInternal(const Cylinder& cyl, double extensionHeight, int chosenDirection, double samplingRadiusMax, std::mt19937& rng)
{
    double startH = cyl.height;
    double endH = cyl.height + extensionHeight;

    double intervalStart = (chosenDirection == 0) ? -startH : startH;
    double intervalEnd = (chosenDirection == 0) ? -endH : endH;

    std::uniform_real_distribution<double> distH(intervalStart, intervalEnd);
    std::uniform_real_distribution<double> distAngle(0.0, 2.0 * M_PI);
    std::uniform_real_distribution<double> distR(0.0, samplingRadiusMax);

    double h = distH(rng);
    double theta = distAngle(rng);
    double r = distR(rng);

    // Check if cylinder is 2D (z component of axis is approximately 0)
    bool is2D = (std::fabs(cyl.az) < 1e-6);

    if (is2D) {
        // 2D case: Cylinder becomes a line
        // Sample along the line: h * axis + perpendicular jitter
        // For 2D, perpendicular to axis (ax, ay) is: (-ay, ax) or (ay, -ax)
        // Normalize the axis first
        double axis_norm = std::sqrt(cyl.ax * cyl.ax + cyl.ay * cyl.ay);
        if (axis_norm < 1e-10) {
            // Degenerate axis, use default
            double px = r * std::cos(theta);
            double py = r * std::sin(theta);
            double pz = 0.0;
            double newRadius_ = std::sqrt(px * px + py * py);
            return Point{px, py, pz, newRadius_, true};
        }
        
        double ax_norm = cyl.ax / axis_norm;
        double ay_norm = cyl.ay / axis_norm;
        
        // Perpendicular vector: (-ay, ax) - this is 90 degree rotation
        double perp_x = -ay_norm;
        double perp_y = ax_norm;
        
        // Sample: h * axis + r * (cos(θ) * perp_x + sin(θ) * perp_y)
        // For 2D line, we sample along axis with perpendicular jitter
        double px = h * ax_norm + r * (std::cos(theta) * perp_x + std::sin(theta) * perp_y);
        double py = h * ay_norm + r * (std::cos(theta) * perp_y - std::sin(theta) * perp_x);
        double pz = 0.0;  // Always 0 for 2D

        double newRadius_ = std::sqrt(px * px + py * py);

        return Point{px, py, pz, newRadius_, true};
    }
    else {
        // 3D case: Original implementation
        // Compute basis vectors
        Point axis{cyl.ax, cyl.ay, cyl.az};
        Point tempX{(std::fabs(cyl.ax) < 0.9) ? 1.0 : 0.0, 
                    (std::fabs(cyl.ay) < 0.9) ? 1.0 : 0.0, 
                    (std::fabs(cyl.az) < 0.9) ? 1.0 : 0.0};
        
        Point u = GeometryUtils::crossProduct(axis, tempX);
        GeometryUtils::normalizePoint(u);
        
        if (u.x == 0.0 && u.y == 0.0 && u.z == 0.0) {
            u = GeometryUtils::crossProduct(axis, Point{0.0, 1.0, 0.0});
            GeometryUtils::normalizePoint(u);
        }

        Point v = GeometryUtils::crossProduct(axis, u);
        GeometryUtils::normalizePoint(v);

        // Construct the sample point
        double px = h * cyl.ax + r * (std::cos(theta) * u.x + std::sin(theta) * v.x);
        double py = h * cyl.ay + r * (std::cos(theta) * u.y + std::sin(theta) * v.y);
        double pz = h * cyl.az + r * (std::cos(theta) * u.z + std::sin(theta) * v.z);

        double newRadius_ = std::sqrt(px * px + py * py + pz * pz);

        return Point{px, py, pz, newRadius_, true};
    }
}
