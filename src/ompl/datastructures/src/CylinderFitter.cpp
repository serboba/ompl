/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/datastructures/statistics/CylinderFitter.h>
#include <ompl/datastructures/geometry/GeometryUtils.h>
#include <cmath>
#include <iostream>

using namespace ompl::statistics;
using namespace ompl::geometry;

void CylinderFitter::computeCovariance(const std::vector<Point3D>& points, Eigen::Matrix3d& cov)
{
    cov.setZero();
    if (points.empty()) return;

    // Compute centroid
    Eigen::Vector3d centroid(0.0, 0.0, 0.0);
    for (const auto& p : points)
    {
        centroid[0] += p.x;
        centroid[1] += p.y;
        centroid[2] += p.z;
    }
    centroid /= static_cast<double>(points.size());

    // Compute covariance
    for (const auto& p : points)
    {
        Eigen::Vector3d v(p.x - centroid[0], p.y - centroid[1], p.z - centroid[2]);
        cov += v * v.transpose();
    }
    cov /= static_cast<double>(points.size());
}

void CylinderFitter::largestEigenVector(double C[3][3], double& ex, double& ey, double& ez)
{
    // Power iteration method
    ex = 1.0; ey = 0.0; ez = 0.0; // Initial guess
    for (int i = 0; i < 50; ++i)
    {
        double nx = C[0][0] * ex + C[0][1] * ey + C[0][2] * ez;
        double ny = C[1][0] * ex + C[1][1] * ey + C[1][2] * ez;
        double nz = C[2][0] * ex + C[2][1] * ey + C[2][2] * ez;
        
        double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (norm < 1e-12) break;
        
        ex = nx / norm;
        ey = ny / norm;
        ez = nz / norm;
    }
}

Cylinder3D CylinderFitter::fitCylinder(const std::vector<Point3D>& points)
{
    Cylinder3D cyl;
    if (points.empty()) return cyl;

    // 1. Compute centroid
    Point3D centroid;
    for (const auto& p : points)
        centroid = centroid + p;
    centroid = centroid / static_cast<double>(points.size());
    cyl.origin = centroid;

    // 2. Compute PCA for direction
    Eigen::Matrix3d cov;
    computeCovariance(points, cov);
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d axis = es.eigenvectors().col(2); // Largest eigenvector
    
    cyl.direction = {axis[0], axis[1], axis[2]};
    cyl.ax = axis[0];
    cyl.ay = axis[1];
    cyl.az = axis[2];

    // 3. Compute radius and height
    double maxDistSq = 0.0;
    double minProj = std::numeric_limits<double>::max();
    double maxProj = -std::numeric_limits<double>::max();

    for (const auto& p : points)
    {
        Point3D diff = p - centroid;
        double proj = GeometryUtils::dotProduct(diff, cyl.direction.x, cyl.direction.y, cyl.direction.z);
        
        if (proj < minProj) minProj = proj;
        if (proj > maxProj) maxProj = proj;
        
        // Distance from axis
        Point3D projVec = cyl.direction * proj;
        Point3D perpVec = diff - projVec;
        double distSq = perpVec.x * perpVec.x + perpVec.y * perpVec.y + perpVec.z * perpVec.z;
        
        if (distSq > maxDistSq) maxDistSq = distSq;
    }

    cyl.radius = std::sqrt(maxDistSq);
    cyl.height = maxProj - minProj;
    
    return cyl;
}

void CylinderFitter::normalize(double& x, double& y, double& z)
{
    GeometryUtils::normalizeVector(x, y, z);
}
