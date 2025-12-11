/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/datastructures/geometry/GeometryUtils.h>
#include <cmath>

using namespace ompl::geometry;

void GeometryUtils::normalizeVector(double& x, double& y, double& z)
{
    double magnitude = std::sqrt(x * x + y * y + z * z);
    if (magnitude > 1e-12)
    {
        double invMagnitude = 1.0 / magnitude;
        x *= invMagnitude;
        y *= invMagnitude;
        z *= invMagnitude;
    }
}

void GeometryUtils::normalizePoint(Point3D& point)
{
    double magnitude = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (magnitude > 1e-12)
    {
        double invMagnitude = 1.0 / magnitude;
        point.x *= invMagnitude;
        point.y *= invMagnitude;
        point.z *= invMagnitude;
    }
}

double GeometryUtils::dotProduct(const Point3D& a, const Point3D& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double GeometryUtils::dotProduct(double px1, double py1, double pz1, double px2, double py2, double pz2)
{
    return px1 * px2 + py1 * py2 + pz1 * pz2;
}

double GeometryUtils::dotProduct(const Point3D& p, double mu_x, double mu_y, double mu_z)
{
    return p.x * mu_x + p.y * mu_y + p.z * mu_z;
}

Point3D GeometryUtils::randomUnitVector(std::mt19937& rng)
{
    std::uniform_real_distribution<> dist(-1.0, 1.0);
    double x = dist(rng);
    double y = dist(rng);
    double z = dist(rng);
    Point3D p{x, y, z};
    return p.normalized();
}

Point3D GeometryUtils::orthogonalize(const Point3D& v, const Point3D& ref)
{
    double dot = dotProduct(v, ref);
    Point3D orthogonalized = {v.x - dot * ref.x, v.y - dot * ref.y, v.z - dot * ref.z};
    normalizePoint(orthogonalized);
    return orthogonalized;
}

Point3D GeometryUtils::crossProduct(const Point3D& a, const Point3D& b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

Point3D GeometryUtils::crossProduct(double ax, double ay, double az, double bx, double by, double bz)
{
    return {
        ay * bz - az * by,
        az * bx - ax * bz,
        ax * by - ay * bx
    };
}
