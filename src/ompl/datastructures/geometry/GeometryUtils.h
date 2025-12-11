/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */
  
#ifndef OMPL_DATASTRUCTURES_GEOMETRY_GEOMETRY_UTILS_H
#define OMPL_DATASTRUCTURES_GEOMETRY_GEOMETRY_UTILS_H

#include <ompl/datastructures/geometry/Point3D.h>
#include <random>

namespace ompl
{
    namespace geometry
    {
        /** @brief Utility functions for 3D geometry operations */
        class GeometryUtils
        {
        public:
            /** Normalize a 3D vector in-place */
            static void normalizeVector(double& x, double& y, double& z);
            
            /** Normalize a Point3D in-place */
            static void normalizePoint(Point3D& point);
            
            /** Dot product between two points */
            static double dotProduct(const Point3D& a, const Point3D& b);
            
            /** Dot product using raw coordinates */
            static double dotProduct(double px1, double py1, double pz1, double px2, double py2, double pz2);
            
            /** Dot product between point and direction vector */
            static double dotProduct(const Point3D& p, double mu_x, double mu_y, double mu_z);
            
            /** Generate a random unit vector */
            static Point3D randomUnitVector(std::mt19937& rng);
            
            /** Orthogonalize vector v with respect to ref */
            static Point3D orthogonalize(const Point3D& v, const Point3D& ref);

            /** Cross product of two vectors */
            static Point3D crossProduct(const Point3D& a, const Point3D& b);

            /** Cross product of two vectors (raw coordinates) */
            static Point3D crossProduct(double ax, double ay, double az, double bx, double by, double bz);
        };
    }
}

#endif // OMPL_DATASTRUCTURES_GEOMETRY_GEOMETRY_UTILS_H
