/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_STATISTICS_CYLINDER_FITTER_H
#define OMPL_DATASTRUCTURES_CYLINDER_FITTER_H

#include <ompl/datastructures/geometry/Point3D.h>
#include <ompl/datastructures/geometry/Cylinder3D.h>
#include <Eigen/Dense>
#include <vector>

namespace ompl
{
    namespace statistics
    {
        /** @brief PCA-based cylinder fitting for point clouds */
        class CylinderFitter
        {
        public:
            /** Compute covariance matrix of point cloud */
           static void computeCovariance(const std::vector<geometry::Point3D>& points, Eigen::Matrix3d& cov);
            
            /** Find largest eigenvector using power iteration */
            static void largestEigenVector(double C[3][3], double& ex, double& ey, double& ez);
            
            /** Fit a cylinder to a set of points using PCA */
            static geometry::Cylinder3D fitCylinder(const std::vector<geometry::Point3D>& points);
            
            /** Normalize a vector */
            static void normalize(double& x, double& y, double& z);
        };
    }
}

#endif // OMPL_DATASTRUCTURES_STATISTICS_CYLINDER_FITTER_H
