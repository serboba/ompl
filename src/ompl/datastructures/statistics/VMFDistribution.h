/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_STATISTICS_VMF_DISTRIBUTION_H
#define OMPL_DATASTRUCTURES_STATISTICS_VMF_DISTRIBUTION_H

#include <ompl/datastructures/geometry/Point3D.h>
#include <vector>
#include <cmath>
#include <random>

namespace ompl
{
    namespace statistics
    {
        /** @brief von Mises-Fisher distribution component */
        struct VMFComponent
        {
            double mu_x, mu_y, mu_z; // Mean direction
            double kappa;             // Concentration parameter
            double pi;                // Mixing weight
            std::vector<geometry::Point3D> points;  // Points assigned to this component
            
            VMFComponent() : mu_x(0.0), mu_y(0.0), mu_z(0.0), kappa(0.0), pi(0.0) {}
        };
        
        /** @brief Single von Mises-Fisher distribution operations */
        class VMFDistribution
        {
        public:
            /** Compute the normalization constant C_d(kappa) for 3D */
            static double computeNormalizationConstant(double kappa);
            
            /** Compute the probability density of a point given VMF parameters */
            static double probability(const geometry::Point3D& x, double mu_x, double mu_y, double mu_z, double kappa);

            /** Sample from a VMF distribution */
            static geometry::Point3D sample(const VMFComponent& component, std::mt19937& rng);
        };
    }
}

#endif // OMPL_DATASTRUCTURES_STATISTICS_VMF_DISTRIBUTION_H
