/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_STATISTICS_VMF_MIXTURE_MODEL_H
#define OMPL_DATASTRUCTURES_STATISTICS_VMF_MIXTURE_MODEL_H

#include <ompl/datastructures/statistics/VMFDistribution.h>
#include <vector>
#include <random>

namespace ompl
{
    namespace statistics
    {
        /** @brief von Mises-Fisher Mixture Model with EM algorithm */
        class VMFMixtureModel
        {
        public:
            /** Initialize parameters using k-means++ style initialization */
            static std::vector<VMFComponent> initializeParameters(
                std::vector<geometry::Point3D>& allPoints, int K, std::mt19937& rng);
            
            /** Compute responsibilities (E-step) */
            static std::vector<double> computeResponsibilities(
                std::vector<geometry::Point3D>& allPoints,
                std::vector<VMFComponent>& components);
            
            /** Update parameters (M-step) */
            static void updateParameters(
                std::vector<geometry::Point3D>& allPoints,
                const std::vector<double>& responsibilities,
                std::vector<VMFComponent>& components);
            
            /** Compute log likelihood */
            static double computeLogLikelihood(
                std::vector<geometry::Point3D>& allPoints,
                const std::vector<VMFComponent>& components);
            
            /** Fit VMF mixture model using EM algorithm */
            static std::vector<VMFComponent> fitVMFMixtureModel(
                std::vector<geometry::Point3D>& allPoints,
                int K, int maxIter, double tol, std::mt19937& rng);
            
            /** Select a component based on mixing weights */
            static int selectComponent(const std::vector<VMFComponent>& components, std::mt19937& rng);
        };
    }
}

#endif // OMPL_DATASTRUCTURES_STATISTICS_VMF_MIXTURE_MODEL_H
