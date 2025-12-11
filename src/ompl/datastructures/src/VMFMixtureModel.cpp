/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/datastructures/statistics/VMFMixtureModel.h>
#include <ompl/datastructures/geometry/GeometryUtils.h>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace ompl::statistics;
using namespace ompl::geometry;

std::vector<VMFComponent> VMFMixtureModel::initializeParameters(
    std::vector<Point3D>& allPoints, int K, std::mt19937& rng)
{
    std::vector<VMFComponent> components(K);
    if (allPoints.empty()) return components;

    // k-means++ initialization
    std::uniform_int_distribution<> dist(0, allPoints.size() - 1);
    int firstCenterIndex = dist(rng);
    
    // First center
    Point3D p = allPoints[firstCenterIndex];
    components[0].mu_x = p.x;
    components[0].mu_y = p.y;
    components[0].mu_z = p.z;
    GeometryUtils::normalizeVector(components[0].mu_x, components[0].mu_y, components[0].mu_z);

    for (int k = 1; k < K; ++k)
    {
        std::vector<double> distances(allPoints.size());
        for (size_t i = 0; i < allPoints.size(); ++i)
        {
            double minDist = std::numeric_limits<double>::max();
            for (int j = 0; j < k; ++j)
            {
                double d = GeometryUtils::dotProduct(allPoints[i], components[j].mu_x, components[j].mu_y, components[j].mu_z);
                // Convert cosine similarity to distance-like metric (1 - cos)
                double distVal = 1.0 - d;
                if (distVal < minDist) minDist = distVal;
            }
            distances[i] = minDist * minDist; // Squared distance for k-means++
        }

        std::discrete_distribution<> d(distances.begin(), distances.end());
        int nextCenterIndex = d(rng);
        
        Point3D nextP = allPoints[nextCenterIndex];
        components[k].mu_x = nextP.x;
        components[k].mu_y = nextP.y;
        components[k].mu_z = nextP.z;
        GeometryUtils::normalizeVector(components[k].mu_x, components[k].mu_y, components[k].mu_z);
    }

    // Initialize other parameters
    for (int k = 0; k < K; ++k)
    {
        components[k].kappa = 10.0; // Initial concentration
        components[k].pi = 1.0 / K; // Uniform mixing weights
    }

    return components;
}

std::vector<double> VMFMixtureModel::computeResponsibilities(
    std::vector<Point3D>& allPoints,
    std::vector<VMFComponent>& components)
{
    size_t N = allPoints.size();
    size_t K = components.size();
    std::vector<double> responsibilities(N * K);

    for (size_t i = 0; i < N; ++i)
    {
        double sumProb = 0.0;
        std::vector<double> probs(K);

        for (size_t k = 0; k < K; ++k)
        {
            double prob = VMFDistribution::probability(allPoints[i], 
                                                      components[k].mu_x, 
                                                      components[k].mu_y, 
                                                      components[k].mu_z, 
                                                      components[k].kappa);
            probs[k] = components[k].pi * prob;
            sumProb += probs[k];
        }

        for (size_t k = 0; k < K; ++k)
        {
            if (sumProb > 1e-12)
                responsibilities[i * K + k] = probs[k] / sumProb;
            else
                responsibilities[i * K + k] = 1.0 / K;
        }
    }
    return responsibilities;
}

void VMFMixtureModel::updateParameters(
    std::vector<Point3D>& allPoints,
    const std::vector<double>& responsibilities,
    std::vector<VMFComponent>& components)
{
    size_t N = allPoints.size();
    size_t K = components.size();

    for (size_t k = 0; k < K; ++k)
    {
        double sumResp = 0.0;
        double r_x = 0.0, r_y = 0.0, r_z = 0.0;

        for (size_t i = 0; i < N; ++i)
        {
            double resp = responsibilities[i * K + k];
            sumResp += resp;
            r_x += resp * allPoints[i].x;
            r_y += resp * allPoints[i].y;
            r_z += resp * allPoints[i].z;
        }
        
        // Update mixing weight
        components[k].pi = sumResp / N;

        // Update mean direction
        double R = std::sqrt(r_x * r_x + r_y * r_y + r_z * r_z);
        if (R > 1e-12)
        {
            components[k].mu_x = r_x / R;
            components[k].mu_y = r_y / R;
            components[k].mu_z = r_z / R;
        }

        // Update kappa (approximation)
        double R_bar = R / sumResp;
        if (R_bar < 0.53) {
             components[k].kappa = 2.0 * R_bar + R_bar * R_bar * R_bar + (5.0 * R_bar * R_bar * R_bar * R_bar * R_bar) / 6.0;
        } else if (R_bar < 0.85) {
             components[k].kappa = -0.4 + 1.39 * R_bar + 0.43 / (1.0 - R_bar);
        } else {
             components[k].kappa = 1.0 / (R_bar * R_bar * R_bar - 4.0 * R_bar * R_bar + 3.0 * R_bar);
        }
        
        // Clamp kappa to avoid numerical issues
        if (components[k].kappa > 700.0) components[k].kappa = 700.0;
        if (components[k].kappa < 0.0) components[k].kappa = 0.0;
    }
}

double VMFMixtureModel::computeLogLikelihood(
    std::vector<Point3D>& allPoints,
    const std::vector<VMFComponent>& components)
{
    double logLikelihood = 0.0;
    size_t K = components.size();

    for (const auto& point : allPoints)
    {
        double prob = 0.0;
        for (size_t k = 0; k < K; ++k)
        {
            prob += components[k].pi * VMFDistribution::probability(point, 
                                                                   components[k].mu_x, 
                                                                   components[k].mu_y, 
                                                                   components[k].mu_z, 
                                                                   components[k].kappa);
        }
        if (prob > 1e-12)
            logLikelihood += std::log(prob);
        else
            logLikelihood += -100.0; // Penalty for zero probability
    }
    return logLikelihood;
}

std::vector<VMFComponent> VMFMixtureModel::fitVMFMixtureModel(
    std::vector<Point3D>& allPoints,
    int K, int maxIter, double tol, std::mt19937& rng)
{
    if (allPoints.empty()) return {};
    
    std::vector<VMFComponent> components = initializeParameters(allPoints, K, rng);
    double prevLogLikelihood = -std::numeric_limits<double>::max();

    for (int iter = 0; iter < maxIter; ++iter)
    {
        std::vector<double> responsibilities = computeResponsibilities(allPoints, components);
        updateParameters(allPoints, responsibilities, components);
        
        double logLikelihood = computeLogLikelihood(allPoints, components);
        if (std::abs(logLikelihood - prevLogLikelihood) < tol)
            break;
        prevLogLikelihood = logLikelihood;
    }
    
    return components;
}

int VMFMixtureModel::selectComponent(const std::vector<VMFComponent>& components, std::mt19937& rng)
{
    if (components.empty()) return -1;
    
    std::vector<double> weights;
    for (const auto& comp : components)
        weights.push_back(comp.pi);
        
    std::discrete_distribution<> dist(weights.begin(), weights.end());
    return dist(rng);
}
