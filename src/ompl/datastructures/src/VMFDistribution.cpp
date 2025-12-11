/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/datastructures/statistics/VMFDistribution.h>
#include <ompl/datastructures/geometry/GeometryUtils.h>
#include <cmath>
#include <random>

using namespace ompl::statistics;
using namespace ompl::geometry;

double VMFDistribution::computeNormalizationConstant(double kappa)
{
    if (std::abs(kappa) < 1e-12) {
        return 1.0 / (4.0 * M_PI); // For kappa=0, uniform distribution on sphere
    }
    double denominator = std::exp(kappa) - std::exp(-kappa);
    if (std::abs(denominator) < 1e-12) {
        return 1.0 / (4.0 * M_PI); // Fallback for numerical instability
    }
    return kappa / (2.0 * M_PI * denominator); // Approximation
}

double VMFDistribution::probability(const geometry::Point3D& x, double mu_x, double mu_y, double mu_z, double kappa)
{
    double dot = geometry::GeometryUtils::dotProduct(x, mu_x, mu_y, mu_z);
    double C_d = computeNormalizationConstant(kappa);
    return C_d * std::exp(kappa * dot);
}

ompl::geometry::Point3D VMFDistribution::sample(const VMFComponent& component, std::mt19937& rng)
{
    std::uniform_real_distribution<double> uniform_neg_pos_1(-1.0, 1.0);
    
    // Step 1: Sample w (cosine of angular distance)
    double kappa = component.kappa;
    double w;
    
    if (kappa < 1e-6) {
        // Uniform sampling on sphere if kappa is near zero
        double z = uniform_neg_pos_1(rng);
        double theta = 2.0 * M_PI * std::uniform_real_distribution<double>(0.0, 1.0)(rng);
        double r = std::sqrt(1.0 - z * z);
        return {r * std::cos(theta), r * std::sin(theta), z};
    }

    double rho = (1.0 + std::sqrt(1.0 + 4.0 * kappa * kappa)) / (2.0 * kappa);
    double z = uniform_neg_pos_1(rng);
    w = (1.0 + rho * z) / (rho + z);

    // Step 2: Sample a uniform direction on the orthogonal subspace
    geometry::Point3D u(uniform_neg_pos_1(rng), uniform_neg_pos_1(rng), uniform_neg_pos_1(rng));
    geometry::GeometryUtils::normalizePoint(u);

    // Step 3: Combine to produce a point
    geometry::Point3D mu(component.mu_x, component.mu_y, component.mu_z);
    double dot = geometry::GeometryUtils::dotProduct(mu, u);
    
    // Subtract projection
    u = u - mu * dot;
    geometry::GeometryUtils::normalizePoint(u);

    // Construct final sample
    geometry::Point3D sample = mu * w + u * std::sqrt(1.0 - w * w);
    return sample;
}
