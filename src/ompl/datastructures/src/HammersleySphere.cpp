//
// Created by serboba on 22.02.25.
//
/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#include <ompl/datastructures/HammersleySphere.h>
#include <ompl/datastructures/statistics/VMFMixtureModel.h>
#include <ompl/datastructures/statistics/CylinderFitter.h>

using namespace ompl;
using namespace ompl::statistics;
using namespace ompl::geometry;

// Constructor implementation
HammersleySphere::HammersleySphere(int sampleSize_, double radius_)
    : rng(std::random_device{}()),
      radius(radius_),
      sampleSize(sampleSize_),
      uniform_0_1_distribution(0.0, 1.0),
      uniform_neg_pos_1_distribution(-1.0, 1.0),
      sampler_(std::make_unique<sampling::AdaptiveSphereSampler>(nullptr, sampleSize_, radius_))
{
}

// Normalize all points in allValidSamples
void HammersleySphere::normalizeAllPoints(std::vector<Point> &allPoints)
{
    for (auto& point : allPoints)
    {
        normalizePoint(point);
    }
}

// Function to initialize parameters
std::vector<HammersleySphere::VMFComponent> HammersleySphere::initializeParameters(std::vector<Point> &allPoints, int K)
{
    // Delegated to VMFMixtureModel via sampler or directly?
    // The original code called VMFMixtureModel::initializeParameters directly.
    // We can keep it that way or add a wrapper in AdaptiveSphereSampler.
    // Since this is a static method in VMFMixtureModel, we can call it directly.
    // But we need to include VMFMixtureModel.h.
    // Wait, AdaptiveSphereSampler includes it.
    // Let's delegate to sampler if possible, but sampler doesn't expose initializeParameters.
    // It exposes fitVMFMixtureModel.
    // Let's just include VMFMixtureModel.h and call it directly for backward compatibility of this specific method.
    return VMFMixtureModel::initializeParameters(allPoints, K, rng);
}

void HammersleySphere::testInitializeVMF(int k_, int max_iter, double tol_)
{
    sphereComponents = fitVMFMixtureModel(k_, max_iter, tol_);
}

// Compute the normalization constant C_d(kappa) for 3D
double HammersleySphere::computeNormalizationConstant(double kappa)
{
    return VMFDistribution::computeNormalizationConstant(kappa);
}

std::vector<double> HammersleySphere::computeResponsibilities(std::vector<Point> &allPoints, std::vector<VMFComponent>& components)
{
    return VMFMixtureModel::computeResponsibilities(allPoints, components);
}

// Update parameters for each component
void HammersleySphere::updateParameters(std::vector<Point> &allPoints,
    const std::vector<double>& responsibilities,
    std::vector<VMFComponent>& components)
{
    VMFMixtureModel::updateParameters(allPoints, responsibilities, components);
}

double HammersleySphere::computeLogLikelihood(std::vector<Point> &allPoints, const std::vector<VMFComponent>& components)
{
    return VMFMixtureModel::computeLogLikelihood(allPoints, components);
}

std::vector<HammersleySphere::VMFComponent> HammersleySphere::fitVMFMixtureModel(int K, int maxIter, double tol)
{
    // Delegate to sampler
    auto components = sampler_->fitVMFMixtureModel(K, maxIter, tol);
    // Sync
    this->sphereComponents = sampler_->getSphereComponents();
    return components;
}

int HammersleySphere::selectComponent()
{
    return sampler_->selectComponent();
}

void HammersleySphere::computeCovariance(const std::vector<Point>& points, Eigen::Matrix3d& cov)
{
    CylinderFitter::computeCovariance(points, cov);
}

void HammersleySphere::normalize(double& x, double& y, double& z)
{
    CylinderFitter::normalize(x, y, z);
}

void HammersleySphere::largestEigenVector(double C[3][3], double& ex, double& ey, double& ez)
{
    CylinderFitter::largestEigenVector(C, ex, ey, ez);
}

HammersleySphere::Cylinder HammersleySphere::fitCylinder(const std::vector<Point>& points)
{
    return CylinderFitter::fitCylinder(points);
}

void HammersleySphere::createSamplesFibonacciLattice(double customRadius)
{
    sampler_->appendFibonacciSamples(customRadius);
    this->spherePoints = sampler_->getSpherePoints();
}

void HammersleySphere::createSamplesUniform(double customRadius)
{
    sampler_->appendUniformSamples(customRadius);
    this->spherePoints = sampler_->getSpherePoints();
}

void HammersleySphere::collectQuasiRandomSamples(double radius_)
{
    sampler_->setFibonacciJitterRadius(this->fibonacciJitterRadius);
    sampler_->collectQuasiRandomSamples(radius_);
    this->spherePoints = sampler_->getSpherePoints();
    this->indices = sampler_->getIndices();
}

void HammersleySphere::popIndexFromIndices(int index, bool valid)
{
    sampler_->popIndexFromIndices(index, valid);
    this->indices = sampler_->getIndices();
    this->allPoints_cached = sampler_->getAllPointsCached();
}

void HammersleySphere::appendCachedValidSamples()
{
    sampler_->appendCachedValidSamples();
    this->allValidPoints = sampler_->getAllValidPoints();
    this->allPoints_cached = sampler_->getAllPointsCached();
}

double HammersleySphere::getValidSampleRate()
{
    return sampler_->getValidSampleRate();
}

std::pair<int, HammersleySphere::Point> HammersleySphere::getSample()
{
    return sampler_->getSample();
}

HammersleySphere::Point HammersleySphere::sampleFromVMF(int componentID, double newRadius_, double kappaMultiplier_)
{
    return sampler_->sampleFromVMF(componentID, newRadius_, kappaMultiplier_);
}

void HammersleySphere::updateComponentCenter(int componentIndex)
{
    sampler_->updateComponentCenter(componentIndex);
    this->sphereComponents = sampler_->getSphereComponents();
}

void HammersleySphere::addValidSampleToComponent(Point& p)
{
    sampler_->addValidSampleToComponent(p);
    this->allValidPoints = sampler_->getAllValidPoints();
    this->sphereComponents = sampler_->getSphereComponents();
    this->bestRadius = sampler_->getBestRadius();
}

HammersleySphere::Point HammersleySphere::getRandomSampleFromCylinder(double extensionHeight, int chosenDirection)
{
    sampler_->setCylinderSamplingRadiusMultiplier(this->cylinderSamplingRadiusMultiplier);
    sampler_->setCylinderRadiusOffsetMultiplier(this->cylinderRadiusOffsetMultiplier);
    return sampler_->getRandomSampleFromCylinder(extensionHeight, chosenDirection);
}

HammersleySphere::Point HammersleySphere::getRandomSampleFromCylinder(double extensionHeight, int chosenDirection, const Eigen::Vector3d &axis)
{
    sampler_->setCylinderSamplingRadiusMultiplier(this->cylinderSamplingRadiusMultiplier);
    sampler_->setCylinderRadiusOffsetMultiplier(this->cylinderRadiusOffsetMultiplier);
    return sampler_->getRandomSampleFromCylinder(extensionHeight, chosenDirection, axis);
}

void HammersleySphere::setPCAFilterTopPercent(double val)
{
    this->pcaFilterTopPercent = val;
    sampler_->setPCAFilterTopPercent(val);
}
