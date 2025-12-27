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

/**
 * @file MAB_RRT.cpp
 * @brief Implementation of MAB-RRT (Multi-Armed Bandit RRT)
 * 
 * This file implements the Multi-Armed Bandit RRT planner with adaptive sampling.
 */

#include "ompl/geometric/planners/disassemblyrrt/MAB_RRT.h"
#include <limits>
#include <filesystem>
#include <ompl/base/spaces/SE3StateSpace.h>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <yaml-cpp/yaml.h>

// =============================================================================
// CONSTRUCTOR
// =============================================================================
/**
 * Initializes the planner with:
 * 1. Base planner configuration
 * 2. YAML-based parameter loading
 * 3. Single 3-arm MAB for sampling strategy selection
 */
ompl::geometric::MAB_RRT::MAB_RRT(
    const base::SpaceInformationPtr& si, 
    const std::string& yamlFilePath)
    : base::Planner(si, "MAB-RRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    
    // Declare configurable parameters for OMPL's parameter system
    Planner::declareParam<double>("range", this, &MAB_RRT::setRange, 
                                  &MAB_RRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &MAB_RRT::setGoalBias, 
                                  &MAB_RRT::getGoalBias, "0.:.05:1.");

    // Load all configuration from YAML file
    loadYAMLConfig(yamlFilePath);

    // Create MAB with 3 arms:
    //   Arm 0 = UNIFORM
    //   Arm 1 = CYLINDER_UP
    //   Arm 2 = CYLINDER_DOWN
    mab_sampler_ = std::make_unique<MAB_SlidingWindowUCB>(kNumMABArms, mabWindowSize_);
}

// =============================================================================
// YAML CONFIGURATION LOADING
// =============================================================================
/**
 * Loads all planner parameters from a YAML configuration file.
 * 
 * Categories:
 * - Goal bias settings
 * - MAB window sizes
 * - Adaptive sphere sampling parameters
 * - Reward system configuration
 * - Cylinder sampling configuration
 */
void ompl::geometric::MAB_RRT::loadYAMLConfig(const std::string& yamlFilePath)
{
    try
    {
        // Resolve the config file path to absolute
        std::filesystem::path configPath(yamlFilePath);
        std::string resolvedPath;
        std::vector<std::string> pathsToTry;
        
        if (configPath.is_absolute()) {
            pathsToTry.push_back(yamlFilePath);
        } else {
            std::filesystem::path cwd = std::filesystem::current_path();
            
            // If path starts with ../, try removing the ../ first (common case when running from root)
            if (yamlFilePath.find("../") == 0) {
                pathsToTry.push_back((cwd / yamlFilePath.substr(3)).lexically_normal().string());
            }
            
            // Try 1: Resolve from current working directory
            pathsToTry.push_back((cwd / configPath).lexically_normal().string());
            
            // Try 2: Original path as-is (might be relative to executable)
            pathsToTry.push_back(yamlFilePath);
            
            // Try 3: If in build/ directory, try from parent
            if (cwd.filename() == "build") {
                pathsToTry.push_back((cwd.parent_path() / configPath).lexically_normal().string());
                if (yamlFilePath.find("../") == 0) {
                    pathsToTry.push_back((cwd.parent_path() / yamlFilePath.substr(3)).lexically_normal().string());
                }
            }
        }
        
        // Find the first path that exists
        bool found = false;
        for (const auto& path : pathsToTry) {
            if (std::filesystem::exists(path) && std::filesystem::is_regular_file(path)) {
                try {
                    resolvedPath = std::filesystem::canonical(path).string();
                    found = true;
                    break;
                } catch (const std::exception&) {
                    resolvedPath = path;
                    found = true;
                    break;
                }
            }
        }
        
        if (!found) {
            std::string errorMsg = "Config file not found. Tried:\n";
            for (const auto& path : pathsToTry) {
                errorMsg += "  - " + path + "\n";
            }
            errorMsg += "Current directory: " + std::filesystem::current_path().string();
            throw YAML::Exception(YAML::Mark::null_mark(), errorMsg);
        }
        
        YAML::Node config = YAML::LoadFile(resolvedPath);

        // ----- Goal Bias -----
        // Different bias values for uniform vs cylinder sampling
        uniformGoalBias_ = config["uniform_goal_bias"].as<double>();
        sphereGoalBias_ = config["sphere_goal_bias"].as<double>();

        // ----- MAB Settings -----
        // Use uniform/sphere window size as MAB window size
        mabWindowSize_ = config["mab_uniform_sphere_window_size"].as<int>();
        
        // ----- Adaptive Sphere Sampling -----
        // These control the burn-in phase that discovers constraint directions
        adaptiveQuasirandomSampleSize_ = config["adaptive_quasirandom_sample_size"].as<int>();
        adaptiveMinExpectedValidityRate_ = config["adaptive_min_expected_validity_rate"].as<double>();
        adaptiveMaxExpectedValidityRate_ = config["adaptive_max_expected_validity_rate"].as<double>();
        adaptiveBurninMaxSteps_ = config["adaptive_burnin_max_steps"] ? 
                                  config["adaptive_burnin_max_steps"].as<int>() : 50;
        
        // Grow/shrink parameters
        adaptiveStartRadius_ = config["adaptive_start_radius"] ? 
                               config["adaptive_start_radius"].as<double>() : 0.1;
        adaptiveMinRadius_ = config["adaptive_min_radius"] ? 
                            config["adaptive_min_radius"].as<double>() : 0.0001;
        adaptiveShrinkStep_ = config["adaptive_shrink_step"] ? 
                             config["adaptive_shrink_step"].as<double>() : 0.1;
        adaptiveGrowStep_ = config["adaptive_grow_step"] ? 
                          config["adaptive_grow_step"].as<double>() : 0.1;
        
        // Maximum radius limit (from YAML, or computed from state space bounds if not provided)
        if (config["adaptive_max_radius"])
        {
            adaptiveMaxRadius_ = config["adaptive_max_radius"].as<double>();
        }

        // ----- Initial Uniform Check -----
        // Early exit optimization if problem is "easy" (high uniform validity)
        initialFreeSamplingProbability_ = config["initialFreeSamplingProbability"] ? 
                                          config["initialFreeSamplingProbability"].as<double>() : 0.5;
        initialNumberOfUniformSampleTrials_ = config["initialNumberOfUniformSampleTrials"] ? 
                                              config["initialNumberOfUniformSampleTrials"].as<int>() : 10;

        // ----- Sphere Extension -----
        // Factor to extend sampling radius after valid samples
        sphereExtensionEps_ = config["sphere_extension_eps"].as<double>();
        
        // ----- MAB Rewards -----
        // Reward values for valid/invalid samples by sampler type
        uniformSamplerInvalidReward_ = config["uniform_sampler_invalid_reward"].as<double>();
        sphereSamplerInvalidReward_ = config["sphere_sampler_invalid_reward"].as<double>();
        uniformSamplerFixedValidReward_ = config["uniform_sampler_fixed_valid_reward"].as<double>();
        sphereSamplerFixedValidReward_ = config["sphere_sampler_fixed_valid_reward"].as<double>();

        // ----- Cylinder Configuration -----
        // Parameters controlling cylinder shape and sampling
        cylinderRadiusOffsetMultiplier_ = config["cylinder_radius_offset_multiplier"].as<double>();
        fibonacciJitterRadius_ = config["fibonacci_jitter_radius"].as<double>();
        cylinderSamplingRadiusMultiplier_ = config["cylinder_sampling_radius_multiplier"].as<double>();
        pcaFilterTopPercent_ = config["pca_filter_top_percent"] ? 
                               config["pca_filter_top_percent"].as<double>() : 1.0;
        
        // ----- Dynamic PCA -----
        // If true, re-compute cylinder axis as new samples are collected
        enableDynamicCylinderPCA_ = config["enableDynamicCylinderPCA"] ? 
                                    config["enableDynamicCylinderPCA"].as<bool>() : false;

        // ----- Burn-in Early Exit -----
        // Exit planning early if burn-in finds very few valid samples
        burninEarlyExitEnabled_ = config["burnin_early_exit_enabled"] ? 
                                  config["burnin_early_exit_enabled"].as<bool>() : false;
        burninMinValidSamplesForContinue_ = config["burnin_min_valid_samples_for_continue"] ? 
                                            config["burnin_min_valid_samples_for_continue"].as<unsigned int>() : 3;

        // ----- Consecutive Full Validity Exit -----
        // Exit burn-in early if all samples are valid (problem is unconstrained)
        earlyExitOnConsecutiveFullValidity_ = config["earlyExitOnConsecutiveFullValidity"] ? 
                                              config["earlyExitOnConsecutiveFullValidity"].as<bool>() : false;
        consecutiveFullValidityMaxIterations_ = config["consecutiveFullValidityMaxIterations"] ? 
                                                config["consecutiveFullValidityMaxIterations"].as<int>() : 5;

        // ----- Forced Uniform After Streak -----
        // Force uniform sample after N consecutive valid cylinder samples
        // (prevents getting stuck in local minima)
        if (config["forced_uniform_after_cylinder_valid_streak"])
        {
            forcedUniformAfterCylinderValidStreak_ = config["forced_uniform_after_cylinder_valid_streak"].as<int>();
        }
        else
        {
            forcedUniformAfterCylinderValidStreak_ = 0;
        }
    }
    catch (const YAML::Exception& e)
    {
        OMPL_ERROR("Error parsing YAML configuration file: %s", e.what());
        throw;
    }
}

// =============================================================================
// DESTRUCTOR & CLEANUP
// =============================================================================

ompl::geometric::MAB_RRT::~MAB_RRT()
{
    freeMemory();
}

/**
 * Resets planner state for a new planning episode.
 * Called by clear() and when replanning.
 */
void ompl::geometric::MAB_RRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
    currentIteration_ = 0;
    cylinderValidStreak_ = 0;
}

void ompl::geometric::MAB_RRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));

    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    
    initializeArms();
    
    // Compute maximum allowed burn-in radius from state space bounds
    // Do this after initializeArms() to ensure state space is fully set up
    // Only compute if not already set from YAML config
    if (adaptiveMaxRadius_ == std::numeric_limits<double>::infinity())
    {
        try {
            adaptiveMaxRadius_ = computeMaxBurninRadius();
        } catch (const std::exception& e) {
            OMPL_WARN("Failed to compute max burn-in radius: %s. Using default.", e.what());
            adaptiveMaxRadius_ = 1000.0;
        }
    }
}

/**
 * Frees all Motion nodes and sampling arms.
 */
void ompl::geometric::MAB_RRT::freeMemory()
{
    std::vector<Motion*> motions;
    
    if (nn_)
    {
        nn_->list(motions);
        for (auto& motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
    
    samplingArms_.clear();
}

// =============================================================================
// SAMPLING ARM INITIALIZATION
// =============================================================================

/**
 * Initializes the two sampling arms:
 * - Arm 0: Cylinder sampler (XYZ only, uses adaptive sphere)
 * - Arm 1: Uniform sampler (all dimensions)
 */
void ompl::geometric::MAB_RRT::initializeArms()
{
    samplingArms_.clear();
    createCylinderSamplingArm();
    createUniformSamplingArm();
}

/**
 * Creates the cylinder sampling arm.
 * 
 * For 6D: The cylinder sampler only samples XYZ (translation), not rotation.
 *         This is controlled by the axesMask {0,0,0,1,1,1} where:
 *         - First 3 zeros: Don't sample RPY (rotation)
 *         - Last 3 ones: Sample XYZ (translation)
 * 
 * For 2D: The cylinder sampler samples XY (translation only).
 *         This is controlled by the axesMask {1,1} where:
 *         - Both ones: Sample XY
 */
void ompl::geometric::MAB_RRT::createCylinderSamplingArm()
{
    int stateDim = si_->getStateDimension();
    std::vector<int> cylinderAxesMask;
    std::vector<int> translationIndices;
    
    if (stateDim == 2) {
        // 2D case: Sample XY
        cylinderAxesMask = {1, 1};
        translationIndices = {0, 1};
    } else {
        // 6D case: Sample XYZ (after RPY)
        cylinderAxesMask = {0, 0, 0, 1, 1, 1};
        translationIndices = {3, 4, 5};
    }

    auto sphere = std::make_unique<sampling::AdaptiveSphereSampler>(
        si_.get(), 
        adaptiveQuasirandomSampleSize_, 
        adaptiveStartRadius_, 
        translationIndices
    );
    
    // Configure cylinder sampling parameters
    sphere->setCylinderRadiusOffsetMultiplier(cylinderRadiusOffsetMultiplier_);
    sphere->setFibonacciJitterRadius(fibonacciJitterRadius_);
    sphere->setCylinderSamplingRadiusMultiplier(cylinderSamplingRadiusMultiplier_);

    samplingArms_.push_back(std::make_unique<SamplingArm>(std::move(sphere), cylinderAxesMask));
}

/**
 * Creates the uniform sampling arm.
 * 
 * The uniform sampler samples all dimensions.
 * - 2D: axesMask {1,1} means sample all 2 dimensions
 * - 6D: axesMask {1,1,1,1,1,1} means sample all 6 dimensions
 */
void ompl::geometric::MAB_RRT::createUniformSamplingArm()
{
    int stateDim = si_->getStateDimension();
    std::vector<int> uniformAxesMask(stateDim, 1);  // Sample all dimensions
    samplingArms_.push_back(std::make_unique<SamplingArm>(nullptr, uniformAxesMask));
}

// =============================================================================
// UTILITY METHODS
// =============================================================================

/**
 * Copies a sample vector (x,y,z) into an OMPL state.
 * Uses the axesMask to determine which dimensions to fill.
 */
void ompl::geometric::MAB_RRT::copySampleVectorIntoState(
    base::State* sample_state, 
    SamplingArm& hypothesis,
    std::vector<double> vec)
{
    auto it = vec.begin();

    std::vector<double> sample_reals(hypothesis.axesIndices.size(), 0);
    for (unsigned int i = 0; i < hypothesis.axesIndices.size(); i++)
    {
        if (hypothesis.axesIndices[i] == 1)
        {
            sample_reals[i] = *it;
            ++it;
        }
        else
            sample_reals[i] = 0.0;
    }

    si_->getStateSpace()->copyFromReals(sample_state, sample_reals);
}

/**
 * Samples uniformly across all state space dimensions.
 */
void ompl::geometric::MAB_RRT::sampleUniformHypothesis(base::State* sample_state)
{
    if (!uniformRealVecSampler_)
    {
        // Fallback: use base sampler if uniformRealVecSampler_ not yet initialized
        if (!sampler_)
            sampler_ = si_->allocStateSampler();
        sampler_->sampleUniform(sample_state);
        return;
    }
    uniformRealVecSampler_->sampleSelectedIndices(sample_state, samplingArms_.at(1)->axesIndices);
}

/**
 * Computes the validity rate of samples at the current radius.
 * Used during burn-in to adjust the sampling radius.
 */
double ompl::geometric::MAB_RRT::computeValidityRate()
{
    base::State* sample_state = si_->allocState();
    base::State* origin_state = si_->allocState();
    si_->getStateSpace()->copyFromReals(origin_state, std::vector<double>(si_->getStateDimension(), 0.0));

    auto& sphereHypothesis = *samplingArms_[0];
    auto& sphere = sphereHypothesis.sphere;

    // Test each sphere sample for validity
    // checkMotion validates both states and the path between them
    for (size_t i = 0; i < sphere->getSpherePoints().size(); i++)
    {
        auto sample = sphere->getSample();
        copySampleVectorIntoState(sample_state, sphereHypothesis,
                                  {sample.second.x, sample.second.y, sample.second.z});
        si_->getStateSpace()->enforceBounds(sample_state);

        // Check if motion from origin to sample is valid (checkMotion already validates both states)
        bool validFlag = si_->checkMotion(origin_state, sample_state);
        
        sphere->popIndexFromIndices(sample.first, validFlag);
    }
    si_->getStateSpace()->enforceBounds(sample_state);

    si_->freeState(sample_state);
    si_->freeState(origin_state);

    return sphere->getValidSampleRate();
}

/**
 * Computes the maximum allowed burn-in radius based on state space bounds.
 * The radius should not exceed the translational bounds to avoid sampling outside the valid region.
 */
double ompl::geometric::MAB_RRT::computeMaxBurninRadius() const
{
    // Safety check: ensure si_ is valid and set up
    if (!si_ || !si_->getStateSpace())
    {
        OMPL_WARN("SpaceInformation not available, using default max radius");
        return 1000.0;
    }
    
    // Ensure state space is set up before accessing bounds
    if (!si_->isSetup())
    {
        OMPL_WARN("SpaceInformation not set up yet, using default max radius");
        return 1000.0;
    }
    
    // Get state space as RealVectorStateSpace
    const auto* rvss = dynamic_cast<const base::RealVectorStateSpace*>(si_->getStateSpace().get());
    if (!rvss)
    {
        // If not RealVectorStateSpace, return a large default value
        OMPL_WARN("State space is not RealVectorStateSpace, using default max radius");
        return 1000.0;
    }
    
    // Get bounds - use const reference (getBounds returns const reference)
    const base::RealVectorBounds& bounds = rvss->getBounds();
    int stateDim = si_->getStateDimension();
    
    // Safety check: ensure bounds are valid
    if (bounds.low.empty() || bounds.high.empty() || bounds.low.size() != bounds.high.size())
    {
        OMPL_WARN("Invalid bounds, using default max radius");
        return 1000.0;
    }
    
    // Determine translation indices (same logic as createCylinderSamplingArm)
    std::vector<int> translationIndices;
    if (stateDim == 2) {
        // 2D case: XY
        translationIndices = {0, 1};
    } else {
        // 6D case: XYZ (after RPY)
        translationIndices = {3, 4, 5};
    }
    
    // Find minimum extent across translation dimensions
    // The max radius should be the minimum of (|low|, |high|) for each translation dimension
    // to ensure we don't exceed bounds in any dimension
    double minExtent = std::numeric_limits<double>::infinity();
    
    for (int idx : translationIndices)
    {
        if (idx >= 0 && idx < static_cast<int>(bounds.low.size()))
        {
            double low = bounds.low[idx];
            double high = bounds.high[idx];
            // Maximum radius is the minimum distance from origin to either bound
            double extent = std::min(std::abs(low), std::abs(high));
            if (extent < minExtent)
            {
                minExtent = extent;
            }
        }
    }
    
    // If we couldn't determine bounds, return a large default
    if (minExtent == std::numeric_limits<double>::infinity())
    {
        OMPL_WARN("Could not determine max radius from bounds, using default");
        return 1000.0;
    }
    
    return minExtent;
}

// =============================================================================
// ADAPTIVE SPHERE SAMPLING SETUP (BURN-IN PHASE)
// =============================================================================
/**
 * Sets up the adaptive sphere sampler.
 * 
 * This is the "burn-in" phase that:
 * 1. Discovers which motion directions are valid
 * 2. Finds the optimal sampling radius
 * 3. Seeds the cylinder sampler with valid samples
 */
void ompl::geometric::MAB_RRT::setupAdaptiveSphereSampling()
{
    // First, check if uniform sampling works well (problem is "easy")
    if (performInitialUniformCheck())
    {
        return;  // Skip adaptive burn-in
    }
    
    // Perform adaptive radius adjustment
    performAdaptiveBurnin();
}

/**
 * Tests if uniform sampling has high validity rate.
 * If yes, skip the expensive adaptive sphere burn-in.
 */
bool ompl::geometric::MAB_RRT::performInitialUniformCheck()
{
    if (initialNumberOfUniformSampleTrials_ <= 0)
    {
        return false;
    }

    base::State* sample_state = si_->allocState();
    Motion* tempMotion = new Motion(si_);

    const int total_samples = initialNumberOfUniformSampleTrials_;
    const int required_valid = static_cast<int>(std::floor(initialFreeSamplingProbability_ * total_samples));

    int valid_count = 0;
    for (int i = 0; i < total_samples; ++i)
    {
        sampleUniformHypothesis(sample_state);
        si_->copyState(tempMotion->state, sample_state);
        Motion* nearest = nn_->nearest(tempMotion);
        
        if (si_->checkMotion(nearest->state, sample_state))
        {
            valid_count++;
            addMotionToTree(sample_state, nearest, MotionOrigin::UNIFORM);

            if (valid_count > required_valid)
                break;
        }
    }
    
    si_->freeState(sample_state);
    if (tempMotion->state) si_->freeState(tempMotion->state);
    delete tempMotion;

    double validity_rate = static_cast<double>(valid_count) / total_samples;

    if (validity_rate > initialFreeSamplingProbability_)
    {
        // Problem is easy - use uniform sampling
        selectedSamplerArm_ = SamplerArm::UNIFORM;
        samplingArms_.at(0)->sphere->clearValidPoints();
        return true;
    }

    return false;
}

/**
 * Iteratively adjusts sampling radius to find optimal sphere size.
 * 
 * Algorithm:
 * 1. Sample at current radius
 * 2. Compute validity rate
 * 3. If too low: shrink radius, keep valid samples
 * 4. If too high: grow radius, discard samples
 * 5. Repeat until in target range or max steps reached
 * 
 * Uses gradient-like log-space updates for stability and speed.
 */
void ompl::geometric::MAB_RRT::performAdaptiveBurnin()
{
    auto& sphere = samplingArms_.at(0)->sphere;
    sphere->setPCAFilterTopPercent(pcaFilterTopPercent_);
    
    selectedSamplerArm_ = SamplerArm::UNIFORM;
    double current_radius = adaptiveStartRadius_;
    
    int current_step = 0;
    int consecutive_full_validity_count = 0;
    
    while (current_radius > adaptiveMinRadius_)
    {
        // Generate samples at current radius
        sphere->collectQuasiRandomSamples(current_radius);
        double validity_rate = computeValidityRate();
        
        // Check for early exit conditions
        if (shouldExitEarlyOnFullValidity(validity_rate, consecutive_full_validity_count, sphere))
            break;

        if (isValidityRateInTargetRange(validity_rate) || current_step >= adaptiveBurninMaxSteps_)
        {
            sphere->appendCachedValidSamples();
            break;
        }

        // Adjust radius based on validity feedback
        current_radius = adjustRadiusBasedOnValidity(sphere, current_radius, validity_rate);
        current_step++;
    }
    
    finalizeBurnin(sphere, current_radius);
}

double ompl::geometric::MAB_RRT::adjustRadiusBasedOnValidity(
    std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere,
    double current_radius,
    double validity_rate)
{
    if (validity_rate < adaptiveMinExpectedValidityRate_)
    {
        // Validity too low → shrink radius (keep valid samples)
        // Use direct multiplication instead of exp()
        current_radius *= adaptiveShrinkStep_;
        sphere->appendCachedValidSamples();
    }
    else if (validity_rate > adaptiveMaxExpectedValidityRate_)
    {
        // Validity too high → grow radius (discard samples)
        // Use direct multiplication instead of exp()
        current_radius *= adaptiveGrowStep_;
        sphere->clearCachedValidSamples();
    }
    // Note: If validity_rate is exactly in target range, no adjustment is made

    // Clamp radius to maximum allowed (based on state space bounds)
    if (current_radius > adaptiveMaxRadius_)
    {
        current_radius = adaptiveMaxRadius_;
    }

    return current_radius;
}

bool ompl::geometric::MAB_RRT::shouldExitEarlyOnFullValidity(
    double validity_rate,
    int& consecutive_count,
    std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere)
{
    if (!earlyExitOnConsecutiveFullValidity_)
    {
        return false;
    }

    if (validity_rate >= 1.0)
    {
        consecutive_count++;
        
        if (consecutive_count >= consecutiveFullValidityMaxIterations_)
        {
            sphere->appendCachedValidSamples();
            return true;
        }
    }
    else
    {
        consecutive_count = 0;
    }

    return false;
}

bool ompl::geometric::MAB_RRT::isValidityRateInTargetRange(double validity_rate) const
{
    return (validity_rate > adaptiveMinExpectedValidityRate_ &&
            validity_rate <= adaptiveMaxExpectedValidityRate_);
}


void ompl::geometric::MAB_RRT::finalizeBurnin(
    std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere,
    double final_radius)
{
    sphere->bestRadius = final_radius;
    burninFinalRadius_ = final_radius;  // Store burn-in radius for extension height calculation
    
    // DEBUG: Print burn-in finalization
    OMPL_INFORM("DEBUG FINALIZE_BURNIN: final_radius=%.6f, bestRadius set to %.6f", 
               final_radius, sphere->bestRadius);
    
    // Fix: Set cylinder height to bestRadius instead of using maxHeight from point projections
    // This ensures extension starts from the correct position (bestRadius) rather than
    // from a potentially larger height that includes samples from higher radii during burn-in
    if (!sphere->getAllValidPoints().empty())
    {
        // Ensure cylinder is fitted first (if not already)
        sphere->updateCylinderAxis();
        // Trigger a fit by calling getRandomSampleFromCylinder with dummy params to ensure cylinder is fitted
        sphere->getRandomSampleFromCylinder(0.0, 0);
        
        // DEBUG: We can't access cylinder directly, but we'll print after setting
        // Now set the height to bestRadius (the final burn-in radius)
        sphere->setCylinderHeight(final_radius);
        OMPL_INFORM("DEBUG FINALIZE_BURNIN: Set cylinder height to bestRadius: %.6f", final_radius);
    }
    
    size_t total_valid_samples = sphere->getAllValidPoints().size();
    
    // Check if we should abort planning (not enough valid samples found)
    if (burninEarlyExitEnabled_ && total_valid_samples < burninMinValidSamplesForContinue_)
    {
        burninEarlyExitTriggered_ = true;
    }
    else
    {
        burninEarlyExitTriggered_ = false;
    }
}

// =============================================================================
// MAB REWARD UPDATE
// =============================================================================
/**
 * Updates MAB rewards based on sample validity.
 * 
 * The MAB has 3 arms:
 *   Arm 0 = UNIFORM
 *   Arm 1 = CYLINDER_UP
 *   Arm 2 = CYLINDER_DOWN
 */
void ompl::geometric::MAB_RRT::updateRewards(bool isValid, MABPath selectedMABPath)
{
    double reward = 0.0;
    int armIndex = -1;
    
    switch (selectedMABPath)
    {
        case MABPath::UNIFORM:
            reward = isValid ? uniformSamplerFixedValidReward_ : uniformSamplerInvalidReward_;
            armIndex = 0;
            break;
        case MABPath::CYLINDER_UP:
            reward = isValid ? sphereSamplerFixedValidReward_ : sphereSamplerInvalidReward_;
            armIndex = 1;
            break;
        case MABPath::CYLINDER_DOWN:
            reward = isValid ? sphereSamplerFixedValidReward_ : sphereSamplerInvalidReward_;
            armIndex = 2;
            break;
        case MABPath::NONE:
            return;  // No update for goal samples
    }
    
    if (armIndex >= 0)
    {
        mab_sampler_->update(armIndex, reward);
    }
}

// =============================================================================
// GET SAMPLE - Main sampling entry point
// =============================================================================
/**
 * Generates a sample state using MAB-guided sampling.
 * 
 * Flow:
 * 1. Check prerequisites (arms initialized, sphere ready)
 * 2. Select arm via MAB (UNIFORM, CYL_UP, or CYL_DOWN)
 * 3. Check goal bias (maybe sample from goal)
 * 4. Generate sample from selected arm
 * 5. Validate sample and update MAB rewards
 * 
 * Returns: true if valid sample generated, false otherwise
 */
bool ompl::geometric::MAB_RRT::getSample(
    base::State *sample_state, 
    Motion* tempMotion, 
    double extensionFactor, 
    base::State* originState, 
    const base::PlannerTerminationCondition& ptc, 
    Motion** nearestMotionOut, 
    bool* isGoalSampleOut)
{
    // Capture validation results for nearest motion query
    Motion* capturedNearest = nullptr;
    bool capturedIsValid = false;
    
    // Lambda for validating states (avoids redundant nearest queries)
    auto validateState = [&](base::State* state) -> bool
    {
        si_->copyState(tempMotion->state, state);
        si_->getStateSpace()->enforceBounds(state);
        
        if (nn_->size() == 0)
        {
            capturedNearest = nullptr;
            capturedIsValid = si_->isValid(state);
            return capturedIsValid;
        }
        
        Motion* nearest = nn_->nearest(tempMotion);
        capturedNearest = nearest;
        bool result = si_->checkMotion(nearest->state, state);
        capturedIsValid = result;
        return result;
    };
    
    // Check prerequisites
    if (!checkSamplingPrerequisites(sample_state, isGoalSampleOut))
    {
        return true;  // Fallback sample was generated
    }
    
    auto& sphere = samplingArms_.at(0)->sphere;
    // Exponential growth: extensionHeight = bestRadius * (exp(extensionFactor - 1.0) - 1.0)
    // This allows fast exponential growth even when bestRadius is very small (e.g., 0.001)
    // extensionFactor = 1.0 → extensionHeight = 0 (no extension)
    // extensionFactor = 2.0 → extensionHeight = bestRadius * (exp(1.0) - 1.0) ≈ bestRadius * 1.718
    // extensionFactor = 3.0 → extensionHeight = bestRadius * (exp(2.0) - 1.0) ≈ bestRadius * 6.389
    double extensionHeight = sphere->bestRadius * (std::exp(extensionFactor - 1.0) - 1.0);
    
    // DEBUG: Print extension calculation
    if (selectedSamplerArm_ == SamplerArm::CYLINDER_UP || selectedSamplerArm_ == SamplerArm::CYLINDER_DOWN) {
        OMPL_INFORM("DEBUG EXTENSION: bestRadius=%.6f, extensionFactor=%.6f, extensionHeight=%.6f", 
                   sphere->bestRadius, extensionFactor, extensionHeight);
    }
    
    // PHASE 1: Select sampling arm using MAB (3 arms)
    selectSamplingArm();
    
    // PHASE 2: Check for goal sampling
    if (shouldSampleGoal(isGoalSampleOut, nearestMotionOut, sample_state))
    {
        return true;  // Goal sample generated
    }
    
    if (isGoalSampleOut) *isGoalSampleOut = false;
    
    // PHASE 3: Generate and validate sample
    MABPath selectedMABPath = MABPath::NONE;
    bool isSampleValid = generateAndValidateSample(
        sample_state, 
        validateState, 
        extensionHeight, 
        selectedMABPath
    );
    
    // PHASE 4: Update statistics and rewards
    updateSamplingStatistics(selectedMABPath, isSampleValid);
    
    // Return captured nearest motion
    if (nearestMotionOut != nullptr)
    {
        *nearestMotionOut = capturedNearest;
    }
    
    return isSampleValid;
}

bool ompl::geometric::MAB_RRT::checkSamplingPrerequisites(
    base::State* sample_state,
    bool* isGoalSampleOut)
{
    if (samplingArms_.empty())
    {
        OMPL_ERROR("samplingArms_ is empty!");
        return false;
    }
    
    auto& sphere = samplingArms_.at(0)->sphere;
    if (!sphere)
    {
        // Fallback to uniform if sphere not initialized
        sampleUniformHypothesis(sample_state);
        if (isGoalSampleOut) *isGoalSampleOut = false;
        return false;
    }
    
    return true;
}

// =============================================================================
// ARM SELECTION
// =============================================================================
/**
 * Selects which arm to use via the MAB.
 * 
 * The MAB directly chooses between 3 arms:
 *   0 = UNIFORM
 *   1 = CYLINDER_UP
 *   2 = CYLINDER_DOWN
 */
ompl::geometric::MAB_RRT::SamplerArm 
ompl::geometric::MAB_RRT::selectSamplingArm()
{
    // Check for forced uniform after consecutive cylinder success
    if (forcedUniformAfterCylinderValidStreak_ > 0 &&
        cylinderValidStreak_ >= forcedUniformAfterCylinderValidStreak_)
    {
        selectedSamplerArm_ = SamplerArm::UNIFORM;
        cylinderValidStreak_ = 0;
    }
    else
    {
        // MAB selection: directly choose from 3 arms
        int armIndex = mab_sampler_->chooseArm();
        
        switch (armIndex)
        {
            case 0:
                selectedSamplerArm_ = SamplerArm::UNIFORM;
                break;
            case 1:
                selectedSamplerArm_ = SamplerArm::CYLINDER_UP;
                break;
            case 2:
                selectedSamplerArm_ = SamplerArm::CYLINDER_DOWN;
                break;
            default:
                selectedSamplerArm_ = SamplerArm::UNIFORM;
                break;
        }
    }
    
    // Fallback if cylinder selected but no valid points exist
    auto& sphere = samplingArms_.at(0)->sphere;
    if ((selectedSamplerArm_ == SamplerArm::CYLINDER_UP || 
         selectedSamplerArm_ == SamplerArm::CYLINDER_DOWN) && 
        sphere->getAllValidPoints().empty())
    {
        selectedSamplerArm_ = SamplerArm::UNIFORM;
    }
    
    return selectedSamplerArm_;
}

/**
 * Checks if we should sample from the goal region.
 * Goal bias is different for uniform vs cylinder arms.
 */
bool ompl::geometric::MAB_RRT::shouldSampleGoal(
    bool* isGoalSampleOut,
    Motion** nearestMotionOut,
    base::State* sample_state)
{
    // Use different goal bias based on selected arm
    double currentGoalBias = (selectedSamplerArm_ == SamplerArm::UNIFORM) 
                           ? uniformGoalBias_ 
                           : sphereGoalBias_;
    
    base::Goal* goal = pdef_->getGoal().get();
    auto* goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    if (goal_s != nullptr && rng_.uniform01() < currentGoalBias && goal_s->canSample())
    {
        goal_s->sampleGoal(sample_state);
        
        if (isGoalSampleOut) *isGoalSampleOut = true;
        if (nearestMotionOut) *nearestMotionOut = nullptr;
        
        return true;
    }
    
    return false;
}

// =============================================================================
// SAMPLE GENERATION AND VALIDATION
// =============================================================================

template <typename ValidateFunc>
bool ompl::geometric::MAB_RRT::generateAndValidateSample(
    base::State* sample_state,
    ValidateFunc& validateState,
    double extensionHeight,
    MABPath& selectedMABPath)
{
    bool isSampleValid = false;
    
    if (selectedSamplerArm_ == SamplerArm::CYLINDER_UP || 
        selectedSamplerArm_ == SamplerArm::CYLINDER_DOWN)
    {
        isSampleValid = sampleFromCylinder(sample_state, validateState, extensionHeight, selectedMABPath);
    }
    else  // UNIFORM
    {
        selectedMABPath = MABPath::UNIFORM;
        sampleUniformHypothesis(sample_state);
        isSampleValid = validateState(sample_state);
    }
    
    return isSampleValid;
}

/**
 * Samples from the cylinder.
 * 
 * In 1L, the direction (UP or DOWN) was already selected by the MAB,
 * so we just use whichever arm was chosen.
 */
template <typename ValidateFunc>
bool ompl::geometric::MAB_RRT::sampleFromCylinder(
    base::State* sample_state,
    ValidateFunc& validateState,
    double extensionHeight,
    MABPath& selectedMABPath)
{
    auto& sphere = samplingArms_.at(0)->sphere;
    
    if (sphere->getAllValidPoints().empty())
    {
        return fallbackToUniform(sample_state, validateState, selectedMABPath);
    }
    
    try
    {
        // Direction already determined by MAB selection
        int cylinderDirection = (selectedSamplerArm_ == SamplerArm::CYLINDER_UP) ? 0 : 1;
        selectedMABPath = (selectedSamplerArm_ == SamplerArm::CYLINDER_UP) 
                        ? MABPath::CYLINDER_UP 
                        : MABPath::CYLINDER_DOWN;
        
        // DEBUG: Print before calling getRandomSampleFromCylinder
        OMPL_INFORM("DEBUG CYLINDER: Calling getRandomSampleFromCylinder with extensionHeight=%.6f", extensionHeight);
        
        sampling::AdaptiveSphereSampler::Point sample_point = 
            sphere->getRandomSampleFromCylinder(extensionHeight, cylinderDirection);
        
        // DEBUG: Print after sampling
        OMPL_INFORM("DEBUG CYLINDER: Sampled point at (%.6f, %.6f, %.6f) with sampledAtRadius=%.6f", 
                   sample_point.x, sample_point.y, sample_point.z, sample_point.sampledAtRadius);
        
        copySampleVectorIntoState(sample_state, *samplingArms_[0], 
                                {sample_point.x, sample_point.y, sample_point.z});
        bool isValid = validateState(sample_state);
        
        if (isValid)
        {
            sphere->addValidSampleToComponent(sample_point);
            
            if (enableDynamicCylinderPCA_)
            {
                sphere->updateCylinderAxis();
            }
        }
        
        return isValid;
    }
    catch (const std::exception& e)
    {
        return fallbackToUniform(sample_state, validateState, selectedMABPath);
    }
}

template <typename ValidateFunc>
bool ompl::geometric::MAB_RRT::fallbackToUniform(
    base::State* sample_state,
    ValidateFunc& validateState,
    MABPath& selectedMABPath)
{
    selectedSamplerArm_ = SamplerArm::UNIFORM;
    selectedMABPath = MABPath::UNIFORM;
    sampleUniformHypothesis(sample_state);
    return validateState(sample_state);
}

void ompl::geometric::MAB_RRT::updateSamplingStatistics(
    MABPath selectedMABPath,
    bool isSampleValid)
{
    updateRewards(isSampleValid, selectedMABPath);
    
    // Track consecutive valid CYLINDER samples (for either direction)
    if ((selectedMABPath == MABPath::CYLINDER_UP || selectedMABPath == MABPath::CYLINDER_DOWN) && isSampleValid)
        cylinderValidStreak_++;
    else
        cylinderValidStreak_ = 0;
}

// =============================================================================
// SOLVE HELPER METHODS
// =============================================================================

ompl::geometric::MAB_RRT::Motion* 
ompl::geometric::MAB_RRT::addMotionToTree(
    base::State* state, Motion* parent, MotionOrigin origin)
{
    auto* motion = new Motion(si_);
    si_->copyState(motion->state, state);
    motion->parent = parent;
    motion->bornFrom = origin;
    nn_->add(motion);
    return motion;
}

bool ompl::geometric::MAB_RRT::checkAndUpdateSolution(
    Motion* motion, base::Goal* goal,
    Motion*& solution, Motion*& approxsol, double& approxdif)
{
    double dist = 0.0;
    bool satisfied = goal->isSatisfied(motion->state, &dist);
    
    if (satisfied)
    {
        approxdif = dist;
        solution = motion;
        return true;
    }
    
    if (dist < approxdif)
    {
        approxdif = dist;
        approxsol = motion;
    }
    
    return false;
}

/**
 * Finds the best motion node for goal connection.
 * 
 * Strategy (3-stage):
 * 1. Prefer non-exhausted UNIFORM nodes (most likely to connect to goal)
 * 2. Try non-exhausted CYLINDER nodes
 * 3. Fall back to nearest neighbor
 */
ompl::geometric::MAB_RRT::Motion* 
ompl::geometric::MAB_RRT::findBestMotionForGoal(
    base::State* goalState, Motion* queryMotion)
{
    std::vector<Motion*> motions;
    nn_->list(motions);
    
    Motion* bestMotion = nullptr;
    double bestDist = std::numeric_limits<double>::infinity();
    
    // Stage 1: Prefer non-exhausted UNIFORM nodes
    for (Motion* m : motions)
    {
        if (m->goalConnectionExhausted) continue;
        if (m->bornFrom != MotionOrigin::UNIFORM) continue;
        
        double dist = si_->distance(m->state, goalState);
        if (dist < bestDist)
        {
            bestDist = dist;
            bestMotion = m;
        }
    }
    
    if (bestMotion != nullptr)
        return bestMotion;
    
    // Stage 2: Look for non-exhausted CYLINDER nodes
    for (Motion* m : motions)
    {
        if (m->goalConnectionExhausted) continue;
        if (m->bornFrom != MotionOrigin::CYLINDER) continue;
        
        double dist = si_->distance(m->state, goalState);
        if (dist < bestDist)
        {
            bestDist = dist;
            bestMotion = m;
        }
    }
    
    if (bestMotion != nullptr)
        return bestMotion;
    
    // Stage 3: All nodes exhausted, fall back to nearest
    return nn_->nearest(queryMotion);
}

ompl::base::State* ompl::geometric::MAB_RRT::applyDistanceLimit(
    Motion* from, base::State* to, base::State* buffer, double distance)
{
    if (distance > maxDistance_)
    {
        si_->getStateSpace()->interpolate(from->state, to, maxDistance_ / distance, buffer);
        return buffer;
    }
    return to;
}

bool ompl::geometric::MAB_RRT::handleGoalSamplePath(
    base::State* rstate, base::State* xstate,
    Motion* rmotion, base::State* originState, base::Goal* goal,
    Motion*& solution, Motion*& approxsol, double& approxdif)
{
    Motion* nmotion = findBestMotionForGoal(rstate, rmotion);
    
    double d = si_->distance(nmotion->state, rstate);
    base::State* dstate = applyDistanceLimit(nmotion, rstate, xstate, d);
    
    bool isMotionValid = si_->checkMotion(nmotion->state, dstate);
    
    if (!isMotionValid)
    {
        nmotion->goalConnectionFailures++;
        nmotion->goalConnectionExhausted = true;
    }
    
    // Update MAB rewards
    MABPath path = MABPath::NONE;
    switch (selectedSamplerArm_)
    {
        case SamplerArm::UNIFORM:
            path = MABPath::UNIFORM;
            break;
        case SamplerArm::CYLINDER_UP:
            path = MABPath::CYLINDER_UP;
            break;
        case SamplerArm::CYLINDER_DOWN:
            path = MABPath::CYLINDER_DOWN;
            break;
    }
    
    if (path != MABPath::NONE)
        updateRewards(isMotionValid, path);

    if (isMotionValid)
    {
        MotionOrigin origin = (selectedSamplerArm_ == SamplerArm::UNIFORM) 
                            ? MotionOrigin::UNIFORM 
                            : MotionOrigin::CYLINDER;
        Motion* motion = addMotionToTree(dstate, nmotion, origin);
        return checkAndUpdateSolution(motion, goal, solution, approxsol, approxdif);
    }
    
    return false;
}

/**
 * Handles normal (non-goal) sample path.
 * 
 * Key difference for CYLINDER samples:
 * - Connect directly (full motion already validated)
 * 
 * For UNIFORM samples:
 * - Apply step-wise distance limit (traditional RRT extension)
 */
bool ompl::geometric::MAB_RRT::handleNormalSamplePath(
    base::State* rstate, base::State* xstate,
    Motion* nearestMotion, base::Goal* goal,
    Motion*& solution, Motion*& approxsol, double& approxdif)
{
    MotionOrigin origin = (selectedSamplerArm_ == SamplerArm::UNIFORM) 
                        ? MotionOrigin::UNIFORM 
                        : MotionOrigin::CYLINDER;

    if (nearestMotion == nullptr)
    {
        Motion* motion = addMotionToTree(rstate, nullptr, origin);
        return checkAndUpdateSolution(motion, goal, solution, approxsol, approxdif);
    }
    
    base::State* dstate;
    
    // For CYLINDER samples (UP or DOWN): connect directly
    // For UNIFORM samples: apply step-wise interpolation
    if (selectedSamplerArm_ == SamplerArm::CYLINDER_UP || 
        selectedSamplerArm_ == SamplerArm::CYLINDER_DOWN)
    {
        dstate = rstate;
    }
    else
    {
        double d = si_->distance(nearestMotion->state, rstate);
        dstate = applyDistanceLimit(nearestMotion, rstate, xstate, d);
    }

    Motion* motion = addMotionToTree(dstate, nearestMotion, origin);
    return checkAndUpdateSolution(motion, goal, solution, approxsol, approxdif);
}

// =============================================================================
// MAIN SOLVE METHOD
// =============================================================================
/**
 * Main planning loop.
 * 
 * Algorithm:
 * 1. Add start states to tree
 * 2. Run burn-in phase (adaptive sphere sampling)
 * 3. While not terminated:
 *    a. Get sample (via MAB-guided sampling)
 *    b. Extend tree toward sample
 *    c. Check for goal connection
 * 4. Build solution path
 */
ompl::base::PlannerStatus ompl::geometric::MAB_RRT::solve(
    const base::PlannerTerminationCondition& ptc)
{
    checkValidity();
    base::Goal* goal = pdef_->getGoal().get();

    // Add start states to tree
    while (const base::State* st = pis_.nextStart())
    {
        auto* motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    uniformRealVecSampler_ = std::dynamic_pointer_cast<base::RealVectorStateSampler>(sampler_);
    if (!uniformRealVecSampler_)
        throw std::runtime_error("Sampler is not RealVectorStateSampler!");

    initializeArms();

    // Allocate working memory
    auto* rmotion = new Motion(si_);
    base::State* rstate = rmotion->state;
    auto* xstate = si_->allocState();
    auto* tempMotion = new Motion(si_);
    base::State* originState = si_->allocState();
    si_->getStateSpace()->copyFromReals(originState, std::vector<double>(si_->getStateDimension(), 0.0));

    // Run burn-in phase
    setupAdaptiveSphereSampling();
    
    // Check for early exit
    if (burninEarlyExitEnabled_ && burninEarlyExitTriggered_)
    {
        OMPL_WARN("%s: Exiting early due to insufficient burn-in valid samples.", getName().c_str());
        
        si_->freeState(xstate);
        si_->freeState(originState);
        if (tempMotion->state) si_->freeState(tempMotion->state);
        delete tempMotion;
        if (rmotion->state) si_->freeState(rmotion->state);
        delete rmotion;

        return base::PlannerStatus(false, false);
    }
    
    // Solution tracking
    Motion* solution = nullptr;
    Motion* approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    
    // Use direct multiplication instead of exp()
    const double extensionFactor = sphereExtensionEps_;
    
    // DEBUG: Print extension factor at start of planning
    OMPL_INFORM("DEBUG PLANNING_START: sphereExtensionEps_=%.6f, extensionFactor=%.6f", 
               sphereExtensionEps_, extensionFactor);

    // Main planning loop
    while (!ptc)
    {
        currentIteration_++;
        
        Motion* nearestMotion = nullptr;
        bool isGoalSample = false;
        
        // Get sample via MAB-guided sampling
        bool foundSample = getSample(rstate, tempMotion, extensionFactor, rmotion->state, ptc, 
                                     &nearestMotion, &isGoalSample);
        if (!foundSample)
            continue;
        
        bool solutionFound = false;
        
        // Process sample
        if (isGoalSample)
            solutionFound = handleGoalSamplePath(rstate, xstate, rmotion, originState, goal,
                                                 solution, approxsol, approxdif);
        else
            solutionFound = handleNormalSamplePath(rstate, xstate, nearestMotion, goal,
                                                   solution, approxsol, approxdif);
        
        if (solutionFound)
            break;
    }

    // Build solution path
    bool solved = false;
    bool approximate = false;
    
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        std::vector<Motion*> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        auto path = std::make_shared<PathGeometric>(si_);
        for (int i = static_cast<int>(mpath.size()) - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    // Cleanup
    si_->freeState(xstate);
    si_->freeState(originState);
    if (rmotion->state) si_->freeState(rmotion->state);
    delete rmotion;
    if (tempMotion->state) si_->freeState(tempMotion->state);
    delete tempMotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return solved ? (approximate ? base::PlannerStatus::APPROXIMATE_SOLUTION : base::PlannerStatus::EXACT_SOLUTION) :
                    base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::MAB_RRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto & motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

