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
 * @file MAB_SSRRT_1L.cpp
 * @brief Implementation of MAB-SSRRT-1L (Single-Layer MAB variant)
 * 
 * This file implements the single-layer Multi-Armed Bandit Sphere-Sampled RRT.
 * Key difference from 2L variant: Uses one flat MAB with 3 arms instead of
 * nested 2-arm MABs.
 */

#include "ompl/geometric/planners/disassemblyrrt/MAB_SSRRT_1L.h"
#include <limits>
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
ompl::geometric::MAB_SSRRT_1L::MAB_SSRRT_1L(
    const base::SpaceInformationPtr& si, 
    const std::string& yamlFilePath)
    : base::Planner(si, "MAB-SSRRT-1L")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    
    // Declare configurable parameters for OMPL's parameter system
    Planner::declareParam<double>("range", this, &MAB_SSRRT_1L::setRange, 
                                  &MAB_SSRRT_1L::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &MAB_SSRRT_1L::setGoalBias, 
                                  &MAB_SSRRT_1L::getGoalBias, "0.:.05:1.");

    // Load all configuration from YAML file
    loadYAMLConfig(yamlFilePath);

    // Create single flat MAB with 3 arms:
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
void ompl::geometric::MAB_SSRRT_1L::loadYAMLConfig(const std::string& yamlFilePath)
{
    try
    {
        YAML::Node config = YAML::LoadFile(yamlFilePath);

        // ----- Goal Bias -----
        // Different bias values for uniform vs cylinder sampling
        uniformGoalBias_ = config["uniform_goal_bias"].as<double>();
        sphereGoalBias_ = config["sphere_goal_bias"].as<double>();

        // ----- MAB Settings -----
        // Use uniform/sphere window size as single window size for flat MAB
        mabWindowSize_ = config["mab_uniform_sphere_window_size"].as<int>();
        
        // ----- Adaptive Sphere Sampling -----
        // These control the burn-in phase that discovers constraint directions
        adaptiveQuasirandomSampleSize_ = config["adaptive_quasirandom_sample_size"].as<int>();
        adaptiveStartRadius_ = config["adaptive_start_radius"].as<double>();
        adaptiveMinRadius_ = config["adaptive_min_radius"].as<double>();
        adaptiveShrinkStep_ = config["adaptive_shrink_step"].as<double>();
        adaptiveGrowStep_ = config["adaptive_grow_step"].as<double>();
        adaptiveMinExpectedValidityRate_ = config["adaptive_min_expected_validity_rate"].as<double>();
        adaptiveMaxExpectedValidityRate_ = config["adaptive_max_expected_validity_rate"].as<double>();
        adaptiveBurninMaxSteps_ = config["adaptive_burnin_max_steps"] ? 
                                  config["adaptive_burnin_max_steps"].as<int>() : 10;

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
        forcedUniformAfterCylinderValidStreak_ = config["forced_uniform_after_cylinder_valid_streak"] ? 
                                                 config["forced_uniform_after_cylinder_valid_streak"].as<int>() : 0;
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

ompl::geometric::MAB_SSRRT_1L::~MAB_SSRRT_1L()
{
    freeMemory();
}

/**
 * Resets planner state for a new planning episode.
 * Called by clear() and when replanning.
 */
void ompl::geometric::MAB_SSRRT_1L::clear()
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

void ompl::geometric::MAB_SSRRT_1L::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));

    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    
    initializeArms();
}

/**
 * Frees all Motion nodes and sampling arms.
 */
void ompl::geometric::MAB_SSRRT_1L::freeMemory()
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
void ompl::geometric::MAB_SSRRT_1L::initializeArms()
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
void ompl::geometric::MAB_SSRRT_1L::createCylinderSamplingArm()
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
void ompl::geometric::MAB_SSRRT_1L::createUniformSamplingArm()
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
void ompl::geometric::MAB_SSRRT_1L::copySampleVectorIntoState(
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
void ompl::geometric::MAB_SSRRT_1L::sampleUniformHypothesis(base::State* sample_state)
{
    uniformRealVecSampler_->sampleSelectedIndices(sample_state, samplingArms_.at(1)->axesIndices);
}

/**
 * Computes the validity rate of samples at the current radius.
 * Used during burn-in to adjust the sampling radius.
 */
double ompl::geometric::MAB_SSRRT_1L::computeValidityRate()
{
    base::State* sample_state = si_->allocState();
    base::State* origin_state = si_->allocState();
    si_->getStateSpace()->copyFromReals(origin_state, std::vector<double>(si_->getStateDimension(), 0.0));

    auto& sphereHypothesis = *samplingArms_[0];
    auto& sphere = sphereHypothesis.sphere;

    // Test each sphere sample for motion validity
    for (size_t i = 0; i < sphere->getSpherePoints().size(); i++)
    {
        auto sample = sphere->getSample();
        copySampleVectorIntoState(sample_state, sphereHypothesis,
                                  {sample.second.x, sample.second.y, sample.second.z});

        bool validFlag = si_->checkMotion(origin_state, sample_state);
        sphere->popIndexFromIndices(sample.first, validFlag);
    }
    si_->getStateSpace()->enforceBounds(sample_state);

    si_->freeState(sample_state);
    si_->freeState(origin_state);

    return sphere->getValidSampleRate();
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
void ompl::geometric::MAB_SSRRT_1L::setupAdaptiveSphereSampling()
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
bool ompl::geometric::MAB_SSRRT_1L::performInitialUniformCheck()
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
 */
void ompl::geometric::MAB_SSRRT_1L::performAdaptiveBurnin()
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

bool ompl::geometric::MAB_SSRRT_1L::shouldExitEarlyOnFullValidity(
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

bool ompl::geometric::MAB_SSRRT_1L::isValidityRateInTargetRange(double validity_rate) const
{
    return (validity_rate > adaptiveMinExpectedValidityRate_ &&
            validity_rate <= adaptiveMaxExpectedValidityRate_);
}

double ompl::geometric::MAB_SSRRT_1L::adjustRadiusBasedOnValidity(
    std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere,
    double current_radius,
    double validity_rate)
{
    if (validity_rate < adaptiveMinExpectedValidityRate_)
    {
        // Validity too low → shrink radius (keep valid samples)
        current_radius *= std::exp(-adaptiveShrinkStep_);
        sphere->appendCachedValidSamples();
    }
    else if (validity_rate > adaptiveMaxExpectedValidityRate_)
    {
        // Validity too high → grow radius (discard samples)
        current_radius *= std::exp(adaptiveGrowStep_);
        sphere->clearCachedValidSamples();
    }

    return current_radius;
}

void ompl::geometric::MAB_SSRRT_1L::finalizeBurnin(
    std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere,
    double final_radius)
{
    sphere->bestRadius = final_radius;
    
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
 * In the 1L variant, we have a flat 3-arm MAB:
 *   Arm 0 = UNIFORM
 *   Arm 1 = CYLINDER_UP
 *   Arm 2 = CYLINDER_DOWN
 */
void ompl::geometric::MAB_SSRRT_1L::updateRewards(bool isValid, MABPath selectedMABPath)
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
bool ompl::geometric::MAB_SSRRT_1L::getSample(
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
    double radius = sphere->bestRadius * extensionFactor;
    
    // PHASE 1: Select sampling arm using single-layer MAB (3 arms)
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
        radius, 
        selectedMABPath
    );
    
    // PHASE 4: Update statistics and rewards
    updateSamplingStatistics(selectedMABPath, isSampleValid, sample_state, originState);
    
    // Return captured nearest motion
    if (nearestMotionOut != nullptr)
    {
        *nearestMotionOut = capturedNearest;
    }
    
    return isSampleValid;
}

bool ompl::geometric::MAB_SSRRT_1L::checkSamplingPrerequisites(
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
 * Selects which arm to use via the single-layer MAB.
 * 
 * In the 1L variant, the MAB directly chooses between 3 arms:
 *   0 = UNIFORM
 *   1 = CYLINDER_UP
 *   2 = CYLINDER_DOWN
 * 
 * This is simpler than the 2L variant which uses nested MABs.
 */
ompl::geometric::MAB_SSRRT_1L::SamplerArm 
ompl::geometric::MAB_SSRRT_1L::selectSamplingArm()
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
        // Single-layer MAB selection: directly choose from 3 arms
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
bool ompl::geometric::MAB_SSRRT_1L::shouldSampleGoal(
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
bool ompl::geometric::MAB_SSRRT_1L::generateAndValidateSample(
    base::State* sample_state,
    ValidateFunc& validateState,
    double radius,
    MABPath& selectedMABPath)
{
    bool isSampleValid = false;
    
    if (selectedSamplerArm_ == SamplerArm::CYLINDER_UP || 
        selectedSamplerArm_ == SamplerArm::CYLINDER_DOWN)
    {
        isSampleValid = sampleFromCylinder(sample_state, validateState, radius, selectedMABPath);
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
bool ompl::geometric::MAB_SSRRT_1L::sampleFromCylinder(
    base::State* sample_state,
    ValidateFunc& validateState,
    double radius,
    MABPath& selectedMABPath)
{
    auto& sphere = samplingArms_.at(0)->sphere;
    
    if (sphere->getAllValidPoints().empty())
    {
        return fallbackToUniform(sample_state, validateState, selectedMABPath);
    }
    
    try
    {
        // Direction already determined by single-layer MAB selection
        int cylinderDirection = (selectedSamplerArm_ == SamplerArm::CYLINDER_UP) ? 0 : 1;
        selectedMABPath = (selectedSamplerArm_ == SamplerArm::CYLINDER_UP) 
                        ? MABPath::CYLINDER_UP 
                        : MABPath::CYLINDER_DOWN;
        
        sampling::AdaptiveSphereSampler::Point sample_point = 
            sphere->getRandomSampleFromCylinder(radius, cylinderDirection);
        
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
bool ompl::geometric::MAB_SSRRT_1L::fallbackToUniform(
    base::State* sample_state,
    ValidateFunc& validateState,
    MABPath& selectedMABPath)
{
    selectedSamplerArm_ = SamplerArm::UNIFORM;
    selectedMABPath = MABPath::UNIFORM;
    sampleUniformHypothesis(sample_state);
    return validateState(sample_state);
}

void ompl::geometric::MAB_SSRRT_1L::updateSamplingStatistics(
    MABPath selectedMABPath,
    bool isSampleValid,
    base::State* /*sample_state*/,
    base::State* /*originState*/)
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

ompl::geometric::MAB_SSRRT_1L::Motion* 
ompl::geometric::MAB_SSRRT_1L::addMotionToTree(
    base::State* state, Motion* parent, MotionOrigin origin)
{
    auto* motion = new Motion(si_);
    si_->copyState(motion->state, state);
    motion->parent = parent;
    motion->bornFrom = origin;
    nn_->add(motion);
    return motion;
}

bool ompl::geometric::MAB_SSRRT_1L::checkAndUpdateSolution(
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
ompl::geometric::MAB_SSRRT_1L::Motion* 
ompl::geometric::MAB_SSRRT_1L::findBestMotionForGoal(
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

ompl::base::State* ompl::geometric::MAB_SSRRT_1L::applyDistanceLimit(
    Motion* from, base::State* to, base::State* buffer, double distance)
{
    if (distance > maxDistance_)
    {
        si_->getStateSpace()->interpolate(from->state, to, maxDistance_ / distance, buffer);
        return buffer;
    }
    return to;
}

bool ompl::geometric::MAB_SSRRT_1L::handleGoalSamplePath(
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
bool ompl::geometric::MAB_SSRRT_1L::handleNormalSamplePath(
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
ompl::base::PlannerStatus ompl::geometric::MAB_SSRRT_1L::solve(
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
    
    const double extensionFactor = std::exp(sphereExtensionEps_);

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

void ompl::geometric::MAB_SSRRT_1L::getPlannerData(base::PlannerData &data) const
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

