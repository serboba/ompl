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
 * @file MAB_RRT_DEBUG.cpp
 * @brief Implementation of MAB-SSRRT (Multi-Armed Bandit Sphere-Sampled RRT)
 * 
 * This file implements the single-layer Multi-Armed Bandit Sphere-Sampled RRT
 * with debug tracking.
 */

 #include "ompl/geometric/planners/disassemblyrrt/MAB_RRT_DEBUG.h"
 #include <fstream>
 #include <iomanip>
 #include <sstream>
 #include <map>
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
 ompl::geometric::MAB_RRT_DEBUG::MAB_RRT_DEBUG(
     const base::SpaceInformationPtr& si, 
     const std::string& yamlFilePath)
     : MAB_RRT(si, yamlFilePath)
 {
     // Base class constructor handles all initialization including mab_sampler_
     // Debug tracking is enabled by default (debugEnabled_ = true)
     
     // Reload config to ensure DEBUG version's loadYAMLConfig is used
     // (Base constructor calls base class loadYAMLConfig which may use defaults)
     loadYAMLConfig(yamlFilePath);
     
     // Verify mab_sampler_ was initialized by base class
     if (!mab_sampler_) {
         OMPL_ERROR("DEBUG: mab_sampler_ is null after base constructor! Re-initializing...");
         mab_sampler_ = std::make_unique<MAB_SlidingWindowUCB>(kNumMABArms, mabWindowSize_);
     }
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
 void ompl::geometric::MAB_RRT_DEBUG::loadYAMLConfig(const std::string& yamlFilePath)
 {
     try
     {
         // Resolve the config file path to absolute (same logic as non-debug version)
         std::filesystem::path configPath(yamlFilePath);
         std::string resolvedConfigPath;
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
                     resolvedConfigPath = std::filesystem::canonical(path).string();
                     found = true;
                     break;
                 } catch (const std::exception&) {
                     resolvedConfigPath = path;
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
         
         // Extract absolute path to demos/disassembly/ directory from resolved config file path
         std::filesystem::path resolvedPath(resolvedConfigPath);
         std::filesystem::path targetDir = resolvedPath.parent_path();
         
         // Convert to absolute path and ensure it ends with /
         targetDir = std::filesystem::absolute(targetDir);
         outputDirectory_ = targetDir.string();
         if (!outputDirectory_.empty() && outputDirectory_.back() != '/') {
             outputDirectory_ += "/";
         }
         
         OMPL_INFORM("DEBUG: Output directory set to: %s", outputDirectory_.c_str());
         OMPL_INFORM("DEBUG: Loading config from: %s", resolvedConfigPath.c_str());
         
         YAML::Node config = YAML::LoadFile(resolvedConfigPath);
 
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
         OMPL_INFORM("DEBUG: Loaded adaptive_quasirandom_sample_size = %d", adaptiveQuasirandomSampleSize_);
         adaptiveStartRadius_ = config["adaptive_start_radius"].as<double>();
         OMPL_INFORM("DEBUG: Loaded adaptive_start_radius = %.10f", adaptiveStartRadius_);
         adaptiveMinRadius_ = config["adaptive_min_radius"].as<double>();
         OMPL_INFORM("DEBUG: Loaded adaptive_min_radius = %.10f", adaptiveMinRadius_);
         adaptiveShrinkStep_ = config["adaptive_shrink_step"].as<double>();
         adaptiveGrowStep_ = config["adaptive_grow_step"].as<double>();
         adaptiveMinExpectedValidityRate_ = config["adaptive_min_expected_validity_rate"].as<double>();
         adaptiveMaxExpectedValidityRate_ = config["adaptive_max_expected_validity_rate"].as<double>();
         adaptiveBurninMaxSteps_ = config["adaptive_burnin_max_steps"] ? 
                                   config["adaptive_burnin_max_steps"].as<int>() : 10;
         
         // Maximum radius limit (from YAML, or computed from state space bounds if not provided)
         if (config["adaptive_max_radius"])
         {
             adaptiveMaxRadius_ = config["adaptive_max_radius"].as<double>();
             OMPL_INFORM("DEBUG: Loaded adaptive_max_radius from YAML = %.10f", adaptiveMaxRadius_);
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
 
 ompl::geometric::MAB_RRT_DEBUG::~MAB_RRT_DEBUG()
 {
     // Base class destructor handles cleanup
 }
 
 /**
  * Resets planner state for a new planning episode.
  * Called by clear() and when replanning.
  */
 void ompl::geometric::MAB_RRT_DEBUG::clear()
 {
     Planner::clear();
     sampler_.reset();
     freeMemory();
     if (nn_)
         nn_->clear();
     lastGoalMotion_ = nullptr;
     currentIteration_ = 0;
     currentBurninStep_ = -1;
     cylinderValidStreak_ = 0;
     
     // DEBUG: Clear debug tracking
     debugSamples_.clear();
     sampleStateToIndex_.clear();
 }
 
 void ompl::geometric::MAB_RRT_DEBUG::setup()
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
        adaptiveMaxRadius_ = computeMaxBurninRadius();
        OMPL_INFORM("DEBUG: Computed max burn-in radius from state space bounds: %.6f", adaptiveMaxRadius_);
    }
    else
    {
        OMPL_INFORM("DEBUG: Using adaptive_max_radius from YAML: %.6f", adaptiveMaxRadius_);
    }
 }
 
 /**
  * Frees all Motion nodes and sampling arms.
  */
 void ompl::geometric::MAB_RRT_DEBUG::freeMemory()
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
 void ompl::geometric::MAB_RRT_DEBUG::initializeArms()
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
 void ompl::geometric::MAB_RRT_DEBUG::createCylinderSamplingArm()
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
 
     OMPL_INFORM("DEBUG: Creating AdaptiveSphereSampler with sampleSize=%d (from config)", adaptiveQuasirandomSampleSize_);
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
 void ompl::geometric::MAB_RRT_DEBUG::createUniformSamplingArm()
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
 void ompl::geometric::MAB_RRT_DEBUG::copySampleVectorIntoState(
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
 void ompl::geometric::MAB_RRT_DEBUG::sampleUniformHypothesis(base::State* sample_state)
 {
     if (uniformRealVecSampler_) {
         uniformRealVecSampler_->sampleSelectedIndices(sample_state, samplingArms_.at(1)->axesIndices);
     } else {
         sampler_->sampleUniform(sample_state);
     }
 }
 
 /**
  * Computes the validity rate of samples at the current radius.
  * Used during burn-in to adjust the sampling radius.
  */
 double ompl::geometric::MAB_RRT_DEBUG::computeValidityRate()
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
         
         // DEBUG: Track sample (these are from AdaptiveSphereSampler during burn-in)
         trackSample(sample_state, validFlag, "burnin", "sphere", 0.0, sample.second.sampledAtRadius);
         
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
 void ompl::geometric::MAB_RRT_DEBUG::setupAdaptiveSphereSampling()
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
 bool ompl::geometric::MAB_RRT_DEBUG::performInitialUniformCheck()
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
 void ompl::geometric::MAB_RRT_DEBUG::performAdaptiveBurnin()
 {
     OMPL_INFORM("DEBUG: Starting adaptive burn-in phase");
     
     auto& sphere = samplingArms_.at(0)->sphere;
     sphere->setPCAFilterTopPercent(pcaFilterTopPercent_);
     
     selectedSamplerArm_ = SamplerArm::UNIFORM;
     double current_radius = adaptiveStartRadius_;
     
     int current_step = 0;
     currentBurninStep_ = 0;  // Start burn-in step tracking
     int consecutive_full_validity_count = 0;
     
     OMPL_INFORM("DEBUG: Initial radius = %.10f, min radius = %.10f", current_radius, adaptiveMinRadius_);
     OMPL_INFORM("DEBUG: adaptiveStartRadius_ member variable = %.10f", adaptiveStartRadius_);
     
     while (current_radius > adaptiveMinRadius_)
     {
         currentBurninStep_ = current_step;  // Update current burn-in step
         OMPL_INFORM("DEBUG: Burn-in step %d: radius = %.10f", current_step, current_radius);
         
         // Generate samples at current radius
         OMPL_INFORM("DEBUG: Collecting %d quasi-random samples at radius %.10f", 
                    adaptiveQuasirandomSampleSize_, current_radius);
         sphere->collectQuasiRandomSamples(current_radius);
         double validity_rate = computeValidityRate();
         
         OMPL_INFORM("DEBUG: Burn-in step %d: validity rate = %.4f", current_step, validity_rate);
         
         // Check for early exit conditions
         if (shouldExitEarlyOnFullValidity(validity_rate, consecutive_full_validity_count, sphere))
         {
             OMPL_INFORM("DEBUG: Early exit triggered (full validity)");
             break;
         }
 
         if (isValidityRateInTargetRange(validity_rate) || current_step >= adaptiveBurninMaxSteps_)
         {
             OMPL_INFORM("DEBUG: Target validity range reached or max steps exceeded");
             sphere->appendCachedValidSamples();
             break;
         }
 
         // Adjust radius based on validity feedback
         double old_radius = current_radius;
         current_radius = adjustRadiusBasedOnValidity(sphere, current_radius, validity_rate);
         OMPL_INFORM("DEBUG: Radius adjusted: %.10f -> %.10f (change: %.2f%%)", 
                     old_radius, current_radius, 
                     old_radius > 0 ? 100.0 * (current_radius - old_radius) / old_radius : 0.0);
         current_step++;
     }
     
     OMPL_INFORM("DEBUG: Burn-in complete. Final radius = %.10f", current_radius);
     currentBurninStep_ = -1;  // End burn-in step tracking
     finalizeBurnin(sphere, current_radius);
 }
 
 bool ompl::geometric::MAB_RRT_DEBUG::shouldExitEarlyOnFullValidity(
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
 
 bool ompl::geometric::MAB_RRT_DEBUG::isValidityRateInTargetRange(double validity_rate) const
 {
     return (validity_rate > adaptiveMinExpectedValidityRate_ &&
             validity_rate <= adaptiveMaxExpectedValidityRate_);
 }
 
 double ompl::geometric::MAB_RRT_DEBUG::adjustRadiusBasedOnValidity(
     std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere,
     double current_radius,
     double validity_rate)
 {
     OMPL_INFORM("DEBUG: adjustRadiusBasedOnValidity: validity_rate=%.6f, min=%.6f, max=%.6f, current_radius=%.10f", 
                 validity_rate, adaptiveMinExpectedValidityRate_, adaptiveMaxExpectedValidityRate_, current_radius);
     
     if (validity_rate < adaptiveMinExpectedValidityRate_)
     {
         // Validity too low → shrink radius (keep valid samples)
         double old_radius = current_radius;
        // Use direct multiplication instead of exp()
        current_radius *= adaptiveShrinkStep_;
        OMPL_INFORM("DEBUG: Shrinking radius: %.10f -> %.10f (multiplier = %.6f)", 
                    old_radius, current_radius, adaptiveShrinkStep_);
        sphere->appendCachedValidSamples();
    }
    else if (validity_rate > adaptiveMaxExpectedValidityRate_)
     {
         // Validity too high → grow radius (discard samples)
         // Use direct multiplication instead of exp()
         double old_radius = current_radius;
         current_radius *= adaptiveGrowStep_;
         OMPL_INFORM("DEBUG: Growing radius: %.10f -> %.10f (multiplier = %.6f)", 
                     old_radius, current_radius, adaptiveGrowStep_);
         sphere->clearCachedValidSamples();
     }
    else
    {
        OMPL_INFORM("DEBUG: Validity rate in target range, no radius adjustment");
    }
    
    // Clamp radius to maximum allowed (based on state space bounds)
    if (current_radius > adaptiveMaxRadius_)
    {
        OMPL_INFORM("DEBUG: Clamping radius from %.10f to max allowed %.10f", current_radius, adaptiveMaxRadius_);
        current_radius = adaptiveMaxRadius_;
    }

    return current_radius;
}
 
void ompl::geometric::MAB_RRT_DEBUG::finalizeBurnin(
    std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere,
    double final_radius)
{
    sphere->bestRadius = final_radius;
    burninFinalRadius_ = final_radius;  // Store burn-in radius for extension height calculation
    
    // Fix: Set cylinder height to bestRadius instead of using maxHeight from point projections
    // This ensures extension starts from the correct position (bestRadius) rather than
    // from a potentially larger height that includes samples from higher radii during burn-in
    if (!sphere->getAllValidPoints().empty())
    {
        // Ensure cylinder is fitted first (if not already)
        sphere->updateCylinderAxis();
        // Trigger a fit by calling getRandomSampleFromCylinder with dummy params to ensure cylinder is fitted
        sphere->getRandomSampleFromCylinder(0.0, 0);
        // Now set the height to bestRadius (the final burn-in radius)
        sphere->setCylinderHeight(final_radius);
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
  * In the 1L variant, we have a flat 3-arm MAB:
  *   Arm 0 = UNIFORM
  *   Arm 1 = CYLINDER_UP
  *   Arm 2 = CYLINDER_DOWN
  */
 void ompl::geometric::MAB_RRT_DEBUG::updateRewards(bool isValid, MABPath selectedMABPath)
 {
     double reward = 0.0;
     int armIndex = -1;
     std::string samplerName;
     
     switch (selectedMABPath)
     {
         case MABPath::UNIFORM:
             reward = isValid ? uniformSamplerFixedValidReward_ : uniformSamplerInvalidReward_;
             armIndex = 0;
             samplerName = "uniform";
             break;
         case MABPath::CYLINDER_UP:
             reward = isValid ? sphereSamplerFixedValidReward_ : sphereSamplerInvalidReward_;
             armIndex = 1;
             samplerName = "cylinder_up";
             break;
         case MABPath::CYLINDER_DOWN:
             reward = isValid ? sphereSamplerFixedValidReward_ : sphereSamplerInvalidReward_;
             armIndex = 2;
             samplerName = "cylinder_down";
             break;
         case MABPath::NONE:
             return;  // No update for goal samples
     }
     
     if (armIndex >= 0)
     {
         if (mab_sampler_) {
             mab_sampler_->update(armIndex, reward);
             OMPL_INFORM("DEBUG: Updated reward for arm %d (%s): %.4f (valid=%d)", 
                         armIndex, samplerName.c_str(), reward, isValid ? 1 : 0);
         } else {
             OMPL_WARN("DEBUG: mab_sampler_ is null, cannot update reward!");
         }
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
 bool ompl::geometric::MAB_RRT_DEBUG::getSample(
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
     // Clamp bestRadius to adaptiveMaxRadius_ to prevent it from exceeding the max limit
     // (bestRadius can be updated from sampledAtRadius which can be very large)
     double clampedBestRadius = std::min(sphere->bestRadius, adaptiveMaxRadius_);
     
     // Exponential growth: extensionHeight = bestRadius * (exp(extensionFactor - 1.0) - 1.0)
     // This allows fast exponential growth even when bestRadius is very small (e.g., 0.001)
     // extensionFactor = 1.0 → extensionHeight = 0 (no extension)
     // extensionFactor = 2.0 → extensionHeight = bestRadius * (exp(1.0) - 1.0) ≈ bestRadius * 1.718
     // extensionFactor = 3.0 → extensionHeight = bestRadius * (exp(2.0) - 1.0) ≈ bestRadius * 6.389
     double extensionHeight = clampedBestRadius * (std::exp(extensionFactor - 1.0) - 1.0);
     
     // DEBUG: Print extension calculation
     if (selectedSamplerArm_ == SamplerArm::CYLINDER_UP || selectedSamplerArm_ == SamplerArm::CYLINDER_DOWN) {
         if (sphere->bestRadius > adaptiveMaxRadius_) {
             OMPL_INFORM("DEBUG EXTENSION: bestRadius=%.6f (clamped to %.6f), extensionFactor=%.6f, extensionHeight=%.6f", 
                        sphere->bestRadius, clampedBestRadius, extensionFactor, extensionHeight);
         } else {
             OMPL_INFORM("DEBUG EXTENSION: bestRadius=%.6f, extensionFactor=%.6f, extensionHeight=%.6f", 
                        clampedBestRadius, extensionFactor, extensionHeight);
         }
     }
     
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
         extensionHeight, 
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
 
 bool ompl::geometric::MAB_RRT_DEBUG::checkSamplingPrerequisites(
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
 ompl::geometric::MAB_RRT_DEBUG::SamplerArm 
 ompl::geometric::MAB_RRT_DEBUG::selectSamplingArm()
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
         if (!mab_sampler_) {
             OMPL_ERROR("DEBUG: mab_sampler_ is null! Falling back to UNIFORM.");
             selectedSamplerArm_ = SamplerArm::UNIFORM;
             return selectedSamplerArm_;
         }
         int armIndex = mab_sampler_->chooseArm();
         
         switch (armIndex)
         {
             case 0:
                 selectedSamplerArm_ = SamplerArm::UNIFORM;
                 OMPL_INFORM("DEBUG: MAB selected arm %d: UNIFORM", armIndex);
                 break;
             case 1:
                 selectedSamplerArm_ = SamplerArm::CYLINDER_UP;
                 OMPL_INFORM("DEBUG: MAB selected arm %d: CYLINDER_UP", armIndex);
                 break;
             case 2:
                 selectedSamplerArm_ = SamplerArm::CYLINDER_DOWN;
                 OMPL_INFORM("DEBUG: MAB selected arm %d: CYLINDER_DOWN", armIndex);
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
 bool ompl::geometric::MAB_RRT_DEBUG::shouldSampleGoal(
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
bool ompl::geometric::MAB_RRT_DEBUG::generateAndValidateSample(
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
         
         // DEBUG tracking moved to updateSamplingStatistics
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
bool ompl::geometric::MAB_RRT_DEBUG::sampleFromCylinder(
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
         // Direction already determined by single-layer MAB selection
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
 bool ompl::geometric::MAB_RRT_DEBUG::fallbackToUniform(
     base::State* sample_state,
     ValidateFunc& validateState,
     MABPath& selectedMABPath)
 {
     selectedSamplerArm_ = SamplerArm::UNIFORM;
     selectedMABPath = MABPath::UNIFORM;
     sampleUniformHypothesis(sample_state);
     return validateState(sample_state);
 }
 
 void ompl::geometric::MAB_RRT_DEBUG::updateSamplingStatistics(
     MABPath selectedMABPath,
     bool isSampleValid,
     base::State* sample_state,
     base::State* /*originState*/)
 {
     // Get reward for tracking
     double reward = 0.0;
     std::string samplerName;
     switch (selectedMABPath) {
         case MABPath::UNIFORM:
             reward = isSampleValid ? uniformSamplerFixedValidReward_ : uniformSamplerInvalidReward_;
             samplerName = "uniform";
             break;
         case MABPath::CYLINDER_UP:
             reward = isSampleValid ? sphereSamplerFixedValidReward_ : sphereSamplerInvalidReward_;
             samplerName = "cylinder_up";
             break;
         case MABPath::CYLINDER_DOWN:
             reward = isSampleValid ? sphereSamplerFixedValidReward_ : sphereSamplerInvalidReward_;
             samplerName = "cylinder_down";
             break;
         case MABPath::NONE:
             return;
     }
     
    // Track planning phase sample (both valid and invalid for debugging)
    // Note: Connection info will be added when sample is actually connected to tree
    if (sample_state != nullptr) {
        trackSample(sample_state, isSampleValid, "planning", samplerName, reward, 0.0, nullptr, false);
    }
     
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
 
 ompl::geometric::MAB_RRT_DEBUG::Motion* 
 ompl::geometric::MAB_RRT_DEBUG::addMotionToTree(
     base::State* state, Motion* parent, MotionOrigin origin)
 {
     auto* motion = new Motion(si_);
     si_->copyState(motion->state, state);
     motion->parent = parent;
     motion->bornFrom = origin;
     nn_->add(motion);
     
     // DEBUG: Track connection for planning phase samples
     if (debugEnabled_ && parent != nullptr) {
         const auto* realState = state->as<base::RealVectorStateSpace::StateType>();
         const auto* parentRealState = parent->state->as<base::RealVectorStateSpace::StateType>();
         if (realState && parentRealState) {
             int stateDim = si_->getStateDimension();
             double sample_x, sample_y, parent_x, parent_y;
             if (stateDim == 2) {
                 sample_x = realState->values[0];
                 sample_y = realState->values[1];
                 parent_x = parentRealState->values[0];
                 parent_y = parentRealState->values[1];
             } else if (stateDim >= 6) {
                 sample_x = realState->values[3];
                 sample_y = realState->values[4];
                 parent_x = parentRealState->values[3];
                 parent_y = parentRealState->values[4];
             } else {
                 return motion;
             }
             
             // Try to find matching sample using state key first
             std::string stateKey = getStateKey(state);
             bool found = false;
             if (!stateKey.empty()) {
                 auto it = sampleStateToIndex_.find(stateKey);
                 if (it != sampleStateToIndex_.end()) {
                     size_t index = it->second;
                     if (index < debugSamples_.size() && !debugSamples_[index].wasConnected) {
                         debugSamples_[index].wasConnected = true;
                         debugSamples_[index].nearest_x = parent_x;
                         debugSamples_[index].nearest_y = parent_y;
                         found = true;
                     }
                 }
             }
             
             // Fallback: search by coordinates with tolerance (for cases where state was modified)
             if (!found) {
                 // Find matching sample in reverse order (most recent first)
                 for (auto it = debugSamples_.rbegin(); it != debugSamples_.rend(); ++it) {
                     if (it->phase == "planning" && !it->wasConnected) {
                         double dx = std::abs(it->x - sample_x);
                         double dy = std::abs(it->y - sample_y);
                         // Use larger tolerance to account for distance limiting
                         if (dx < 0.5 && dy < 0.5) {
                             it->wasConnected = true;
                             it->nearest_x = parent_x;
                             it->nearest_y = parent_y;
                             found = true;
                             break;
                         }
                     }
                 }
             }
             
             // If still not found, infer sampler from the origin parameter passed to this function
             // This should rarely happen if tracking is working correctly, but use origin as fallback
             if (!found) {
                 std::string inferredSampler = "uniform";  // Default fallback
                 if (origin == MotionOrigin::UNIFORM) {
                     inferredSampler = "uniform";
                 } else if (origin == MotionOrigin::CYLINDER) {
                     // For CYLINDER, we need to check which direction was used
                     // Use selectedSamplerArm_ to determine if it's up or down
                     if (selectedSamplerArm_ == SamplerArm::CYLINDER_UP) {
                         inferredSampler = "cylinder_up";
                     } else if (selectedSamplerArm_ == SamplerArm::CYLINDER_DOWN) {
                         inferredSampler = "cylinder_down";
                     } else {
                         inferredSampler = "cylinder_up";  // Default to up if unknown
                     }
                 }
                 trackSample(state, true, "planning", inferredSampler, 0.0, 0.0, parent->state, true);
             }
         }
     } else if (debugEnabled_ && parent == nullptr) {
         // Root node - track it
         trackSample(state, true, "planning", "root", 0.0, 0.0, nullptr, false);
     }
     
     return motion;
 }
 
 bool ompl::geometric::MAB_RRT_DEBUG::checkAndUpdateSolution(
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
 ompl::geometric::MAB_RRT_DEBUG::Motion* 
 ompl::geometric::MAB_RRT_DEBUG::findBestMotionForGoal(
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
 
 ompl::base::State* ompl::geometric::MAB_RRT_DEBUG::applyDistanceLimit(
     Motion* from, base::State* to, base::State* buffer, double distance)
 {
     if (distance > maxDistance_)
     {
         si_->getStateSpace()->interpolate(from->state, to, maxDistance_ / distance, buffer);
         return buffer;
     }
     return to;
 }
 
 bool ompl::geometric::MAB_RRT_DEBUG::handleGoalSamplePath(
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
 bool ompl::geometric::MAB_RRT_DEBUG::handleNormalSamplePath(
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
         
         // DEBUG: If distance limiting modified the state, track the actual connected state
         // The original rstate was already tracked, but dstate might be different
         if (debugEnabled_ && dstate != rstate) {
             // dstate is the actual state that will be connected - track it if different
             const auto* dRealState = dstate->as<base::RealVectorStateSpace::StateType>();
             const auto* rRealState = rstate->as<base::RealVectorStateSpace::StateType>();
             if (dRealState && rRealState) {
                 int stateDim = si_->getStateDimension();
                 double dx, dy, rx, ry;
                 if (stateDim == 2) {
                     dx = dRealState->values[0];
                     dy = dRealState->values[1];
                     rx = rRealState->values[0];
                     ry = rRealState->values[1];
                 } else {
                     dx = dRealState->values[3];
                     dy = dRealState->values[4];
                     rx = rRealState->values[3];
                     ry = rRealState->values[4];
                 }
                 // If significantly different, track the actual connected state
                 if (std::abs(dx - rx) > 0.01 || std::abs(dy - ry) > 0.01) {
                     std::string dKey = getStateKey(dstate);
                     if (!dKey.empty() && sampleStateToIndex_.find(dKey) == sampleStateToIndex_.end()) {
                         // Track the actual state that gets connected (might be intermediate node)
                         std::string samplerName = (selectedSamplerArm_ == SamplerArm::UNIFORM) ? "uniform" : 
                                                  (selectedSamplerArm_ == SamplerArm::CYLINDER_UP) ? "cylinder_up" : "cylinder_down";
                         trackSample(dstate, true, "planning", samplerName, 0.0, 0.0, nearestMotion->state, true);
                     }
                 }
             }
         }
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
 ompl::base::PlannerStatus ompl::geometric::MAB_RRT_DEBUG::solve(
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
         
         // DEBUG: Track start state as a connected sample (connected to itself/root)
         if (debugEnabled_) {
             base::State* originState = si_->allocState();
             si_->getStateSpace()->copyFromReals(originState, std::vector<double>(si_->getStateDimension(), 0.0));
             trackSample(st, true, "planning", "start", 0.0, 0.0, originState, true);
             si_->freeState(originState);
         }
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
         
         // DEBUG: Export path with sampler information
         if (debugEnabled_) {
             // Build base class path for export
             std::vector<MAB_RRT::Motion*> baseMpath;
             for (Motion* m : mpath) {
                 // Access the base class Motion through inheritance
                 // Since Motion in DEBUG inherits from base Motion, we can use reinterpret_cast
                 // or better: access the state and rebuild the path from PlannerData
                 baseMpath.push_back(reinterpret_cast<MAB_RRT::Motion*>(m));
             }
             exportPathWithSamplers("bugtrap_path.csv", baseMpath);
         }
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
 
 void ompl::geometric::MAB_RRT_DEBUG::getPlannerData(base::PlannerData &data) const
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
 
 
 // =============================================================================
 // DEBUG TRACKING METHODS
 // =============================================================================
 
 std::string ompl::geometric::MAB_RRT_DEBUG::getStateKey(const base::State* state) const
 {
     if (state == nullptr) return "";
     
     const auto* realState = state->as<base::RealVectorStateSpace::StateType>();
     if (realState == nullptr) return "";
     
     int stateDim = si_->getStateDimension();
     std::stringstream ss;
     ss << std::fixed << std::setprecision(6);
     
     if (stateDim == 2) {
         ss << realState->values[0] << "," << realState->values[1];
     } else if (stateDim >= 6) {
         ss << realState->values[3] << "," << realState->values[4];
     } else {
         return "";
     }
     
     return ss.str();
 }
 
 void ompl::geometric::MAB_RRT_DEBUG::trackSample(
     const base::State* state, 
     bool isValid, 
     const std::string& phase,
     const std::string& sampler,
     double reward,
     double radius,
     const base::State* nearestState,
     bool wasConnected)
 {
     if (!debugEnabled_ || state == nullptr) return;
     
     try {
         DebugSample sample;
         const auto* realState = state->as<base::RealVectorStateSpace::StateType>();
         if (realState == nullptr) return;
         
         int stateDim = si_->getStateDimension();
         if (stateDim == 2) {
             sample.x = realState->values[0];
             sample.y = realState->values[1];
         } else if (stateDim >= 6) {
             // For 6D, extract translation components
             sample.x = realState->values[3];
             sample.y = realState->values[4];
             // sample.z would be values[5] but we're only tracking 2D for visualization
         } else {
             return;  // Unsupported dimension
         }
         
         sample.isValid = isValid;
         sample.phase = phase;
         sample.sampler = sampler;
         sample.iteration = currentIteration_;
         sample.reward = reward;
         sample.radius = radius;
         sample.burnin_step = (phase == "burnin") ? currentBurninStep_ : -1;
         sample.wasConnected = wasConnected;
         
         // Extract nearest neighbor coordinates if provided
         if (nearestState != nullptr && wasConnected) {
             const auto* nearestRealState = nearestState->as<base::RealVectorStateSpace::StateType>();
             if (nearestRealState != nullptr) {
                 if (stateDim == 2) {
                     sample.nearest_x = nearestRealState->values[0];
                     sample.nearest_y = nearestRealState->values[1];
                 } else if (stateDim >= 6) {
                     sample.nearest_x = nearestRealState->values[3];
                     sample.nearest_y = nearestRealState->values[4];
                 }
             }
         } else {
             sample.nearest_x = 0.0;
             sample.nearest_y = 0.0;
         }
         
         size_t index = debugSamples_.size();
         debugSamples_.push_back(sample);
         
         // Store mapping for planning phase samples to enable connection tracking
         if (phase == "planning") {
             std::string key = getStateKey(state);
             if (!key.empty()) {
                 sampleStateToIndex_[key] = index;
             }
         }
         
         // Debug output for burn-in samples (first few only to avoid spam)
         if (phase == "burnin" && debugSamples_.size() <= 5) {
             OMPL_INFORM("DEBUG: Tracked burn-in sample #%zu: sampler=%s, valid=%d, radius=%.10f", 
                        debugSamples_.size(), sampler.c_str(), isValid ? 1 : 0, radius);
         }
     } catch (...) {
         // Silently ignore tracking errors to avoid crashes
     }
 }
 
 void ompl::geometric::MAB_RRT_DEBUG::exportPathWithSamplers(
     const std::string& filename, const std::vector<MAB_RRT::Motion*>& mpath) const
 {
     // Always save to demos/disassembly/ directory (absolute path from config)
     std::string pathFile = outputDirectory_ + filename;
     
     std::ofstream outFile(pathFile);
     if (!outFile.is_open()) {
         OMPL_WARN("Could not open file for writing: %s", pathFile.c_str());
         return;
     }
         
         outFile << std::fixed << std::setprecision(6);
         outFile << "x,y,sampler\n";
         
         // Write path in reverse order (from start to goal)
         for (int i = static_cast<int>(mpath.size()) - 1; i >= 0; --i)
         {
             MAB_RRT::Motion* motion = mpath[i];
             const auto* realState = motion->state->as<base::RealVectorStateSpace::StateType>();
             if (realState == nullptr) continue;
             
             int stateDim = si_->getStateDimension();
             double x, y;
             if (stateDim == 2) {
                 x = realState->values[0];
                 y = realState->values[1];
             } else if (stateDim >= 6) {
                 x = realState->values[3];
                 y = realState->values[4];
             } else {
                 continue;
             }
             
             // Determine sampler by looking up in debug samples
             // This gives us the exact sampler (uniform, cylinder_up, cylinder_down) used
             std::string sampler = "unknown";
             
             // First try to find match in debug samples (check all samples, not just connected ones)
             // Use reverse iteration to get the most recent match
             bool found = false;
             double bestDist = 1e10;
             for (auto it = debugSamples_.rbegin(); it != debugSamples_.rend(); ++it) {
                 if (it->phase == "planning") {
                     double dx = std::abs(it->x - x);
                     double dy = std::abs(it->y - y);
                     double dist = std::sqrt(dx*dx + dy*dy);
                     // Use larger tolerance to account for distance limiting and state interpolation
                     // Take the closest match within tolerance
                     if (dist < 2.0 && dist < bestDist) {
                         sampler = it->sampler;
                         bestDist = dist;
                         found = true;
                     }
                 }
             }
             
             // If still not found, try to match with any sample (including burn-in)
             if (!found) {
                 for (auto it = debugSamples_.rbegin(); it != debugSamples_.rend(); ++it) {
                     double dx = std::abs(it->x - x);
                     double dy = std::abs(it->y - y);
                     double dist = std::sqrt(dx*dx + dy*dy);
                     if (dist < 2.0 && dist < bestDist) {
                         sampler = it->sampler;
                         bestDist = dist;
                         found = true;
                     }
                 }
             }
             
             // If still not found, fall back to MotionOrigin (but this loses UP/DOWN distinction)
             if (!found) {
                 auto origin = motion->bornFrom;
                 if (origin == MAB_RRT::MotionOrigin::UNIFORM) {
                     sampler = "uniform";
                 } else if (origin == MAB_RRT::MotionOrigin::CYLINDER) {
                     // Can't determine UP/DOWN from MotionOrigin, default to cylinder
                     sampler = "cylinder";
                 }
             }
             
             // Special case: first state is always "start"
             if (i == static_cast<int>(mpath.size()) - 1) {
                 sampler = "start";
             }
             
             outFile << x << "," << y << "," << sampler << "\n";
         }
         
         outFile.close();
         OMPL_INFORM("Path with samplers exported to: %s", pathFile.c_str());
 }
 
 void ompl::geometric::MAB_RRT_DEBUG::exportSampleData(const std::string& filename) const
 {
     // Always save to demos/disassembly/ directory (absolute path from config)
     std::string sampleFile = outputDirectory_ + filename;
     
     std::ofstream outFile(sampleFile);
     if (!outFile.is_open()) {
         OMPL_WARN("Could not open file for writing: %s", sampleFile.c_str());
         return;
     }
     
     outFile << std::fixed << std::setprecision(6);
     outFile << "x,y,is_valid,phase,sampler,iteration,reward,radius,burnin_step,was_connected,nearest_x,nearest_y\n";
     
     for (const auto& sample : debugSamples_) {
         outFile << sample.x << "," << sample.y << ","
                 << (sample.isValid ? "1" : "0") << ","
                 << sample.phase << ","
                 << sample.sampler << ","
                 << sample.iteration << ","
                 << sample.reward << ","
                 << sample.radius << ","
                 << sample.burnin_step << ","
                 << (sample.wasConnected ? "1" : "0") << ","
                 << sample.nearest_x << ","
                 << sample.nearest_y << "\n";
     }
     
     outFile.close();
     OMPL_INFORM("Exported %zu debug samples to %s", debugSamples_.size(), sampleFile.c_str());
     
     // Debug: Count samples by phase
     size_t burnin_count = 0, planning_count = 0;
     for (const auto& s : debugSamples_) {
         if (s.phase == "burnin") burnin_count++;
         else if (s.phase == "planning") planning_count++;
     }
     OMPL_INFORM("DEBUG: Sample breakdown - burnin: %zu, planning: %zu", burnin_count, planning_count);
 }