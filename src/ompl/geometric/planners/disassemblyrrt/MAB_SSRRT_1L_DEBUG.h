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
 * @file MAB_SSRRT_1L.h
 * @brief Multi-Armed Bandit Sphere-Sampled RRT with Single-Layer MAB (1L)
 * 
 * =============================================================================
 * ALGORITHM OVERVIEW
 * =============================================================================
 * 
 * MAB-SSRRT-1L is an RRT-based motion planner that uses Multi-Armed Bandit (MAB)
 * reinforcement learning to adaptively select between different sampling strategies.
 * 
 * The "1L" (Single Layer) variant uses a FLAT 3-arm MAB:
 *   - Arm 0: UNIFORM sampling (random samples across entire state space)
 *   - Arm 1: CYLINDER_UP sampling (directed samples along +Z axis)
 *   - Arm 2: CYLINDER_DOWN sampling (directed samples along -Z axis)
 * 
 * This is in contrast to the 2L (Two Layer) variant which uses nested MABs:
 *   - Layer 1: UNIFORM vs CYLINDER selection
 *   - Layer 2: UP vs DOWN direction (only when CYLINDER is selected)
 * 
 * =============================================================================
 * KEY CONCEPTS
 * =============================================================================
 * 
 * 1. ADAPTIVE SPHERE SAMPLING:
 *    During burn-in, the planner discovers constraint directions by probing
 *    samples at varying radii. Valid samples are collected to seed the 
 *    cylinder sampler with known free-motion directions.
 * 
 * 2. CYLINDER SAMPLING:
 *    Uses PCA on valid samples to identify the principal axis of motion.
 *    Samples are generated along a cylinder aligned with this axis, enabling
 *    efficient exploration in constrained environments (e.g., disassembly).
 * 
 * 3. MAB REWARD SYSTEM:
 *    - Valid samples receive positive rewards
 *    - Invalid samples receive low/zero rewards
 *    - UCB (Upper Confidence Bound) balances exploration vs exploitation
 * 
 * =============================================================================
 * USAGE
 * =============================================================================
 * 
 * @code
 * auto planner = std::make_shared<og::MAB_SSRRT_1L>(si, "/path/to/config.yaml");
 * planner->setProblemDefinition(pdef);
 * planner->setup();
 * auto status = planner->solve(timeout);
 * @endcode
 * 
 * =============================================================================
 */

#ifndef OMPL_GEOMETRIC_PLANNERS_DISASSEMBLYRRT_MAB_SSRRT_1L_DEBUG_
#define OMPL_GEOMETRIC_PLANNERS_DISASSEMBLYRRT_MAB_SSRRT_1L_DEBUG_

#include "ompl/geometric/planners/disassemblyrrt/MAB_SSRRT_1L.h"
#include <vector>
#include <string>
#include <map>

namespace ompl {
    namespace geometric {

        /**
         * @class MAB_SSRRT_1L_DEBUG
         * @brief DEBUG VERSION: Multi-Armed Bandit Sphere-Sampled RRT with Single-Layer MAB
         * 
         * This is a debug version with extensive logging and sample tracking.
         * Use MAB_SSRRT_1L for production code.
         * 
         * This planner extends RRT with:
         * - Adaptive sphere sampling for constraint discovery
         * - Cylinder sampling along principal constraint axis
         * - Single-layer MAB for arm selection (3 arms: UNIFORM, CYL_UP, CYL_DOWN)
         * 
         * Particularly effective for:
         * - Disassembly planning where objects have limited motion directions
         * - Narrow passage problems with known constraint structure
         * - Environments where directed exploration outperforms uniform
         */
        class MAB_SSRRT_1L_DEBUG : public MAB_SSRRT_1L {
        public:
            /** @brief Constructor with YAML configuration file path */
            MAB_SSRRT_1L_DEBUG(const base::SpaceInformationPtr &si, const std::string &yamlFilePath);

            ~MAB_SSRRT_1L_DEBUG() override;
            
            /** @brief Export debug sample data to CSV file */
            void exportSampleData(const std::string& filename) const;
            
            /** @brief Export path with sampler information to CSV */
            void exportPathWithSamplers(const std::string& filename, const std::vector<MAB_SSRRT_1L::Motion*>& mpath) const;
            
            // Override base class methods to add debug tracking
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void getPlannerData(base::PlannerData &data) const override;
            void clear() override;

            /** @brief Set the goal bias (probability of sampling goal) */
            void setGoalBias(double goalBias) { goalBias_ = goalBias; }

            /** @brief Get current goal bias */
            double getGoalBias() const { return goalBias_; }

            /** @brief Set maximum extension distance per step */
            void setRange(double distance) { maxDistance_ = distance; }

            /** @brief Get maximum extension distance */
            double getRange() const { return maxDistance_; }

            template<template<typename T> class NN>
            void setNearestNeighbors() {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

            // ====================================================================
            // ENUMERATIONS
            // ====================================================================

            /**
             * @brief Sampling arm selection for flat MAB (3 arms)
             * 
             * In the 1L (single-layer) variant, all three options are at the same level:
             * - UNIFORM: Random sampling across entire state space
             * - CYLINDER_UP: Directed sampling along +Z axis (primary motion direction)
             * - CYLINDER_DOWN: Directed sampling along -Z axis (opposite direction)
             */
            enum class SamplerArm
            {
                UNIFORM,        ///< Sample uniformly in entire state space
                CYLINDER_UP,    ///< Sample along positive cylinder axis
                CYLINDER_DOWN   ///< Sample along negative cylinder axis
            };

            /**
             * @brief Simplified origin tracking for Motion nodes
             * 
             * Used to track which sampling strategy created each tree node.
             * This enables the goal connection heuristic to prefer UNIFORM nodes.
             */
            enum class MotionOrigin
            {
                UNIFORM,   ///< Node created by uniform sampling
                CYLINDER   ///< Node created by cylinder sampling (either direction)
            };

            /**
             * @brief MAB path tracking for reward updates
             * 
             * Tracks which specific arm was used, enabling proper reward attribution.
             */
            enum class MABPath
            {
                UNIFORM,       ///< Arm 0: Uniform sampling
                CYLINDER_UP,   ///< Arm 1: Cylinder +Z
                CYLINDER_DOWN, ///< Arm 2: Cylinder -Z
                NONE           ///< No arm selected (goal sample)
            };

        protected:
            // ====================================================================
            // CONSTANTS
            // ====================================================================
            
            /** @brief Number of arms in the flat MAB (UNIFORM, CYL_UP, CYL_DOWN) */
            static inline constexpr int kNumMABArms = 3;
            
            /** @brief Default state space dimension (6D: RPY + XYZ) */
            static inline constexpr int kDefaultStateDimension = 6;
            
            // ====================================================================
            // INNER CLASSES
            // ====================================================================
            
            /**
             * @class Motion
             * @brief Represents a node in the RRT search tree
             * 
             * Each Motion stores:
             * - The sampled state
             * - Parent pointer for path reconstruction
             * - Origin tracking for goal connection heuristics
             * - Goal connection failure tracking to avoid repeated failed attempts
             */
            class Motion {
            public:
                Motion() = default;
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()) {}
                ~Motion() = default;

                base::State *state{nullptr};    ///< The state in this node
                Motion *parent{nullptr};        ///< Parent node in the tree
                
                // Goal connection tracking
                unsigned int goalConnectionFailures{0};  ///< Number of failed goal connections
                bool goalConnectionExhausted{false};     ///< True if should not try goal from here
                
                // Track which sampler arm created this node
                MotionOrigin bornFrom{MotionOrigin::UNIFORM};
            };

            /**
             * @struct SamplingArm
             * @brief Configuration for a sampling strategy
             * 
             * Each arm has:
             * - sphere: The adaptive sphere sampler (nullptr for uniform)
             * - axesIndices: Mask indicating which dimensions to sample
             */
            struct SamplingArm {
                std::unique_ptr<sampling::AdaptiveSphereSampler> sphere;
                std::vector<int> axesIndices;
                
                SamplingArm() = default;
                SamplingArm(std::unique_ptr<sampling::AdaptiveSphereSampler> s, std::vector<int> axes)
                    : sphere(std::move(s)), axesIndices(std::move(axes)) {}
            };

            // ====================================================================
            // INITIALIZATION & CLEANUP
            // ====================================================================
            
            /** @brief Load configuration parameters from YAML file */
            void loadYAMLConfig(const std::string& yamlFilePath);
            
            /** @brief Free all allocated memory */
            void freeMemory();
            
            /** @brief Initialize sampling arms (cylinder + uniform) */
            void initializeArms();
            
            /** @brief Create the cylinder sampling arm (XYZ only) */
            void createCylinderSamplingArm();
            
            /** @brief Create the uniform sampling arm (all dimensions) */
            void createUniformSamplingArm();
            
            /** @brief Distance function for nearest neighbor queries */
            double distanceFunction(const Motion *a, const Motion *b) const {
                return si_->distance(a->state, b->state);
            }

            // ====================================================================
            // ADAPTIVE SPHERE SAMPLING (BURN-IN PHASE)
            // ====================================================================
            
            /** @brief Main entry point for adaptive sphere setup */
            void setupAdaptiveSphereSampling();
            
            /** @brief Test if uniform sampling is sufficient (early exit) */
            bool performInitialUniformCheck();
            
            /** @brief Iteratively adjust radius to find optimal sampling sphere */
            void performAdaptiveBurnin();
            
            /** @brief Check for early exit on consecutive full validity */
            bool shouldExitEarlyOnFullValidity(double validity_rate, int& consecutive_count,
                                              std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere);
            
            /** @brief Check if validity rate is in acceptable range */
            bool isValidityRateInTargetRange(double validity_rate) const;
            
            /** @brief Shrink or grow radius based on validity feedback */
            double adjustRadiusBasedOnValidity(std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere,
                                              double current_radius, double validity_rate);
            
            /** @brief Finalize burn-in and set best radius */
            void finalizeBurnin(std::unique_ptr<sampling::AdaptiveSphereSampler>& sphere, double final_radius);
            
            /** @brief Compute sample validity rate at current radius */
            double computeValidityRate();

            // ====================================================================
            // SAMPLING METHODS
            // ====================================================================
            
            /** @brief Main sampling entry point - generates and validates samples */
            bool getSample(base::State *sample_state, Motion* tempMotion, double extensionFactor, 
                          base::State* originState, const base::PlannerTerminationCondition& ptc, 
                          Motion** nearestMotionOut = nullptr, bool* isGoalSampleOut = nullptr);
            
            /** @brief Verify sampling prerequisites are met */
            bool checkSamplingPrerequisites(base::State* sample_state, bool* isGoalSampleOut);
            
            /** @brief Select which arm to use via MAB */
            SamplerArm selectSamplingArm();
            
            /** @brief Check if should sample from goal region */
            bool shouldSampleGoal(bool* isGoalSampleOut, Motion** nearestMotionOut, base::State* sample_state);
            
            /** @brief Generate sample and validate motion */
            template <typename ValidateFunc>
            bool generateAndValidateSample(base::State* sample_state, ValidateFunc& validateState,
                                          double radius, MABPath& selectedMABPath);
            
            /** @brief Sample from cylinder (direction already selected) */
            template <typename ValidateFunc>
            bool sampleFromCylinder(base::State* sample_state, ValidateFunc& validateState,
                                   double radius, MABPath& selectedMABPath);
            
            /** @brief Fallback to uniform if cylinder fails */
            template <typename ValidateFunc>
            bool fallbackToUniform(base::State* sample_state, ValidateFunc& validateState,
                                  MABPath& selectedMABPath);
                                  
            /** @brief Update MAB rewards and statistics */
            void updateSamplingStatistics(MABPath selectedMABPath, bool isSampleValid,
                                         base::State* sample_state, base::State* originState);
            
            /** @brief Sample uniformly across all dimensions */
            void sampleUniformHypothesis(base::State *state);
            
            /** @brief Copy sample vector into OMPL state */
            void copySampleVectorIntoState(base::State *sample_state, SamplingArm &arm,
                                           std::vector<double> vec);
            
            /** @brief Update MAB rewards based on sample validity */
            void updateRewards(bool isValid, MABPath selectedMABPath);

            // ====================================================================
            // SOLVE HELPER METHODS
            // ====================================================================
            
            /** @brief Find best node for goal connection (prefers UNIFORM nodes) */
            Motion* findBestMotionForGoal(base::State* goalState, Motion* queryMotion);
            
            /** @brief Add new motion to the tree */
            Motion* addMotionToTree(base::State* state, Motion* parent, MotionOrigin origin);
            
            /** @brief Check goal satisfaction and update solution tracking */
            bool checkAndUpdateSolution(Motion* motion, base::Goal* goal,
                                       Motion*& solution, Motion*& approxsol, double& approxdif);
            
            /** @brief Limit extension distance (for uniform samples) */
            base::State* applyDistanceLimit(Motion* from, base::State* to, 
                                           base::State* buffer, double distance);
            
            /** @brief Handle goal-biased sample path */
            bool handleGoalSamplePath(base::State* rstate, base::State* xstate,
                                     Motion* rmotion, base::State* originState, base::Goal* goal,
                                     Motion*& solution, Motion*& approxsol, double& approxdif);
            
            /** @brief Handle normal (non-goal) sample path */
            bool handleNormalSamplePath(base::State* rstate, base::State* xstate,
                                       Motion* nearestMotion, base::Goal* goal,
                                       Motion*& solution, Motion*& approxsol, double& approxdif);

            // ====================================================================
            // MEMBER VARIABLES
            // ====================================================================
            
            base::StateSamplerPtr sampler_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            double goalBias_{.05};
            double maxDistance_{0.};
            bool addIntermediateStates_{true};

            RNG rng_;
            Motion *lastGoalMotion_{nullptr};

            std::vector<std::unique_ptr<SamplingArm>> samplingArms_;
            
            /** @brief Single flat MAB with 3 arms (replaces nested MABs from 2L version) */
            std::unique_ptr<MAB_SlidingWindowUCB> mab_sampler_;

            // ====================================================================
            // CONFIGURATION (loaded from YAML)
            // ====================================================================
            
            double uniformGoalBias_{0.05};   ///< Goal bias when uniform arm selected
            double sphereGoalBias_{0.05};    ///< Goal bias when cylinder arm selected
            
            int mabWindowSize_{64};  ///< Sliding window size for MAB
            
            // Adaptive sampling parameters
            int adaptiveQuasirandomSampleSize_{128};
            double adaptiveStartRadius_{0.5};
            double adaptiveMinRadius_{0.01};
            double adaptiveShrinkStep_{0.1};
            double adaptiveGrowStep_{0.1};
            double adaptiveMinExpectedValidityRate_{0.3};
            double adaptiveMaxExpectedValidityRate_{0.7};
            int adaptiveBurninMaxSteps_{10};

            // Initial uniform check
            double initialFreeSamplingProbability_{0.5};
            int initialNumberOfUniformSampleTrials_{10};

            double sphereExtensionEps_{0.0};
            
            // MAB rewards
            double uniformSamplerInvalidReward_{0.0};
            double sphereSamplerInvalidReward_{0.0};
            double uniformSamplerFixedValidReward_{1.0};
            double sphereSamplerFixedValidReward_{1.0};

            // Cylinder configuration
            double cylinderRadiusOffsetMultiplier_{0.1};
            double fibonacciJitterRadius_{0.39269908169872414};
            double cylinderSamplingRadiusMultiplier_{1.5};
            double pcaFilterTopPercent_{1.0};

            bool enableDynamicCylinderPCA_{false};

            // Burn-in early exit
            bool burninEarlyExitEnabled_{false};
            unsigned int burninMinValidSamplesForContinue_{3};
            bool burninEarlyExitTriggered_{false};
            
            bool earlyExitOnConsecutiveFullValidity_{false};
            int consecutiveFullValidityMaxIterations_{5};

            int forcedUniformAfterCylinderValidStreak_{0};

            // ====================================================================
            // RUNTIME STATE
            // ====================================================================
            
            unsigned int currentIteration_{0};
            int cylinderValidStreak_{0};
            SamplerArm selectedSamplerArm_{SamplerArm::UNIFORM};
            std::shared_ptr<base::RealVectorStateSampler> uniformRealVecSampler_;
            
            // ====================================================================
            // DEBUG TRACKING
            // ====================================================================
            
            /** @brief Structure to track a sample for debugging */
            struct DebugSample {
                double x, y;  // For 2D, or x,y,z for 3D
                bool isValid;
                std::string phase;  // "burnin" or "planning"
                std::string sampler; // "uniform", "cylinder_up", "cylinder_down"
                int iteration;
                double reward;
                double radius;  // For sphere samples
                // Connection/edge information
                bool wasConnected;  // Whether this sample was connected to the tree
                double nearest_x, nearest_y;  // Coordinates of nearest neighbor (if connected)
            };
            
            std::vector<DebugSample> debugSamples_;  ///< All tracked samples
            bool debugEnabled_{true};  ///< Enable debug tracking
            
            // Map to track which sample state corresponds to which debug sample index
            // Key: state pointer (as string representation), Value: index in debugSamples_
            std::map<std::string, size_t> sampleStateToIndex_;  ///< Map state to sample index for connection tracking
            
            /** @brief Track a sample for debugging */
            void trackSample(const base::State* state, bool isValid, 
                           const std::string& phase, const std::string& sampler,
                           double reward = 0.0, double radius = 0.0,
                           const base::State* nearestState = nullptr, bool wasConnected = false);
            
            /** @brief Get state key for mapping */
            std::string getStateKey(const base::State* state) const;
        };
    }
}

#endif

