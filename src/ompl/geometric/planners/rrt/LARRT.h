/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2022, Servet Bora Bayraktar
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
*   * Neither the name of the copyright holder nor the names of its
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

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_LARRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_LARRT_

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/PathDefragmenter.h>
#include <ompl/base/objectives/MinimalActionsObjective.h>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gLARRT
           @par Short description
           LA-RRT (Low-Actions RRT) is a bidirectional RRT for rearrangement-style problems posed
           over a fragmented (factored) state space, where the full state is partitioned into
           independent groups of indices (one group per movable object / degree of freedom). Rather
           than minimizing geometric path length, LA-RRT minimizes the number of actions, i.e. the
           number of mode switches between groups along the path (see MinimalActionsObjective). As
           the start and goal trees are grown, motions that would change several groups at once are
           "isolated" into single-group steps, and discovered solutions are post-processed by the
           PathDefragmenter, which reorders and merges same-group path segments to further reduce
           the action count while remaining collision-free.
        */

        /** \brief Low-Actions RRT (LARRT) */
        class LARRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            LARRT(const base::SpaceInformationPtr &si, std::vector<std::vector<int>> group_indices,
                  bool useIsolation = true, int goalIndex = 0);

            ~LARRT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Return the cost (number of actions) of the best solution path found so far. */
            ompl::base::Cost bestCost() const
            {
                return bestCost_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                tStart_ = std::make_shared<NN<Motion *>>();
                tGoal_ = std::make_shared<NN<Motion *>>();
                setup();
            }


            /** \brief Get the number of iterations the planner performed */
            unsigned int numIterations() const
            {
                return iterations_;
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion */
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                const base::State *root{nullptr};
                base::State *state{nullptr};
                Motion *parent{nullptr};
                base::Cost cost;
                int index_changed{0};
            };

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            using TreeData = std::shared_ptr<NearestNeighbors<Motion *>>;

            /** \brief Information attached to growing a tree of motions (used internally) */
            struct TreeGrowingInfo
            {
                base::State *xstate;
                Motion *xmotion;
                bool start;
            };


            /** \brief The state of the tree after an attempt to extend it */
            enum GrowState
            {
                /// no progress has been made
                TRAPPED,
                /// progress has been made towards the randomly sampled state
                ADVANCED,
                /// the randomly sampled state was reached
                REACHED
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Grow a tree towards a random state */
            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);


            void getMotionVectors(Motion * mot_,std::vector<Motion*> &vec);


            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief The start tree */
            TreeData tStart_;

            /** \brief The goal tree */
            TreeData tGoal_;

            /** \brief A flag that toggles between expanding the start tree (true) or goal tree (false). */
            bool startTree_{true};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool useIsolation_;
            int goalIndex_;
            /** \brief The random number generator */
            RNG rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State *, base::State *> connectionPoint_;

            /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
            double distanceBetweenTrees_;

            base::OptimizationObjectivePtr opt_;


            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};
            base::Cost incCost{0};

            std::vector<ompl::base::State*> bestPath;
            std::string bestCostProgressProperty() const;


            std::vector<std::vector<int>> group_indices;
            void createNewMotion(const base::State *st, Motion *premotion,
                                 ompl::geometric::LARRT::Motion *newmotion);

            bool validMotionCheck(const bool start, const base::State *from_, const base::State *to_);


            unsigned int iterations_{0u};


            int getChangedIndex(const base::State *from, const base::State *to);

            std::vector<int>
            getChangedGroups(const std::vector<double> &from_, const std::vector<double> &to_);


            int getCostPath(std::vector<base::State *> &states_);

            std::string numIterationsProperty() const
            {
                return std::to_string(numIterations());
            }


            void constructSolutionPath(PathGeometric &path, Motion *startMotion, Motion *goalMotion);


            void buildIsoStates( base::State *from_, const base::State *to, std::vector<int> &changed_index_groups,
                                std::vector<ompl::base::State *> &iso_);

            void freeStates(std::vector<ompl::base::State *> &states);

        };
    }
}

#endif  // OMPL_GEOMETRIC_PLANNERS_RRT_LARRT_