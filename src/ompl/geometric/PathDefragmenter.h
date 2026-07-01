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

#ifndef OMPL_GEOMETRIC_PATH_DEFRAGMENTER_
#define OMPL_GEOMETRIC_PATH_DEFRAGMENTER_

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>

namespace ompl
{
    namespace geometric {

        /** \brief Post-processes an LARRT solution path expressed over a fragmented state space.
            It reorders and merges consecutive path segments that act on the same group (fragment)
            and skips redundant fragments, reducing the number of actions / mode-switches while
            keeping every motion collision-free. */
        class PathDefragmenter {
        public:


            struct FragmentIntervals
            {
                int start_index;
                int end_index;
                int id;

                FragmentIntervals(int start_, int end_, int id_) : start_index(start_), end_index(end_),id(id_){}

            };


            PathDefragmenter(base::SpaceInformationPtr si, std::vector<std::vector<int>> fragment_indices, int goalIndex = 0,
                             const base::OptimizationObjectivePtr &opt = nullptr);



            void startPathDefrag(std::vector<ompl::base::State *> &path);

            void checkRepairPath(std::vector<ompl::base::State *> &path_);

            void trySkipFragment(std::vector<ompl::base::State *> &mainPath);

            int pathDefrag(std::vector<ompl::base::State *> &mainPath);

            void simplifyActionIntervals(std::vector<ompl::base::State *> &mainPath);

            void cutOffIfGoalReached(std::vector<ompl::base::State *> &mainPath);


            void doPathDefragComplete(std::vector<ompl::base::State *> &path_);


        private:


            void
            getIntermediateState(const ompl::base::State *from, const ompl::base::State *to, ompl::base::State *state,
                                 int index_group);


            int getChangedIndex(const ompl::base::State *from, const ompl::base::State *to);


            void freeStates(std::vector<ompl::base::State *> &states);

            void freeStates(std::vector<std::pair<ompl::base::State *, int>> &states);

            bool reConnect(base::State *from, std::vector<std::pair<ompl::base::State *, int>> &queue_,
                           std::vector<ompl::base::State *> &rewireResult);

            void reConnect(base::State *from, std::vector<std::pair<ompl::base::State *, int>> &prio_,
                           std::vector<std::pair<ompl::base::State *, int>> &stack_,
                           std::vector<ompl::base::State *> &rewireResult);

            void
            findNextFragment(int start_index, int prev_index, std::vector<ompl::base::State *> &mainPath,
                             bool sameFragmentType,
                             std::vector<std::pair<ompl::base::State *, int>> &foundFragment);


            int getCostPath(std::vector<ompl::base::State *> &states_);

            void getFragment(int start_index, int end_index, std::vector<ompl::base::State *> &mainPath,
                             std::vector<std::pair<ompl::base::State *, int>> &fragment);


            void getFragmentIDs(std::vector<ompl::base::State *> &path, std::vector<FragmentIntervals> &fragmentIDs,
                                bool goalFragment);


            void
            isolateStates(const base::State *rfrom, const base::State *rto, std::vector<ompl::base::State *> &iso_);

            /** \brief Collision-checked isolation of a multi-group edge into single-group
                steps, trying group orderings; returns the states to insert (excluding the
                final one, which equals \e to), or false if no ordering is collision-free. */
            bool isolateChecked(const base::State *from, const base::State *to,
                                std::vector<base::State *> &out);

            std::vector<int> getChangedGroups(const std::vector<double> &from_, const std::vector<double> &to_);

            void
            buildIsoStates(const std::vector<double> &from_, const std::vector<double> &to_,
                           std::vector<int> &changed_index_groups,
                           std::vector<ompl::base::State *> &iso_);

            void skipFragments(std::vector<ompl::base::State *> &mainPath);

            /** \brief Deep-copy a path (allocates fresh states). */
            std::vector<ompl::base::State *> clonePath(const std::vector<ompl::base::State *> &path);

            /** \brief True if \e path starts at \e from, ends at \e to and every motion is collision-free. */
            bool isPathValid(const std::vector<ompl::base::State *> &path, const base::State *from,
                             const base::State *to);



        protected:

            ompl::base::SpaceInformationPtr si_;
            std::vector<std::vector<int>> fragment_indices;
            int goalIndex_;

            base::OptimizationObjectivePtr opt_;

            // Captured start/goal configurations of the path currently being defragmented;
            // valid only for the duration of doPathDefragComplete(). Used to keep every
            // optimisation step from truncating the goal or breaking start connectivity.
            base::State *startState_{nullptr};
            base::State *goalState_{nullptr};

        };

    }
}



#endif  // OMPL_GEOMETRIC_PATH_DEFRAGMENTER_