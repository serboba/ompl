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

#ifndef OMPL_BASE_OBJECTIVES_MINIMAL_ACTIONS_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_MINIMAL_ACTIONS_OBJECTIVE_

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective that counts actions: the cost of a motion is the
            number of index groups (objects) whose values change between the two states. Minimizing
            it yields paths that move as few objects as possible per step. */
        class MinimalActionsObjective : public OptimizationObjective
        {
        public:
            MinimalActionsObjective(const SpaceInformationPtr &si, std::vector<std::vector<int>> group_indices);

            Cost stateCost(const State *s) const override;


            Cost motionCost(const State *s1, const State *s2) const override;

            Cost motionCostHeuristic(const State *s1, const State *s2) const override;


            Cost identityCost() const override;

        protected:


            std::vector<std::vector<int>> group_indices_;
        };
    }
}

#endif  // OMPL_BASE_OBJECTIVES_MINIMAL_ACTIONS_OBJECTIVE_