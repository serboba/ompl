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

#ifndef OMPL_BASE_SPACES_FRAGMENTED_STATE_SPACE_H
#define OMPL_BASE_SPACES_FRAGMENTED_STATE_SPACE_H

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {
        /** \brief A RealVectorStateSpace whose dimensions are partitioned into independent
            groups ("fragments"/"factors"), one per movable object. Distance and interpolation
            operate group-by-group so that motions traverse a single group at a time, which lets
            LARRT reason about actions (per-group changes) rather than raw Euclidean distance. */
        class FragmentedStateSpace : public ompl::base::RealVectorStateSpace
        {
        public:

           FragmentedStateSpace(std::vector<std::vector<int>> groups_) :
                grouped_indices(groups_)
            {}

            virtual ~FragmentedStateSpace() = default;



            double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

            void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t,
                             ompl::base::State *state) const override;

        private:
            double distance(double v1, double v2) const;

            int findIndex(std::vector<double> &distances, double t) const;

            std::vector<double> getDistances(const FragmentedStateSpace::StateType *const rfrom,
                                             const FragmentedStateSpace::StateType *const rto) const;


        protected:
            std::vector<std::vector<int>> grouped_indices;
        };
    }
}


#endif  // OMPL_BASE_SPACES_FRAGMENTED_STATE_SPACE_H