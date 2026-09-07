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

#define BOOST_TEST_MODULE "LARRT"
#include <boost/test/unit_test.hpp>
#include <cmath>
#include <vector>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/FactoredStateSpace.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/geometric/planners/rrt/LARRT.h"
#include "ompl/util/RandomNumbers.h"

using namespace ompl;

// Three single-index groups (objects). Object 0's straight run collides unless
// objects 1 and 2 are moved out of the way first, so a valid minimal-action plan
// must order the single-object moves correctly, exactly what LARRT and the
// PathDefragmenter produce. This mirrors the LowActionsPlanning demo.
static bool isStateValid(const base::State *state)
{
    const auto *s = state->as<base::FactoredStateSpace::StateType>();
    if (s->values[0] > 0.7 && s->values[0] < 1.1 && s->values[1] < 0.6)
        return false;
    if (s->values[0] > 1.3 && s->values[0] < 1.8 && s->values[2] > -0.5)
        return false;
    return true;
}

// Number of index groups that differ between two states of a 3-object space.
static int changedGroups(const base::State *a, const base::State *b)
{
    const auto *sa = a->as<base::FactoredStateSpace::StateType>();
    const auto *sb = b->as<base::FactoredStateSpace::StateType>();
    int n = 0;
    for (int i = 0; i < 3; ++i)
        if (std::fabs(sa->values[i] - sb->values[i]) > 1e-6)
            ++n;
    return n;
}

BOOST_AUTO_TEST_CASE(FactoredRearrangementIsValidGoalReachingAndMinimalActions)
{
    RNG::setSeed(1);  // deterministic

    std::vector<std::vector<int>> groups = {{0}, {1}, {2}};
    auto space = std::make_shared<base::FactoredStateSpace>(groups);
    space->addDimension(0.0, 2.0);
    space->addDimension(0.0, 2.0);
    space->addDimension(-2.0, 0.0);

    auto si = std::make_shared<base::SpaceInformation>(space);
    si->setStateValidityChecker([](const base::State *s) { return isStateValid(s); });

    geometric::SimpleSetup ss(si);
    base::ScopedState<> start(space), goal(space);
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 0.0;
    goal[0] = 2.0;
    goal[1] = 1.0;
    goal[2] = -1.0;
    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<geometric::LARRT>(si, groups));

    base::PlannerStatus solved = ss.solve(5.0);
    BOOST_REQUIRE(bool(solved));

    // NOTE: do NOT interpolate. LA-RRT's solution is a sequence of single-object
    // moves and each state is meaningful; interpolation would subdivide the segments.
    auto &path = ss.getSolutionPath();
    const std::size_t n = path.getStateCount();
    BOOST_REQUIRE(n >= 2);

    // (1) endpoints preserved, catches the goal-truncation bug
    BOOST_CHECK(space->equalStates(path.getState(0), start.get()));
    BOOST_CHECK(space->equalStates(path.getState(n - 1), goal.get()));

    // (2) every state collision-free, catches the collision-reintroduction bug
    for (std::size_t i = 0; i < n; ++i)
        BOOST_CHECK(isStateValid(path.getState(i)));

    // (3) each edge moves exactly ONE object (the factored/defrag invariant) and the
    //     total action count is minimal (each of the 3 objects moves once). This is the
    //     property the PathDefragmenter exists to achieve.
    int actions = 0;
    for (std::size_t i = 1; i < n; ++i)
    {
        int c = changedGroups(path.getState(i - 1), path.getState(i));
        BOOST_CHECK_EQUAL(c, 1);
        actions += c;
    }
    BOOST_CHECK_EQUAL(actions, 3);
}
