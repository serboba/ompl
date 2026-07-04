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

// Regression tests for the LA-RRT PathDefragmenter. Each test feeds a hand-crafted
// factored path (consecutive states differ in a single group) to
// doPathDefragComplete and asserts the three invariants the defragmenter must keep:
//   * it never crashes,
//   * the returned path still starts at the start and ends at the goal,
//   * every motion is collision-free,
// and that it does not increase the action count. The three scenarios reproduce the
// historical failure modes: a single-fragment crash, goal-tail truncation, and a
// straightened detour reintroducing a collision.

#define BOOST_TEST_MODULE "LARRTPathDefragmenter"
#include <boost/test/unit_test.hpp>

#include <vector>
#include <cmath>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/FactoredStateSpace.h"
#include "ompl/geometric/PathDefragmenter.h"

using namespace ompl;

namespace
{
struct Rect
{
    double xmin, xmax, ymin, ymax;
    bool contains(double x, double y) const { return x > xmin && x < xmax && y > ymin && y < ymax; }
};

// A central box obstacle, used by the collision scenario; empty for the others.
std::vector<Rect> g_obstacles;

bool pointFree(double x, double y)
{
    for (const auto &r : g_obstacles)
        if (r.contains(x, y))
            return false;
    return true;
}

base::SpaceInformationPtr makeSI(int dim, const std::vector<std::vector<int>> &groups,
                                 const base::StateValidityCheckerFn &vc)
{
    auto space = std::make_shared<base::FactoredStateSpace>(groups);
    for (int d = 0; d < dim; ++d)
        space->addDimension(0.0, 10.0);
    auto si = std::make_shared<base::SpaceInformation>(space);
    si->setStateValidityChecker(vc);
    si->setStateValidityCheckingResolution(0.002);
    si->setup();
    return si;
}

// A plain RealVectorStateSpace (NOT a FactoredStateSpace) -- used to prove the
// defragmenter's factoring is backend-agnostic.
base::SpaceInformationPtr makePlainSI(int dim, double low, double high,
                                      const base::StateValidityCheckerFn &vc)
{
    auto space = std::make_shared<base::RealVectorStateSpace>(dim);
    base::RealVectorBounds b(dim);
    b.setLow(low);
    b.setHigh(high);
    space->setBounds(b);
    auto si = std::make_shared<base::SpaceInformation>(space);
    si->setStateValidityChecker(vc);
    si->setStateValidityCheckingResolution(0.001);
    si->setup();
    return si;
}

std::vector<base::State *> buildPath(const base::SpaceInformationPtr &si,
                                     const std::vector<std::vector<double>> &pts)
{
    std::vector<base::State *> p;
    for (const auto &v : pts)
    {
        base::State *s = si->allocState();
        si->getStateSpace()->copyFromReals(s, v);
        p.push_back(s);
    }
    return p;
}

int changedGroup(const std::vector<std::vector<int>> &groups, const std::vector<double> &a,
                 const std::vector<double> &b)
{
    for (size_t g = 0; g < groups.size(); ++g)
        for (int idx : groups[g])
            if (std::abs(a[idx] - b[idx]) > 1e-10)
                return static_cast<int>(g);
    return -1;
}

int actionCount(const base::SpaceInformationPtr &si, const std::vector<std::vector<int>> &groups,
                const std::vector<base::State *> &path)
{
    if (path.size() < 2)
        return static_cast<int>(path.size());
    std::vector<std::vector<double>> v(path.size());
    for (size_t i = 0; i < path.size(); ++i)
        si->getStateSpace()->copyToReals(v[i], path[i]);
    int prev = changedGroup(groups, v[0], v[1]);
    int count = 1;
    for (size_t i = 1; i + 1 < v.size(); ++i)
    {
        int cur = changedGroup(groups, v[i], v[i + 1]);
        if (cur != prev) { ++count; prev = cur; }
    }
    return count;
}

bool collisionFree(const base::SpaceInformationPtr &si, const std::vector<base::State *> &path)
{
    for (size_t i = 0; i + 1 < path.size(); ++i)
        if (!si->checkMotion(path[i], path[i + 1]))
            return false;
    return true;
}

bool atConfig(const base::SpaceInformationPtr &si, const base::State *s, const std::vector<double> &exp)
{
    std::vector<double> v;
    si->getStateSpace()->copyToReals(v, s);
    for (size_t d = 0; d < exp.size(); ++d)
        if (std::abs(v[d] - exp[d]) > 1e-3)
            return false;
    return true;
}

void freePath(const base::SpaceInformationPtr &si, std::vector<base::State *> &p)
{
    for (auto *s : p)
        si->freeState(s);
    p.clear();
}

// Run doPathDefragComplete on a crafted path and assert all invariants.
void checkDefrag(const base::SpaceInformationPtr &si, const std::vector<std::vector<int>> &groups,
                 const std::vector<std::vector<double>> &pts, const std::vector<double> &startExp,
                 const std::vector<double> &goalExp)
{
    auto path = buildPath(si, pts);
    int rawActions = actionCount(si, groups, path);
    BOOST_REQUIRE(collisionFree(si, path));   // the crafted input must itself be valid

    geometric::PathDefragmenter pd(si, groups, 0);
    BOOST_REQUIRE_NO_THROW(pd.doPathDefragComplete(path));   // (1) never crashes

    BOOST_REQUIRE_GE(path.size(), 2u);
    BOOST_CHECK(atConfig(si, path.front(), startExp));        // (2) keeps the start
    BOOST_CHECK(atConfig(si, path.back(), goalExp));          //     keeps the goal
    BOOST_CHECK(collisionFree(si, path));                     // (3) stays collision-free
    BOOST_CHECK_LE(actionCount(si, groups, path), rawActions); // never worse on actions

    freePath(si, path);
}
}  // namespace

// Historical failure mode #1: a path whose transitions never change group is a single
// fragment; getFragmentIDs used to dereference an empty vector and crash.
BOOST_AUTO_TEST_CASE(SingleFragmentDoesNotCrash)
{
    g_obstacles.clear();
    std::vector<std::vector<int>> groups = {{0}, {1}};
    auto si = makeSI(2, groups, [](const base::State *) { return true; });
    // object 0 only: one fragment, several states.
    checkDefrag(si, groups, {{5, 5}, {6, 5}, {7, 5}, {9, 5}}, {5, 5}, {9, 5});
}

// Historical failure mode #2: cutOffIfGoalReached used to drop every trailing move that
// did not touch goal index 0, truncating the second object short of its goal (and then
// crashing on the collapsed single-fragment remainder).
BOOST_AUTO_TEST_CASE(GoalTailNotTruncated)
{
    g_obstacles.clear();
    std::vector<std::vector<int>> groups = {{0}, {1}};
    auto si = makeSI(2, groups, [](const base::State *) { return true; });
    // alternating fragments; the LAST action is object 1 reaching its goal y = 1.
    checkDefrag(si, groups, {{5, 5}, {7, 5}, {7, 3}, {9, 3}, {9, 1}}, {5, 5}, {9, 1});
}

// Historical failure mode #3: simplifyActionIntervals straightened a same-object detour
// (object A's L-shaped path around a box) into a straight line through the obstacle.
BOOST_AUTO_TEST_CASE(DetourNotStraightenedThroughObstacle)
{
    g_obstacles = {{4.0, 6.0, 4.0, 6.0}};   // central box on the straight A=(2,5)->(8,5) line
    std::vector<std::vector<int>> groups = {{0, 1}, {2, 3}};
    auto si = makeSI(4, groups, [](const base::State *state) {
        const auto *s = state->as<base::FactoredStateSpace::StateType>();
        return pointFree(s->values[0], s->values[1]) && pointFree(s->values[2], s->values[3]);
    });
    // object A detours low around the box (a single A-fragment of 3 edges), then object B moves.
    checkDefrag(si, groups,
                {{2, 5, 0, 0}, {2, 2, 0, 0}, {8, 2, 0, 0}, {8, 5, 0, 0}, {8, 5, 1, 0}},
                {2, 5, 0, 0}, {8, 5, 1, 0});
    g_obstacles.clear();
}

// checkRepairPath must isolate a multi-group edge collision-free on a PLAIN
// RealVectorStateSpace (no FactoredStateSpace, no group-by-group interpolate).
// Here moving group 0 first (x then y) drives through an obstacle at the (5,0)
// corner, while moving group 1 first (y then x) is clear -- so isolation must pick
// the collision-free ordering rather than blindly splitting x-first.
BOOST_AUTO_TEST_CASE(IsolationPicksCollisionFreeOrderOnPlainSpace)
{
    g_obstacles = {{4.0, 6.0, -1.0, 1.0}};   // blocks the x-first corner (5,0)
    auto si = makePlainSI(2, -2.0, 12.0, [](const base::State *state) {
        const auto *s = state->as<base::RealVectorStateSpace::StateType>();
        return pointFree(s->values[0], s->values[1]);
    });
    std::vector<std::vector<int>> groups = {{0}, {1}};

    // one edge (0,0) -> (5,5) changing BOTH groups (motion cost 2)
    auto path = buildPath(si, {{0, 0}, {5, 5}});
    BOOST_REQUIRE(si->checkMotion(path.front(), path.back()));   // combined diagonal is free

    geometric::PathDefragmenter pd(si, groups, 0);
    BOOST_REQUIRE_NO_THROW(pd.checkRepairPath(path));

    // isolated into 3 states, every single-group step collision-free, endpoints kept
    BOOST_REQUIRE_EQUAL(path.size(), 3u);
    BOOST_CHECK(collisionFree(si, path));
    BOOST_CHECK(atConfig(si, path.front(), {0, 0}));
    BOOST_CHECK(atConfig(si, path.back(), {5, 5}));
    // the y-first waypoint is (0,5); the colliding x-first waypoint (5,0) must NOT appear
    BOOST_CHECK(atConfig(si, path.at(1), {0, 5}));

    freePath(si, path);
    g_obstacles.clear();
}
