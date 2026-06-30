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

// LA-RRT 2D demo A -- "configuration space" explainer.
//
// Two 1-DOF objects, groups = {{0}, {1}}. The full state is (object0, object1),
// which is directly the 2D CONFIGURATION SPACE: x = position of object 0,
// y = position of object 1. Because the fragmented state space moves one group
// (one object) at a time, every solution segment is axis-aligned: a horizontal
// move changes object 0, a vertical move changes object 1.
//
// LA-RRT minimizes the NUMBER OF ACTIONS = the number of times control switches
// between the two objects (mode switches), NOT geometric length. After the path
// is found, the PathDefragmenter (run internally by LARRT::solve) reorders and
// merges same-object segments to drive the action count down. Visually, "minimal
// actions" reads as "fewest axis-aligned segments / direction changes".
//
// After solving, this demo writes a JSON description of the problem and the
// solution path so it can be plotted / animated by the scripts in viz/.

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/FragmentedStateSpace.h>
#include <ompl/geometric/planners/rrt/LARRT.h>
#include <ompl/base/objectives/MinimalActionsObjective.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Physical picture: two objects share a single 1-D track. Object 0's coordinate
// is x, object 1's is y, so the full state is the 2-D configuration space. The
// objects may not come within BAND_GAP of each other (a collision) -- EXCEPT in
// a "passing zone" near the high end of the track, where the track widens. We
// model that as: the diagonal band |x - y| < BAND_GAP is forbidden, but only
// while x + y < BAND_SUM (the low/narrow end). The upper-right corner is the
// passing zone where the two objects can swap which one is ahead.
//
// To rearrange from "object 1 ahead" (1,5) to "object 0 ahead" (5,1), the plan
// must route up through the passing zone and back, one object moving per action.
// The minimal-action solution is a 5-action axis-aligned weave around the band.
static constexpr double BAND_GAP = 2.0;
static constexpr double BAND_SUM = 10.0;

static bool pointFree(double x, double y)
{
    return !(std::abs(x - y) < BAND_GAP && (x + y) < BAND_SUM);
}

static bool isStateValid(const ob::State *state)
{
    const auto *s = state->as<ob::FragmentedStateSpace::StateType>();
    return pointFree(s->values[0], s->values[1]);   // x = object 0, y = object 1
}

// Replicates LARRT's notion of "which group changed between two states".
static int changedGroup(const std::vector<std::vector<int>> &groups,
                        const std::vector<double> &a, const std::vector<double> &b)
{
    for (size_t g = 0; g < groups.size(); ++g)
        for (int idx : groups[g])
            if (std::abs(a[idx] - b[idx]) > 1e-10)
                return static_cast<int>(g);
    return -1;
}

// Action count = number of contiguous runs where the same group keeps moving.
static int actionCount(const std::vector<std::vector<int>> &groups,
                       const std::vector<std::vector<double>> &path)
{
    if (path.size() < 2)
        return static_cast<int>(path.size());
    int prev = changedGroup(groups, path[0], path[1]);
    int count = 1;
    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        int cur = changedGroup(groups, path[i], path[i + 1]);
        if (cur != prev)
        {
            ++count;
            prev = cur;
        }
    }
    return count;
}

// True if the straight segment a->b stays out of the forbidden band (dense sample).
static bool segmentFree(const std::vector<double> &a, const std::vector<double> &b)
{
    const int N = 400;
    for (int k = 0; k <= N; ++k)
    {
        double t = double(k) / N;
        double x = a[0] + t * (b[0] - a[0]);
        double y = a[1] + t * (b[1] - a[1]);
        if (!pointFree(x, y))
            return false;
    }
    return true;
}

// Validate that the returned path is a genuine, collision-free solution.
// The LA-RRT PathDefragmenter occasionally returns a truncated/garbled path for
// these small 2-group problems, so we verify every solution and re-plan on a bad
// one (see the retry loop in main).
static bool pathValid(const std::vector<std::vector<double>> &path,
                     const std::vector<double> &startExp,
                     const std::vector<double> &goalExp)
{
    if (path.size() < 2)
        return false;
    for (size_t d = 0; d < 2; ++d)
        if (std::abs(path.front()[d] - startExp[d]) > 1e-3 ||
            std::abs(path.back()[d] - goalExp[d]) > 1e-3)
            return false;
    for (size_t i = 0; i + 1 < path.size(); ++i)
        if (!segmentFree(path[i], path[i + 1]))
            return false;
    return true;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::vector<std::vector<int>> groups = {{0}, {1}};
    const std::vector<double> startExp = {1.0, 5.0};   // object 1 is "ahead"
    const std::vector<double> goalExp = {5.0, 1.0};    // object 0 must end up "ahead"

    auto space = std::make_shared<ob::FragmentedStateSpace>(groups);
    space->addDimension(0.0, 10.0);   // object 0
    space->addDimension(0.0, 10.0);   // object 1

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
    // Fine resolution so straight single-object moves are reliably collision-checked.
    si->setStateValidityCheckingResolution(0.002);

    og::SimpleSetup ss(si);

    ob::ScopedState<> start(space);
    start[0] = startExp[0];
    start[1] = startExp[1];

    ob::ScopedState<> goal(space);
    goal[0] = goalExp[0];
    goal[1] = goalExp[1];

    ss.setStartAndGoalStates(start, goal);
    auto larrt = new og::LARRT(si, groups);
    ss.setPlanner(ob::PlannerPtr(larrt));

    // Retry-until-validated: re-plan (fresh seed) until the solution checks out.
    std::vector<std::vector<double>> path;
    int actions = 0;
    bool ok = false;
    const int maxAttempts = 400;
    for (int attempt = 1; attempt <= maxAttempts && !ok; ++attempt)
    {
        ss.clear();
        try
        {
            if (!ss.solve(2.0))
                continue;
        }
        catch (const std::exception &e)
        {
            // The defragmenter can throw on some seeds; discard and retry.
            continue;
        }
        path.clear();
        for (auto *st : ss.getSolutionPath().getStates())
        {
            std::vector<double> v;
            space->copyToReals(v, st);
            path.push_back(v);
        }
        if (pathValid(path, startExp, goalExp))
        {
            actions = actionCount(groups, path);
            ok = true;
            std::cout << "Found a validated solution on attempt " << attempt << "." << std::endl;
        }
    }

    if (!ok)
    {
        std::cout << "No validated solution found within " << maxAttempts
                  << " attempts." << std::endl;
        return 1;
    }

    std::cout << "States: " << path.size()
              << "  |  final action count (group switches): " << actions
              << "  |  planner bestCost: " << larrt->bestCost().value() << std::endl;
    ss.getSolutionPath().print(std::cout);

    // ---- write JSON ------------------------------------------------------
    const std::string outPath = "demos/larrt2d/out/configspace.json";
    std::ofstream out(outPath);
    if (!out)
    {
        std::cerr << "Could not open " << outPath
                  << " (run from the repository root)." << std::endl;
        return 1;
    }
    out << "{\n";
    out << "  \"demo\": \"configspace\",\n";
    out << "  \"groups\": [[0], [1]],\n";
    out << "  \"axis_labels\": [\"object 0 position\", \"object 1 position\"],\n";
    out << "  \"bounds\": [[0.0, 10.0], [0.0, 10.0]],\n";
    out << "  \"obstacles\": [],\n";
    // The forbidden region is the diagonal band |x - y| < gap while x + y < sum.
    out << "  \"forbidden_band\": {\"gap\": " << BAND_GAP
        << ", \"sum_limit\": " << BAND_SUM << "},\n";
    out << "  \"start\": [" << path.front()[0] << ", " << path.front()[1] << "],\n";
    out << "  \"goal\": [" << path.back()[0] << ", " << path.back()[1] << "],\n";
    out << "  \"action_count\": " << actions << ",\n";
    out << "  \"path\": [\n";
    for (size_t i = 0; i < path.size(); ++i)
    {
        out << "    [" << path[i][0] << ", " << path[i][1] << "]"
            << (i + 1 < path.size() ? "," : "") << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    out.close();
    std::cout << "Wrote " << outPath << std::endl;
    return 0;
}
