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

// LA-RRT 2D demo B -- "workspace rearrangement" explainer.
//
// Two pucks live in a shared 2D workspace. The state is 4-dimensional:
// puck A = (dim0, dim1), puck B = (dim2, dim3), with groups = {{0,1}, {2,3}}.
// Each ACTION moves exactly one puck (one group) while the other stays put.
//
// LA-RRT minimizes the number of actions = the number of times control switches
// between the two pucks. Moving a single puck from its start straight to its goal
// counts as ONE action even if it traces an L-shaped detour around an obstacle,
// because the same group keeps moving. So the natural minimal plan here is "move
// puck A out of the way / to its goal, then move puck B" -> a small action count.
// The PathDefragmenter (run internally by LARRT::solve) reorders/merges per-puck
// segments to remove redundant back-and-forth switching.
//
// After solving, this demo writes a JSON description of the problem and the full
// solution path for the scripts in viz/.

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/FragmentedStateSpace.h>
#include <ompl/geometric/planners/rrt/LARRT.h>
#include <ompl/base/objectives/MinimalActionsObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/RandomNumbers.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Axis-aligned rectangular obstacle in the shared 2D workspace.
struct Rect
{
    double xmin, xmax, ymin, ymax;
    bool contains(double x, double y) const
    {
        return x > xmin && x < xmax && y > ymin && y < ymax;
    }
};

// Obstacles framing the free workspace: four blocks in the quadrant corners
// leave a central cross-shaped free space. Puck A is relocated along the row
// y = 5 and puck B along the column x = 5, so their geometric paths cross at the
// centre. Because the two pucks may not overlap, they cannot both sit at the
// centre at once -- but since they move ONE puck per action, the trees are never
// at the centre simultaneously. After defragmentation the plan reduces to the
// 2-action optimum: move puck B up the column, then puck A across the row.
static const std::vector<Rect> obstacles = {
    {1.0, 3.0, 1.0, 3.0},   // bottom-left
    {7.0, 9.0, 1.0, 3.0},   // bottom-right
    {1.0, 3.0, 7.0, 9.0},   // top-left
    {7.0, 9.0, 7.0, 9.0}    // top-right
};

static bool pointFree(double x, double y)
{
    for (const auto &r : obstacles)
        if (r.contains(x, y))
            return false;
    return true;
}

static constexpr double PUCK_CLEAR = 0.7;   // min centre-to-centre distance

static bool isStateValid(const ob::State *state)
{
    const auto *s = state->as<ob::FragmentedStateSpace::StateType>();
    const double ax = s->values[0], ay = s->values[1];   // puck A
    const double bx = s->values[2], by = s->values[3];   // puck B
    if (!pointFree(ax, ay) || !pointFree(bx, by))
        return false;
    // The two pucks may not overlap (simple disc collision).
    const double dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy >= PUCK_CLEAR * PUCK_CLEAR;
}

static int changedGroup(const std::vector<std::vector<int>> &groups,
                        const std::vector<double> &a, const std::vector<double> &b)
{
    for (size_t g = 0; g < groups.size(); ++g)
        for (int idx : groups[g])
            if (std::abs(a[idx] - b[idx]) > 1e-10)
                return static_cast<int>(g);
    return -1;
}

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

// True if the straight 4-D segment a->b keeps both pucks off every obstacle and
// keeps the pucks clear of each other along the whole motion (dense sample).
static bool segmentFree(const std::vector<double> &a, const std::vector<double> &b)
{
    const int N = 400;
    for (int k = 0; k <= N; ++k)
    {
        double t = double(k) / N;
        double ax = a[0] + t * (b[0] - a[0]), ay = a[1] + t * (b[1] - a[1]);
        double bx = a[2] + t * (b[2] - a[2]), by = a[3] + t * (b[3] - a[3]);
        if (!pointFree(ax, ay) || !pointFree(bx, by))
            return false;
        double dx = ax - bx, dy = ay - by;
        if (dx * dx + dy * dy < PUCK_CLEAR * PUCK_CLEAR)
            return false;
    }
    return true;
}

// Sanity-check that the returned path is a genuine, collision-free solution that
// reaches the goal -- the invariants the PathDefragmenter is required to preserve.
static bool pathValid(const std::vector<std::vector<double>> &path,
                     const std::vector<double> &startExp,
                     const std::vector<double> &goalExp)
{
    if (path.size() < 2)
        return false;
    for (size_t d = 0; d < 4; ++d)
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
    std::vector<std::vector<int>> groups = {{0, 1}, {2, 3}};
    const std::vector<double> startExp = {1.0, 5.0, 5.0, 1.0};
    const std::vector<double> goalExp = {9.0, 5.0, 5.0, 9.0};

    auto space = std::make_shared<ob::FragmentedStateSpace>(groups);
    space->addDimension(0.0, 10.0);   // puck A x
    space->addDimension(0.0, 10.0);   // puck A y
    space->addDimension(0.0, 10.0);   // puck B x
    space->addDimension(0.0, 10.0);   // puck B y

    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
    // Fine resolution so straight single-puck moves are reliably collision-checked.
    si->setStateValidityCheckingResolution(0.002);

    og::SimpleSetup ss(si);

    ob::ScopedState<> start(space);
    start[0] = startExp[0]; start[1] = startExp[1];   // puck A on the left of row y = 5
    start[2] = startExp[2]; start[3] = startExp[3];   // puck B at the bottom of column x = 5

    ob::ScopedState<> goal(space);
    goal[0] = goalExp[0]; goal[1] = goalExp[1];        // puck A -> right of the row
    goal[2] = goalExp[2]; goal[3] = goalExp[3];        // puck B -> top of the column

    ss.setStartAndGoalStates(start, goal);
    auto larrt = new og::LARRT(si, groups);
    ss.setPlanner(ob::PlannerPtr(larrt));

    // Fixed seed so this explainer reproduces the same minimal-action plan every run.
    ompl::RNG::setSeed(1);

    // A single solve is enough: the PathDefragmenter returns a valid, minimal-action
    // path directly. We still sanity-check it below to document the invariants.
    if (!ss.solve(2.0))
    {
        std::cout << "No solution found." << std::endl;
        return 1;
    }

    std::vector<std::vector<double>> path;
    for (auto *st : ss.getSolutionPath().getStates())
    {
        std::vector<double> v;
        space->copyToReals(v, st);
        path.push_back(v);
    }

    if (!pathValid(path, startExp, goalExp))
    {
        std::cout << "Solution failed validation (should not happen)." << std::endl;
        return 1;
    }
    int actions = actionCount(groups, path);

    std::cout << "States: " << path.size()
              << "  |  final action count (puck switches): " << actions
              << "  |  planner bestCost: " << larrt->bestCost().value() << std::endl;
    ss.getSolutionPath().print(std::cout);

    // ---- write JSON ------------------------------------------------------
    const std::string outPath = "demos/larrt2d/out/workspace.json";
    std::ofstream out(outPath);
    if (!out)
    {
        std::cerr << "Could not open " << outPath
                  << " (run from the repository root)." << std::endl;
        return 1;
    }
    out << "{\n";
    out << "  \"demo\": \"workspace\",\n";
    out << "  \"groups\": [[0, 1], [2, 3]],\n";
    out << "  \"object_names\": [\"puck A\", \"puck B\"],\n";
    out << "  \"bounds\": [[0.0, 10.0], [0.0, 10.0], [0.0, 10.0], [0.0, 10.0]],\n";
    out << "  \"obstacles\": [\n";
    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        const auto &r = obstacles[i];
        out << "    {\"xmin\": " << r.xmin << ", \"xmax\": " << r.xmax
            << ", \"ymin\": " << r.ymin << ", \"ymax\": " << r.ymax << "}"
            << (i + 1 < obstacles.size() ? "," : "") << "\n";
    }
    out << "  ],\n";
    out << "  \"start\": [" << path.front()[0] << ", " << path.front()[1] << ", "
        << path.front()[2] << ", " << path.front()[3] << "],\n";
    out << "  \"goal\": [" << path.back()[0] << ", " << path.back()[1] << ", "
        << path.back()[2] << ", " << path.back()[3] << "],\n";
    out << "  \"action_count\": " << actions << ",\n";
    out << "  \"path\": [\n";
    for (size_t i = 0; i < path.size(); ++i)
    {
        out << "    [" << path[i][0] << ", " << path[i][1] << ", "
            << path[i][2] << ", " << path[i][3] << "]"
            << (i + 1 < path.size() ? "," : "") << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    out.close();
    std::cout << "Wrote " << outPath << std::endl;
    return 0;
}
