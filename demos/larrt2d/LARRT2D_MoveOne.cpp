/*********************************************************************
 * LA-RRT 2D -- single-object optimal motion primitive (Phase 2 building block).
 *
 * The per-object sub-problem of rearrangement: move ONE object from its start to a
 * given goal (x, y) with every OTHER object frozen at its start pose (as an obstacle).
 * This is a full-dimensional, length-optimal geometric query -- the regime where BIT*
 * / AIT* are strong, and (unlike the full factored space) there is no locked-DOF
 * informed-sampler pathology because we plan only the mover's 2 DOF.
 *
 * Usage: demo_LARRT2D_moveone <scene.json> <mover_index> <gx> <gy> [planTime=2] [planner=bitstar]
 * Prints solved / path length / time and whether the motion is collision-free.
 *********************************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "json.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using json = nlohmann::json;

struct OBB { double cx, cy, hx, hy, theta; };
static constexpr double SAT_EPS = 1e-6;

static void corners(const OBB &b, double xs[4], double ys[4]) {
    const double c = std::cos(b.theta), s = std::sin(b.theta);
    const double sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, 1, -1};
    for (int k = 0; k < 4; ++k) {
        const double lx = sx[k] * b.hx, ly = sy[k] * b.hy;
        xs[k] = b.cx + c * lx - s * ly; ys[k] = b.cy + s * lx + c * ly;
    }
}
static void proj(const double xs[4], const double ys[4], double ax, double ay, double &mn, double &mx) {
    mn = mx = xs[0] * ax + ys[0] * ay;
    for (int k = 1; k < 4; ++k) { double p = xs[k] * ax + ys[k] * ay; mn = std::min(mn, p); mx = std::max(mx, p); }
}
static bool overlap(const OBB &a, const OBB &b) {
    double ax[4], ay[4], bx[4], by[4]; corners(a, ax, ay); corners(b, bx, by);
    const double axes[4][2] = {{std::cos(a.theta), std::sin(a.theta)}, {-std::sin(a.theta), std::cos(a.theta)},
                               {std::cos(b.theta), std::sin(b.theta)}, {-std::sin(b.theta), std::cos(b.theta)}};
    for (auto &ax2 : axes) {
        double mnA, mxA, mnB, mxB;
        proj(ax, ay, ax2[0], ax2[1], mnA, mxA); proj(bx, by, ax2[0], ax2[1], mnB, mxB);
        if (mxA <= mnB + SAT_EPS || mxB <= mnA + SAT_EPS) return false;
    }
    return true;
}

int main(int argc, char **argv) {
    if (argc < 5) {
        std::cerr << "usage: " << argv[0]
                  << " <scene.json> <mover_index> <gx> <gy> [planTime=2] [planner=bitstar]\n";
        return 1;
    }
    const std::string scenePath = argv[1];
    const int mover = std::atoi(argv[2]);
    const double gx = std::atof(argv[3]), gy = std::atof(argv[4]);
    const double planTime = (argc > 5) ? std::atof(argv[5]) : 2.0;
    const std::string plannerName = (argc > 6) ? argv[6] : "bitstar";

    std::ifstream in(scenePath); json j; in >> j;

    // static obstacles = walls + every non-mover object frozen at its start pose
    std::vector<OBB> statics;
    for (const auto &o : j["obstacles"])
        statics.push_back({0.5 * (double(o["xmin"]) + double(o["xmax"])),
                           0.5 * (double(o["ymin"]) + double(o["ymax"])),
                           0.5 * (double(o["xmax"]) - double(o["xmin"])),
                           0.5 * (double(o["ymax"]) - double(o["ymin"])), 0.0});
    double mhx = 0, mhy = 0, mtheta = 0, sx0 = 0, sy0 = 0;
    double xlo = 0, xhi = 0, ylo = 0, yhi = 0;
    int idx = 0;
    for (const auto &o : j["objects"]) {
        const double hx = o["hx"], hy = o["hy"];
        const auto st = o["start"];
        if (idx == mover) {
            mhx = hx; mhy = hy; mtheta = st[2]; sx0 = st[0]; sy0 = st[1];
            xlo = o["xbounds"][0]; xhi = o["xbounds"][1]; ylo = o["ybounds"][0]; yhi = o["ybounds"][1];
        } else {
            statics.push_back({double(st[0]), double(st[1]), hx, hy, double(st[2])});
        }
        ++idx;
    }

    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds b(2);
    b.setLow(0, xlo); b.setHigh(0, xhi); b.setLow(1, ylo); b.setHigh(1, yhi);
    space->setBounds(b);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([statics, mhx, mhy, mtheta](const ob::State *s) {
        const auto *r = s->as<ob::RealVectorStateSpace::StateType>();
        OBB m{r->values[0], r->values[1], mhx, mhy, mtheta};
        for (const auto &o : statics) if (overlap(m, o)) return false;
        return true;
    });
    si->setStateValidityCheckingResolution(0.002);

    og::SimpleSetup ss(si);
    ob::ScopedState<> start(space), goal(space);
    start[0] = sx0; start[1] = sy0; goal[0] = gx; goal[1] = gy;
    ss.setStartAndGoalStates(start, goal);
    ss.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    if (plannerName == "bitstar")      ss.setPlanner(std::make_shared<og::BITstar>(si));
    else if (plannerName == "aitstar") ss.setPlanner(std::make_shared<og::AITstar>(si));
    else if (plannerName == "rrtstar") ss.setPlanner(std::make_shared<og::RRTstar>(si));
    else                               ss.setPlanner(std::make_shared<og::RRTConnect>(si));

    // stop at first exact solution or the time budget (measures time-to-solution)
    auto ptc = ob::plannerOrTerminationCondition(
        ob::timedPlannerTerminationCondition(planTime),
        ob::exactSolnPlannerTerminationCondition(ss.getProblemDefinition()));
    const auto t0 = std::chrono::steady_clock::now();
    ob::PlannerStatus status = ss.solve(ptc);
    const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    const bool solved = bool(status) &&
        status.operator ob::PlannerStatus::StatusType() == ob::PlannerStatus::EXACT_SOLUTION;
    double len = -1;
    bool collFree = false;
    if (solved) {
        auto &p = ss.getSolutionPath();
        p.interpolate(400);
        len = p.length();
        collFree = p.check();   // dense collision check
    }
    std::cout << "planner=" << plannerName << "  mover=" << mover
              << "  solved=" << (solved ? "true" : "false")
              << "  path_len=" << len << "  collision_free=" << (collFree ? "true" : "false")
              << "  time_s=" << dt << std::endl;
    return solved ? 0 : 1;
}
