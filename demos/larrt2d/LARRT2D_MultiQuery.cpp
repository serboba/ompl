/*********************************************************************
 * LA-RRT 2D -- multi-query roadmap-reuse test (EIRM* vs BIT* from scratch).
 *
 * The rearrangement inner loop solves "move object i through this space" MANY times.
 * EIRM* (Effort-Informed Roadmaps) is a multiquery planner: it reuses one roadmap +
 * validation effort across queries, so per-query time should drop as it warms up --
 * exactly the amortization the from-scratch BIT* connector lacked.
 *
 * This isolates that: one mover, a FIXED scene (others frozen at start), K random goals.
 *   A) EIRM* with multiquery ON  -- one instance, roadmap reused across all K goals.
 *   B) BIT* from scratch         -- a fresh planner per goal (the naive connector).
 * Prints per-query times and totals.
 *
 * Usage: demo_LARRT2D_multiquery <scene.json> <mover_index> [K=40] [perQueryTime=0.5]
 *********************************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <memory>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/informedtrees/EIRMstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/util/RandomNumbers.h>

#include "json.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using json = nlohmann::json;

struct OBB { double cx, cy, hx, hy, theta; };
static constexpr double SAT_EPS = 1e-6;
static void corners(const OBB &b, double xs[4], double ys[4]) {
    const double c = std::cos(b.theta), s = std::sin(b.theta);
    const double sx[4] = {1, 1, -1, -1}, sy[4] = {1, -1, 1, -1};
    for (int k = 0; k < 4; ++k) { double lx = sx[k]*b.hx, ly = sy[k]*b.hy;
        xs[k] = b.cx + c*lx - s*ly; ys[k] = b.cy + s*lx + c*ly; }
}
static void pr(const double xs[4], const double ys[4], double ax, double ay, double &mn, double &mx) {
    mn = mx = xs[0]*ax + ys[0]*ay;
    for (int k = 1; k < 4; ++k) { double p = xs[k]*ax + ys[k]*ay; mn = std::min(mn,p); mx = std::max(mx,p); }
}
static bool overlap(const OBB &a, const OBB &b) {
    double ax[4], ay[4], bx[4], by[4]; corners(a, ax, ay); corners(b, bx, by);
    const double axes[4][2] = {{std::cos(a.theta),std::sin(a.theta)},{-std::sin(a.theta),std::cos(a.theta)},
                               {std::cos(b.theta),std::sin(b.theta)},{-std::sin(b.theta),std::cos(b.theta)}};
    for (auto &x : axes) { double mnA,mxA,mnB,mxB; pr(ax,ay,x[0],x[1],mnA,mxA); pr(bx,by,x[0],x[1],mnB,mxB);
        if (mxA <= mnB + SAT_EPS || mxB <= mnA + SAT_EPS) return false; }
    return true;
}

int main(int argc, char **argv) {
    if (argc < 3) { std::cerr << "usage: " << argv[0]
        << " <scene.json> <mover_index> [K=40] [perQueryTime=0.5]\n"; return 1; }
    const std::string scenePath = argv[1];
    const int mover = std::atoi(argv[2]);
    const int K = (argc > 3) ? std::atoi(argv[3]) : 40;
    const double qt = (argc > 4) ? std::atof(argv[4]) : 0.5;

    std::ifstream in(scenePath); json j; in >> j;
    std::vector<OBB> statics;
    for (const auto &o : j["obstacles"])
        statics.push_back({0.5*(double(o["xmin"])+double(o["xmax"])), 0.5*(double(o["ymin"])+double(o["ymax"])),
                           0.5*(double(o["xmax"])-double(o["xmin"])), 0.5*(double(o["ymax"])-double(o["ymin"])), 0.0});
    double mhx=0,mhy=0,mth=0,sx0=0,sy0=0,xlo=0,xhi=0,ylo=0,yhi=0; int idx=0;
    for (const auto &o : j["objects"]) {
        const double hx=o["hx"],hy=o["hy"]; const auto st=o["start"];
        if (idx==mover) { mhx=hx; mhy=hy; mth=st[2]; sx0=st[0]; sy0=st[1];
            xlo=o["xbounds"][0]; xhi=o["xbounds"][1]; ylo=o["ybounds"][0]; yhi=o["ybounds"][1]; }
        else statics.push_back({double(st[0]),double(st[1]),hx,hy,double(st[2])});
        ++idx;
    }

    auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds b(2); b.setLow(0,xlo); b.setHigh(0,xhi); b.setLow(1,ylo); b.setHigh(1,yhi);
    space->setBounds(b);
    auto si = std::make_shared<ob::SpaceInformation>(space);
    auto valid = [statics,mhx,mhy,mth](const ob::State *s) {
        const auto *r = s->as<ob::RealVectorStateSpace::StateType>();
        OBB m{r->values[0],r->values[1],mhx,mhy,mth};
        for (const auto &o : statics) if (overlap(m,o)) return false;
        return true;
    };
    si->setStateValidityChecker(valid);
    si->setStateValidityCheckingResolution(0.002);
    si->setup();

    // K random valid goals (same set used by both methods)
    ompl::RNG rng(1);
    std::vector<std::pair<double,double>> goals;
    while ((int)goals.size() < K) {
        double gx = rng.uniformReal(xlo,xhi), gy = rng.uniformReal(ylo,yhi);
        ob::ScopedState<ob::RealVectorStateSpace> t(space); t[0]=gx; t[1]=gy;
        if (si->isValid(t.get())) goals.emplace_back(gx,gy);
    }

    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    auto makePdef = [&](double gx, double gy) {
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        ob::ScopedState<ob::RealVectorStateSpace> s(space), g(space);
        s[0]=sx0; s[1]=sy0; g[0]=gx; g[1]=gy;
        pdef->setStartAndGoalStates(s, g);
        pdef->setOptimizationObjective(objective);
        return pdef;
    };
    auto solveTimed = [&](const std::shared_ptr<ob::Planner> &pl, const std::shared_ptr<ob::ProblemDefinition> &pdef) {
        auto ptc = ob::plannerOrTerminationCondition(ob::timedPlannerTerminationCondition(qt),
                                                     ob::exactSolnPlannerTerminationCondition(pdef));
        auto t0 = std::chrono::steady_clock::now();
        auto st = pl->solve(ptc);
        double dt = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
        bool ok = bool(st) && st.operator ob::PlannerStatus::StatusType() == ob::PlannerStatus::EXACT_SOLUTION;
        return std::make_pair(ok, dt);
    };

    // ---- A) EIRM* multiquery: one instance, roadmap reused ----
    std::cout << "K=" << K << " goals, per-query budget " << qt << "s, scene " << std::string(j["name"]) << "\n";
    std::cout << "query   EIRM*(reuse)   BIT*(scratch)\n";
    auto eirm = std::make_shared<og::EIRMstar>(si);   // multiquery is on by default in EIRM*
    double totA = 0, totB = 0; int okA = 0, okB = 0;
    for (int k = 0; k < K; ++k) {
        auto pdef = makePdef(goals[k].first, goals[k].second);
        eirm->setProblemDefinition(pdef);
        eirm->setup();
        auto [oka, dta] = solveTimed(eirm, pdef);
        eirm->clearQuery();
        totA += dta; okA += oka;

        auto bit = std::make_shared<og::BITstar>(si);
        auto pdefB = makePdef(goals[k].first, goals[k].second);
        bit->setProblemDefinition(pdefB); bit->setup();
        auto [okb, dtb] = solveTimed(bit, pdefB);
        totB += dtb; okB += okb;

        if (k < 12 || k % 5 == 0)
            printf("%4d   %10.4f    %10.4f\n", k, dta, dtb);
    }
    printf("----\nEIRM* multiquery: solved %d/%d  total %.3fs  mean %.4fs\n", okA, K, totA, totA/K);
    printf("BIT*  from scratch: solved %d/%d  total %.3fs  mean %.4fs\n", okB, K, totB, totB/K);
    printf("speedup (mean per query): %.2fx\n", (totB/K) / std::max(totA/K, 1e-9));
    return 0;
}
