/*********************************************************************
 * LA-RRT 2D rearrangement-puzzle driver (data-driven, multi-kind objects).
 *
 * Usage: demo_LARRT2D_puzzle <scene.json> [planTime=10] [planner=larrt]
 *   planner in {larrt, larrteirm, rrtconnect, rrtstar, bitstar, aitstar, fmt}
 *
 * Object kinds (each maps its DOF sub-vector to an oriented rectangle = OBB):
 *   box    : free body, DOF (x, y[, theta]); OBB centred at (x,y), angle theta.
 *   door   : revolute, DOF (angle); rectangle hinged at a fixed pivot, length L,
 *            width W; OBB centre = hinge + (L/2)(cos a, sin a), angle a. (move-n-times
 *            "door" that swings open around a hinge.)
 *   slider : prismatic, DOF (disp); rectangle translating along a fixed axis from a base.
 *
 * One group per object (moving any of its DOF = one action). Goal region: each TARGET
 * object at its goal DOF values; every non-target object anywhere collision-free.
 * See demos/larrt2d/PUZZLE_PIPELINE_SPEC.md.
 *********************************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <limits>
#include <map>
#include <memory>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/FactoredStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/LARRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/EIRMstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/RandomNumbers.h>

#include "json.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Oriented bounding box + SAT overlap (spec section 1).
// ---------------------------------------------------------------------------
struct OBB { double cx, cy, hx, hy, theta; };
static constexpr double SAT_EPS = 1e-6;

static void obbCorners(const OBB &b, double xs[4], double ys[4])
{
    const double c = std::cos(b.theta), s = std::sin(b.theta);
    const double sx[4] = {+1, +1, -1, -1}, sy[4] = {+1, -1, +1, -1};
    for (int k = 0; k < 4; ++k)
    {
        const double lx = sx[k] * b.hx, ly = sy[k] * b.hy;
        xs[k] = b.cx + c * lx - s * ly;
        ys[k] = b.cy + s * lx + c * ly;
    }
}
static void projectRange(const double xs[4], const double ys[4], double ax, double ay,
                         double &mn, double &mx)
{
    mn = mx = xs[0] * ax + ys[0] * ay;
    for (int k = 1; k < 4; ++k) { double p = xs[k] * ax + ys[k] * ay; mn = std::min(mn, p); mx = std::max(mx, p); }
}
static bool obbOverlap(const OBB &a, const OBB &b)
{
    double ax[4], ay[4], bx[4], by[4];
    obbCorners(a, ax, ay); obbCorners(b, bx, by);
    const double axes[4][2] = {
        {std::cos(a.theta), std::sin(a.theta)}, {-std::sin(a.theta), std::cos(a.theta)},
        {std::cos(b.theta), std::sin(b.theta)}, {-std::sin(b.theta), std::cos(b.theta)}};
    for (auto &x : axes)
    {
        double mnA, mxA, mnB, mxB;
        projectRange(ax, ay, x[0], x[1], mnA, mxA);
        projectRange(bx, by, x[0], x[1], mnB, mxB);
        if (mxA <= mnB + SAT_EPS || mxB <= mnA + SAT_EPS) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Object model: each object owns a contiguous block of DOF in the full state
// ([off, off+ndof)) and maps them to an OBB according to its kind.
// ---------------------------------------------------------------------------
enum class Kind { Box, Door, Slider, Revolute };

struct ObjSpec
{
    std::string name;
    Kind kind;
    bool isTarget;
    int off, ndof;                       // DOF block in the full state
    std::vector<double> startVals, goalVals, lo, hi;   // per-DOF
    // fixed geometry
    double hx = 0, hy = 0;               // box / slider / revolute half-extents
    double fixedAngle = 0;               // box (non-rot) orientation / slider rect angle
    double hingeX = 0, hingeY = 0, length = 0, width = 0;   // door
    double baseX = 0, baseY = 0, axisX = 1, axisY = 0;      // slider
    double anchorX = 0, anchorY = 0, comX = 0, comY = 0;    // revolute: pivot + body-centre offset

    OBB obb(const double *v) const       // v = &fullReals[off]
    {
        if (kind == Kind::Box)
        {
            const double th = (ndof == 3) ? v[2] : fixedAngle;
            return OBB{v[0], v[1], hx, hy, th};
        }
        if (kind == Kind::Door)
        {
            const double a = v[0];
            return OBB{hingeX + 0.5 * length * std::cos(a), hingeY + 0.5 * length * std::sin(a),
                       0.5 * length, 0.5 * width, a};
        }
        if (kind == Kind::Revolute)
        {
            // General hinge: body centre = anchor + R(a)*com; body rotates with the joint.
            // (A door is the special case com = (length/2, 0), hx = length/2, hy = width/2.)
            const double a = v[0], c = std::cos(a), s = std::sin(a);
            return OBB{anchorX + c * comX - s * comY, anchorY + s * comX + c * comY, hx, hy, a};
        }
        // Slider
        const double d = v[0];
        return OBB{baseX + d * axisX, baseY + d * axisY, hx, hy, fixedAngle};
    }
};

struct Scene
{
    std::string name;
    double worldXmin, worldXmax, worldYmin, worldYmax;
    std::vector<OBB> obstacles;   // static walls
    std::vector<ObjSpec> objects;
    int dim = 0;                  // total DOF
};

// Valid iff no object OBB overlaps any wall and no two object OBBs overlap.
static bool configValid(const Scene &sc, const std::vector<double> &vals)
{
    std::vector<OBB> objs;
    objs.reserve(sc.objects.size());
    for (const auto &o : sc.objects) objs.push_back(o.obb(&vals[o.off]));
    for (size_t i = 0; i < objs.size(); ++i)
    {
        for (const auto &obs : sc.obstacles) if (obbOverlap(objs[i], obs)) return false;
        for (size_t j = i + 1; j < objs.size(); ++j) if (obbOverlap(objs[i], objs[j])) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Rearrangement goal region: targets pinned to their goal DOF, others sampled
// at random valid poses. (See spec 1b.)
// ---------------------------------------------------------------------------
class RearrangementGoal : public ob::GoalSampleableRegion, public og::GoalProjection
{
public:
    RearrangementGoal(const ob::SpaceInformationPtr &si, const Scene &scene)
        : ob::GoalSampleableRegion(si), scene_(scene) { setThreshold(1e-3); }

    double distanceGoal(const ob::State *st) const override
    {
        const auto *s = st->as<ob::FactoredStateSpace::StateType>();
        double d = 0.0;
        for (const auto &o : scene_.objects)
            if (o.isTarget)
                for (int k = 0; k < o.ndof; ++k) d += std::abs(s->values[o.off + k] - o.goalVals[k]);
        return d;
    }

    void sampleGoal(ob::State *st) const override
    {
        auto *s = st->as<ob::FactoredStateSpace::StateType>();
        for (int attempt = 0; attempt < 200; ++attempt)
        {
            for (const auto &o : scene_.objects)
                for (int k = 0; k < o.ndof; ++k)
                    s->values[o.off + k] = o.isTarget ? o.goalVals[k]
                                                      : rng_.uniformReal(o.lo[k], o.hi[k]);
            if (si_->isValid(st)) return;
        }
    }

    bool projectToGoal(const ob::State *from, ob::State *out) const override
    {
        si_->copyState(out, from);
        auto *s = out->as<ob::FactoredStateSpace::StateType>();
        for (const auto &o : scene_.objects)
            if (o.isTarget)
                for (int k = 0; k < o.ndof; ++k) s->values[o.off + k] = o.goalVals[k];
        return true;
    }

    unsigned int maxSampleCount() const override { return std::numeric_limits<unsigned int>::max(); }

private:
    Scene scene_;
    mutable ompl::RNG rng_;
};

// ---------------------------------------------------------------------------
// EIRM* single-object connector (two-level low level). Persistent per-object
// roadmap over the object's FREE DOF, validated against STATIC walls only, reused
// across the run; each query re-checks the returned path against the movable
// objects at their current poses. See demos/larrt2d/TWO_LEVEL_DESIGN.md.
// ---------------------------------------------------------------------------
class EirmConnector : public og::FactorConnector
{
public:
    EirmConnector(const ob::SpaceInformationPtr &fullSi, const Scene &scene)
        : fullSi_(fullSi), scene_(scene) {}

    bool connect(const ob::State *fromFull, const std::vector<double> &targetReals, int g,
                 std::vector<ob::State *> &waypoints) override
    {
        const ObjSpec &obj = scene_.objects[g];
        std::vector<double> fromReals;
        fullSi_->getStateSpace()->copyToReals(fromReals, fromFull);

        RM &rm = roadmap(g);
        const std::size_t sd = rm.free.size();
        ob::ScopedState<ob::RealVectorStateSpace> s(rm.space), t(rm.space);
        for (std::size_t i = 0; i < sd; ++i)
        {
            s[i] = fromReals[obj.off + rm.free[i]];
            t[i] = targetReals[obj.off + rm.free[i]];
        }
        auto pdef = std::make_shared<ob::ProblemDefinition>(rm.si);
        pdef->setStartAndGoalStates(s, t);
        pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(rm.si));
        rm.planner->setProblemDefinition(pdef);
        rm.planner->setup();
        auto ptc = ob::plannerOrTerminationCondition(
            ob::timedPlannerTerminationCondition(0.05),
            ob::exactSolnPlannerTerminationCondition(pdef));
        ob::PlannerStatus st = rm.planner->solve(ptc);
        const bool exact = bool(st) &&
            st.operator ob::PlannerStatus::StatusType() == ob::PlannerStatus::EXACT_SOLUTION;
        if (!exact) { rm.planner->clearQuery(); return false; }
        auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
        if (!path) { rm.planner->clearQuery(); return false; }

        // Assemble a helper to turn a sub-state (free DOF of g) into g's OBB.
        std::vector<double> full = fromReals;
        auto gObb = [&](const double *sub) {
            for (std::size_t i = 0; i < sd; ++i) full[obj.off + rm.free[i]] = sub[i];
            return obj.obb(&full[obj.off]);
        };
        // Re-check the wall-free path against the OTHER objects at their fromFull poses.
        og::PathGeometric dense(*path);
        dense.interpolate(std::max<std::size_t>(2, path->getStateCount()) * 30);
        for (const auto *ps : dense.getStates())
        {
            const auto *r = ps->as<ob::RealVectorStateSpace::StateType>();
            OBB gm = gObb(r->values);
            for (size_t j = 0; j < scene_.objects.size(); ++j)
                if ((int)j != g && obbOverlap(gm, scene_.objects[j].obb(&fromReals[scene_.objects[j].off])))
                { rm.planner->clearQuery(); return false; }
        }
        // Accept: map path waypoints to full states (only g's free DOF change).
        const auto &states = path->getStates();
        for (std::size_t k = 1; k < states.size(); ++k)
        {
            const auto *r = states[k]->as<ob::RealVectorStateSpace::StateType>();
            for (std::size_t i = 0; i < sd; ++i) full[obj.off + rm.free[i]] = r->values[i];
            ob::State *fs = fullSi_->allocState();
            fullSi_->getStateSpace()->copyFromReals(fs, full);
            waypoints.push_back(fs);
        }
        rm.planner->clearQuery();
        return !waypoints.empty();
    }

private:
    struct RM
    {
        std::vector<int> free;   // indices (within the object) of non-degenerate DOF
        std::shared_ptr<ob::RealVectorStateSpace> space;
        ob::SpaceInformationPtr si;
        std::shared_ptr<og::EIRMstar> planner;
    };

    RM &roadmap(int g)
    {
        auto it = rms_.find(g);
        if (it != rms_.end()) return it->second;
        const ObjSpec &o = scene_.objects[g];
        RM rm;
        for (int k = 0; k < o.ndof; ++k) if (o.hi[k] - o.lo[k] > 1e-6) rm.free.push_back(k);
        const std::size_t sd = rm.free.size();
        rm.space = std::make_shared<ob::RealVectorStateSpace>(sd);
        ob::RealVectorBounds b(sd);
        for (std::size_t i = 0; i < sd; ++i) { b.setLow(i, o.lo[rm.free[i]]); b.setHigh(i, o.hi[rm.free[i]]); }
        rm.space->setBounds(b);
        rm.si = std::make_shared<ob::SpaceInformation>(rm.space);
        // Validity: object g vs STATIC walls only -> reusable roadmap.
        std::vector<OBB> walls = scene_.obstacles;
        ObjSpec obj = o;
        std::vector<int> free = rm.free;
        std::vector<double> base = o.startVals;   // full DOF vector for g; free ones overwritten
        rm.si->setStateValidityChecker([walls, obj, free, base](const ob::State *st) {
            const auto *r = st->as<ob::RealVectorStateSpace::StateType>();
            std::vector<double> v = base;
            for (std::size_t i = 0; i < free.size(); ++i) v[free[i]] = r->values[i];
            OBB m = obj.obb(v.data());
            for (const auto &w : walls) if (obbOverlap(m, w)) return false;
            return true;
        });
        rm.si->setStateValidityCheckingResolution(0.002);
        rm.si->setup();
        rm.planner = std::make_shared<og::EIRMstar>(rm.si);
        rms_[g] = rm;
        return rms_[g];
    }

    ob::SpaceInformationPtr fullSi_;
    Scene scene_;
    std::map<int, RM> rms_;
};

// ---------------------------------------------------------------------------
// Action counting (spec section 5).
// ---------------------------------------------------------------------------
static int changedGroup(const std::vector<std::vector<int>> &groups,
                        const std::vector<double> &a, const std::vector<double> &b)
{
    for (size_t g = 0; g < groups.size(); ++g)
        for (int idx : groups[g])
            if (std::abs(a[idx] - b[idx]) > 1e-9) return static_cast<int>(g);
    return -1;
}
static int actionCount(const std::vector<std::vector<int>> &groups,
                       const std::vector<std::vector<double>> &path)
{
    if (path.size() < 2) return static_cast<int>(path.size());
    int prev = changedGroup(groups, path[0], path[1]), count = 1;
    for (size_t i = 1; i + 1 < path.size(); ++i)
    {
        int cur = changedGroup(groups, path[i], path[i + 1]);
        if (cur != prev) { ++count; prev = cur; }
    }
    return count;
}

// Dense SAT check of a straight segment in the full space.
static bool segmentFree(const Scene &sc, const std::vector<double> &a, const std::vector<double> &b)
{
    const int N = 200;
    std::vector<double> v(a.size());
    for (int k = 0; k <= N; ++k)
    {
        const double t = double(k) / N;
        for (size_t d = 0; d < a.size(); ++d) v[d] = a[d] + t * (b[d] - a[d]);
        if (!configValid(sc, v)) return false;
    }
    return true;
}
static bool pathValid(const Scene &sc, const std::vector<std::vector<double>> &path)
{
    if (path.size() < 2) return false;
    for (const auto &o : sc.objects)
    {
        for (int k = 0; k < o.ndof; ++k)
            if (std::abs(path.front()[o.off + k] - o.startVals[k]) > 1e-3) return false;
        if (o.isTarget)
            for (int k = 0; k < o.ndof; ++k)
                if (std::abs(path.back()[o.off + k] - o.goalVals[k]) > 1e-3) return false;
    }
    for (size_t i = 0; i + 1 < path.size(); ++i)
        if (!segmentFree(sc, path[i], path[i + 1])) return false;
    return true;
}

// ---------------------------------------------------------------------------
static double num(const json &j, const char *k, double def) { return j.contains(k) ? double(j[k]) : def; }

int main(int argc, char **argv)
{
    if (argc < 2) { std::cerr << "Usage: " << argv[0] << " <scene.json> [planTime=10] [planner=larrt]\n"; return 1; }
    const std::string scenePath = argv[1];
    const double planTime = (argc > 2) ? std::atof(argv[2]) : 10.0;
    const std::string plannerName = (argc > 3) ? argv[3] : "larrt";

    std::ifstream in(scenePath);
    if (!in) { std::cerr << "Could not open scene: " << scenePath << "\n"; return 1; }
    json j; try { in >> j; } catch (const std::exception &e) { std::cerr << "JSON error: " << e.what() << "\n"; return 1; }

    Scene scene;
    scene.name = j.value("name", "scene");
    scene.worldXmin = j["world"]["xmin"]; scene.worldXmax = j["world"]["xmax"];
    scene.worldYmin = j["world"]["ymin"]; scene.worldYmax = j["world"]["ymax"];
    for (const auto &o : j["obstacles"])
    {
        const double xmin = o["xmin"], xmax = o["xmax"], ymin = o["ymin"], ymax = o["ymax"];
        scene.obstacles.push_back(OBB{0.5 * (xmin + xmax), 0.5 * (ymin + ymax),
                                      0.5 * (xmax - xmin), 0.5 * (ymax - ymin), 0.0});
    }

    int off = 0;
    for (const auto &o : j["objects"])
    {
        ObjSpec s;
        s.name = o.value("name", "obj");
        s.isTarget = o.value("target", false);
        const std::string type = o.value("type", "box");
        s.off = off;

        if (type == "door")
        {
            s.kind = Kind::Door;
            s.ndof = 1;
            s.hingeX = o["hinge"][0]; s.hingeY = o["hinge"][1];
            s.length = o["length"]; s.width = o["width"];
            s.startVals = { double(o["start_angle"]) };
            s.lo = { double(o["angle_bounds"][0]) }; s.hi = { double(o["angle_bounds"][1]) };
            if (s.isTarget) s.goalVals = { double(o["goal_angle"]) };
        }
        else if (type == "revolute")
        {
            s.kind = Kind::Revolute;
            s.ndof = 1;
            s.anchorX = o["anchor"][0]; s.anchorY = o["anchor"][1];
            s.comX = o["com"][0]; s.comY = o["com"][1];
            s.hx = o["hx"]; s.hy = o["hy"];
            s.startVals = { double(o["start_angle"]) };
            s.lo = { double(o["angle_bounds"][0]) }; s.hi = { double(o["angle_bounds"][1]) };
            if (s.isTarget) s.goalVals = { double(o["goal_angle"]) };
        }
        else if (type == "slider")
        {
            s.kind = Kind::Slider;
            s.ndof = 1;
            s.baseX = o["base"][0]; s.baseY = o["base"][1];
            s.axisX = o["axis"][0]; s.axisY = o["axis"][1];
            const double an = std::hypot(s.axisX, s.axisY); s.axisX /= an; s.axisY /= an;
            s.hx = o["hx"]; s.hy = o["hy"]; s.fixedAngle = num(o, "angle", 0.0);
            s.startVals = { double(o["start_disp"]) };
            s.lo = { double(o["disp_bounds"][0]) }; s.hi = { double(o["disp_bounds"][1]) };
            if (s.isTarget) s.goalVals = { double(o["goal_disp"]) };
        }
        else   // box
        {
            s.kind = Kind::Box;
            s.hx = o["hx"]; s.hy = o["hy"];
            const bool rotates = o.value("rotates", false);
            s.ndof = rotates ? 3 : 2;
            s.fixedAngle = o["start"].size() > 2 ? double(o["start"][2]) : 0.0;
            s.startVals = { double(o["start"][0]), double(o["start"][1]) };
            s.lo = { double(o["xbounds"][0]), double(o["ybounds"][0]) };
            s.hi = { double(o["xbounds"][1]), double(o["ybounds"][1]) };
            if (rotates) { s.startVals.push_back(s.fixedAngle);
                           s.lo.push_back(o["tbounds"][0]); s.hi.push_back(o["tbounds"][1]); }
            if (s.isTarget)
            {
                s.goalVals = { double(o["goal"][0]), double(o["goal"][1]) };
                if (rotates) s.goalVals.push_back(o["goal"].size() > 2 ? double(o["goal"][2]) : 0.0);
            }
        }
        off += s.ndof;
        scene.objects.push_back(std::move(s));
    }
    scene.dim = off;
    const size_t nObj = scene.objects.size();
    if (nObj == 0 || scene.dim == 0) { std::cerr << "Scene has no movable DOF.\n"; return 1; }

    // groups: one per object (its DOF block)
    std::vector<std::vector<int>> groups(nObj);
    for (size_t i = 0; i < nObj; ++i)
        for (int k = 0; k < scene.objects[i].ndof; ++k) groups[i].push_back(scene.objects[i].off + k);

    auto space = std::make_shared<ob::FactoredStateSpace>(groups);
    {
        int di = 0;
        for (const auto &o : scene.objects)
            for (int k = 0; k < o.ndof; ++k)
            {
                double lo = o.lo[k], hi = o.hi[k];
                if (hi - lo < 1e-9) { lo -= 1e-9; hi += 1e-9; }   // locked DOF -> tiny non-degenerate
                space->addDimension(lo, hi);
                // A door's DOF (and a rotating box's theta) is an angle -> treat as SO(2):
                // shortest-arc distance/interpolation, no linear boundary artefact.
                if (o.kind == Kind::Door || o.kind == Kind::Revolute ||
                    (o.kind == Kind::Box && o.ndof == 3 && k == 2))
                    space->markAngleDim(di);
                ++di;
            }
    }

    auto si = std::make_shared<ob::SpaceInformation>(space);
    Scene sc = scene;
    si->setStateValidityChecker([sc](const ob::State *state) {
        const auto *s = state->as<ob::FactoredStateSpace::StateType>();
        std::vector<double> vals(sc.dim);
        for (int d = 0; d < sc.dim; ++d) vals[d] = s->values[d];
        return configValid(sc, vals);
    });
    si->setStateValidityCheckingResolution(0.002);

    size_t nTargets = 0;
    for (const auto &o : scene.objects) nTargets += o.isTarget ? 1 : 0;
    if (nTargets == 0) std::cerr << "WARNING: no target object; goal is trivially the start.\n";

    og::SimpleSetup ss(si);
    ob::ScopedState<> start(space);
    for (const auto &o : scene.objects)
        for (int k = 0; k < o.ndof; ++k) start[o.off + k] = o.startVals[k];
    ss.addStartState(start);
    ss.setGoal(std::make_shared<RearrangementGoal>(si, scene));

    og::LARRT *larrt = nullptr;
    std::unique_ptr<EirmConnector> eirmConn;
    if (plannerName == "rrtconnect")      ss.setPlanner(ob::PlannerPtr(new og::RRTConnect(si)));
    else if (plannerName == "bitstar")    ss.setPlanner(ob::PlannerPtr(new og::BITstar(si)));
    else if (plannerName == "aitstar")    ss.setPlanner(ob::PlannerPtr(new og::AITstar(si)));
    else if (plannerName == "fmt")        ss.setPlanner(ob::PlannerPtr(new og::FMT(si)));
    else if (plannerName == "rrtstar")    ss.setPlanner(ob::PlannerPtr(new og::RRTstar(si)));
    else
    {
        larrt = new og::LARRT(si, groups);
        if (plannerName == "larrteirm") { eirmConn.reset(new EirmConnector(si, scene)); larrt->setFactorConnector(eirmConn.get()); }
        ss.setPlanner(ob::PlannerPtr(larrt));
    }
    if (plannerName == "bitstar" || plannerName == "aitstar" || plannerName == "fmt" || plannerName == "rrtstar")
        ss.setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(si));

    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(planTime);
    if (plannerName != "larrt" && plannerName != "larrteirm")
        ptc = ob::plannerOrTerminationCondition(ptc, ob::exactSolnPlannerTerminationCondition(ss.getProblemDefinition()));

    const auto t0 = std::chrono::steady_clock::now();
    ob::PlannerStatus status = ss.solve(ptc);
    const double solveTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const bool solved = bool(status) &&
        status.operator ob::PlannerStatus::StatusType() == ob::PlannerStatus::EXACT_SOLUTION;

    std::vector<std::vector<double>> path;
    int actions = -1;
    if (solved)
    {
        for (auto *st : ss.getSolutionPath().getStates())
        {
            std::vector<double> v; space->copyToReals(v, st); path.push_back(v);
        }
        actions = actionCount(groups, path);
        if (!pathValid(scene, path))
            std::cout << "WARNING: returned path failed independent SAT validation.\n";
    }

    std::cout << "solved=" << (solved ? "true" : "false")
              << "  action_count=" << actions << "  solve_time_s=" << solveTime << std::endl;

    // ---- solution JSON (echo geometry so viz/validator can rebuild OBBs) ----
    json out;
    out["name"] = scene.name;
    out["world"] = {{"xmin", scene.worldXmin}, {"xmax", scene.worldXmax},
                    {"ymin", scene.worldYmin}, {"ymax", scene.worldYmax}};
    out["obstacles"] = json::array();
    for (const auto &o : j["obstacles"])
        out["obstacles"].push_back({{"xmin", o["xmin"]}, {"xmax", o["xmax"]},
                                    {"ymin", o["ymin"]}, {"ymax", o["ymax"]}});
    out["objects"] = json::array();
    for (const auto &o : scene.objects)
    {
        json oj;
        oj["name"] = o.name;
        oj["target"] = o.isTarget;
        oj["dims"] = json::array();
        for (int k = 0; k < o.ndof; ++k) oj["dims"].push_back(o.off + k);
        oj["start"] = o.startVals;
        oj["goal"] = o.isTarget ? json(o.goalVals) : json(nullptr);
        if (o.kind == Kind::Box)
        {
            oj["type"] = "box"; oj["hx"] = o.hx; oj["hy"] = o.hy;
            oj["rotates"] = (o.ndof == 3); oj["angle"] = o.fixedAngle;
        }
        else if (o.kind == Kind::Door)
        {
            oj["type"] = "door"; oj["hinge"] = {o.hingeX, o.hingeY};
            oj["length"] = o.length; oj["width"] = o.width;
        }
        else if (o.kind == Kind::Revolute)
        {
            oj["type"] = "revolute"; oj["anchor"] = {o.anchorX, o.anchorY};
            oj["com"] = {o.comX, o.comY}; oj["hx"] = o.hx; oj["hy"] = o.hy;
        }
        else
        {
            oj["type"] = "slider"; oj["base"] = {o.baseX, o.baseY}; oj["axis"] = {o.axisX, o.axisY};
            oj["hx"] = o.hx; oj["hy"] = o.hy; oj["angle"] = o.fixedAngle;
        }
        out["objects"].push_back(oj);
    }
    out["groups"] = groups;
    out["planner"] = plannerName;
    out["solved"] = solved;
    out["action_count"] = solved ? actions : -1;
    out["solve_time_s"] = solveTime;
    out["path"] = json::array();
    if (solved) for (const auto &v : path) out["path"].push_back(v);

    const std::string outPath = "demos/larrt2d/out/" + scene.name + ".json";
    std::ofstream ofs(outPath);
    if (!ofs) { std::cerr << "Could not open " << outPath << " (run from repo root).\n"; return 1; }
    ofs << out.dump(2) << std::endl;
    std::cout << "Wrote " << outPath << std::endl;
    return solved ? 0 : 1;
}
