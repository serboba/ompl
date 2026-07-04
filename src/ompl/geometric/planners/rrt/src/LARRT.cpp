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

#include "ompl/geometric/planners/rrt/LARRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/spaces/FactoredStateSpace.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"
#include <cmath>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <unordered_map>

ompl::geometric::LARRT::LARRT(const base::SpaceInformationPtr &si, std::vector<std::vector<int>> gr_indices,
                              bool useIsolation, int goalIndex)
  : base::Planner(si, "LARRT")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;
    group_indices = gr_indices;

    Planner::declareParam<double>("range", this, &LARRT::setRange, &LARRT::getRange, "0.:1.:10000.");
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    useIsolation_ = useIsolation;
    goalIndex_ = goalIndex;
    addPlannerProgressProperty("best cost DOUBLE", [this] { return bestCostProgressProperty(); });
    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
}

ompl::geometric::LARRT::~LARRT()
{
    freeMemory();
}

void ompl::geometric::LARRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    opt_ = std::make_shared<ompl::base::MinimalActionsObjective>(si_,group_indices);
    pdef_->setOptimizationObjective(opt_);

    bestCost_ = opt_->infiniteCost();
    incCost = opt_->infiniteCost();
}

void ompl::geometric::LARRT::freeMemory()
{
    std::vector<Motion *> motions;
    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

}

void ompl::geometric::LARRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();

    iterations_ = 0;
    bestCost_ = base::Cost(INFINITY);
}



void ompl::geometric::LARRT::buildIsoStates(ompl::base::State * from_, const ompl::base::State *to,
                                            std::vector<int> &changed_index_groups,
                                            std::vector<ompl::base::State* > &iso_ )
{
    std::vector<double> intermediate_st;
    std::vector<double> to_;
    base::State * it_ = from_;
    si_->getStateSpace()->copyToReals(intermediate_st,from_);
    si_->getStateSpace()->copyToReals(to_,to);

    for (size_t v = 0; v < changed_index_groups.size(); v++) {
        for (int i: group_indices.at(changed_index_groups.at(v))) {
            intermediate_st[i] = to_[i];
        }
        ompl::base::State * newState = si_->getStateSpace()->allocState();
        si_->getStateSpace()->copyFromReals(newState,intermediate_st);

        if (si_->checkMotion(it_,newState))
            iso_.push_back(newState);
        else{
            freeStates(iso_);
            si_->freeState(newState);
            return;
        }
        it_ = newState;
    }

}


std::vector<int> ompl::geometric::LARRT::getChangedGroups(const std::vector<double> &from_, const std::vector<double> &to_){
    std::vector<int> groups;
    for(size_t i = 0 ; i < group_indices.size(); i++){
        for(auto index : group_indices.at(i)){
            if(abs(from_.at(index)-to_.at(index))  > 0.0001){
                groups.push_back(i);
                break;
            }
        }
    }
    return groups;
}


double ompl::geometric::LARRT::groupDistance(const base::State *a, const base::State *b) const
{
    std::vector<double> va, vb;
    si_->getStateSpace()->copyToReals(va, a);
    si_->getStateSpace()->copyToReals(vb, b);
    // Angular (SO(2)) DOF use the shortest-arc distance; linear DOF use |diff|. If the space is a
    // FactoredStateSpace it tells us which dims are angular; otherwise everything is linear.
    const auto *fss = dynamic_cast<const base::FactoredStateSpace *>(si_->getStateSpace().get());
    double d = 0.0;
    for (auto const &group : group_indices)
        for (int idx : group)
        {
            double diff = std::abs(va[idx] - vb[idx]);
            if (fss != nullptr && fss->isAngleDim(idx))
            {
                diff = std::fmod(diff, 2.0 * M_PI);
                if (diff > M_PI) diff = 2.0 * M_PI - diff;
            }
            d += diff;
        }
    return d;
}

void ompl::geometric::LARRT::createNewMotion(const base::State *st, ompl::geometric::LARRT::Motion *premotion,
                                             ompl::geometric::LARRT::Motion *newmotion){
    si_->copyState(newmotion->state, st);

    newmotion->parent = premotion;
    newmotion->root = premotion->root;
    newmotion->cost = opt_->motionCost(premotion->state,st);
    newmotion->index_changed = getChangedIndex(premotion->state,st);

}


bool ompl::geometric::LARRT::validMotionCheck(const bool start, const base::State *from_, const base::State *to_){
    bool validmo= start ? si_->checkMotion(from_, to_) :
                           si_->isValid(to_) && si_->checkMotion(to_, from_);

    return validmo;

}

void ompl::geometric::LARRT::getMotionVectors(Motion * mot_, std::vector<Motion *> &vec)
{
    /* construct the motion vec */
    Motion *solution = mot_;
    while (solution != nullptr)
    {
        vec.push_back(solution);
        solution = solution->parent;
    }

}


int ompl::geometric::LARRT::getChangedIndex(const ompl::base::State *from, const ompl::base::State * to){

    // assumes a motion cost of 1 between from and to, and that they are not equal states

    std::vector<double> s_from,s_to;
    si_->getStateSpace()->copyToReals(s_from,from);
    si_->getStateSpace()->copyToReals(s_to,to);

    for(size_t i = 0; i < group_indices.size() ; i++)
    {
        for(auto const &index_in_gr : group_indices.at(i))
        {
            if(abs(s_from.at(index_in_gr) - s_to.at(index_in_gr)) > 1e-10)
                return i;
        }
    }

    return -1;
}


void ompl::geometric::LARRT::freeStates(std::vector<ompl::base::State*> &states)
{
    for(auto &st : states)
        si_->freeState(st);
    states.clear();
}


int ompl::geometric::LARRT::getCostPath(std::vector<ompl::base::State * > &states_)
{

    if(states_.size()<2)
        return 1;
    int prev_index = getChangedIndex(states_.at(0),states_.at(1));
    int cost = 1;
    for (size_t i = 1; i < states_.size()-1 ; ++i) {

        if(prev_index != getChangedIndex(states_.at(i),states_.at(i+1)))
        {
            cost++;
            prev_index = getChangedIndex(states_.at(i),states_.at(i+1));
        }
    }

    return cost;
}


ompl::geometric::LARRT::GrowState ompl::geometric::LARRT::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                   Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    std::vector<double> nreals, rreals;
    si_->getStateSpace()->copyToReals(nreals, nmotion->state);
    si_->getStateSpace()->copyToReals(rreals, rmotion->state);

    /* factors (groups/objects) that differ between the tree node and the sample */
    std::vector<int> diff = getChangedGroups(nreals, rreals);
    if (diff.empty())
        return TRAPPED;

    // --- diagnostics (LARRT_DBG) : count branch outcomes for the START tree only ----------
    static bool DBG = std::getenv("LARRT_DBG") != nullptr;
    static long dCall = 0, dIso = 0, dIsoOK = 0, dSingle = 0, dSingleOK = 0, dDoorPick = 0, dDoorOK = 0;
    const bool dbgThis = DBG && tgi.start;   // start tree only
    if (dbgThis)
    {
        ++dCall;
        if ((dCall % 40000) == 0)
            OMPL_INFORM("DBG start: calls=%ld iso=%ld(ok %ld) single=%ld(ok %ld) doorPick=%ld(ok %ld) useIso=%d range=%.3f",
                        dCall, dIso, dIsoOK, dSingle, dSingleOK, dDoorPick, dDoorOK, (int)useIsolation_, maxDistance_);
    }

    /* If several factors differ, ISOLATE the multi-factor extension to the sample into an ordered
       sequence of single-group steps (buildIsoStates), each collision-checked. This is what lets
       coupled sequences be discovered -- e.g. "swing the door open, THEN move the target through".
       The order is shuffled each call so that over iterations every ordering (obstacle-first vs
       target-first) is tried; buildIsoStates returns empty if the chosen order is blocked. */
    static bool NO_ISO = std::getenv("LARRT_NO_ISO") != nullptr;
    if (diff.size() > 1 && useIsolation_ && !NO_ISO)
    {
        if (dbgThis) ++dIso;
        si_->getStateSpace()->copyFromReals(tgi.xstate, rreals);   // full sample as the target
        std::vector<int> order = diff;
        for (int i = static_cast<int>(order.size()) - 1; i > 0; --i)   // Fisher-Yates shuffle
        {
            int j = rng_.uniformInt(0, i);
            int tmp = order[i]; order[i] = order[j]; order[j] = tmp;
        }
        std::vector<base::State *> dstates;
        buildIsoStates(nmotion->state, tgi.xstate, order, dstates);
        if (dstates.empty())
            return TRAPPED;   // this ordering is blocked; another sample/order may succeed

        Motion *prev = nmotion;
        for (base::State *st : dstates)
        {
            auto *m = new Motion(si_);
            createNewMotion(st, prev, m);                 // sets parent/root/cost/index(=that group)
            tree->add(m);
            incCost = opt_->combineCosts(incCost, base::Cost(1.0));
            prev = m;
        }
        freeStates(dstates);
        tgi.xmotion = prev;
        if (dbgThis) ++dIsoOK;
        return REACHED;   // the full sample (every differing factor) was matched
    }

    /* Advance ONE factor toward the sample, chosen at random among the differing
       factors and range-limited to maxDistance_ within that factor's L1. Picking a
       random factor (rather than always the lowest-index one) is what makes the
       factored search actually explore every object; moving a single factor keeps
       every tree edge at action cost 1 (no isolation needed). */
    int g = diff[rng_.uniformInt(0, static_cast<int>(diff.size()) - 1)];
    if (dbgThis) { ++dSingle; if (group_indices.at(g).size() == 1) ++dDoorPick; }
    double dg = 0.0;
    for (int idx : group_indices.at(g))
        dg += std::abs(nreals[idx] - rreals[idx]);
    double frac = (dg > maxDistance_) ? (maxDistance_ / dg) : 1.0;

    std::vector<double> dreals = nreals;
    for (int idx : group_indices.at(g))
        dreals[idx] = nreals[idx] + frac * (rreals[idx] - nreals[idx]);
    si_->getStateSpace()->copyFromReals(tgi.xstate, dreals);
    base::State *dstate = tgi.xstate;

    if (si_->equalStates(nmotion->state, dstate))
        return TRAPPED;

    if (dbgThis && group_indices.at(g).size() == 1 && dDoorPick < 12)
    {
        bool ev = si_->isValid(dstate);
        bool cm = si_->checkMotion(nmotion->state, dstate);
        OMPL_INFORM("DBG door step: from=%.4f to=%.4f (sample=%.4f) endpointValid=%d checkMotion=%d",
                    nreals[group_indices.at(g)[0]], dreals[group_indices.at(g)[0]],
                    rreals[group_indices.at(g)[0]], (int)ev, (int)cm);
    }
    if (!validMotionCheck(tgi.start, nmotion->state, dstate))
    {
        // The straight-line move of object g is blocked. Common case: give up (TRAPPED).
        // If a FactorConnector is installed, instead ask it to route object g AROUND the
        // obstruction to the sample's g-pose (others held at nmotion). The result is a chain
        // of same-object edges (still one object per action). See TWO_LEVEL_DESIGN.md.
        if (factorConnector_ == nullptr)
            return TRAPPED;

        std::vector<base::State *> wps;
        if (!factorConnector_->connect(nmotion->state, rreals, g, wps))
            return TRAPPED;

        Motion *prev = nmotion;
        for (base::State *ws : wps)
        {
            auto *m = new Motion(si_);
            createNewMotion(ws, prev, m);         // copies ws; sets parent/root/cost/index(=g)
            si_->freeState(ws);
            incCost = opt_->combineCosts(incCost, opt_->motionCost(prev->state, m->state));
            tree->add(m);
            prev = m;
        }
        tgi.xmotion = prev;
        // g was routed all the way to the sample's g-pose, so if it was the only differing
        // factor the sample is now matched -> REACHED, else more factors remain -> ADVANCED.
        return (diff.size() == 1) ? REACHED : ADVANCED;
    }

    incCost = opt_->combineCosts(incCost, opt_->motionCost(nmotion->state, dstate));
    auto *motion = new Motion(si_);
    createNewMotion(dstate, nmotion, motion);
    tree->add(motion);
    tgi.xmotion = motion;
    if (dbgThis) { ++dSingleOK; if (group_indices.at(g).size() == 1) ++dDoorOK; }

    /* REACHED only once the sample is matched on every factor: this was the last
       remaining differing factor and it was completed in a single step. */
    const bool reach = (frac >= 1.0 && diff.size() == 1);
    return reach ? REACHED : ADVANCED;
}



void ompl::geometric::LARRT::constructSolutionPath(ompl::geometric::PathGeometric &path, Motion * startMotion, Motion * goalMotion)
{
    if (startMotion->parent != nullptr)
        startMotion = startMotion->parent;
    else
        goalMotion = goalMotion->parent;

    connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

    std::vector<Motion *> mpath1;
    getMotionVectors(startMotion,mpath1);
    std::vector<Motion *> mpath2;
    getMotionVectors(goalMotion,mpath2);


    path.getStates().reserve(mpath1.size() + mpath2.size());
    for (int i = mpath1.size() - 1; i >= 0; --i){
        path.append(mpath1[i]->state);
    }
    for (auto &i : mpath2){
        path.append(i->state);
    }

}

ompl::base::PlannerStatus ompl::geometric::LARRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());
    // Optional: a goal that can project a state onto the goal manifold (targets pinned, free
    // dims kept) enables the PC-safe goal-biased projection below. Null for ordinary goals.
    auto *goalProj = dynamic_cast<GoalProjection *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        motion->cost = opt_->identityCost();
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    if (std::getenv("LARRT_SELFTEST"))
    {
        // Swing the door with every other object HELD at the start, find first invalid angle.
        base::State *a = si_->allocState();
        base::State *b = si_->allocState();
        std::vector<double> sv;
        si_->getStateSpace()->copyToReals(sv, tStart_->size() ? [&]{ std::vector<Motion*> m; tStart_->list(m); return m[0]->state; }() : a);
        const int doorIdx = static_cast<int>(sv.size()) - 1;   // door is last dim in door_easy
        OMPL_INFORM("SELFTEST start reals: A=(%.3f,%.3f) door=%.4f  startValid=%d",
                    sv[0], sv[1], sv[doorIdx], (int)([&]{ si_->getStateSpace()->copyFromReals(a, sv); return si_->isValid(a); }()));
        double prev = sv[doorIdx];
        for (double ang = sv[doorIdx]; ang >= -3.14159; ang -= 0.01)
        {
            std::vector<double> w = sv; w[doorIdx] = ang;
            si_->getStateSpace()->copyFromReals(b, w);
            std::vector<double> w0 = sv; w0[doorIdx] = prev;
            si_->getStateSpace()->copyFromReals(a, w0);
            bool ev = si_->isValid(b), cm = si_->checkMotion(a, b);
            if (!ev || !cm)
            {
                OMPL_INFORM("SELFTEST swing DOWN first failure at door=%.4f (endpointValid=%d stepMotion=%d)", ang, (int)ev, (int)cm);
                break;
            }
            prev = ang;
        }
        si_->freeState(a); si_->freeState(b);
    }

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *projState = si_->allocState();   // scratch for goal-biased projection
    bool solved = false;

    auto best_path(std::make_shared<PathGeometric>(si_));
    bestCost_ = opt_->infiniteCost();

    auto pd_(std::make_shared<ompl::geometric::PathDefragmenter>(si_,group_indices,goalIndex_,opt_)); // pd_ as p(ath) d(efragmenter)



    while (!ptc)
    {
        iterations_++;
        TreeData &tree = startTree_ ? tStart_ : tGoal_;
        tgi.start = startTree_;
        startTree_ = !startTree_;
        TreeData &otherTree = startTree_ ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                motion->cost = opt_->identityCost();
                motion->index_changed = 0;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }
        /* sample random state */

        sampler_->sampleUniform(rstate);

        GrowState gs = growTree(tree, tgi, rmotion);


        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            // --- PC-safe goal-biased projection onto the goal manifold ---------------------
            // If the goal supports projection and we just grew the START tree, occasionally try
            // to jump this new node straight onto the goal manifold (targets -> goal, free
            // objects left where they are). Only accept it when the projection edge moves at
            // most ONE object (single action); multi-target goals fall back to the goal-tree
            // connection below. The sampled goal tree remains the fallback, and goalBias_ > 0 is
            // fixed, so probabilistic completeness is preserved. See
            // demos/larrt2d/GOAL_REGION_AND_COMPLETENESS.md.
            if (goalProj != nullptr && tree == tStart_ && rng_.uniformReal(0.0, 1.0) < goalBias_ &&
                goalProj->projectToGoal(addedMotion->state, projState))
            {
                // count how many groups differ between the node and its projection
                std::vector<double> a, b;
                si_->getStateSpace()->copyToReals(a, addedMotion->state);
                si_->getStateSpace()->copyToReals(b, projState);
                int changedGroups = 0;
                for (const auto &grp : group_indices)
                    for (int idx : grp)
                        if (std::abs(a[idx] - b[idx]) > 1e-10) { ++changedGroups; break; }

                const bool alreadyAtGoal = (changedGroups == 0);              // node itself is a goal
                const bool oneMove = (changedGroups == 1) &&
                                     si_->isValid(projState) &&
                                     si_->checkMotion(addedMotion->state, projState);

                if (alreadyAtGoal || oneMove)
                {
                    // The solution is the start-tree chain up to addedMotion, plus (if a move is
                    // needed) the single projection edge onto the goal manifold.
                    Motion *last = addedMotion;
                    if (oneMove)
                    {
                        auto *pm = new Motion(si_);
                        createNewMotion(projState, addedMotion, pm);   // sets parent/root/cost/index
                        tStart_->add(pm);
                        last = pm;
                    }

                    auto path(std::make_shared<PathGeometric>(si_));
                    std::vector<Motion *> chain;
                    for (Motion *m = last; m != nullptr; m = m->parent)
                        chain.push_back(m);
                    for (auto it = chain.rbegin(); it != chain.rend(); ++it)
                        path->append((*it)->state);

                    pd_->doPathDefragComplete(path->getStates());
                    if (getCostPath(path->getStates()) < bestCost_.value())
                    {
                        OMPL_DEBUG("Projection solution, cost: %d (old %g)",
                                   getCostPath(path->getStates()), bestCost_.value());
                        freeStates(best_path->getStates());
                        best_path = path;
                        bestCost_ = base::Cost(getCostPath(best_path->getStates()));
                    }
                    // Solution found and defragmented to convergence -> stop planning.
                    // LA-RRT minimises actions through single-object moves + the
                    // PathDefragmenter, not by running the tree search for the whole time
                    // budget; the best_path is emitted after the loop.
                    break;
                }
            }

            /* attempt to connect trees */
            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);

            GrowState gsc = ADVANCED;
            tgi.start = startTree_;

            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = tgi.start ? tgi.xmotion : addedMotion;
            Motion *goalMotion = tgi.start ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root) )
            {
                auto path(std::make_shared<PathGeometric>(si_));

                constructSolutionPath(*path,startMotion,goalMotion);

                pd_->doPathDefragComplete(path->getStates());

                if(getCostPath(path->getStates()) < bestCost_.value())
                {
                    // bestCost_ starts at +inf, so print it as a real (avoids the
                    // INT_MIN that casting infinity to int produced).
                    OMPL_DEBUG("Better path found, new path cost: %d, old path cost: %g", getCostPath(path->getStates()), bestCost_.value());
                    freeStates(best_path->getStates());
                    best_path = path;
                    bestCost_ = base::Cost(getCostPath(best_path->getStates()));
                }

                // The trees connected and the path was defragmented to convergence
                // (doPathDefragComplete). That is the whole plan -- stop here instead of
                // spending the remaining time budget; the best_path is emitted after the loop.
                break;
            }
            else
            {
                if(ptc && best_path->getStates().size() > 1)
                {
                    pdef_->addSolutionPath(best_path, false, 0.0, getName());
                    solved = true;
                    break;

                }
                else if (tgi.start)
                {
                    // We were working from the startTree.
                    // NOTE: a start-tree node that already satisfies the goal is only recorded as
                    // an APPROXIMATE solution here, never accepted as exact. For rearrangement goals
                    // where non-target objects may be anywhere, accepting it exactly would leave the
                    // free objects in place (fewer actions) -- but doing so soundly requires a
                    // goal-BIASED projection extension onto the goal manifold, not just loosening
                    // this test: the goal set is measure-zero in the target dims, so an unbiased
                    // sampler reaches it with probability 0. See
                    // demos/larrt2d/GOAL_REGION_AND_COMPLETENESS.md for the probabilistic-
                    // completeness analysis before changing this.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
        }


    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    si_->freeState(projState);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

    // --- Optional tree dump for diagnostics (set env LARRT_TREE_DUMP=<path>) ---------------
    // Writes every node of both trees (full real vector + parent index) as JSON so the search
    // frontier can be visualised in the workspace. No effect unless the env var is set.
    if (const char *dumpPath = std::getenv("LARRT_TREE_DUMP"))
    {
        std::ofstream df(dumpPath);
        if (df)
        {
            auto writeTree = [&](TreeData &tree, const char *key) {
                std::vector<Motion *> ms;
                if (tree) tree->list(ms);
                std::unordered_map<const Motion *, int> idx;
                for (size_t i = 0; i < ms.size(); ++i) idx[ms[i]] = static_cast<int>(i);
                df << "\"" << key << "\":[";
                for (size_t i = 0; i < ms.size(); ++i)
                {
                    std::vector<double> v;
                    si_->getStateSpace()->copyToReals(v, ms[i]->state);
                    df << (i ? "," : "") << "{\"v\":[";
                    for (size_t d = 0; d < v.size(); ++d) df << (d ? "," : "") << v[d];
                    int p = ms[i]->parent ? idx[ms[i]->parent] : -1;
                    df << "],\"p\":" << p << "}";
                }
                df << "]";
            };
            df << "{";
            writeTree(tStart_, "start");
            df << ",";
            writeTree(tGoal_, "goal");
            df << "}\n";
            OMPL_INFORM("%s: dumped trees to %s", getName().c_str(), dumpPath);
        }
    }

    if(best_path->getStateCount()>1)
    {
        pdef_->addSolutionPath(best_path, false, 0.0, getName());
        solved = true;
    }
    else if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
        return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::LARRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}

std::string ompl::geometric::LARRT::bestCostProgressProperty() const
{
    return std::to_string(this->bestCost_.value());
}