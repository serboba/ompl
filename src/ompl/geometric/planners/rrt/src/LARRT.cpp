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
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"

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

    /* assume we can reach the state we go towards */
    bool reach = true;
    /* find state to add */
    base::State *dstate = rmotion->state;
    double d = si_->distance(nmotion->state, rmotion->state);
    // interpolate by maxDistance_ (not maxDistance_ / d) so a step is taken even for distant samples
    si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_, tgi.xstate);

    if (si_->equalStates(nmotion->state, tgi.xstate))
        return TRAPPED;

    dstate = tgi.xstate;

    if(d>maxDistance_){
        reach = false;
    }

    if (si_->equalStates(nmotion->state, dstate))
        return TRAPPED;


    if(!validMotionCheck(tgi.start,nmotion->state,dstate)){

        return TRAPPED;
    }

    auto newCost = opt_->motionCost(nmotion->state,dstate);

    // cost > 1.0 means the motion changes more than one group; otherwise the transition
    // already affects a single group and needs no isolation
    if(newCost.value() > 1.0){

        if(useIsolation_)
        {
            std::vector<ompl::base::State *> dstates;
            std::vector<double> s1,s2;
            si_->getStateSpace()->copyToReals(s1,nmotion->state);
            si_->getStateSpace()->copyToReals(s2,dstate);
            std::vector<int> g1 = getChangedGroups(s1,s2);
            buildIsoStates(nmotion->state,dstate,g1,dstates);

            std::vector<Motion *> stack_motion;
            if (dstates.size() == 0) { // nothing could be isolated
                return TRAPPED;
            }

            Motion *premotion = nmotion;
            for (auto st: dstates) { 
                auto *motion = new Motion(si_);
                createNewMotion(st, premotion,motion);
                stack_motion.push_back(motion);
                premotion = motion;
            }

            for (auto const &mot_: stack_motion) { // add motions at the end only if states could be isolated, only if loop passed without trapped
                tree->add(mot_);
                incCost = opt_->combineCosts(incCost, base::Cost(1.0));
            }
            freeStates(dstates);

            tgi.xmotion = stack_motion.back();

            return reach ? REACHED : ADVANCED;
        }
        return TRAPPED;
    }

    incCost = opt_->combineCosts(incCost, opt_->motionCost(nmotion->state, dstate));
    auto * motion = new Motion(si_);
    createNewMotion(dstate,nmotion,motion);
    tree->add(motion);
    tgi.xmotion = motion;

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

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
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

                if(ptc)
                {
                    pdef_->addSolutionPath(best_path, false, 0.0, getName());
                    solved = true;
                    break;
                }
                else
                    continue;
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
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());

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