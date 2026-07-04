/* Author: Servet Bora Bayraktar */

#include <ompl/geometric/PathDefragmenter.h>
#include <ompl/base/objectives/MinimalActionsObjective.h>
#include <algorithm>

ompl::geometric::PathDefragmenter::PathDefragmenter(base::SpaceInformationPtr si,
                                                    std::vector<std::vector<int>> fragment_indices, int goalIndex,
                                                    const base::OptimizationObjectivePtr & /*opt*/)
  :si_(std::move(si)), fragment_indices(fragment_indices)
{
    goalIndex_ = goalIndex;
    opt_ = std::make_shared<base::MinimalActionsObjective>(si_, fragment_indices);
}




std::vector<int> ompl::geometric::PathDefragmenter::getChangedGroups(const std::vector<double> &from_,const std::vector<double> &to_){
    std::vector<int> groups;
    for(size_t i = 0 ; i < fragment_indices.size(); i++){
        for(auto index : fragment_indices.at(i)){
            if(abs(from_.at(index)-to_.at(index))  > 0.0001){
                groups.push_back(i);
                break;
            }
        }
    }
    return groups;
}

void ompl::geometric::PathDefragmenter::buildIsoStates(const std::vector<double> &from_,const std::vector<double> &to_,
                                                       std::vector<int> &changed_index_groups,
                                                       std::vector<ompl::base::State* > &iso_ ) {
    std::vector<double> intermediate_st(from_.size());
    intermediate_st = from_;


    for (size_t v = 0; v < changed_index_groups.size(); v++) {
        for (int i: fragment_indices.at(changed_index_groups.at(v))) {
            intermediate_st[i] = to_[i];
        }
        base::State *temp = si_->getStateSpace()->allocState(); // todo mem leak?
        si_->getStateSpace()->copyFromReals(temp, intermediate_st);
        iso_.push_back(temp);
    }
}


void ompl::geometric::PathDefragmenter::isolateStates(const base::State* rfrom, const base::State* rto, std::vector<ompl::base::State*> &iso_){

    std::vector<double> from_,to_;
    si_->getStateSpace()->copyToReals(from_,rfrom);
    si_->getStateSpace()->copyToReals(to_,rto);

    std::vector<int> groups = getChangedGroups(from_,to_);
    buildIsoStates(from_,to_,groups,iso_);
}


// Isolate the multi-group motion from->to into a chain of single-group steps, moving one
// group at a time and collision-checking every step. The group order matters: moving group
// A before B may collide where B-before-A does not, so we try orderings until one is fully
// collision-free. Returns the intermediate states to insert (all but the final, which equals
// `to` and is already present in the path); returns false if no ordering is collision-free.
//
// This is deliberately backend-agnostic: it works on any state space (a plain
// RealVectorStateSpace as well as a FactoredStateSpace) because the factoring is expressed
// here via the group indices, not delegated to the space's interpolate().
bool ompl::geometric::PathDefragmenter::isolateChecked(const base::State *from, const base::State *to,
                                                       std::vector<base::State *> &out)
{
    std::vector<double> fromV, toV;
    si_->getStateSpace()->copyToReals(fromV, from);
    si_->getStateSpace()->copyToReals(toV, to);

    std::vector<int> order = getChangedGroups(fromV, toV);   // ascending group indices
    if (order.empty())
        return false;

    // getChangedGroups returns the groups sorted, so next_permutation enumerates all of
    // them. Cap the factorial cost: for many simultaneously-changing groups just try the
    // natural order (such motions are rare and usually already collision-free).
    const bool tryAllOrders = order.size() <= 4;
    do
    {
        std::vector<double> cur = fromV;
        const base::State *prev = from;
        std::vector<base::State *> chain;
        bool ok = true;
        for (int g : order)
        {
            for (int idx : fragment_indices.at(g))
                cur[idx] = toV[idx];
            base::State *s = si_->allocState();
            si_->getStateSpace()->copyFromReals(s, cur);
            if (!si_->checkMotion(prev, s))
            {
                si_->freeState(s);
                ok = false;
                break;
            }
            chain.push_back(s);
            prev = s;
        }
        if (ok && !chain.empty())
        {
            si_->freeState(chain.back());   // == to, already in the path
            chain.pop_back();
            out = chain;
            return true;
        }
        for (base::State *s : chain)
            si_->freeState(s);
    } while (tryAllOrders && std::next_permutation(order.begin(), order.end()));

    return false;
}


void ompl::geometric::PathDefragmenter::getIntermediateState(const ompl::base::State *from,const ompl::base::State * to, ompl::base::State *state, int index_group)
{
    std::vector<double> s_new_v,to_;

    si_->getStateSpace()->copyToReals(s_new_v,from);
    si_->getStateSpace()->copyToReals(to_,to);

    std::vector<int> group = fragment_indices.at(index_group);
    for(size_t i = 0; i< group.size(); i++){
        s_new_v[group.at(i)] = to_[group.at(i)];
    }

    si_->getStateSpace()->copyFromReals(state,s_new_v);
}


int ompl::geometric::PathDefragmenter::getChangedIndex(const ompl::base::State *from,const ompl::base::State * to){

    // setze voraus dass cost 1 zw. from und to, keine equal states

    std::vector<double> s_from,s_to;
    si_->getStateSpace()->copyToReals(s_from,from);
    si_->getStateSpace()->copyToReals(s_to,to);

    for(size_t i = 0; i < fragment_indices.size() ; i++)
    {
        for(auto const &index_in_gr : fragment_indices.at(i))
        {
            if(abs(s_from.at(index_in_gr) - s_to.at(index_in_gr)) > 1e-10)
                return i;
        }
    }
    return -1;
}


void ompl::geometric::PathDefragmenter::freeStates(std::vector<ompl::base::State*> &states)
{
    for(auto &st : states)
        si_->freeState(st);
    states.clear();
}

void ompl::geometric::PathDefragmenter::freeStates(std::vector<std::pair<ompl::base::State *,int >> &states)
{
    for(auto &st : states)
        si_->freeState(st.first);
    states.clear();
}

bool ompl::geometric::PathDefragmenter::reConnect(ompl::base::State *from,
                                                  std::vector<std::pair<ompl::base::State *,int >> &queue_,
                                                  std::vector<ompl::base::State * > &rewireResult )
{
    for(size_t i = 0; i < queue_.size(); i++)
    {
        ompl::base::State * newState = si_->allocState(); // todo mem leak
        getIntermediateState(from,queue_.at(i).first,newState,queue_.at(i).second);
        if(!si_->checkMotion(from,newState))
        {
            si_->freeState(newState);
            return false;
        }

        rewireResult.push_back(newState);
        from = newState;
    }
    return true;
}


void ompl::geometric::PathDefragmenter::reConnect(ompl::base::State *from, std::vector<std::pair<ompl::base::State *,int >> &prio_,
                                                  std::vector<std::pair<ompl::base::State *,int >> &stack_,
                                                  std::vector<ompl::base::State *> &rewireResult) {

    if (reConnect(from, prio_, rewireResult)) {
        ompl::base::State * temp_from = si_->getStateSpace()->allocState();
        si_->getStateSpace()->copyState(temp_from,rewireResult.back());

        if (reConnect(temp_from, stack_, rewireResult)) { // reconnection successful
            si_->freeState(temp_from);
        }
        else{ // collision during reconnection
            si_->freeState(temp_from);
            freeStates(rewireResult);
        }
    }else{ // collision during first reconnection
        freeStates(rewireResult);
    }

    // all states newly allocated delete old
    freeStates(prio_);
    freeStates(stack_);
}

void ompl::geometric::PathDefragmenter::findNextFragment(int start_index, int prev_index, std::vector<ompl::base::State *> &mainPath, bool sameFragmentType,
                                                         std::vector<std::pair<ompl::base::State *, int>> &foundFragment)
{
    if(sameFragmentType)
    {
        // Bound is size(), not size()-1: the merge fragment may be the path's trailing
        // fragment, whose last state is the goal state at index size()-1. Excluding it
        // would truncate that fragment (dropping the goal), so a final-fragment merge
        // could never succeed and the path kept one redundant action.
        for (size_t i = start_index; i < mainPath.size(); i++) {
            if (getChangedIndex(mainPath.at(i - 1), mainPath.at(i)) == prev_index) {

                ompl::base::State *xstate = si_->getStateSpace()->allocState();
                si_->getStateSpace()->copyState(xstate, mainPath.at(i));
                foundFragment.push_back(std::make_pair(xstate, getChangedIndex(mainPath.at(i - 1), mainPath.at(i))));
            }
            else
                return;
        }
    }
    else
    {
        for (size_t i = start_index; i < mainPath.size() - 1; i++) {
            if (getChangedIndex(mainPath.at(i - 1), mainPath.at(i)) != prev_index) {

                ompl::base::State *xstate = si_->getStateSpace()->allocState();
                si_->getStateSpace()->copyState(xstate, mainPath.at(i));
                foundFragment.push_back(std::make_pair(xstate, getChangedIndex(mainPath.at(i - 1), mainPath.at(i))));
            }
            else
                return;
        }
    }

}

int ompl::geometric::PathDefragmenter::pathDefrag(std::vector<ompl::base::State *> &mainPath) {

    int rewireCount = 0;
    int prev_index = getChangedIndex(mainPath.at(0),mainPath.at(1));

    for(size_t toID = 1; toID < mainPath.size()-1; ++toID){

        size_t fromID = toID-1;

        ompl::base::State *fromState = mainPath.at(fromID);
        ompl::base::State  *toState = mainPath.at(toID);

        if(getChangedIndex(fromState,toState) != prev_index)
        {
            size_t subsearch_id = toID;
            std::vector<std::pair<ompl::base::State *,int>> merge_fragment;
            std::vector<std::pair<ompl::base::State *,int>> push_fragment;

            findNextFragment(subsearch_id,prev_index,mainPath,false,push_fragment);

            if(!push_fragment.empty())
                subsearch_id += push_fragment.size();
            else{
                prev_index = getChangedIndex(mainPath.at(toID-1),mainPath.at(toID));
                continue;
            }

            findNextFragment(subsearch_id,prev_index,mainPath, true, merge_fragment);

            if(merge_fragment.empty()){
                freeStates(push_fragment);

                prev_index = getChangedIndex(mainPath.at(toID-1),mainPath.at(toID));
                continue;
            }

            if(subsearch_id >= mainPath.size())
                OMPL_ERROR("SUB SEARCH ID OVER SIZE ERR"); // should never occur

            std::vector<ompl::base::State * > rewiredConnection;

            reConnect(fromState,merge_fragment,push_fragment,rewiredConnection);

            if(rewiredConnection.empty()){ // no rewiring possible because of invalid motion but no problem, just start with next the next state and go on
                prev_index = getChangedIndex(mainPath.at(toID-1),mainPath.at(toID));
                continue;
            }

            rewireCount+=rewiredConnection.size();
            int l = 0;


            for(size_t j = toID; j < toID+rewiredConnection.size(); j++)
                si_->freeState(mainPath.at(j));

            int end_index = mainPath.size()-toID  -rewiredConnection.size();
            mainPath.erase(mainPath.begin()+toID, mainPath.end()-end_index);

            for(size_t j = toID; j <toID+rewiredConnection.size(); j++)
            {
                mainPath.insert(mainPath.begin()+j,rewiredConnection.at(l));
                l++;
            }

            fromID = fromID+merge_fragment.size(); //
            if(toID>= mainPath.size()-1)
                return rewireCount;
            prev_index = getChangedIndex(mainPath.at(fromID),mainPath.at(fromID+1));
            toID = fromID+1;
        }

    }
    return rewireCount;

}

int ompl::geometric::PathDefragmenter::getCostPath(std::vector<ompl::base::State * > &states_)
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



void ompl::geometric::PathDefragmenter::getFragmentIDs(std::vector<ompl::base::State*> &path,std::vector<FragmentIntervals> &fragmentIDs, bool goalFragment = false)
{
    fragmentIDs.clear();
    int prev_index = getChangedIndex(path.at(0),path.at(1));

    for(size_t i = 1; i < path.size()-1; i++)
    {
        if(prev_index != getChangedIndex(path.at(i),path.at(i+1)))
        {
            // check for equal states, that may occur if we skip fragments
            if(si_->equalStates(path.at(i),path.at(i+1))){
                // equal states can appear when fragments are skipped; drop the duplicate
                path.erase(path.begin()+(i+1));
                continue;
            }
            if(fragmentIDs.size()>0)
                fragmentIDs.push_back(FragmentIntervals(fragmentIDs.back().end_index+1,i, prev_index));
            else
                fragmentIDs.push_back(FragmentIntervals(0,i, prev_index));

            prev_index = getChangedIndex(path.at(i),path.at(i+1));
        }
    }

    // The trailing run is its own fragment. Guard against an empty fragmentIDs:
    // a path whose transitions never change group has a single fragment, and
    // dereferencing fragmentIDs.back() here would be undefined behaviour.
    if(fragmentIDs.empty())
        fragmentIDs.push_back(FragmentIntervals(0, path.size()-1, prev_index));
    else if(prev_index != fragmentIDs.back().id)
        fragmentIDs.push_back(FragmentIntervals(fragmentIDs.back().end_index+1,path.size()-1, prev_index));

    if(goalFragment)
    {
        std::vector<FragmentIntervals> goalfrags;
        for(size_t i = 0; i < fragmentIDs.size(); i++)
        {
            if(fragmentIDs.at(i).id == goalIndex_)
                goalfrags.push_back(fragmentIDs.at(i));
        }
        fragmentIDs = goalfrags;
    }
}

void ompl::geometric::PathDefragmenter::getFragment(int start_index, int end_index,
                                                    std::vector<ompl::base::State*> &mainPath, std::vector<std::pair<ompl::base::State *,int>> &fragment)
{
    for(int j = start_index; j< end_index; j++){
        ompl::base::State * temp = si_->allocState();
        si_->getStateSpace()->copyState(temp,mainPath.at(j));
        fragment.push_back(std::make_pair(temp, getChangedIndex(mainPath.at(j-1),mainPath.at(j))));
    }
}


void ompl::geometric::PathDefragmenter::skipFragments(std::vector<ompl::base::State*> &mainPath)
{
    if(mainPath.size() < 3)
        return;
    std::vector<FragmentIntervals> fragmentIDs;
    getFragmentIDs(mainPath,fragmentIDs);

    // i + 1 < size() (never size() - 1): a single-fragment path has size() == 1,
    // and the unsigned subtraction would wrap around and run the loop out of range.
    for(size_t i = 0;  i + 1 < fragmentIDs.size();  i++)
    {
        std::vector<std::pair<ompl::base::State *,int>> pathA;
        std::vector<std::pair<ompl::base::State *,int>> pathB;
        std::vector<ompl::base::State *> result_;

        // The reconnection replays the surviving fragments starting from the real
        // path start. Using a copy of mainPath[0] (not a zeroed state) is essential:
        // it becomes the first state of the rewired path, so a zeroed state would
        // silently corrupt the start configuration.
        ompl::base::State *temp_from = si_->getStateSpace()->allocState();
        si_->getStateSpace()->copyState(temp_from, mainPath.at(0));

        if(i != 0){
            getFragment(1, fragmentIDs.at(i).start_index, mainPath, pathA);
            getFragment(fragmentIDs.at(i).end_index + 1, mainPath.size(), mainPath, pathB);

            if (pathA.size() < 1 || pathB.size() < 1) {
                freeStates(pathA);
                freeStates(pathB);
                si_->freeState(temp_from);
                continue;
            }
            reConnect(temp_from, pathA, pathB, result_);

        }
        else
        {
            getFragment(fragmentIDs.at(i).end_index+1,mainPath.size(),mainPath,pathB);
            if(pathB.size()<1){
                si_->freeState(temp_from);
                continue;
            }
            pathB.front().second = getChangedIndex(mainPath.at(fragmentIDs.at(i).end_index),pathB.front().first);

            if(!reConnect(temp_from,pathB,result_)){ // reconnect only one path segment directly to the state
                freeStates(pathB);
                freeStates(result_);
                si_->freeState(temp_from);
                continue;
            }
            freeStates(pathB);
        }

        // A fragment may only be skipped if it is not mandatory to reach the goal:
        // the rewired path must still end at the goal configuration. Otherwise the
        // skip drops a goal-reaching motion and must be rejected.
        if(!result_.empty() && goalState_ != nullptr && !si_->equalStates(result_.back(), goalState_))
        {
            freeStates(result_);
            si_->freeState(temp_from);
            continue;
        }

        if(!result_.empty())
        {
            freeStates(mainPath);

            mainPath = result_;
            mainPath.insert(mainPath.begin(),temp_from);
            getFragmentIDs(mainPath,fragmentIDs);

            if(mainPath.size() < 3)
                return;
        }
        else
            si_->freeState(temp_from);
        // no reconnection / jump over possible
    }

}


void ompl::geometric::PathDefragmenter::trySkipFragment(std::vector<ompl::base::State *> &mainPath) {


    size_t prev_size;

    do{
        prev_size = mainPath.size();
        skipFragments(mainPath);
    }while(mainPath.size() < prev_size );

}


void ompl::geometric::PathDefragmenter::simplifyActionIntervals(std::vector<ompl::base::State*> &mainPath) // finds intervals of same action index, builds shortcuts
{
    const size_t n = mainPath.size();
    if(n < 3)
        return;

    // Keep the endpoints and every group-transition boundary. Within a same-group run
    // the intermediate waypoints are candidates for removal (one run = one action), but
    // collapsing them straightens the run start -> run end, which can cut the corner of
    // an obstacle detour. We therefore keep a run's interior unless the straight
    // shortcut between its boundaries is collision-free.
    std::vector<char> keep(n, 0);
    keep[0] = 1;
    keep[n - 1] = 1;

    int prev_index = getChangedIndex(mainPath.at(0), mainPath.at(1));
    for(size_t i = 1; i + 1 < n; ++i)
    {
        int idx = getChangedIndex(mainPath.at(i), mainPath.at(i + 1));
        if(idx != prev_index)
        {
            keep[i] = 1;
            prev_index = idx;
        }
    }

    std::vector<size_t> boundaries;
    for(size_t i = 0; i < n; ++i)
        if(keep[i])
            boundaries.push_back(i);

    for(size_t k = 0; k + 1 < boundaries.size(); ++k)
    {
        size_t a = boundaries.at(k);
        size_t b = boundaries.at(k + 1);
        if(b > a + 1 && !si_->checkMotion(mainPath.at(a), mainPath.at(b)))
            for(size_t j = a + 1; j < b; ++j)
                keep[j] = 1;   // shortcut would collide: keep the detour waypoints
    }

    std::vector<ompl::base::State *> simplifiedPath;
    for(size_t i = 0; i < n; ++i)
    {
        if(keep[i])
            simplifiedPath.push_back(mainPath.at(i));
        else
            si_->freeState(mainPath.at(i));
    }

    mainPath = simplifiedPath;
}


void ompl::geometric::PathDefragmenter::cutOffIfGoalReached(std::vector<ompl::base::State*> &mainPath)
{
    // If the path passes through the goal configuration before its end, every state
    // afterwards is a redundant excursion and can be dropped. We compare against the
    // captured goal state (the full configuration), not a single goal index: the old
    // index-based test removed the final move of every non-goal-index group, which on
    // a full rearrangement problem truncates objects short of their goal.
    if(goalState_ == nullptr || mainPath.size() < 3)
        return;

    for(size_t i = 1; i + 1 < mainPath.size(); i++)
    {
        if(si_->equalStates(mainPath.at(i), goalState_))
        {
            for(size_t j = i + 1; j < mainPath.size(); j++)
                si_->freeState(mainPath.at(j));
            mainPath.erase(mainPath.begin() + i + 1, mainPath.end());
            return;
        }
    }
}

void ompl::geometric::PathDefragmenter::startPathDefrag(std::vector<ompl::base::State *> &path)
{

    int counter =0;
    base::Cost maxCost_(getCostPath(path));

    while(true){
        pathDefrag(path);
        base::Cost temp_cost(getCostPath(path));
        if(temp_cost.value() <= maxCost_.value())
        {
            if(temp_cost.value() == maxCost_.value())
                counter++;
            if(counter ==2) // if the cost stays the same twice,
                return;
            maxCost_ = temp_cost;
        }
        else
            return;
    }

}

void ompl::geometric::PathDefragmenter::checkRepairPath(std::vector<ompl::base::State *> &path_)
{

    for(size_t i = 0; i + 1 < path_.size(); )
    {
        if(si_->equalStates(path_.at(i),path_.at(i+1)))
        {
            // Drop the duplicate (keep path_[i] so the start state is never removed).
            si_->freeState(path_.at(i+1));
            path_.erase(path_.begin()+i+1);
            continue;
        }

        if(opt_->motionCost(path_.at(i),path_.at(i+1)).value()> 1.0) // path defrag needs every edge to have motion cost 1.0
        {
            // Split the multi-group edge into collision-free single-group steps. If no
            // ordering is collision-free, keep the (planner-validated) combined edge rather
            // than inserting a colliding isolated path -- so checkRepairPath never makes the
            // path collide, on any state space.
            std::vector<ompl::base::State *> iso_;
            if(isolateChecked(path_.at(i),path_.at(i+1),iso_) && !iso_.empty())
            {
                path_.insert(path_.begin()+i+1,iso_.begin(),iso_.end());
                i += iso_.size() + 1;
                continue;
            }
        }
        ++i;
    }
}



std::vector<ompl::base::State *> ompl::geometric::PathDefragmenter::clonePath(const std::vector<ompl::base::State *> &path)
{
    std::vector<ompl::base::State *> copy;
    copy.reserve(path.size());
    for(auto *s : path)
    {
        ompl::base::State *c = si_->allocState();
        si_->copyState(c, s);
        copy.push_back(c);
    }
    return copy;
}

bool ompl::geometric::PathDefragmenter::isPathValid(const std::vector<ompl::base::State *> &path,
                                                    const ompl::base::State *from, const ompl::base::State *to)
{
    if(path.size() < 2)
        return false;
    if(!si_->equalStates(path.front(), from) || !si_->equalStates(path.back(), to))
        return false;
    for(size_t i = 0; i + 1 < path.size(); ++i)
        if(!si_->checkMotion(path.at(i), path.at(i + 1)))
            return false;
    return true;
}

void ompl::geometric::PathDefragmenter::doPathDefragComplete(std::vector<ompl::base::State *> &path_)
{
    checkRepairPath(path_);

    if(path_.size() < 2)
        return;

    // Capture the true endpoints. Every optimisation below must preserve them and keep
    // the path collision-free; the guard rolls a stage back if it ever does not, so a
    // buggy or degenerate stage can never truncate the goal or reintroduce a collision.
    startState_ = si_->allocState();
    si_->copyState(startState_, path_.front());
    goalState_ = si_->allocState();
    si_->copyState(goalState_, path_.back());

    std::vector<ompl::base::State *> snapshot = clonePath(path_);   // last known-valid path

    // The baseline we fall back to must itself be valid. checkRepairPath() now isolates
    // multi-group edges with per-step collision checking, so this should not fire; keep it
    // as a backstop that warns (rather than silently returning a bad path) if it ever does.
    if(!isPathValid(snapshot, startState_, goalState_))
        OMPL_WARN("PathDefragmenter: path is not collision-free after repair; "
                  "defragmentation may return an invalid path");

    // Accept a stage's output only if it preserves the endpoints, stays collision-free
    // AND does not increase the action count; otherwise roll the stage back. This makes
    // the wrapper enforce all three invariants the defragmentation must keep.
    auto guard = [&]()
    {
        if(isPathValid(path_, startState_, goalState_) && getCostPath(path_) <= getCostPath(snapshot))
        {
            freeStates(snapshot);
            snapshot = clonePath(path_);
        }
        else
        {
            freeStates(path_);
            path_ = clonePath(snapshot);
        }
    };

    // Run the whole simplification pipeline repeatedly until the action count no longer
    // drops: a later stage (e.g. simplifyActionIntervals) can re-enable an earlier one
    // (e.g. trySkipFragment), so a single pass is not a fixed point. guard() guarantees the
    // action count is monotone non-increasing, so this loop terminates (cost is bounded below).
    int prevCost;
    do
    {
        prevCost = getCostPath(path_);
        startPathDefrag(path_);          guard();
        cutOffIfGoalReached(path_);      guard();
        trySkipFragment(path_);          guard();
        simplifyActionIntervals(path_);  guard();
    } while (getCostPath(path_) < prevCost);

    freeStates(snapshot);
    si_->freeState(startState_); startState_ = nullptr;
    si_->freeState(goalState_);  goalState_ = nullptr;
}
