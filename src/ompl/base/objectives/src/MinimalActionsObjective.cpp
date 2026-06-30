/* Author: Servet Bora Bayraktar */

#include <ompl/base/objectives/MinimalActionsObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>


ompl::base::MinimalActionsObjective::MinimalActionsObjective(const SpaceInformationPtr &si, std::vector<std::vector<int>> group_indices)
        : ompl::base::OptimizationObjective(si), group_indices_(group_indices)
{
    description_ = "Minimal Actions";
}

ompl::base::Cost ompl::base::MinimalActionsObjective::stateCost(const State * /*s*/) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::MinimalActionsObjective::identityCost() const
{
    return Cost(0.0);
}
ompl::base::Cost ompl::base::MinimalActionsObjective::motionCost(const State *s1, const State *s2) const
{

    if(s1 == nullptr || s2 == nullptr) {
        return infiniteCost();
    }
    int action_cost = 0;
    const base::StateSpacePtr &space = si_->getStateSpace();
    std::vector<double> s1_vals,s2_vals;
    space->copyToReals(s1_vals,s1);
    space->copyToReals(s2_vals,s2);

    for(auto const &group: group_indices_){
        for(auto const &index_in_gr : group){
            if(abs(s1_vals.at(index_in_gr) - s2_vals.at(index_in_gr)) > 1e-10){
                action_cost++;
                break;
            }
        }
    }

    return Cost(double(action_cost));
}


ompl::base::Cost ompl::base::MinimalActionsObjective::motionCostHeuristic(const State *s1, const State *s2) const {

    return motionCost(s1,s2);
}