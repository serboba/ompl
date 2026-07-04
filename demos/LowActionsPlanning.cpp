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

// Minimal LA-RRT demo: a 3-dimensional factored state space with three single-index
// groups (objects). Each object must reach its goal value; LARRT minimizes the number of
// actions (per-object moves) needed to rearrange all three from start to goal.

#include <iostream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/FactoredStateSpace.h>
#include <ompl/geometric/planners/rrt/LARRT.h>
#include <ompl/base/objectives/MinimalActionsObjective.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


bool isStateValid(const ob::State *state){
    const auto *s = state->as<ob::FactoredStateSpace::StateType>();
    if(s->values[0] > 0.7 && s->values[0] < 1.1 && s->values[1] < 0.6){
        return false;
    }else if(s->values[0] > 1.3 && s->values[0] < 1.8 && s->values[2] > -0.5){
        return false;
    }else{
        return true;
    }
}



int main (int /*argc*/, char ** /*argv*/){

    std::vector<std::vector<int>> groups = {{0}, {1}, {2}};

    auto space = std::make_shared<ob::FactoredStateSpace>(groups);

    space->addDimension(0.0,2.0);

    space->addDimension(0.0,2.0);

    space->addDimension(-2.0,0.0);


    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);

    si->setStateValidityChecker([] (const ob::State *state) {return isStateValid(state);});

    og::SimpleSetup ss(si);

    ob::ScopedState<> start(space);
    start[0] = 0.0;
    start[1] = 0.0;
    start[2] = 0.0;

    ob::ScopedState<> goal(space);
    goal[0] = 2.0;
    goal[1] = 1.0;
    goal[2] = -1.0;

    ss.setStartAndGoalStates(start,goal);
    auto larrt = new og::LARRT(si,groups);

    ss.setPlanner(ob::PlannerPtr(larrt));

    ob::PlannerStatus solved = ss.solve(1.0);

    if(solved){
        std::cout<<"Found solution! Solution path:" << std::endl;
        ss.getSolutionPath().print(std::cout);
    }else{
        std::cout << "No solution." << std::endl;
    }
}