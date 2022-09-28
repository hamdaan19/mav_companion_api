#pragma once

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "mav_companion_api/TrajPlanner.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class PathPlan {
    public: 

        PathPlan();

        void initStart();
        void setGoal(double x, double y, double z);
        void setStates(mav_companion_api::TrajPlanner::Request req);
        std::vector<std::vector<double>> plan();
        void printMsg();

    private:
        ob::StateSpacePtr space; 
        ob::SpaceInformationPtr si; 
 
        ob::ProblemDefinitionPtr pdef;

        // Flag to store whether an initial start state has been set. 
        // Planning will not happen until this is set to true. 
        bool startInit = false; 

        std::vector<std::vector<double>> waypoints;
};