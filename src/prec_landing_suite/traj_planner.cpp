#include <iostream>

#include <ompl/geometric/planners/prm/PRM.h>

#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

#include "mav_companion_api/PathPlanner.h"


bool isStateValid(const ob::State *state){
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    // extract the first component of the state and cast it to what we expect
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    // extract the second component of the state and cast it to what we expect
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state defined by pos & rot

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}

void PathPlan::printMsg(){
    std::cout << "Hello World!" << std::endl;
}

PathPlan::PathPlan(){
    space = ob::StateSpacePtr(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);

    bounds.setLow(0, -20);
    bounds.setHigh(0, 20);
    bounds.setLow(1, -20);
    bounds.setHigh(1, 20);
    bounds.setLow(2, 0);
    bounds.setHigh(2, 50);

    space->as<ob::SE3StateSpace>()->setBounds(bounds);

    // Defining space information
    //si = ob::SpaceInformationPtr(std::make_shared<ob::SpaceInformation>(space));
    si = ob::SpaceInformationPtr(std::make_shared<ob::SpaceInformation>(space));

    si->setStateValidityChecker(isStateValid);
    
    pdef = ob::ProblemDefinitionPtr(std::make_shared<ob::ProblemDefinition>(si));
    // create a start state
    ob::ScopedState<ob::SE3StateSpace> start(space);
    
    // create a goal state
    ob::ScopedState<ob::SE3StateSpace> goal(space);

    // Set start and goal points
    start->setXYZ(0, 0, 0);
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity(); // Sets rotation to identity
    goal->setXYZ(0, 0, 0); 
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity(); // Sets rotation to identity

    // ss.setStartAndGoalStates(start, goal);
    pdef->setStartAndGoalStates(start, goal);
}

void PathPlan::setStates(mav_companion_api::TrajPlanner::Request req) {

    // Set Start State
    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setXYZ(req.start.x, req.start.y, req.start.z);
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    // Set Goal State
    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setXYZ(req.goal.x, req.goal.y, req.goal.z);
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
    
    // Clear Start and Goal States
    pdef->clearStartStates();
    pdef->clearGoal();
    
    // Setting Start and Goal States
    pdef->setStartAndGoalStates(start, goal);
    
}


std::vector<std::vector<double>> PathPlan::plan(){

    std::vector<std::vector<double>> waypointList;
    

    auto rrtC = std::make_shared<og::RRTConnect>(si);
    rrtC-> setRange(0.5);
    std::cout << "RANGE: " << rrtC->getRange() << std::endl;

    const ob::PlannerPtr planner(rrtC);

    planner->setProblemDefinition(pdef);
    planner->setup();

    si->printSettings(std::cout);
    pdef->print(std::cout);

    ob::PlannerStatus solved = planner->solve(1.0);

    if (solved) {
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Solution has been found: " << std::endl;
        path->print(std::cout);

        og::PathGeometric* traj = pdef->getSolutionPath()->as<og::PathGeometric>();
        traj->printAsMatrix(std::cout);

        std::vector<ob::State *> states = traj->getStates();
        std::cout << "Number of states: " << traj->getStateCount() << std::endl;

        for (std::size_t idx = 0; idx < traj->getStateCount(); idx++){    
            std::vector<double> waypoint;
            ob::State* state = traj->getState(idx);
            const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

            // extract the first component of the state and cast it to what we expect
            const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            // extract the second component of the state and cast it to what we expect
            const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

            waypoint.push_back(pos->values[0]);
            waypoint.push_back(pos->values[1]);
            waypoint.push_back(pos->values[2]);
            waypointList.push_back(waypoint);

        }
        pdef->clearSolutionPaths();

    } else {
        std::cout << "No solution found" << std::endl;}


    std::vector<std::vector<double>> wpL;
    double x=0, y=0, z=0, inc=0.5; 

    // for (int num = 1; num <= 2; num++){
    //     std::vector<double> vec; 
    //     vec.push_back(x);
    //     vec.push_back(y);
    //     vec.push_back(z);
    //     x += 0.01;
    //     y += 0.01;
    //     z += 0.01;
    //     wpL.push_back(vec); 
    // }

    for (double i = 0; i < 10; i++){
        std::vector<double> pt;
        pt.push_back(x);
        pt.push_back(y);
        pt.push_back(z);

        std::cout << "Pt: " << x << ", " << y << ", " << z << std::endl;

        x += inc*1; 
        y += inc*2;
        z += inc*2.5; 

        wpL.push_back(pt); 

    }
    return waypointList;
    //return wpL;
}

