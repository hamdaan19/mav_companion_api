#include <iostream>
#include <ompl/config.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ros/ros.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

void setStart(geometry_msgs::PoseStamped data);
void setGoal();

class Planner {
    public:
        Planner() {
            space = std::make_shared<ob::SE3StateSpace>();

            ob::RealVectorBounds bounds(3);

            bounds.setLow(0, -20);
            bounds.setHigh(0, 20);
            bounds.setLow(1, -20);
            bounds.setHigh(1, 20);
            bounds.setLow(2, 0);
            bounds.setHigh(2, 20);

            // create a start state
            ob::ScopedState<ob::SE3StateSpace> start(space);
            
            // create a goal state
            ob::ScopedState<ob::SE3StateSpace> goal(space);

            space->setBounds(bounds);

            // Defining space information
            si = ob::SpaceInformationPtr(std::make_shared<ob::SpaceInformation>(space));

            // Initializing your Simple Setup Object
            ss = og::SimpleSetup(si);

            // Set start and goal points
            start->setXYZ(0, 0, 0);
            start->as<ob::SO3StateSpace::StateType>(1)->setIdentity(); // Sets rotation to identity
            goal->setXYZ(0, 0, 0); 
            goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity(); // Sets rotation to identity

            ss.setStateValidityChecker([&](const ob::State *state) { return isStateValid(state); }); // Hard to understand this line of code

            ss.setStartAndGoalStates(start, goal);
        }

        void setStart(double x, double y, double z) {
            ob::ScopedState<ob::SE3StateSpace> start(space);
            start->setXYZ(x,y,z);
            start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
            ss.setStartState(start);
        }

        void setGoal(double x, double y, double z) {
            ob::ScopedState<ob::SE3StateSpace> goal(space);
            goal->setXYZ(x,y,z);
            goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
            ss.setGoalState(goal);
        }

        void plan(){
            const ob::PlannerPtr planner(std::make_shared<og::PRM>(si));

            ss.setPlanner(planner);
            ss.setup();
            ss.print();

            ob::PlannerStatus solved = ss.solve(1.0);

            if (solved) {
                std::cout << "Solution has been found: " << std::endl;
                ss.simplifySolution();
                ss.getSolutionPath().print(std::cout);
            } else 
                std::cout << "No solution found" << std::endl;
        }

    private: 
        ob::StateSpacePtr space; 
        ob::SpaceInformationPtr si; 

        og::SimpleSetup ss;

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
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_planner_node");
    ros::NodeHandle n; 

    Planner planner; 

    ros::Subscriber odom_sub = n.subscribe("/mavros/imu/data", 10, boost::bind(&start_cb, _1, planner));
    ros::Subscriber goal_sub = n.subscribe("/map/marker_location/goal", 10, boost::bind(&goal_cb, _1, planner));

    ros::spin();
}

void start_cb(geometry_msgs::PoseStamped data, Planner *planner_obj) { 
    planner_obj->setStart(data.pose.position.x, data.pose.position.y, data.pose.position.z);
}

void goal_cb(geometry_msgs::Point data, Planner *planner_obj) {
    planner_obj->setGoal(data.x, data.y, data.z);
    planner_obj->plan();
}