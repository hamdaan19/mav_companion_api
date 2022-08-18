#include <iostream>
#include <ompl/config.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>

#include <ros/ros.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class TrajPlan;
void start_cb(geometry_msgs::PoseStamped data);
void goal_cb(geometry_msgs::Point data); 

ros::Publisher traj_pub;

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

class TrajPlan {
    public:

        static double startX;
        static double startY;
        static double startZ;

        TrajPlan() {
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

            si->setStateValidityChecker(&isStateValid);
            
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

        void initStart() {
            startInit = true; 
            
        }

        void setGoal(double x, double y, double z) {
            if (!startInit) {
                ROS_WARN("Start State has not been set yet.");
                return;
            }

            // Set Start State
            ob::ScopedState<ob::SE3StateSpace> start(space);
            start->setXYZ(startX, startY, startZ);
            start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

            // Set Goal State
            ob::ScopedState<ob::SE3StateSpace> goal(space);
            goal->setXYZ(x,y,z);
            goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
            
            // Setting Start and Goal States
            pdef->setStartAndGoalStates(start, goal);
            
            if (startInit) {
                std::cout << "startX: " << startX << " startY: " << startY << " startZ: " << startZ << std::endl;
                std::cout << "goalX: " << x << " goalY: " << y << " goalZ: " << z << std::endl;
            }
        }

        void plan(){

            auto rrtC = std::make_shared<og::RRTConnect>(si);
            rrtC-> setRange(0.1);
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

                trajectory_msgs::MultiDOFJointTrajectory msg;
                trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg; 

                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "map";
                msg.joint_names.clear();
                msg.joint_names.push_back("iris");
                msg.points.clear();


                std::vector<ob::State *> states = traj->getStates();
                std::cout << "Number of states: " << traj->getStateCount() << std::endl;

                for (ob::State* state : states) {
                    const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

                    // extract the first component of the state and cast it to what we expect
                    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                    // extract the second component of the state and cast it to what we expect
                    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                    point_msg.time_from_start.fromSec(ros::Time::now().toSec());
                    point_msg.transforms.resize(1); // This function will %resize the %vector to the specified number of elements.
                    point_msg.transforms[0].translation.x = pos->values[0];
                    point_msg.transforms[0].translation.y = pos->values[1];
                    point_msg.transforms[0].translation.z = pos->values[2];

                    point_msg.transforms[0].rotation.x = rot->x;
                    point_msg.transforms[0].rotation.y = rot->y;
                    point_msg.transforms[0].rotation.z = rot->z;
                    point_msg.transforms[0].rotation.w = rot->w;

                    msg.points.push_back(point_msg);
                    std::cout << "X: " << pos->values[0] << "  Y: " << pos->values[1] << "  Z: " << pos->values[2] << std::endl;
                }

                traj_pub.publish(msg);

                

            } else 
                std::cout << "No solution found" << std::endl;
        }

    private: 
        ob::StateSpacePtr space; 
        ob::SpaceInformationPtr si; 
 
        ob::ProblemDefinitionPtr pdef;

        // Flag to store whether an initial start state has been set. 
        // Planning will not happen until this is set to true. 
        bool startInit = false; 

};

// Creating a global-scope object for TrajPlan class 
TrajPlan planner;
double TrajPlan::startX, TrajPlan::startY, TrajPlan::startZ;

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_planner_node");
    ros::NodeHandle n; 

    ros::Subscriber odom_sub = n.subscribe("/mavros/local_position/pose", 10, start_cb);
    ros::Subscriber goal_sub = n.subscribe("/map/marker_location/goal", 10, goal_cb);

    traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("trajectory/waypoints",1);

    ros::spin();
}


void start_cb(geometry_msgs::PoseStamped data) { 

    TrajPlan::startX = data.pose.position.x;
    TrajPlan::startY = data.pose.position.y;
    TrajPlan::startZ = data.pose.position.z;

    planner.initStart();
}

void goal_cb(geometry_msgs::Point data) {
    
    planner.setGoal(data.x,data.y,data.z);
    planner.plan();
}

