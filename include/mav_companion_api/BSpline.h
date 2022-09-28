#pragma once 

#include <iostream> 
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <unsupported/Eigen/Splines>
#include <Eigen/Dense>

typedef Eigen::Spline<double, 3, 5> Spline3D;
typedef Eigen::SplineFitting<Spline3D> SplineFitting3D;

class BSpline {
    public:

        Eigen::RowVectorXd getStamps(int lenOfArr);
        Eigen::Matrix<double, Eigen::Dynamic, 3> getWaypointMatrix(std::vector<std::vector<double>> wpList); 

        Spline3D fitSpline(std::vector<std::vector<double>> waypoints);

        double getPathLength(Spline3D s);

        double getChordLength(Eigen::Matrix<double, Eigen::Dynamic, 3> points);
        Eigen::VectorXd genParamVec1(Eigen::Matrix<double, Eigen::Dynamic, 3> points, double TCL);
        Eigen::VectorXd genParamVec2(int size); 



};