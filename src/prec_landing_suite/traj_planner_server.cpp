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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <gsl/gsl_integration.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_deriv.h>

#include "mav_companion_api/PathPlanner.h"
#include "mav_companion_api/BSpline.h"
#include "mav_companion_api/TrajOpt.h"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

double MAX_VELOCITY_LIMIT = 2; // Velocity in ms^-1
double MAX_ACCELERATION_LIMIT = 1; // Acceleration in ms^-2 

int DEGREE_OF_TEMPORAL_CURVE = 3;
int NUMBER_OF_PIECES = 3; 
int NUMBER_OF_INTERVALS = NUMBER_OF_PIECES - 1; 

bool service_cb(mav_companion_api::TrajPlanner::Request &req, mav_companion_api::TrajPlanner::Response &res);

double objectiveFunc(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);
double monoConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);

double velXConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);
double velYConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);
double velZConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);

double accXConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);
double accYConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);
double accZConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);

double initTimeConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double minTimeConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);

double initVelocityXConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double initVelocityYConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double initVelocityZConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);

double pointContinuityConstraint1(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double pointContinuityConstraint2(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double pointContinuityConstraint3(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double pointContinuityConstraint4(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double pointContinuityConstraint5(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double pointContinuityConstraint6(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double pointContinuityConstraint7(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);

double grad1ContinuityConstraint1(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad1ContinuityConstraint2(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad1ContinuityConstraint3(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad1ContinuityConstraint4(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad1ContinuityConstraint5(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad1ContinuityConstraint6(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad1ContinuityConstraint7(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);

double grad2ContinuityConstraint1(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad2ContinuityConstraint2(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad2ContinuityConstraint3(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad2ContinuityConstraint4(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad2ContinuityConstraint5(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad2ContinuityConstraint6(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);
double grad2ContinuityConstraint7(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data);

void setPointContinuityConstraints(nlopt::opt* optimizer, std::vector<double> intervalVec, int numIntervals);
double pointContinuityConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data);

void pointConstraints(unsigned m, double *result, unsigned dim, const double* alpha, double* grad, void* more_data);
void grad1Constraints(unsigned m, double *result, unsigned dim, const double* alpha, double* grad, void* more_data);
void grad2Constraints(unsigned m, double *result, unsigned dim, const double* alpha, double* grad, void* more_data);

// Creating a global-scope object for PathPlan class 
PathPlan planner;
BSpline spline_obj;

typedef struct {
    Spline3D H;
    GFunc G;
} velData; 

typedef struct {
    std::vector<double> intervals; 
} intervalData; 

void plotSpline(Spline3D* H){
    std::vector<double> tVec, jerkX, jerkY, jerkZ;
}

void plotJerks(Spline3D* H, GFunc* G_obj){
    std::vector<double> tVec, jerkX, jerkY, jerkZ; 
    for(double tau = 0; tau <= 1; tau+=1e-2){
        auto deriv = H->derivatives(tau, 3);

        auto d1H = deriv.col(1);
        auto d2H = deriv.col(2);
        auto d3H = deriv.col(3);

        double D = pow(G_obj->gradG(tau), 3); 
        double gradD = 3*pow(G_obj->gradG(tau), 2)*G_obj->grad2G(tau);
        Eigen::Vector3d N = G_obj->gradG(tau)*d2H - d1H*G_obj->grad2G(tau);
        Eigen::Vector3d gradN = G_obj->gradG(tau)*d3H - d1H*G_obj->grad3G(tau);

        Eigen::Vector3d jerk = (1/G_obj->gradG(tau)) * (D*gradN - N*gradD)/pow(D, 2); 

        jerkX.push_back( jerk(0) );
        jerkY.push_back( jerk(1) );
        jerkZ.push_back( jerk(2) );
        tVec.push_back( G_obj->G(tau) );
    }
    
    plt::title("Jerk Plots");
    plt::suptitle("Temporal Curve: Piecewise Cubic");
    plt::named_plot("Jerk X", tVec, jerkX);
    plt::named_plot("Jerk Y", tVec, jerkY);
    plt::named_plot("Jerk Z", tVec, jerkZ);
    plt::xlabel("Time (s)");
    plt::ylabel("Jerk (ms^-3)");
    plt::legend();
    plt::show();
}

void plotVelocities(Spline3D* H, GFunc* G_obj){
    std::vector<double> tVec, velX, velY, velZ; 
    for(double tau = 0; tau <= 1; tau+=1e-4){
        auto deriv = H->derivatives(tau, 1);

        Eigen::Vector3d vel = deriv.col(1) * ( 1/G_obj->gradG(tau) );

        velX.push_back( vel(0) );
        velY.push_back( vel(1) );
        velZ.push_back( vel(2) );
        tVec.push_back( G_obj->G(tau) );
        //tVec.push_back( tau );
    }

    // push back final velocities for tau = 1
    auto derivF = H->derivatives(1, 1);
    Eigen::Vector3d velF = derivF.col(1) * ( 1/G_obj->gradG(1) );
    velX.push_back( velF(0) );
    velY.push_back( velF(1) );
    velZ.push_back( velF(2) );
    tVec.push_back( G_obj->G(1) );
    
    std::cout << "size of t: " << tVec.size() << std::endl;
    std::cout << "size of velX: " << velX.size() << std::endl;

    plt::title("Velocity Plots");
    plt::suptitle("Temporal Curve: Piecewise Cubic");
    plt::named_plot("Velocity X", tVec, velX);
    plt::named_plot("Velocity Y", tVec, velY);
    plt::named_plot("Velocity Z", tVec, velZ);
    plt::xlabel("Time (s)");
    plt::ylabel("Velocity (ms^-1)");
    plt::legend();
    plt::show();
}

void plotSplineDeriv(Spline3D* H, GFunc* G_obj){
    std::vector<double> tVec, velX, velY, velZ, numDerZ, num; 
    std::vector<double> accZ, accZ2, numVel, numAcc, jerkZ; 
    for(double tau = 0; tau <= 1; tau+=1e-3){
        auto deriv = H->derivatives(tau, 3);

        velX.push_back( deriv(0, 1) );
        velY.push_back( deriv(1, 1) );
        velZ.push_back( deriv(2, 1) * ( 1/G_obj->gradG(tau) ) );

        double accelZ = ( 1/G_obj->gradG(tau) ) * ( (deriv(2, 2)/G_obj->gradG(tau)) - ( (deriv(2, 1)*G_obj->grad2G(tau))/pow(G_obj->gradG(tau), 2) ) ); 
        accZ.push_back(accelZ);

        double accelZ2 = (1/pow(G_obj->gradG(tau), 3)) * (G_obj->gradG(tau)*deriv(2, 2) - deriv(2, 1)*G_obj->grad2G(tau));
        accZ2.push_back(accelZ2);

        double D = pow(G_obj->gradG(tau), 3); 
        double gradD = 3*pow(G_obj->gradG(tau), 2)*G_obj->grad2G(tau);
        double N = G_obj->gradG(tau)*deriv(2, 2) - deriv(2, 1)*G_obj->grad2G(tau);
        double gradN = G_obj->gradG(tau)*deriv(2, 3) - deriv(2, 1)*G_obj->grad3G(tau);

        double jZ = (1/G_obj->gradG(tau)) * (D*gradN - N*gradD)/pow(D, 2); 
        jerkZ.push_back(jZ);

        tVec.push_back( G_obj->G(tau) );
        //tVec.push_back( tau );
    }

    std::cout << "Acc: " << accZ.size() << " Vel: " << velZ.size() << " tVec: " << tVec.size() << std::endl;

    for (double tau = 0; tau <= 1-2e-3; tau += 1e-3){
        double tau1 = tau;
        double tau2 = tau + 1e-3; 
        double tau3 = tau + 2e-3;
        double t1 = G_obj->G(tau1);
        double t2 = G_obj->G(tau2);
        double t3 = G_obj->G(tau3);
        auto deriv1 = H->derivatives(tau1, 2);
        auto deriv2 = H->derivatives(tau2, 2);
        auto deriv3 = H->derivatives(tau3, 2);
        double p1 = deriv1(2, 0);
        double p2 = deriv2(2, 0);
        double p3 = deriv3(2, 0);
        double v1 = (p2-p1)/(t2-t1); 
        double v2 = (p3-p2)/(t3-t2);
        double a1 = (v2-v1)/(t2-t1);
        numVel.push_back(v1);
        numAcc.push_back(a1); 
        num.push_back(G_obj->G(tau));
    }

    // push back final velocities for tau = 1
    auto derivF = H->derivatives(1, 3);
    velX.push_back( derivF(0, 1) );
    velY.push_back( derivF(2, 0) );
    velZ.push_back( derivF(2, 1) * ( 1/G_obj->gradG(1) ) );

    double accelZFinal = ( 1/G_obj->gradG(1) ) * ( (derivF(2, 2)/G_obj->gradG(1)) - ( (derivF(2, 1)*G_obj->grad2G(1))/pow(G_obj->gradG(1), 2) ) );
    accZ.push_back(accelZFinal);

    double accelZ2Final = (1/pow(G_obj->gradG(1), 3)) * (G_obj->gradG(1)*derivF(2, 2) - derivF(2, 1)*G_obj->grad2G(1));
    accZ2.push_back(accelZ2Final);

    //Final Jerk Appending
    double D = pow(G_obj->gradG(1), 3); 
    double gradD = 3*pow(G_obj->gradG(1), 2)*G_obj->grad2G(1);
    double N = G_obj->gradG(1)*derivF(2, 2) - derivF(2, 1)*G_obj->grad2G(1);
    double gradN = G_obj->gradG(1)*derivF(2, 3) - derivF(2, 1)*G_obj->grad3G(1);

    double jZ = (1/G_obj->gradG(1)) * (D*gradN - N*gradD)/pow(D, 2); 
    jerkZ.push_back(jZ);

    tVec.push_back( G_obj->G(1) );
    
    std::cout << "size of t: " << tVec.size() << std::endl;
    std::cout << "size of velX: " << velX.size() << std::endl;
    //plt::named_plot("velX", tVec, velX);
    plt::plot(num, numVel, "r*");
    //plt::plot(num, numAcc, "^");
    plt::plot(tVec, jerkZ, "^");
    //plt::named_plot("velY", tVec, velY);
    plt::named_plot("velZ", tVec, velZ);
    //plt::named_plot("accZ", tVec, accZ, "^");
    plt::named_plot("accZ2", tVec, accZ2);
    plt::show();
}

void plotGDeriv(Spline3D* H, GFunc* G_obj){
    std::vector<double> tVec, velX, velY, velZ; 
    for(double tau = 0; tau <= 1; tau+=1e-3){
        auto deriv = H->derivatives(tau, 2);

        std::cout << "Vel: " << tau << std::endl << deriv << std::endl;

        velX.push_back(G_obj->gradG(tau) );
        //tVec.push_back( G_obj->G(tau) );
        tVec.push_back( tau );
    }

    // push back final velocities for tau = 1
    auto derivF = H->derivatives(1, 2);
    velX.push_back( G_obj->gradG(1) );
    //tVec.push_back( G_obj->G(1) );
    tVec.push_back( 1 );
    
    std::cout << "size of t: " << tVec.size() << std::endl;
    std::cout << "size of velX: " << velX.size() << std::endl;
    plt::named_plot("velX", tVec, velX);
    plt::legend();
    plt::show();
}

void plotAccels(Spline3D* H, GFunc* G_obj){
    std::vector<double> tVec, accX, accY, accZ; 
    for(double tau = 0; tau <= 1; tau+=1e-3){
        auto deriv = H->derivatives(tau, 2);

        Eigen::Vector3d d1H = deriv.col(1);
        Eigen::Vector3d d2H = deriv.col(2); 

        Eigen::Vector3d accel = (1/pow(G_obj->gradG(tau), 3)) * (G_obj->gradG(tau)*d2H - d1H*G_obj->grad2G(tau));

        accX.push_back( accel(0) ); 
        accY.push_back( accel(1) ); 
        accZ.push_back( accel(2) ); 
        tVec.push_back( G_obj->G(tau) );

    }
    
    plt::title("Acceleration Plots");
    plt::suptitle("Temporal Curve: Piecewise Cubic");
    plt::named_plot("Acceleration X", tVec, accX);
    plt::named_plot("Acceleration Y", tVec, accY);
    plt::named_plot("Acceleration Z", tVec, accZ);
    plt::xlabel("Time (s)");
    plt::ylabel("Acceleration (ms^-2)");
    plt::legend();
    plt::show();
}

void plotPositions(Spline3D* H, GFunc* G_obj){
    std::vector<double> tVec, posX, posY, posZ; 
    for(double tau = 0; tau <= 1; tau+=1e-2){

        posX.push_back( (*H)(tau)[0] );
        posY.push_back( (*H)(tau)[1] );
        posZ.push_back( (*H)(tau)[2] );
        tVec.push_back( G_obj->G(tau) );
        //tVec.push_back( tau );
    }
    
    plt::title("Position Plots");
    plt::suptitle("Temporal Curve: Piecewise Cubic");
    plt::named_plot("Position X", tVec, posX);
    plt::named_plot("Position Y", tVec, posY);
    plt::named_plot("Position Z", tVec, posZ);
    plt::xlabel("Time (s)");
    plt::ylabel("Position (m)");
    plt::legend();
    plt::show();
}

void plotTimes(GFunc* G_obj, std::vector<double> intVec){
    std::vector<double> tVec, tauVec;
    for (double tau = 0; tau <= 1; tau+=1e-2){
        tVec.push_back( G_obj->G(tau) );
        tauVec.push_back(tau);
    }

    std::vector<double> times;
    for (double i : intVec){
        times.push_back( G_obj->G(i) ); 
    }

    // Appending final values at tau=1
    tVec.push_back( G_obj->G(1) );
    tauVec.push_back(1);

    plt::title("Temporal Curve");
    plt::suptitle("Temporal Curve: Piecewise Cubic");
    plt::plot(tauVec, tVec);
    plt::plot(intVec, times, "r*");
    plt::xlabel("Tau ∈ [0,1]");
    plt::ylabel("Time (s)");
    plt::show();
}

int main(int argc, char **argv) {
    ROS_INFO("Starting Trajectory Planning Server.");
    ros::init(argc, argv, "TrajectoryPlannerServerr");
    ros::NodeHandle n; 

    //vis_pub = n.advertise<visualization_msgs::Marker>("trajectory/visualization/marker",1);
    ros::ServiceServer service = n.advertiseService("trajectory_planner", service_cb);

    ros::spin(); 
}


bool service_cb(mav_companion_api::TrajPlanner::Request &req, mav_companion_api::TrajPlanner::Response &res) {
    planner.setStates(req);
    
    trajectory_msgs::MultiDOFJointTrajectory traj_msg;
    
    std::vector<std::vector<double>> waypoints = planner.plan();

    if ( !waypoints.empty() ) { // if my waypoints varible is not empty
        
        Spline3D s = spline_obj.fitSpline(waypoints);

        std::cout << s.knots() << std::endl;
        //std::cout << s.ctrls() << std::endl;
        //std::cout << s.operator()(0.12) << std::endl;
        //std::cout << "Coefficients: " << s(0.12) << std::endl << std::endl;
        Eigen::Matrix<double, 3, 4> deriv = s.derivatives(0, 3);
        //std::cout << "Derivative Coef: " << std::endl << derivatives(0,2) << std::endl << std::endl;
        std::cout << "Derivative Coef: " << std::endl << deriv << std::endl;

        constraint_data data;
        data.H = s; 

        int dim = (DEGREE_OF_TEMPORAL_CURVE+1)*(NUMBER_OF_PIECES);

        std::vector<double> intervalVector; 
        for (double interval = 0; interval <= 1; interval += (1/(double)NUMBER_OF_PIECES)){
            intervalVector.push_back(interval);
            std::cout << interval << std::endl; 
        }

        std::vector<double> lb(dim);
        std::vector<double> ub(dim);
        for (int i = 0; i < dim; i++){
            lb[i] = -HUGE_VAL; 
            ub[i] = HUGE_VAL;  
        }

        
        // -------------------------------------------------
        std::cout << "Hello Hello \n"; 
        nlopt::opt opt(nlopt::LN_COBYLA, dim);
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
        opt.set_min_objective(objectiveFunc, &data);

        opt.add_inequality_constraint(monoConstraint, &data, 1e-6);
        opt.add_inequality_constraint(velXConstraint, &data, 1e-6);
        opt.add_inequality_constraint(velYConstraint, &data, 1e-6);
        opt.add_inequality_constraint(velZConstraint, &data, 1e-6);

        opt.add_inequality_constraint(accXConstraint, &data, 1e-6);
        opt.add_inequality_constraint(accYConstraint, &data, 1e-6);
        opt.add_inequality_constraint(accZConstraint, &data, 1e-6);

        opt.add_inequality_constraint(minTimeConstraint, &data, 1e-6);
        opt.add_equality_constraint(initTimeConstraint, &data, 1e-6);

        std::vector<double> tols(NUMBER_OF_INTERVALS);

        for (int i=0; i < NUMBER_OF_INTERVALS; i++) { tols.at(i) = 1e-3; }
        std::cout << "tols: " << tols.size() << std::endl; 
        opt.add_equality_mconstraint(pointConstraints, &intervalVector, tols); 
        opt.add_equality_mconstraint(grad1Constraints, &intervalVector, tols); 
        opt.add_equality_mconstraint(grad2Constraints, &intervalVector, tols); 

        opt.set_xtol_rel(1e-2);

        // Dynmically setting initial values for my optimization parameters
        std::vector<double> alpha(dim);
        int t = DEGREE_OF_TEMPORAL_CURVE-1;
        for (int i = 0; i < dim; i++){
            if (i == t){
                alpha[i] = 1;
                t = t+DEGREE_OF_TEMPORAL_CURVE+1;
            } else {
                alpha[i] = 0;
            }
        }

        double minf;

        // GFunc g(alpha, 3); 
        // std::cout << "G: " << g.grad2G(0) << std::endl; 


        try{
            nlopt::result result = opt.optimize(alpha, minf);
            std::cout << "found minimum" << std::endl;
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

        std::cout << "\nMin: " << minf << std::endl;

        GFunc optG(alpha, DEGREE_OF_TEMPORAL_CURVE); // optimized G
        plotTimes(&optG, intervalVector);
        plotPositions(&s, &optG);
        plotVelocities(&s, &optG);
        plotAccels(&s, &optG);
        plotJerks(&s, &optG);


    } else {
        ROS_WARN("Planner returned empty array.");
    }


    res.trajectory = traj_msg;
    return true;
}

/* 
_________________________________________________________________

 Writing my CONSTRAINT functions
_________________________________________________________________

*/ 

void pointConstraints(unsigned m, double *result, unsigned dim, const double* alpha, double* grad, void* more_data){
    
    std::vector<double>* data = reinterpret_cast<std::vector<double>*>(more_data);
    for (int interval = 1; interval <= NUMBER_OF_INTERVALS; interval++){

        double tau = data->at(interval); 

        double seg1 = 0;
        double seg2 = 0;

        int i = (DEGREE_OF_TEMPORAL_CURVE+1)*(interval-1);
        int j = (DEGREE_OF_TEMPORAL_CURVE+1)*(interval);

        for (int k = 0; k <= DEGREE_OF_TEMPORAL_CURVE; k++){
            seg1 += alpha[i+k]*pow(tau, DEGREE_OF_TEMPORAL_CURVE-k);
        }

        for (int k = 0; k <= DEGREE_OF_TEMPORAL_CURVE; k++){
            seg2 += alpha[j+k]*pow(tau, DEGREE_OF_TEMPORAL_CURVE-k);
        }

        result[interval-1] = seg1-seg2; 
    }

}

void grad1Constraints(unsigned m, double *result, unsigned dim, const double* alpha, double* grad, void* more_data){
    //intervalData* data = reinterpret_cast<intervalData*>(more_data);
    std::vector<double>* data = reinterpret_cast<std::vector<double>*>(more_data);
    for (int interval = 1; interval <= NUMBER_OF_INTERVALS; interval++){

        double tau = data->at(interval); 

        double seg1 = 0;
        double seg2 = 0;

        int i = (DEGREE_OF_TEMPORAL_CURVE+1)*(interval-1);
        int j = (DEGREE_OF_TEMPORAL_CURVE+1)*(interval);

        for (int k = 0; k <= DEGREE_OF_TEMPORAL_CURVE-1; k++){
            seg1 += (DEGREE_OF_TEMPORAL_CURVE-k)*alpha[i+k]*pow(tau, DEGREE_OF_TEMPORAL_CURVE-k-1);
        }

        for (int k = 0; k <= DEGREE_OF_TEMPORAL_CURVE-1; k++){
            seg2 += (DEGREE_OF_TEMPORAL_CURVE-k)*alpha[j+k]*pow(tau, DEGREE_OF_TEMPORAL_CURVE-k-1);
        }

        result[interval-1] = seg1-seg2; 
    }

}

void grad2Constraints(unsigned m, double *result, unsigned dim, const double* alpha, double* grad, void* more_data){
    //intervalData* data = reinterpret_cast<intervalData*>(more_data);
    std::vector<double>* data = reinterpret_cast<std::vector<double>*>(more_data);
    for (int interval = 1; interval <= NUMBER_OF_INTERVALS; interval++){

        double tau = data->at(interval); 

        double seg1 = 0;
        double seg2 = 0;

        int i = (DEGREE_OF_TEMPORAL_CURVE+1)*(interval-1);
        int j = (DEGREE_OF_TEMPORAL_CURVE+1)*(interval);

        for (int k = 0; k<=DEGREE_OF_TEMPORAL_CURVE-2; k++){
            seg1 += (DEGREE_OF_TEMPORAL_CURVE-k-1)*(DEGREE_OF_TEMPORAL_CURVE-k)*alpha[i+k]*pow(tau, DEGREE_OF_TEMPORAL_CURVE-k-2);  
        }

        for (int k = 0; k<=DEGREE_OF_TEMPORAL_CURVE-2; k++){
            seg2 += (DEGREE_OF_TEMPORAL_CURVE-k-1)*(DEGREE_OF_TEMPORAL_CURVE-k)*alpha[j+k]*pow(tau, DEGREE_OF_TEMPORAL_CURVE-k-2);  
        }


        result[interval-1] = seg1-seg2; 
    }

}


double monoConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data){
    // Constraint which ensures whether dt/dtau increases 'mono'tonically. 
    // std::cout << "Entering monoConstraint\n";
    if (!grad.empty()){
        // do something
    }

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    double result = G_obj.minimumGradG(); 

    //std::cout << "mono constraint: " << -1 * result << std::endl;
    return -1 * result; // NLopt always expects constraints to be of the form myconstraint(x) ≤ 0. 
}

double velXConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data){
    // Constraint which ensures my velocities over all tau (tau exists [0, 1]) remain within limits
    // std::cout << "Entering velX\n";

    if (!grad.empty()){
        // do something
    }

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    double result = G_obj.maximumPositionXGradient(data->H);

    // std::cout << "X velocity constraint: " << result - MAX_VELOCITY_LIMIT << std::endl; 
    return result - MAX_VELOCITY_LIMIT;
}

double velYConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data){
    // Constraint which ensures my velocities over all tau (tau exists [0, 1]) remain within limits
    // std::cout << "Entering velY\n";

    if (!grad.empty()){
        // do something
    }

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    double result = G_obj.maximumPositionYGradient(data->H);

    // std::cout << "Y velocity constraint: " << result - MAX_VELOCITY_LIMIT << std::endl;
    return result - MAX_VELOCITY_LIMIT;
}

double velZConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data){
    // Constraint which ensures my velocities over all tau (tau exists [0, 1]) remain within limits
    // std::cout << "Entering velZ\n";

    if (!grad.empty()){
        // do something
    }

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    double result = G_obj.maximumPositionZGradient(data->H);

    // std::cout << "Z velocity constraint: " << result - MAX_VELOCITY_LIMIT << std::endl;
    return result - MAX_VELOCITY_LIMIT;
}

double accXConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data){
    // Constraint which ensures my accelerations over all tau (tau exists [0, 1]) remain within limits
    if (!grad.empty()){
        // do something
        std::cout << "Hello"; 
    }

    // std::cout << "Entering accX\n";

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    double result = G_obj.maximumVelocityXGradient(data->H);

    //std::cout << "X accel constraint: " << result - MAX_VELOCITY_LIMIT << std::endl;
    // std::cout << "Exiting accX\n";

    return result - MAX_ACCELERATION_LIMIT; 
}

double accYConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data){
    // Constraint which ensures my accelerations over all tau (tau exists [0, 1]) remain within limits
    if (!grad.empty()){
        // do somethingstd::cout << "X accel constraint: " << result - MAX_VELOCITY_LIMIT;
        std::cout << "Hello"; 
    }

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    double result = G_obj.maximumVelocityYGradient(data->H);

    //std::cout << "Y accel constraint: " << result - MAX_VELOCITY_LIMIT << std::endl;
    return result - MAX_ACCELERATION_LIMIT; 
}

double accZConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data){
    // Constraint which ensures my accelerations over all tau (tau exists [0, 1]) remain within limits
    if (!grad.empty()){
        // do something
    }

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    double result = G_obj.maximumVelocityZGradient(data->H);

    //std::cout << "Z accel constraint: " << result - MAX_VELOCITY_LIMIT << std::endl;
    return result - MAX_ACCELERATION_LIMIT; 
}

double minTimeConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){
    
    // std::cout << "Entering minTime constraint\n";

    if (!grad.empty()){
        // do something
    }

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);  
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    Eigen::Vector3d diff_vec = data->H(0) - data->H(1);
    double dist = diff_vec.norm();
    double Tmin = dist/MAX_VELOCITY_LIMIT; 

    // std::cout << "Exiting minTime constraint\n";
    return Tmin - G_obj.G(1);
}

double initTimeConstraint(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){
    if (!grad.empty()){
        // do something
    }

    // std::cout << "Entering initTime constraint\n";

    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    //std::cout << "Init time: "<< G_obj.G(0) << std::endl; 
    // std::cout << "Exiting initTime constraint\n"; 
    return G_obj.G(0);
}

// double pointContinuityConstraint1(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
//     double seg1, seg2, seg2_, seg3, seg3_, seg4; 

//     std::cout << "Entered into POINT CONT\n";

//     seg1 = G_obj.alpha[0]*pow(0.25, 3) +  G_obj.alpha[1]*pow(0.25, 2) + G_obj.alpha[2]*0.25 + G_obj.alpha[3]; 
//     seg2 = G_obj.alpha[4]*pow(0.25, 3) +  G_obj.alpha[5]*pow(0.25, 2) + G_obj.alpha[6]*0.25 + G_obj.alpha[7];  

//     seg2_ = G_obj.alpha[4]*pow(0.5, 3) +  G_obj.alpha[5]*pow(0.5, 2) + G_obj.alpha[6]*0.5 + G_obj.alpha[7];
//     seg3 = G_obj.alpha[8]*pow(0.5, 3) +  G_obj.alpha[9]*pow(0.5, 2) + G_obj.alpha[10]*0.5 + G_obj.alpha[11];

//     seg3_ = G_obj.alpha[8]*pow(0.75, 3) +  G_obj.alpha[9]*pow(0.75, 2) + G_obj.alpha[10]*0.75 + G_obj.alpha[11];
//     seg4 = G_obj.alpha[12]*pow(0.75, 3) +  G_obj.alpha[13]*pow(0.75, 2) + G_obj.alpha[14]*0.75 + G_obj.alpha[15];

//     Eigen::Vector3d diff {seg1-seg2, seg2_-seg3, seg3_-seg4};
//     double d = diff.norm(); 
 

//     return d; 
// }

// double pointContinuityConstraint2(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = G_obj.b1*pow(0.25, 3) +  G_obj.b2*pow(0.25, 2) + G_obj.b3*0.25 + G_obj.b4; 
//     seg2 = G_obj.c1*pow(0.25, 3) +  G_obj.c2*pow(0.25, 2) + G_obj.c3*0.25 + G_obj.c4;  

//     return seg1 - seg2; 
// }

// double pointContinuityConstraint3(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg2 = G_obj.d1*pow(0.375, 3) +  G_obj.d2*pow(0.375, 2) + G_obj.d3*0.375 + G_obj.d4;  
//     seg1 = G_obj.c1*pow(0.375, 3) +  G_obj.c2*pow(0.375, 2) + G_obj.c3*0.375 + G_obj.c4;  

//     return seg1 - seg2; 
// }

// double pointContinuityConstraint4(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = G_obj.d1*pow(0.5, 3) +  G_obj.d2*pow(0.5, 2) + G_obj.d3*0.5 + G_obj.d4;  
//     seg2 = G_obj.e1*pow(0.5, 3) +  G_obj.e2*pow(0.5, 2) + G_obj.e3*0.5 + G_obj.e4;  

//     return seg1 - seg2; 
// }

// double pointContinuityConstraint5(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 
 
//     seg1 = G_obj.e1*pow(0.625, 3) +  G_obj.e2*pow(0.625, 2) + G_obj.e3*0.625 + G_obj.e4; 
//     seg2 = G_obj.f1*pow(0.625, 3) +  G_obj.f2*pow(0.625, 2) + G_obj.f3*0.625 + G_obj.f4;  

//     return seg1 - seg2; 
// }

// double pointContinuityConstraint6(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 
 
//     seg1 = G_obj.f1*pow(0.75, 3) +  G_obj.f2*pow(0.75, 2) + G_obj.f3*0.75 + G_obj.f4; 
//     seg2 = G_obj.g1*pow(0.75, 3) +  G_obj.g2*pow(0.75, 2) + G_obj.g3*0.75 + G_obj.g4;  

//     return seg1 - seg2; 
// }

// double pointContinuityConstraint7(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 
 
//     seg1 = G_obj.g1*pow(0.875, 3) +  G_obj.g2*pow(0.875, 2) + G_obj.g3*0.875 + G_obj.g4; 
//     seg2 = G_obj.h1*pow(0.875, 3) +  G_obj.h2*pow(0.875, 2) + G_obj.h3*0.875 + G_obj.h4;  

//     return seg1 - seg2; 
// }

// double grad1ContinuityConstraint1(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
//     double seg1, seg2; 

//     seg1 = 3*G_obj.alpha[0]*pow(0.125, 2) + 2*G_obj.alpha[1]*0.125 + G_obj.alpha[2];
//     seg2 = 3*G_obj.alpha[4]*pow(0.125, 2) + 2*G_obj.alpha[5]*0.125 + G_obj.alpha[6];  

//     std::cout << "First Derivative Diff: " << abs(seg1 - seg2) << std::endl;  

//     return seg1 - seg2; 
// }

// double grad1ContinuityConstraint2(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 3*G_obj.b1*pow(0.25, 2) + 2*G_obj.b2*0.25 + G_obj.b3; 
//     seg2 = 3*G_obj.c1*pow(0.25, 2) + 2*G_obj.c2*0.25 + G_obj.c3;  

//     return seg1 - seg2; 
// }

// double grad1ContinuityConstraint3(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 3*G_obj.c1*pow(0.375, 2) + 2*G_obj.c2*0.375 + G_obj.c3; 
//     seg2 = 3*G_obj.d1*pow(0.375, 2) + 2*G_obj.d2*0.375 + G_obj.d3;  

//     return seg1 - seg2; 
// }

// double grad1ContinuityConstraint4(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 3*G_obj.d1*pow(0.5, 2) + 2*G_obj.d2*0.5 + G_obj.d3;  
//     seg2 = 3*G_obj.e1*pow(0.5, 2) + 2*G_obj.e2*0.5 + G_obj.e3; 

//     return seg1 - seg2; 
// }

// double grad1ContinuityConstraint5(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 3*G_obj.e1*pow(0.625, 2) + 2*G_obj.e2*0.625 + G_obj.e3;  
//     seg2 = 3*G_obj.f1*pow(0.625, 2) + 2*G_obj.f2*0.625 + G_obj.f3; 

//     return seg1 - seg2; 
// }

// double grad1ContinuityConstraint6(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 3*G_obj.f1*pow(0.75, 2) + 2*G_obj.f2*0.75 + G_obj.f3;  
//     seg2 = 3*G_obj.g1*pow(0.75, 2) + 2*G_obj.g2*0.75 + G_obj.g3; 

//     return seg1 - seg2; 
// }

// double grad1ContinuityConstraint7(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 
 
//     seg1 = 3*G_obj.g1*pow(0.875, 2) + 2*G_obj.g2*0.875 + G_obj.g3; 
//     seg2 = 3*G_obj.h1*pow(0.875, 2) + 2*G_obj.h2*0.875 + G_obj.h3;

//     return seg1 - seg2; 
// }


// double grad2ContinuityConstraint1(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
//     double seg1, seg2; 

//     seg1 = 6*G_obj.alpha[0]*pow(0.125, 1) + 2*G_obj.alpha[1];
//     seg2 = 6*G_obj.alpha[4]*pow(0.125, 1) + 2*G_obj.alpha[5]; 

//     std::cout << "Second Derivative Diff: " << abs(seg1 - seg2) << std::endl;

//     return seg1 - seg2; 
// }

// double grad2ContinuityConstraint2(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 6*G_obj.b1*pow(0.25, 1) + 2*G_obj.b2;  
//     seg2 = 6*G_obj.c1*pow(0.25, 1) + 2*G_obj.c2;  

//     return seg1 - seg2; 
// }

// double grad2ContinuityConstraint3(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 6*G_obj.c1*pow(0.375, 1) + 2*G_obj.c2; 
//     seg2 = 6*G_obj.d1*pow(0.375, 1) + 2*G_obj.d2;  

//     return seg1 - seg2; 
// }

// double grad2ContinuityConstraint4(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

    
//     seg1 = 6*G_obj.d1*pow(0.5, 1) + 2*G_obj.d2;  
//     seg2 = 6*G_obj.e1*pow(0.5, 1) + 2*G_obj.e2; 

//     return seg1 - seg2; 
// }

// double grad2ContinuityConstraint5(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 6*G_obj.e1*pow(0.625, 1) + 2*G_obj.e2;
//     seg2 = 6*G_obj.f1*pow(0.625, 1) + 2*G_obj.f2; 

//     return seg1 - seg2; 
// }

// double grad2ContinuityConstraint6(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 6*G_obj.f1*pow(0.75, 1) + 2*G_obj.f2; 
//     seg2 = 6*G_obj.g1*pow(0.75, 1) + 2*G_obj.g2; 

//     return seg1 - seg2; 
// }

// double grad2ContinuityConstraint7(const std::vector<double> &alpha, std::vector<double> &grad, void* more_data){

//     GFunc G_obj(alpha);
//     double seg1, seg2; 

//     seg1 = 6*G_obj.g1*pow(0.875, 1) + 2*G_obj.g2; 
//     seg2 = 6*G_obj.h1*pow(0.875, 1) + 2*G_obj.h2; 

//     return seg1 - seg2; 
// }


/* 
_________________________________________________________________

 Writing my OBJECTIVE function
_________________________________________________________________

*/ 

double i = 0; 

double objectiveFunc(const std::vector<double> &alpha, std::vector<double> &grad, void *more_data){

    std::cout << "Iteration: " <<  i << std::endl;

    double l1 = 1, l2 = 1;

    constraint_data *data = reinterpret_cast<constraint_data*>(more_data);
    GFunc G_obj(alpha, DEGREE_OF_TEMPORAL_CURVE);
    double jerkCost = G_obj.sumOfJerks(data->H);
    double scaledJerkCost = l2*jerkCost; 
    double timeCost = G_obj.G(1);
    double scaledTimeCost = l1*timeCost;
    double cost = scaledTimeCost + scaledJerkCost;

    int ch = 97; // Decimal ASCII Value for 'a' 
    int num = 49; // Decimal ASCII Value for '1'
    int num_ = 49; 
    int idx = 0; 
    for (int seg = 1; seg <= NUMBER_OF_PIECES; seg++){
        while (num <= num_+DEGREE_OF_TEMPORAL_CURVE){
            char c[2] = {ch, num};
            std::cout << c << ": " << alpha[idx] << std::endl;
            num += 1; 
            idx += 1; 
        }
        num -= (DEGREE_OF_TEMPORAL_CURVE+1); 
        ch += 1; 
    }

    std::cout << "Jerk Cost: " << jerkCost << std::endl;
    std::cout << "Time Cost: " << timeCost << std::endl;
    i++; 

    return cost;
} 