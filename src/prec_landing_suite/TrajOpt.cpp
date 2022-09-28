#include <iostream>
#include <nlopt.hpp>
#include <cmath>

#include "mav_companion_api/TrajOpt.h"

GFunc::GFunc(std::vector<double> alpha, int degree) {

    this->alpha = alpha; 
    this->degree = degree; 

    totalSegs = alpha.size()/(degree+1);

    for (double interval = 0; interval <= 1; interval += (1/totalSegs)){
        segIntervals.push_back(interval);
    }

    // Four piecewise quadratic functions
    // a1 = alpha[0];
    // a2 = alpha[1];
    // a3 = alpha[2];
    // a4 = alpha[3];

    // b1 = alpha[4];
    // b2 = alpha[5];
    // b3 = alpha[6];
    // b4 = alpha[7];

    // c1 = alpha[8];
    // c2 = alpha[9];
    // c3 = alpha[10];
    // c4 = alpha[11];
    
    // d1 = alpha[12];
    // d2 = alpha[13];
    // d3 = alpha[14];
    // d4 = alpha[15]; 

    // e1 = alpha[16];
    // e2 = alpha[17];
    // e3 = alpha[18]; 
    // e4 = alpha[19];

    // f1 = alpha[20];
    // f2 = alpha[21];
    // f3 = alpha[22];
    // f4 = alpha[23];

    // g1 = alpha[24];
    // g2 = alpha[25];
    // g3 = alpha[26];
    // g4 = alpha[27]; 

    // h1 = alpha[28];
    // h2 = alpha[29];
    // h3 = alpha[30];
    // h4 = alpha[31];
}

int GFunc::checkWhichSegment(double tau) {
    int seg; 
    for (int idx = 1; idx < segIntervals.size(); idx++){
        if (tau <= segIntervals.at(idx)){
            seg = idx;
            break;  
        }
    }
    return seg; 
}

double GFunc::G(double tau){ //tau exists between 0 and 1

    double t = 0; 
    int seg = checkWhichSegment(tau);

    int i = (degree+1)*(seg-1);
    for (int k = 0; k <= degree; k++){
        t += alpha[i+k]*pow(tau, degree-k);
    }

    return t;
}

double GFunc::gradG(double tau){
    double t = 0; 
    int seg = checkWhichSegment(tau);

    int i = (degree+1)*(seg-1);
    for (int k = 0; k <= degree-1; k++){
        t += (degree-k)*alpha[i+k]*pow(tau, degree-k-1);
    }

    return t; 
}

double GFunc::grad2G(double tau){
    double t = 0;
    int seg = checkWhichSegment(tau);

    int i = (degree+1)*(seg-1);
    for (int k = 0; k<=degree-2; k++){
        t += (degree-k-1)*(degree-k)*alpha[i+k]*pow(tau, degree-k-2);  
    }

    return t; 
}

double GFunc::grad3G(double tau){
    double t = 0; 
    int seg = checkWhichSegment(tau);
    
    int i = (degree+1)*(seg-1);
    for (int k = 0; k<=degree-3; k++){
        t += (degree-k-2)*(degree-k-1)*(degree-k)*alpha[i+k]*pow(tau, degree-k-3);
    }

    return t; 
}

// double GFunc::G(double tau){ //tau exists between 0 and 1

//     /* G is basically a cubic polynomial which is a direct relation
//     between my time 't' domain and tau domain. */ 

//     // a1, a2, a3, b1, b2, b3, c1, c2, c3, d1, d2, d3

//     double t; 

//     if (tau <= 0.125) {
//         t = a1*pow(tau, 3) + a2*pow(tau, 2) + a3*pow(tau, 1) + a4; 
//     } else if (tau > 0.125 && tau <= 0.25) {
//         t = b1*pow(tau, 3) + b2*pow(tau, 2) + b3*pow(tau, 1) + b4; 
//     } else if (tau > 0.25 && tau <= 0.375) {
//         t = c1*pow(tau, 3) + c2*pow(tau, 2) + c3*pow(tau, 1) + c4; 
//     } else if (tau > 0.375 && tau <= 0.5) {
//         t = d1*pow(tau, 3) + d2*pow(tau, 2) + d3*pow(tau, 1) + d4; 
//     } else if (tau > 0.5 && tau <= 0.625) {
//         t = e1*pow(tau, 3) + e2*pow(tau, 2) + e3*pow(tau, 1) + e4;
//     } else if (tau > 0.625 && tau <= 0.75) {
//         t = f1*pow(tau, 3) + f2*pow(tau, 2) + f3*pow(tau, 1) + f4;
//     } else if (tau > 0.75 && tau <= 0.875) {
//         t = g1*pow(tau, 3) + g2*pow(tau, 2) + g3*pow(tau, 1) + g4;
//     } else if (tau > 0.875) {
//         t = h1*pow(tau, 3) + h2*pow(tau, 2) + h3*pow(tau, 1) + h4;
//     } 

//     return t;
// }

// double GFunc::gradG(double tau){ //tau exists between 0 and 1

//     double t;

//     // if (tau <= 0.25) {
//     //     t = 3*a1*pow(tau, 2) + 2*a2*tau + a3; 
//     // } else if (tau > 0.25 && tau <= 0.5) {
//     //     t = 3*b1*pow(tau, 2) + 2*b2*tau + b3;  
//     // } else if (tau > 0.5 && tau <= 0.75) {
//     //     t = 3*c1*pow(tau, 2) + 2*c2*tau + c3; 
//     // } else if (tau > 0.75) {
//     //     t = 3*d1*pow(tau, 2) + 2*d2*tau + d3; 
//     // }

//     if (tau <= 0.125) {
//         t = 3*a1*pow(tau, 2) + 2*a2*tau + a3; 
//     } else if (tau > 0.125 && tau <= 0.25) {
//         t = 3*b1*pow(tau, 2) + 2*b2*tau + b3; 
//     } else if (tau > 0.25 && tau <= 0.375) {
//         t = 3*c1*pow(tau, 2) + 2*c2*tau + c3; 
//     } else if (tau > 0.375 && tau <= 0.5) {
//         t = 3*d1*pow(tau, 2) + 2*d2*tau + d3; 
//     } else if (tau > 0.5 && tau <= 0.625) {
//         t = 3*e1*pow(tau, 2) + 2*e2*tau + e3;
//     } else if (tau > 0.625 && tau <= 0.75) {
//         t = 3*f1*pow(tau, 2) + 2*f2*tau + f3;
//     } else if (tau > 0.75 && tau <= 0.875) {
//         t = 3*g1*pow(tau, 2) + 2*g2*tau + g3;
//     } else if (tau > 0.875) {
//         t = 3*h1*pow(tau, 2) + 2*h2*tau + h3;
//     } 

//     return t;
// }

// double GFunc::grad2G(double tau){
//     double t;

//     // if (tau <= 0.25) {
//     //     t = 6*a1*tau + 2*a2; 
//     // } else if (tau > 0.25 && tau <= 0.5) {
//     //     t = 6*b1*tau + 2*b2;
//     // } else if (tau > 0.5 && tau <= 0.75) {
//     //     t = 6*c1*tau + 2*c2;
//     // } else if (tau > 0.75) {
//     //     t = 6*d1*tau + 2*d2;
//     // }

//     if (tau <= 0.125) {
//         t = 6*a1*tau + 2*a2; 
//     } else if (tau > 0.125 && tau <= 0.25) {
//         t = 6*b1*tau + 2*b2; 
//     } else if (tau > 0.25 && tau <= 0.375) {
//         t = 6*c1*tau + 2*c2; 
//     } else if (tau > 0.375 && tau <= 0.5) {
//         t = 6*d1*tau + 2*d2; 
//     } else if (tau > 0.5 && tau <= 0.625) {
//         t = 6*e1*tau + 2*e2;
//     } else if (tau > 0.625 && tau <= 0.75) {
//         t = 6*f1*tau + 2*f2;
//     } else if (tau > 0.75 && tau <= 0.875) {
//         t = 6*g1*tau + 2*g2;
//     } else if (tau > 0.875) {
//         t = 6*h1*tau + 2*h2;
//     } 

//     return t;
// }

// double GFunc::grad3G(double tau){
//     double t;

//     // if (tau <= 0.25) {
//     //     t = 6*a1; 
//     // } else if (tau > 0.25 && tau <= 0.5) {
//     //     t = 6*b1;
//     // } else if (tau > 0.5 && tau <= 0.75) {
//     //     t = 6*c1;
//     // } else if (tau > 0.75) {
//     //     t = 6*d1;
//     // }

//     if (tau <= 0.125) {
//         t = 6*a1; 
//     } else if (tau > 0.125 && tau <= 0.25) {
//         t = 6*b1; 
//     } else if (tau > 0.25 && tau <= 0.375) {
//         t = 6*c1; 
//     } else if (tau > 0.375 && tau <= 0.5) {
//         t = 6*d1; 
//     } else if (tau > 0.5 && tau <= 0.625) {
//         t = 6*e1;
//     } else if (tau > 0.625 && tau <= 0.75) {
//         t = 6*f1;
//     } else if (tau > 0.75 && tau <= 0.875) {
//         t = 6*g1;
//     } else if (tau > 0.875) {
//         t = 6*h1;
//     } 
    
//     return t;
// }

double GFunc::minimumGradG(){ 
    // Incrementally checks the value of dG with step size of dtau.
    double minGrad = gradG(0);
    for (double tau = 0; tau <= 1; tau += dtau){
        if (gradG(tau) < minGrad) {
            minGrad = gradG(tau);
        }
    }
    return minGrad; 
}

double GFunc::maximumPositionXGradient(Spline3D H){

    double maxVel = 0;
    
    for (double tau = 0; tau <= 1; tau += dtau){
        auto deriv = H.derivatives(tau, 1);

        double velX = deriv(0, 1) * (1/gradG(tau));

        if (abs(velX) > maxVel){
            maxVel = abs(velX);
        }
    }

    return maxVel;
}

double GFunc::maximumPositionYGradient(Spline3D H){

    double maxVel = 0;
    
    for (double tau = 0; tau <= 1; tau += dtau){
        auto deriv = H.derivatives(tau, 1);

        double velY = deriv(1, 1) * (1/gradG(tau));

        if (abs(velY) > maxVel){
            maxVel = abs(velY);
        }
    }

    return maxVel;
}

double GFunc::maximumPositionZGradient(Spline3D H){

    double maxVel = 0;
    
    for (double tau = 0; tau <= 1; tau += dtau){
        auto deriv = H.derivatives(tau, 1);

        double velZ = deriv(2, 1) * (1/gradG(tau));

        if (abs(velZ) > maxVel){
            maxVel = abs(velZ);
        }
    }

    return maxVel;
}

double GFunc::maximumVelocityXGradient(Spline3D H){
    double maxAcc = 0;

    for (double tau = 0; tau <= 1 - 2*dtau; tau += dtau){
        auto deriv = H.derivatives(tau, 2);

        double accX = ( 1/gradG(tau) ) * ( (deriv(0, 2)/gradG(tau)) - ( (deriv(0, 1)*grad2G(tau))/pow(gradG(tau), 2) ) );
        
        if (abs(accX) > maxAcc) {
            maxAcc = abs(accX);
        } 
    }

    return maxAcc;
}

double GFunc::maximumVelocityYGradient(Spline3D H){
    double maxAcc = 0;

    for (double tau = 0; tau <= 1 - 2*dtau; tau += dtau){
        auto deriv = H.derivatives(tau, 2);

        double accY = ( 1/gradG(tau) ) * ( (deriv(1, 2)/gradG(tau)) - ( (deriv(1, 1)*grad2G(tau))/pow(gradG(tau), 2) ) );

        if (abs(accY) > maxAcc) {
            maxAcc = abs(accY);
        } 
    }

    return maxAcc;
}

double GFunc::maximumVelocityZGradient(Spline3D H){
    double maxAcc = 0;

    for (double tau = 0; tau <= 1 - 2*dtau; tau += dtau){
        auto deriv = H.derivatives(tau, 2);

        double accZ = ( 1/gradG(tau) ) * ( (deriv(2, 2)/gradG(tau)) - ( (deriv(2, 1)*grad2G(tau))/pow(gradG(tau), 2) ) );

        if (abs(accZ) > maxAcc) {
            maxAcc = abs(accZ);
        } 
    }

    return maxAcc;
}



double GFunc::sumOfJerks(Spline3D H){
    double sum = 0;

    for (double tau = 0; tau <= 1; tau += dtau){
        auto deriv = H.derivatives(tau, 3);

        auto H = deriv.col(0);
        auto d1H = deriv.col(1);
        auto d2H = deriv.col(2);
        auto d3H = deriv.col(3);

        double D = pow(gradG(tau), 3); 
        double gradD = 3*pow(gradG(tau), 2)*grad2G(tau);
        Eigen::Vector3d N = gradG(tau)*d2H - d1H*grad2G(tau);
        Eigen::Vector3d gradN = gradG(tau)*d3H - d1H*grad3G(tau);

        Eigen::Vector3d jerk = (1/gradG(tau)) * (D*gradN - N*gradD)/pow(D, 2); 

        sum += abs(jerk.squaredNorm());
 
    }

    return sum;
}


Optimizer::Optimizer(unsigned n, std::vector<double> lower_bounds) : opt(nlopt::LN_COBYLA, n) {

    opt.set_lower_bounds(lower_bounds);
    opt.set_xtol_rel(xtol_rel); 
}

void Optimizer::setConstraintFunction(constraint_data data, double (*constraint_func)(
                                                            const std::vector<double> &, 
                                                            std::vector<double> &, 
                                                            void *
                                                        )
                                                    ){

    opt.add_inequality_constraint(constraint_func, &data, 1e-8);
}

void Optimizer::setObjectiveFunction(constraint_data data, double (*objective_func)(
                                                            const std::vector<double> &, 
                                                            std::vector<double> &, 
                                                            void *
                                                        )
                                                    ){

    opt.set_min_objective(objective_func, &data);
}


void Optimizer::startOptimization(std::vector<double>& alpha, double& minf){
    //try{
        nlopt::result result = opt.optimize(alpha, minf);
        std::cout << "Optimization finished." << std::endl;
    //}
    // catch(std::exception &e) {
    //     std::cout << "nlopt failed: " << e.what() << std::endl;
    // }
}



