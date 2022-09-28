#include "mav_companion_api/BSpline.h"


Eigen::RowVectorXd BSpline::getStamps(int lenOfArr){
  double dt = 1/(double)(lenOfArr-1);
  Eigen::RowVectorXd stamps(lenOfArr); 

  for (int i = 0; i < lenOfArr; i++){
    stamps(i) = dt*i;
  }

  return stamps;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> BSpline::getWaypointMatrix(std::vector<std::vector<double>> wpList) {
  int numOfPoints = wpList.size(); 

  Eigen::Matrix<double, Eigen::Dynamic, 3> waypointMatrix(numOfPoints, 3); 
  for (int i=0; i<numOfPoints; i++){
    for (int j=0; j<3; j++){
      waypointMatrix(i, j) = wpList[i][j]; 
    }
  }

  return waypointMatrix;
}

double BSpline::getPathLength(Spline3D s){
  int resolution = 1000;
  double resNorm = 1/(double)resolution;
  double pathLength = 0;

  for (double i = 0; i<=resolution-1; i++){
    Eigen::Vector3d diff = s( i*resNorm ) - s( (i+1)*resNorm ); 
    std::cout << (double)diff.norm() << std::endl;
    pathLength += (double)diff.norm();
  }

  return pathLength;
}

Spline3D BSpline::fitSpline(std::vector<std::vector<double>> waypoints){
  Eigen::RowVectorXd wpStamps = this->getStamps(waypoints.size());
  Eigen::MatrixXd wpPointMatrix = this->getWaypointMatrix(waypoints);

  
  Eigen::MatrixXd derivs {{0., 0., 0.}, {0., 0., 0.}};
  Eigen::VectorXi derivsIdx {{0, waypoints.size()-1}};

  //Generating Parameters
  double chordLen = this->getChordLength(wpPointMatrix);
  auto knotParams1 = this->genParamVec1(wpPointMatrix, chordLen); // Based on the algo
  int numOfPoints = waypoints.size();
  auto knotParams2 = this->genParamVec2(numOfPoints); // Based on dt

  const auto fit = SplineFitting3D::InterpolateWithDerivatives(wpPointMatrix.transpose(), derivs.transpose(), derivsIdx, 5, knotParams1);
  //const auto fit = SplineFitting3D::Interpolate(wpPointMatrix.transpose(), 5, knotParams2);
  Spline3D spline(fit);

  return spline;
}


double BSpline::getChordLength(Eigen::Matrix<double, Eigen::Dynamic, 3> points){
  double sum = 0;
  for (int j = 0; j < points.rows()-1; j++){
      auto Qj = points.row(j);
      auto Qj1 = points.row(j+1);

      auto diff = Qj1-Qj; 
      sum += diff.norm();
  }

  return sum;
}

Eigen::VectorXd BSpline::genParamVec1(Eigen::Matrix<double, Eigen::Dynamic, 3> points, double TCL){
  double t = 0; 

  Eigen::VectorXd params(points.rows());
  params(0) = t; 
  std::cout << "Num of points: " << points.rows() << std::endl;

  for (int j=1; j < points.rows(); j++){
      auto diff = points.row(j) - points.row(j-1);
      params(j) = t + diff.norm()/TCL;

      t = params(j);

  }

  return params;
}

Eigen::VectorXd BSpline::genParamVec2(int size){
  double t_end = 1;
  double dt = t_end/(size-1); 
  double t = 0;
  
  Eigen::VectorXd params(size);

  for (int idx=0; idx<size; idx+=1){
      params(idx) = t;
      t += dt; 
  }

  std::cout << "Size of params: " << params.size() << std::endl;

  return params; 
}
