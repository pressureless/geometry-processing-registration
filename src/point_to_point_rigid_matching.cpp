#include "point_to_point_rigid_matching.h"
#include <igl/polar_svd.h>
#include "closest_rotation.h"
#include <iostream>

void point_to_point_rigid_matching(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & P,
  Eigen::Matrix3d & R,
  Eigen::RowVector3d & t)
{
  // Replace with your code

  R = Eigen::Matrix3d::Identity();
  t = Eigen::RowVector3d::Zero();

  Eigen::MatrixXd d1 = X.col(0) - P.col(0);
  Eigen::MatrixXd d2 = X.col(1) - P.col(1);
  Eigen::MatrixXd d3 = X.col(2) - P.col(2);

  Eigen::MatrixXd del(3*X.rows(), 1);
  del << d1, d2, d3;

  Eigen::MatrixXd A(3*X.rows(), 6);
  Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(X.rows(), 1);
  Eigen::MatrixXd I = Eigen::VectorXd::Ones(X.rows());

  A << Z, X.col(2), -X.col(1), I, Z, Z,
       -X.col(2), Z, X.col(0), Z, I, Z,
       X.col(1), -X.col(0), Z, Z, Z, I; 
  
  Eigen::MatrixXd u = (A.transpose()*A).inverse()*(-A.transpose()*del);
  Eigen::Matrix3d M = Eigen::Matrix3d::Identity();
 
  M(0,1) = -u(2);
  M(0,2) = u(1);
  M(1,0) = u(2);
  M(1,2) = -u(0);
  M(2,0) = - u(1);
  M(2,1) = u(0);
  closest_rotation(M, R);

  t(0) = u(3);
  t(1) = u(4);
  t(2) = u(5);
}

