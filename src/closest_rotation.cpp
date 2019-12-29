#include "closest_rotation.h"
#include <Eigen/SVD>  
#include <Eigen/Dense>
#include <iostream>

void closest_rotation(
  const Eigen::Matrix3d & M,
  Eigen::Matrix3d & R)
{
  // Replace with your code
  // R = Eigen::Matrix3d::Identity();
  Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::ColPivHouseholderQRPreconditioner> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd V = svd.matrixV();
  Eigen::MatrixXd Vt = V.transpose();

  Eigen::MatrixXd omigel = Eigen::MatrixXd(U.cols(), Vt.rows());
  omigel.setZero();
  int index = std::min(omigel.rows(), omigel.cols());

  for (int i = 0; i < index; ++i)
   {
   	if (i < index-1)
   	{
   		omigel(i,i) = 1;
   	}
   	else if (i == index - 1)
   	{
   		// Eigen::MatrixXd result = U * Vt;
   		Eigen::MatrixXd result = V * U.transpose();
   		omigel(i,i) = result.determinant(); 
   	}
   } 
   // R = U * omigel * Vt;
   R = V * omigel * U.transpose(); 
}
