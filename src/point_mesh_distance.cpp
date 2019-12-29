#include "point_mesh_distance.h"
#include "point_triangle_distance.h"
#include <Eigen/Geometry> 

void point_mesh_distance(
  const Eigen::MatrixXd & X,
  const Eigen::MatrixXd & VY,
  const Eigen::MatrixXi & FY,
  Eigen::VectorXd & D,
  Eigen::MatrixXd & P,
  Eigen::MatrixXd & N)
{
  // Replace with your code
  P.resizeLike(X);
  D.resize(X.rows(), 1);
  N = Eigen::MatrixXd::Zero(X.rows(),X.cols());
  
  for(int i = 0; i<X.rows(); i++) {
    D(i) = INT_MAX;
    for (int j = 0; j < FY.rows(); ++j)
    {
      double d = 0;
      Eigen::RowVector3d p;
      point_triangle_distance(X.row(i), VY.row(FY(j,0)), VY.row(FY(j,1)), VY.row(FY(j,2)), d, p);

      if (d < D(i))
      {
        D(i) = d;
        P.row(i) = p;

        Eigen::RowVector3d e1 = VY.row(FY(j,1)) - VY.row(FY(j,0));
        Eigen::RowVector3d e2 = VY.row(FY(j,2)) - VY.row(FY(j,0));
        Eigen::RowVector3d direction = e1.cross(e2);
        N.row(i) = p + direction/direction.norm();

        N.row(i) = X.row(i) - p;
      }
    } 
  } 
}
