#include "random_points_on_mesh.h"
#include <stdlib.h>
#include <igl/cumsum.h>
#include <igl/doublearea.h>
#include <iostream>

void random_points_on_mesh(
  const int n,
  const Eigen::MatrixXd & V,
  const Eigen::MatrixXi & F,
  Eigen::MatrixXd & X)
{
	  // REPLACE WITH YOUR CODE:
   	X.resize(n,3);
	  Eigen::VectorXd A;
    igl::doublearea(V,F,A);

    //cumulative sum of the relative areas
    Eigen::VectorXd C;
    igl::cumsum(A,1,C);
    C = C / C(C.rows() - 1);

    for(int i = 0; i<X.rows(); i++){
    	//choose a triangle
  		double r1 = (double) rand() / (RAND_MAX);
  		int start = 0, end = C.rows() - 1, index = 0;
  		while(end > start + 1){
  			int mid = (start+end)/2;
  			if (C(mid) < r1)
  			{
  				start = mid;
  			}
  			else{
  				end = mid;
  			}
  		}
  		if (C(start) >= r1)
  		{
  			index = start;
  		}
  		else{
  			index = end;
  		}
  		//inside a triangle
  		double r2 = (double) rand() / (RAND_MAX);
  		double r3 = (double) rand() / (RAND_MAX);
  		if (r2 + r3 > 1)
  		{
  			r2 = 1 - r2;
  			r3 = 1 - r3;
  		}
  		
  		X.row(i) = V.row(F(index, 0)) + r2*(V.row(F(index, 1)) - V.row(F(index, 0))) + r3*(V.row(F(index, 2)) - V.row(F(index, 0)));
    } 
}

