#include "point_triangle_distance.h"
#include <Eigen/Geometry> 
#include <iostream>

void point_triangle_distance(
  const Eigen::RowVector3d & x,
  const Eigen::RowVector3d & a,
  const Eigen::RowVector3d & b,
  const Eigen::RowVector3d & c,
  double & d,
  Eigen::RowVector3d & p)
{
	// a:P1, b:P2, c:P3, x:P0, x projection poing:P, closet point in triangle:Pt
	Eigen::RowVector3d P1P2 = b - a;
	Eigen::RowVector3d P1P3 = c - a;
	Eigen::RowVector3d P2P3 = c - b;
	Eigen::RowVector3d P1P0 = x - a;

	Eigen::RowVector3d Np = P1P2.cross(P1P3); 
	double cos = P1P0.dot(Np)/(P1P0.norm()*Np.norm());
	Eigen::RowVector3d P0P = -1*P1P0.norm()*cos*Np/Np.norm();
	Eigen::RowVector3d P = x + P0P;

	Eigen::RowVector3d V1 = -P1P2/P1P2.norm() - P1P3/P1P3.norm();
	Eigen::RowVector3d V2 = -P2P3/P2P3.norm() + P1P2/P1P2.norm();
	Eigen::RowVector3d V3 = P1P3/P1P3.norm() + P2P3/P2P3.norm();

    Eigen::RowVector3d P1P = P-a, P2P = P-b, P3P = P-c;

    Eigen::RowVector3d left, right;
    if (V1.cross(P1P).dot(Np) > 0)
    {
    	if (V2.cross(P2P).dot(Np) < 0)
    	{
    		left = b, right = a;
    	}
    	else{
    		left = c, right = b;	
    	}
    }
    else{
    	if (V3.cross(P3P).dot(Np) > 0)
    	{
    		left = a, right = c;	
    	}
    	else{
    		left = c, right = b;
    	}
    } 
    //
    Eigen::RowVector3d right_left = left - right;
    Eigen::RowVector3d Pright = right - P, Pleft = left - P;
    if (Pright.cross(Pleft).dot(Np) < 0)
    {
    	//outside
    	Eigen::RowVector3d R = Pleft.cross(Pright).cross(right_left);
    	Eigen::RowVector3d PPt = (Pright.dot(R)/R.norm()) * R /R.norm();
    	Eigen::RowVector3d Pt = P + PPt;

    	Eigen::RowVector3d PrPt = Pt - right;
    	Eigen::RowVector3d PrPl = left - right;

    	double t = PrPt(0)/PrPl(0);
    	if (t >=0 && t <= 1)
    	{
    		p = Pt; 
    	}
    	else if (t < 0)
    	{
    		//P1
    		p = right;
    	}
    	else{
    		//P2
    		p = left;
    	}
    }
    else{
    	p = P;
    }

    Eigen::RowVector3d direction = x - p;
    d = direction.norm(); 
}
