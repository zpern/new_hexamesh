#include"geometryfunction.h"
#include <cmath>
#include<algorithm>
namespace GEEOMETRY_FUNCTION{
#define EPS 1e-5

	BLVector solveCenterPointOfSphereCap(BLVector A, BLVector B, BLVector C) {
		//only call this funtion when ||A||==||B||=||C||=1
		BLVector ans = (B - A) ^ (C - B);
		return ans.normalized();
	}

	BLVector solveCenterPointOfCircle(BLVector A, BLVector B, BLVector C) {
	BLVector pd[3] = { A , B , C };
	double a1, b1, c1, d1;
	double a2, b2, c2, d2;
	double a3, b3, c3, d3;

	double x1 = pd[0].x, y1 = pd[0].y, z1 = pd[0].z;
	double x2 = pd[1].x, y2 = pd[1].y, z2 = pd[1].z;
	double x3 = pd[2].x, y3 = pd[2].y, z3 = pd[2].z;

	a1 = (y1*z2 - y2 * z1 - y1 * z3 + y3 * z1 + y2 * z3 - y3 * z2);
	b1 = -(x1*z2 - x2 * z1 - x1 * z3 + x3 * z1 + x2 * z3 - x3 * z2);
	c1 = (x1*y2 - x2 * y1 - x1 * y3 + x3 * y1 + x2 * y3 - x3 * y2);
	d1 = -(x1*y2*z3 - x1 * y3*z2 - x2 * y1*z3 + x2 * y3*z1 + x3 * y1*z2 - x3 * y2*z1);

	a2 = 2 * (x2 - x1);
	b2 = 2 * (y2 - y1);
	c2 = 2 * (z2 - z1);
	d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

	a3 = 2 * (x3 - x1);
	b3 = 2 * (y3 - y1);
	c3 = 2 * (z3 - z1);
	d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;
	BLVector centerpoint;
	centerpoint[0] = -(b1*c2*d3 - b1 * c3*d2 - b2 * c1*d3 + b2 * c3*d1 + b3 * c1*d2 - b3 * c2*d1)
		/ (a1*b2*c3 - a1 * b3*c2 - a2 * b1*c3 + a2 * b3*c1 + a3 * b1*c2 - a3 * b2*c1);
	centerpoint[1] = (a1*c2*d3 - a1 * c3*d2 - a2 * c1*d3 + a2 * c3*d1 + a3 * c1*d2 - a3 * c2*d1)
		/ (a1*b2*c3 - a1 * b3*c2 - a2 * b1*c3 + a2 * b3*c1 + a3 * b1*c2 - a3 * b2*c1);
	centerpoint[2] = -(a1*b2*d3 - a1 * b3*d2 - a2 * b1*d3 + a2 * b3*d1 + a3 * b1*d2 - a3 * b2*d1)
		/ (a1*b2*c3 - a1 * b3*c2 - a2 * b1*c3 + a2 * b3*c1 + a3 * b1*c2 - a3 * b2*c1);

	//centerpoint[3] = sqrt(pow(pd[0].x - centerpoint[0], 2) + pow(pd[0].y - centerpoint[1], 2) + pow(pd[0].z - centerpoint[2], 2));

	return centerpoint;
}

BLVector getMostNormal(const std::vector<BLVector>& neighbour_normal)
{
	BLVector ans;
	if (neighbour_normal.size() == 1) {
		return  neighbour_normal[0];
	}
	else if (neighbour_normal.size() == 2) {
		return (neighbour_normal[0] + neighbour_normal[1]).normalized();
	}
	else {

		int size = neighbour_normal.size();
		double min_bei_vise = -10;

		for (int i = 0; i < size; i++) {
			for (int j = i + 1; j < size; j++) {
				for (int k = j + 1; k < size; k++) {
					BLVector centor = GEEOMETRY_FUNCTION::solveCenterPointOfCircle(neighbour_normal[i], neighbour_normal[j], neighbour_normal[k]);
					BLVector normal = centor.normalized();
					if (isnan(normal.x))
						continue;
					double d = getMinCos(normal, neighbour_normal);
					if (min_bei_vise < d) {
						min_bei_vise = d;
						ans = normal;
					}
				}
			}
		}
		for (int i = 0; i < size; i++) {
			for (int j = i + 1; j < size; j++) {
				BLVector centor = (neighbour_normal[i] + neighbour_normal[j]);

				BLVector normal = centor.normalized();
				if (isnan(normal.x))
					continue;
				double d = getMinCos(normal, neighbour_normal);
				if (min_bei_vise < d) {
					min_bei_vise = d;
					ans = normal;
				}
			}
		}

	}



	return ans;
}

double getMinCos(const BLVector & direction, const std::vector<BLVector>& neighbour_normal)
{
	double minBeita = 1.0;
	for (int i = 0; i < neighbour_normal.size(); i++)
	{
		minBeita = std::min(minBeita, (neighbour_normal[i])* direction);
	}
	return minBeita;
}


bool pointInEdge(BLVector pos, BLVector ep1, BLVector ep2) {
	auto p1 = ep1;
	auto p2 = ep2;

	auto n1 = (p1 ^ pos).normalized();
	auto n2 = (pos ^ p2).normalized();

	if ((n1 - n2).magnitude2() < EPS)
		return true;
	return false;
}


bool isEdgeintersected(BLVector p1, BLVector p2, BLVector p3, BLVector p4)
{;

	BLVector intersected_coordinate = ((p1 ^ p2) ^ (p3 ^ p4)).normalized();

	return pointInEdge(intersected_coordinate, p1,p2) && pointInEdge(intersected_coordinate, p3,p4);

}







}
