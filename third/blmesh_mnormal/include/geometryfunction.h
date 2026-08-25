#pragma once
#ifndef _geeometryfunction_H_
#define _geeometryfunction_H_
#include "complexnode.h"
#include <cmath>
#include "common.h"
namespace GEEOMETRY_FUNCTION{
	enum INTER_STATUE {
		disjoin = 0x00,
		tangent = 0x01,
		one_intersection = 0x02,
		two_intersections = 0x04,
		fully_coincide = 0x08,
		mul_intersections = 0x10,
		others=0x20
	};


	bool pointInEdge(BLVector pos, BLVector ep1, BLVector ep2);



	bool isEdgeintersected(BLVector p1, BLVector p2, BLVector p3, BLVector p4);


	BLVector solveCenterPointOfSphereCap(BLVector A, BLVector B, BLVector C);

	BLVector solveCenterPointOfCircle(BLVector A, BLVector B, BLVector C);

	/**
	 * @brief get the geometry center normal
	 *
	 */
	BLVector getMostNormal(const std::vector<BLVector>& neighbour_normal);

	/**
	 * @brief get min cos value between direction and neighbour
	 *
	 */
	double getMinCos(const BLVector& direction, const  std::vector<BLVector>& neighbour_normal);
}
#endif
