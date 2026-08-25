#pragma once
#ifndef _capoverlapregion_H_
#define _capoverlapregion_H_
#include <list>
#include <vector>
#include "common.h"
#include "VirtualSphereMeshGenerator.h"
#include "sphericalCap.h"

using std::vector;
class MultBLNode {
public:
	BLVector coordinate;
	
protected:
	
};

class ComplexNode:public MultBLNode {
public:
    /**
    * @brief Construct a new Complex Node object
    *
    */
    ComplexNode();

	/**
	* @brief return the min asin(visable angle) 0--bad 1--best
	* @normal the marching direction of node
	*/
	double CaculateVisableAngle(BLVector normal);

	/*
	* @brife try to split node into two virtual node
	*/
	void SplitNode();
	void ConfigureSplit(double plane_skewness, double convex_skewness,
		int maximum_strategy_count);

	/*
	* @brife get final virtual mesh from optimization
	*/
	VirtualSphereMesh& getFinalMesh();

	vector<vector<ComplexNode>::iterator> neighbour_node_; /// An array store the ordered neighbour font direction.
	vector<BLVector> neighbour_front_direction_; /// An array store the ordered neighbour font direction.
	vector<int> neighbour_front_index_;

	BLVector single_normal_;
	double visible_angle_;
	double original_skewness_;
	int node_id_;
    double highRatio = 1;


private:
	double plane_skewness_ = PLANE_SKEWNESS;
	double convex_skewness_ = CONVEX_SKEWNESS;
	int maximum_strategy_count_ = MAX_STATEGY;
	VirtualSphereMeshGenerator generator_;/// Marching direction must lie on the regio. In geometry, direction must pointer to outer region of model
};
#endif
