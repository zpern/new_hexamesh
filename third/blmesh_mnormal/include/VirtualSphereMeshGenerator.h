#pragma once
#ifndef _VIRTUAL_MESH_GENERATOR_H_
#define _VIRTUAL_MESH_GENERATOR_H_
#include "VirtualSphereMesh.h"
#include "VirtualSphereMeshStrategy.h"
#include <map>

#define NO_INSERT -1
#define ONE_INSERT -2

/************************************************************************/
/* sphere mesh in one complex node                                      */
/************************************************************************/
class VirtualSphereMeshGenerator
{
public:
	VirtualSphereMeshGenerator();
	~VirtualSphereMeshGenerator();

	std::size_t getNumberofProblem();
	void addStrategy(VirtualSphereStrategy vs);

	/*
	* @brief: if problem size is too big, we just solve the most gifted [max_number_of_problem] problems in 
	*/
	void cutStrategies(int max_number_of_problem);

	void setSingleSkew(double d);
	double getSingleSkew();


	void generate();

	VirtualSphereMesh& getFinalMesh();
protected:
	void AddNodeSphereSpr(VirtualSphereMesh& input_boundary);
	void LawsonFlip();
	VirtualSphereMesh generateAddOneNodeTriangles();

protected:
	double single_normal_skewness_;
	VirtualSphereMesh output_;

	std::vector<VirtualSphereStrategy> strategies_;
	std::vector<VPoint> virutal_point_list_;
	std::vector<VTriangle> virtual_triangles_list_;
	std::map<std::array<int,2>,VEDGE> edge_map_;
};
#endif
