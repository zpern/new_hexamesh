/**
 * @file TopologyOptimizer.cpp
 * @author yhf (hfye@zju.edu.cn)
 * @brief This optimizer only change topology, the point location is unchanged after optimization. Sphere small polygen reconnection(SSPR) is utilized for transverse all the topology connection.
 * @version 1.6
 * @date 2022-01-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "../include/TopologyOptimizer.h"
#include "../include/VirtualSphereMeshHasher.h"
#include "../include/MeshEvaluation.h"

void TopologyOptimer::Optimize(VirtualSphereMesh &node) {
	VirtualSphereMesh no_insert_front = node;

	VirtualSphereMesh::best_result.clear();
	double initial_quality = 1;// MeshEvaluator::GetSingleton().GetQuality(node);
	MeshBoundaryMap::instance()->clear();
	VirtualSphereMesh::best_result.clear();
	VirtualSphereMesh test = node;
	VirtualSphereMesh shell = node;

	VirtualSphereMesh init = node;
	init.clearInner();
	shell.clearInner();

	auto tris = init.LocalTriangulation();
	//input_boundary.saveBoundaryVtk("BOUNDARY.vtk");
	//for (int i = 0; i < tris.size(); i++) {

	//	tris[i].saveTriVtk("TRI" + std::to_string(i) + ".vtk");
	//}
//	std::vector<>
	double min_max_skewness = 1.0;
	double min_ave_skewness = 1.0;

	for (int i = 0; i < tris.size(); i++) {
		if (!tris[i].triangle_lists_.size())
			continue;
		shell.triangle_lists_ = tris[i].triangle_lists_;
		/************************************************************************/
		/* Quality of new topology must be the best                             */
		/************************************************************************/
		double quality =	MeshEvaluator::GetSingleton().GetQuality(shell);
		if (quality < initial_quality) {
			initial_quality = quality;
			no_insert_front = shell;
		}
	}
	node = no_insert_front;
}


