#include "../include/mostnormaloptimizer.h"
#include "../include/geometryfunction.h"
#include "../include/MeshEvaluation.h"

void MostNormalOptimer::Optimize(VirtualSphereMesh & node)
{
	VirtualSphereMesh bak = node;
	auto node_to_triangle = node.neighbour_tri_index_;
	for (int i = 0; i < node.triangle_lists_.size(); i++) {
		for (int k = 0; k < 3; k++) {
			node_to_triangle[node.triangle_lists_[i].point_index_[k]].insert(\
				std::array<int, 2>{node.triangle_lists_[i].point_index_[(k + 1) % 3], node.triangle_lists_[i].point_index_[(k + 2) % 3]});
		}
	}
	for (int i = 0; i < node.boundary_edges_.size(); i++) {
		int id = node.boundary_edges_[i][0];
		vector<BLVector> neighbour_normals;
		if (node.virtual_point_lists_[id].isFarNode())
			continue;
		for (auto j : node_to_triangle[id]) {
			BLVector n1 = node.virtual_point_lists_[j[0]].getCoord() - node.virtual_point_lists_[id].getCoord();
			BLVector n2 = node.virtual_point_lists_[j[1]].getCoord() - node.virtual_point_lists_[j[0]].getCoord();
			neighbour_normals.push_back((n1^n2).normalized());
		}
		node.virtual_point_lists_[id].setCoord(GEEOMETRY_FUNCTION::getMostNormal(neighbour_normals));
	}
	if (MeshEvaluator::GetSingleton().GetQuality(bak) < MeshEvaluator::GetSingleton().GetQuality(node))
		node = bak;
}

