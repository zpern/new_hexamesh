#ifndef  VIRTUAL_SPHERE_MESH_HASHER_H_
#define VIRTUAL_SPHERE_MESH_HASHER_H_
#include "VirtualSphereMesh.h"
#include "singleton.h"
#include <set>
#include <unordered_map>
struct MeshBoundaryHasher
{
	std::size_t operator()(const VirtualSphereMesh& mesh) const
	{		
		std::size_t add_count=mesh.isAddnode();
		std::size_t num = 0;
		for (auto i : mesh.neighbour_tri_index_) {
			num += std::hash<int>()(i.first*i.first);
			for (auto j : i.second)
				num += (j[0] * j[0]+j[1]*j[1])*i.first;
		}
		return std::hash<int>()(num * 3 + add_count);
	}
};
struct MeshBoundaryEdgeHasher
{
	std::size_t operator()(const VirtualSphereMesh& mesh) const
	{


		std::size_t add_count = mesh.isAddnode();
		std::size_t num = 0;
		for (auto i : mesh.boundary_edges_) {
			num += std::hash<int>()(i[0]* i[0]*10+ i[1]* i[1]);
			
		}
		return std::hash<int>()(num * 3 + add_count);
	}
};



struct EqualMeshBoundary {
	bool operator()(const VirtualSphereMesh&a, const VirtualSphereMesh&b) const {


		if (a.neighbour_tri_index_ != b.neighbour_tri_index_)
			return false;
		if (a.isAddnode() != b.isAddnode())
			return false;
		return true;
	}
};

struct EqualMeshBoundaryEdge {
	bool operator()(const VirtualSphereMesh&a, const VirtualSphereMesh&b) const {
		std::set<int> node_set[2];
		for (auto i : a.boundary_edges_)
			node_set[0].insert(i[0]);
		for (auto i : b.boundary_edges_)
			node_set[1].insert(i[0]);
		if (node_set[0] != node_set[1])
			return false;
		if (a.isAddnode() != b.isAddnode())
			return false;
		return true;
	}
};


class MeshBoundaryMap:public Singleton<MeshBoundaryMap>
{
public:
	void clear() {
		min_max_local_skewness = 0.99999;
		best_result.clear();
		all_result.clear();
		boundary_edge_result.clear();
	}
	double min_max_local_skewness;
	unordered_map<VirtualSphereMesh, VirtualSphereMesh, MeshBoundaryHasher, EqualMeshBoundary> best_result;
	unordered_map<VirtualSphereMesh, std::vector<VirtualSphereMesh>, MeshBoundaryHasher, EqualMeshBoundary> all_result;
	unordered_map<VirtualSphereMesh, std::vector<VirtualSphereMesh>, MeshBoundaryEdgeHasher, EqualMeshBoundaryEdge> boundary_edge_result;
};


#endif // ! MESH_SPLITTER_H_


