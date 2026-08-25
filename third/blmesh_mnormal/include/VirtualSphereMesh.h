#ifndef _VIRTUAL_SPHERE_BOUNDARY_H_
#define _VIRTUAL_SPHERE_BOUNDARY_H_
#include "sphericalCap.h"
#include <fstream>
#include <string>
#include <set>
#include <map>

class VirtualSphereMesh {
public:
	VirtualSphereMesh();

	bool operator<(const VirtualSphereMesh& obj);
	bool operator>(const VirtualSphereMesh& obj);

	VirtualSphereMesh(const vector<BLVector>& convex_edge, const std::vector< BLVector>& facets_set_normal, vector<vector<int>> edge2face, vector<int> splitter);

	/*
	* @brife clear inner triangles
	*
	*/
	void clearInner();
	/*
	* @brife smooth the node in boundary
	*
	*/
	void smoothBoundaryNode();

	double caculateMaxSkewness();
	double caculateAveragySkewness();


	void addBoundaryTriangles(std::vector<std::vector<std::array<int, 2>>> manifold_triangles, map<int, BLVector> coordinate);

	std::vector<VirtualSphereMesh> getAllPossibleSubBoundary();


	void addNodeOpt();
	double AddNodeSphereSpr();
	double SphereSpr();

	bool isFullBoundary();

	bool isBoundaryValid();
	/**
	 * @brief merge two sphere mesh incluing boundary and triangle mesh.
	 * 
	 * @param b1 the first mesh
	 * @param b2 the second mesh
	 * @return the result mesh 
	 */
	static VirtualSphereMesh mergeMesh(VirtualSphereMesh& b1, VirtualSphereMesh& b2);

	static std::vector<VirtualSphereMesh> MeshBoundaryComplementary(VirtualSphereMesh& b1, int total);

	const int getExtraPointCount()  const;
	const int getValidPointCount()  const;
	bool isPointInBoundary(BLVector pos);
	void saveBoundaryVtk(std::string filename);
	void saveTriVtk(std::string filename);
	map<int, int> getMapNodeAfterFarNode();

	std::set<int> valid;
	std::vector<VEDGE> boundary_edges_;
	std::vector<VPoint> virtual_point_lists_;
	std::map<int, std::set<std::array<int,2>>> neighbour_tri_index_;
	std::vector<VTriangle> triangle_lists_;
	double max_skewness_;
	double ave_skewness_;

	//point index map, the last num has special meaning: -2 add a point, -1 without add a point
	static std::map<std::set<int>, VirtualSphereMesh> best_result;
	int isAddnode() const;
	void removeInvalidNode();

	vector<VirtualSphereMesh> LocalTriangulation() const;
protected:

	bool intersectionCheck();
	void addNode();
	void addTriangle(std::array<int, 3> tri);
	double getEdgeSkewness(int n1, int n2) const;
	double getEdgeSkewness(int i) const;
	double getTriSkewness(int i) const;
	double getTriSkewness(std::array<int,3> tri) const;
	

	int add_node_count = 0;
	std::vector<VEDGE> findBoundaryBetween(int start, int end) const;
};
#endif
