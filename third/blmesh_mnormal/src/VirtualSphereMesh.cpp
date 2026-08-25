
#include "../include/VirtualSphereMesh.h"
#include "../include/PointOptimizer.h"
#include "../include/combineoptimizer.h"
#include "../include/geometryfunction.h"
#include "../include/VirtualSphereMeshHasher.h"
#include "geom_func.h"
#include <set>
#include <map>
#include <algorithm>
#include <limits>
#include <../include/Splitter.h>
#define BAD_QUALITY 0.85
map<set<int>, VirtualSphereMesh> VirtualSphereMesh::best_result = map<set<int>, VirtualSphereMesh>();
void VirtualSphereMesh::removeInvalidNode()
{
	valid.clear();
	for (auto i : boundary_edges_) {
		valid.insert(i[0]);
		valid.insert(i[1]);
	}
	for (auto i : triangle_lists_) {
		valid.insert(i.point_index_[0]);
		valid.insert(i.point_index_[1]);
		valid.insert(i.point_index_[2]);
	}
	valid.insert(-1 - add_node_count);


}
std::vector<VEDGE> VirtualSphereMesh::findBoundaryBetween(int start, int end) const
{
	int left_start_index, left_end_index;
	for (int i = 0; i < boundary_edges_.size(); i++) {
		if (start == boundary_edges_[i][1]) {
			left_end_index = i;
		}
		if (end == boundary_edges_[i][0]) {
			left_start_index = i;
		}
	}
	std::vector<VEDGE> leaf_edges;
	for (int ptr = left_start_index; ptr%boundary_edges_.size() != (left_end_index+1) % boundary_edges_.size(); ptr++) {
		leaf_edges.push_back(boundary_edges_[ptr%boundary_edges_.size()]);
	}
	leaf_edges.push_back(VEDGE{ start,end });
	return leaf_edges;
}
VirtualSphereMesh VirtualSphereMesh::mergeMesh(VirtualSphereMesh & b1, VirtualSphereMesh & b2)
{

	VirtualSphereMesh ans=b1;
	// merge points
	for (auto i : b2.triangle_lists_)
		ans.triangle_lists_.push_back(i);
	for (auto i : b2.valid)
		ans.valid.insert(i);


	if (b2.virtual_point_lists_.size() > ans.virtual_point_lists_.size()) {
		ans.virtual_point_lists_ = b2.virtual_point_lists_;
		ans.valid = b2.valid;
	}

	//modify negative bit
	if (ans.valid.find(ONE_INSERT) != ans.valid.end())
		ans.valid.erase(NO_INSERT);
	ans.add_node_count = b1.add_node_count + b2.add_node_count;
	

	//merge boundary edge, by xor
	ans.boundary_edges_.clear();
	for (int j = 0; j < b2.boundary_edges_.size();j++) {
		bool in = false;
		for (int i = 0; i < b1.boundary_edges_.size(); i++) {
			if (b2.boundary_edges_[j][0] == b1.boundary_edges_[i][1] && b2.boundary_edges_[j][1] == b1.boundary_edges_[i][0])
				in = true;
		}
		if (!in)
			ans.boundary_edges_.push_back(b2.boundary_edges_[j]);
	}
	for (int j = 0; j < b1.boundary_edges_.size(); j++) {
		bool in = false;
		for (int i = 0; i < b2.boundary_edges_.size(); i++) {
			if (b2.boundary_edges_[i][0] == b1.boundary_edges_[j][1] && b2.boundary_edges_[i][1] == b1.boundary_edges_[j][0])
				in = true;
		}
		if (!in)
			ans.boundary_edges_.push_back(b1.boundary_edges_[j]);
	}

	ans.max_skewness_ = max(b1.max_skewness_, b2.max_skewness_);
	return ans;


}









/*
*        get all subset of b1 named ans that ans+b2=b1, only support boundary
*
*
*
*
*/
//#pragma optimize("",off)
std::vector<VirtualSphereMesh> VirtualSphereMesh::MeshBoundaryComplementary(VirtualSphereMesh & b1, int total)
{
	std::vector<VirtualSphereMesh> ans;
	std::set<int> is_in_b1;

	for (int i = 0; i < b1.boundary_edges_.size(); i++) {
		is_in_b1.insert(b1.boundary_edges_[i][0]);
		is_in_b1.insert(b1.boundary_edges_[i][1]);
	}

	int start = *is_in_b1.begin();
	int s = -1;
	int e = -1;
	for (int i = start; i < total + start; i++) {
		int id = i % total;
		if (is_in_b1.find(id) != is_in_b1.end()&& is_in_b1.find((id+1)%total) == is_in_b1.end()) {
			s = id;
		}
		else if (is_in_b1.find(id) == is_in_b1.end() && is_in_b1.find((id + 1) % total) == is_in_b1.end()) {
			e = (id + 1) % total;
		}
		else if (is_in_b1.find(id) == is_in_b1.end() && is_in_b1.find((id + 1) % total) != is_in_b1.end()) {
			e = (id + 1) % total;
			VirtualSphereMesh vs;
			vs = b1;
			vs.boundary_edges_.clear();
			int p = s;
			while (true) {
				if (p != e)
					vs.boundary_edges_.push_back(std::array<int, 2>{p%total, (p + 1) % total});
				else {
					vs.boundary_edges_.push_back(std::array<int, 2>{e, s});
					break;
				}
				p++;
				p = p % total;
			}
			s = -1;
			e = -1;
			ans.push_back(vs);
		}
	}
	if (s != -1 && e != -1) {
		VirtualSphereMesh vs;
		vs = b1;
		vs.boundary_edges_.clear();
		int p = s;
		while (true) {
			if (p != e)
				vs.boundary_edges_.push_back(std::array<int, 2>{p%total, (p + 1) % total});
			else {
				vs.boundary_edges_.push_back(std::array<int, 2>{e, s});
				break;
			}
			p++;
			p = p % total;
		}
		s = -1;
		e = -1;
		ans.push_back(vs);
	}

	
	

	return ans;
}
//#pragma optimize("",on)
const int VirtualSphereMesh::getExtraPointCount() const
{
	if (valid.size())
		return (valid.size())/2;
	return 0;
}
const int VirtualSphereMesh::getValidPointCount() const
{
	if(valid.size())
	return valid.size()-1;
	return 1;
}
bool VirtualSphereMesh::isPointInBoundary(BLVector pos)
{
	for (auto j : boundary_edges_) {

		bool valid = true;

		
		for (auto i : boundary_edges_) {
			
			if ((!GEEOMETRY_FUNCTION::isEdgeintersected(pos,virtual_point_lists_[j[0]].getCoord(), virtual_point_lists_[i[0]].getCoord(), virtual_point_lists_[i[1]].getCoord()))&&\
				(!GEEOMETRY_FUNCTION::isEdgeintersected(pos, virtual_point_lists_[j[1]].getCoord(), virtual_point_lists_[i[0]].getCoord(), virtual_point_lists_[i[1]].getCoord()))) {
				valid = false;
			}
		}
		if (valid) {
			BLVector v1 = virtual_point_lists_[j[0]].getCoord() - pos;
			BLVector v2 = virtual_point_lists_[j[1]].getCoord() - pos;
			return (v1^v2)*virtual_point_lists_[j[1]].getCoord() >0;
		}
	}

	return false;
}

void VirtualSphereMesh::saveBoundaryVtk(std::string filename)
{
	int nBdry = boundary_edges_.size();

	const char *filename_ = filename.data();
	int i, idx;
	std::set<int> setpnt;
	std::set<int>::iterator sit;
	FILE *fout = nullptr;
	
	fout = fopen(filename_, "w");
	if (fout == nullptr)
	{
		throw(std::string("no authority to write file"));
	}
	//fprintf(fout, "%d %d 0 0 0 0 0\n", setpnt.size(), nBdry);
	fprintf(fout, "# vtk DataFile Version 2.0\nboundary layer mesh\nASCII\nDATASET UNSTRUCTURED_GRID\nPOINTS %d double\n", virtual_point_lists_.size() +1);
	sit = setpnt.begin();
	i = 0;
	fprintf(fout, "0 0 0\n");
	for(int i=0;i< virtual_point_lists_.size();i++)
	{
		idx = i;
		VPoint* ptr = dynamic_cast<VPoint*>(&virtual_point_lists_[idx]);
		fprintf(fout, "%f %f %f\n", ptr->getCoord()[0], ptr->getCoord()[1], ptr->getCoord()[2]);
	}
	nBdry = boundary_edges_.size();
	fprintf(fout, "CELLS %d %d\n", nBdry, nBdry * 3);

	for (i = 0; i < nBdry; i++)
	{
		fprintf(fout, " 2 %d %d\n", boundary_edges_[i][0]+1, boundary_edges_[i][1]+1);
	}
	fprintf(fout, "CELL_TYPES %d\n", nBdry);
	for (i = 0; i < nBdry; i++)
	{
		fprintf(fout, " 3\n");
	}

	fclose(fout);
	fout = nullptr;
}

void VirtualSphereMesh::saveTriVtk(std::string filename)
{

	int nBdry = triangle_lists_.size();

	int neighbour_triangle_size = 0;
	for (auto i : neighbour_tri_index_) {
		neighbour_triangle_size+=i.second.size();
	}
	nBdry += neighbour_triangle_size;

	const char *filename_ = filename.data();
	int i, idx;
	std::set<int> setpnt;
	std::set<int>::iterator sit;
	FILE *fout = nullptr;

	for (i = 0; i < virtual_point_lists_.size(); i++)
	{
		setpnt.insert(i);
	}
	map<int, int> ptmap;


	fout = fopen(filename_, "w");
	if (fout == nullptr)
	{
		throw(std::string("no authority to write file"));
	}
	//fprintf(fout, "%d %d 0 0 0 0 0\n", setpnt.size(), nBdry);
	fprintf(fout, "# vtk DataFile Version 2.0\nboundary layer mesh\nASCII\nDATASET UNSTRUCTURED_GRID\nPOINTS %d double\n", setpnt.size() + 1);
	sit = setpnt.begin();
	i = 0;
	fprintf(fout, "0 0 0\n");
	while (sit != setpnt.end())
	{
		idx = *sit;
		VPoint* ptr = dynamic_cast<VPoint*>(&virtual_point_lists_[idx]);
		fprintf(fout, "%f %f %f\n", ptr->getCoord()[0], ptr->getCoord()[1], ptr->getCoord()[2]);
		ptmap[idx] = i++;
		++sit;
	}

	fprintf(fout, "CELLS %d %d\n", nBdry+ boundary_edges_.size(), nBdry * 4+ boundary_edges_.size()*3);

	for (i = 0; i < triangle_lists_.size(); i++)
	{
		fprintf(fout, " 3 %d %d %d\n", ptmap[triangle_lists_[i].point_index_[0]] + 1, ptmap[triangle_lists_[i].point_index_[1]] + 1, ptmap[triangle_lists_[i].point_index_[2]] + 1);
	}

	for (auto i : neighbour_tri_index_) {
		for(auto j:i.second)
		{
			fprintf(fout, " 3 %d %d %d\n", ptmap[i.first] + 1, ptmap[j[0]] + 1, ptmap[j[1]] + 1);
		}
	}


	for (i = 0; i < boundary_edges_.size(); i++)
	{
		fprintf(fout, " 2 %d %d\n", ptmap[boundary_edges_[i][0]] + 1, ptmap[boundary_edges_[i][1]] + 1);
	}
	fprintf(fout, "CELL_TYPES %d\n", nBdry+ boundary_edges_.size());
	for (i = 0; i < nBdry; i++)
	{
		fprintf(fout, " 5\n");
	}

	for (i = 0; i < boundary_edges_.size(); i++)
	{
		fprintf(fout, " 3\n");
	}
	fclose(fout);
	fout = nullptr;

}

map<int, int> VirtualSphereMesh::getMapNodeAfterFarNode()
{
	map<int, int> ans;



	return ans;
}

void VirtualSphereMesh::addTriangle(std::array<int, 3> tri)
{
	triangle_lists_.push_back(VTriangle());
	triangle_lists_.back().point_index_ = tri;
}

double VirtualSphereMesh::getEdgeSkewness(int n1,int n2) const
{
	double ans = 0;
	if ((!virtual_point_lists_[n1].isFarNode()) && (!virtual_point_lists_[n2].isFarNode())) {
		ans = max(ans, abs(ANGLE(virtual_point_lists_[n1].getCoord(), virtual_point_lists_[n2].getCoord()) - 90) / 90);
	}
	if ((virtual_point_lists_[n1].isFarNode()) && (virtual_point_lists_[n2].isFarNode())) {
		ans = 1;
	}
	return ans;

}
double VirtualSphereMesh::getEdgeSkewness(int i) const
{
	return getEdgeSkewness(boundary_edges_[i][0], boundary_edges_[i][1]);
}

// skweness 




double VirtualSphereMesh::getTriSkewness(int i) const
{
	return getTriSkewness(triangle_lists_[i].point_index_);
}

double VirtualSphereMesh::getTriSkewness(std::array<int, 3> tri) const
{
	double ans = 0;
	BLVector p[3];
	int count = 0;
	for (int j = 0; j < 3; j++) {
		p[j] = virtual_point_lists_[tri[j]].getCoord();
	}
	for (int k = 0; k < 3; k++)
		for (int j = k + 1; j < 3; j++)
			if ((!virtual_point_lists_[tri[j]].isFarNode()) && (!virtual_point_lists_[tri[k]].isFarNode())) {
				ans = max(ans, abs(ANGLE(p[k], p[j]) - 90) / 90);
			}
	for (int k = 0; k < 3; k++)
		if (!virtual_point_lists_[tri[k]].isFarNode()) {
			if ((!virtual_point_lists_[tri[(k + 1) % 3]].isFarNode()) && (!virtual_point_lists_[tri[(k + 2) % 3]].isFarNode())) {
				ans = max(ans, (ANGLE(p[(k + 1) % 3] - p[k], p[(k + 2) % 3] - p[k]) - 60) / 120);
				ans = max(ans, (60 - ANGLE(p[(k + 1) % 3] - p[k], p[(k + 2) % 3] - p[k])) / 60);

				ans += TWO_ORDER_TRI_COST;
			}
			else {
				ans = max(ans, abs((ANGLE(p[(k + 1) % 3] - p[k], p[(k + 2) % 3] - p[k]) - 90)) / 90);
				if ((!virtual_point_lists_[tri[(k + 1) % 3]].isFarNode()) || (!virtual_point_lists_[tri[(k + 2) % 3]].isFarNode())) {
					ans += ONE_ORDER_TRI_COST;
				}
			}
		}
	for (int k = 0; k < 3; k++) {
		if (virtual_point_lists_[tri[k]].isFarNode() && virtual_point_lists_[tri[(k + 1) % 3]].isFarNode()) {
			ans = 1;
		}
	}
	
	/************************************************************************/
	/* if THREE nodes are not all far node, we deploy a cost on quality function*/
	/************************************************************************/


	return ans;
}

VirtualSphereMesh::VirtualSphereMesh()
{
	max_skewness_ = 1;
}

bool VirtualSphereMesh::operator<(const VirtualSphereMesh & obj)
{
	if(max_skewness_ == obj.max_skewness_)
		return max_skewness_ < obj.max_skewness_;
	return ave_skewness_< obj.ave_skewness_;
}

bool VirtualSphereMesh::operator>(const VirtualSphereMesh & obj)
{
	if (max_skewness_ == obj.max_skewness_)
		return max_skewness_ > obj.max_skewness_;
	return ave_skewness_ > obj.ave_skewness_;
}

VirtualSphereMesh::VirtualSphereMesh(const vector<BLVector>& convex_edge, const std::vector<BLVector>& facets_set_normal, vector<vector<int>> edge2face, vector<int> splitter)
{

	max_skewness_ = 1;
	if (convex_edge.size() != facets_set_normal.size())
		throw std::logic_error("impossbile input!");
	for (int i = 0; i < convex_edge.size(); i++) {
		// build point lists
		VPoint point ;
		point.setCoord(convex_edge[i]);
		point.setIndex(virtual_point_lists_.size());
		point.setGlobalIndex(splitter[i]);
		virtual_point_lists_.push_back(point);
		


		VPoint point_sphere;

		point_sphere.setCoord( facets_set_normal[i]);
		
		point_sphere.setIndex(virtual_point_lists_.size());
		point_sphere.setGlobalNeighbourTriIndex(edge2face[i]);
		virtual_point_lists_.push_back(point_sphere);
		
	}

	for (int i = 0; i < convex_edge.size(); i++) {
		// build edge lists
		auto edge_1 = VEDGE();
		auto edge_2 = VEDGE();
		(edge_1)[0] = 2 * i + 0;
		(edge_1)[1] = 2 * i + 1;
		int j = (i + 1) % convex_edge.size();
		(edge_2)[0] = 2 * i + 1;
		(edge_2)[1] = 2 * j + 0;
		boundary_edges_.push_back(edge_1);
		boundary_edges_.push_back(edge_2);
	}
}

double VirtualSphereMesh::caculateAveragySkewness()
{
	double ave_skewness = 0;
	int count = 0;
	for (int i = 0; i < triangle_lists_.size(); i++) {
		ave_skewness += getTriSkewness(i);
		for (int k = 0; k < 3; k++)
			ave_skewness += getEdgeSkewness(triangle_lists_[i].point_index_[(k + 1) % 3], triangle_lists_[i].point_index_[(k + 2) % 3]);
	}
	count += triangle_lists_.size() * 4;

	// visibility cone
	auto node_to_triangle = neighbour_tri_index_;
	for (int i = 0; i < triangle_lists_.size(); i++) {
		for (int k = 0; k < 3; k++) {
			node_to_triangle[triangle_lists_[i].point_index_[k]].insert(\
				std::array<int, 2>{triangle_lists_[i].point_index_[(k + 1) % 3], triangle_lists_[i].point_index_[(k + 2) % 3]});
		}
	}
	for (auto i : node_to_triangle) {
		vector<BLVector> neighbour_normals;
		for (auto j : i.second) {
			BLVector n1 = virtual_point_lists_[j[0]].getCoord() - virtual_point_lists_[i.first].getCoord();
			BLVector n2 = virtual_point_lists_[j[1]].getCoord() - virtual_point_lists_[j[0]].getCoord();
			neighbour_normals.push_back((n1^n2).normalized());
		}
		double minCos = GEEOMETRY_FUNCTION::getMinCos(GEEOMETRY_FUNCTION::getMostNormal(neighbour_normals), neighbour_normals);
		ave_skewness += abs(acos(minCos) - PI / 2) * 2 / PI;
	}
	count += node_to_triangle.size();
	ave_skewness_ = ave_skewness / count;
	return ave_skewness_;
}

void VirtualSphereMesh::clearInner()
{
	triangle_lists_.clear();
	max_skewness_ = 0;
	removeInvalidNode();
}

void VirtualSphereMesh::smoothBoundaryNode()
{
	
	auto node_to_triangle = neighbour_tri_index_;
	for (int i = 0; i < triangle_lists_.size(); i++) {
		for (int k = 0; k < 3; k++) {
			node_to_triangle[triangle_lists_[i].point_index_[k]].insert(\
				std::array<int, 2>{triangle_lists_[i].point_index_[(k + 1) % 3], triangle_lists_[i].point_index_[(k + 2) % 3]});
		}
	}
	int count = MAX_INTERATION/2;
	while (count--) {
		CombineOptimer c_opt;
		c_opt.Optimize(*this);
	}

}
//#pragma optimize("",off)
double VirtualSphereMesh::caculateMaxSkewness()
{
	double max_skewness = 0;
	if (!triangle_lists_.empty()) { // if triangle exist , just caculate the triangles
		for (int i = 0; i < triangle_lists_.size(); i++) {
			max_skewness = max(max_skewness, getTriSkewness(i));
			for (int k = 0; k < 3; k++)
				max_skewness = max(max_skewness, getEdgeSkewness(triangle_lists_[i].point_index_[(k + 1) % 3], triangle_lists_[i].point_index_[(k + 2) % 3]));
		}
	}
	else { // if no triangles in mesh  , we caculate the edge skewness
		for (int i = 0; i < boundary_edges_.size(); i++) {
			max_skewness = max(max_skewness, getEdgeSkewness(boundary_edges_[i][0], boundary_edges_[i][1]));
		}
	}


	// visibility cone
	auto node_to_triangle = neighbour_tri_index_;
	for (int i = 0; i < triangle_lists_.size(); i++) {
		for (int k = 0; k < 3; k++) {
			node_to_triangle[triangle_lists_[i].point_index_[k]].insert(\
				std::array<int, 2>{triangle_lists_[i].point_index_[(k + 1) % 3], triangle_lists_[i].point_index_[(k + 2) % 3]});
		}
	}
	for (auto i : node_to_triangle) {
		vector<BLVector> neighbour_normals;
		for (auto j : i.second) {
			BLVector n1 = virtual_point_lists_[j[0]].getCoord() - virtual_point_lists_[i.first].getCoord();
			BLVector n2 = virtual_point_lists_[j[1]].getCoord() - virtual_point_lists_[j[0]].getCoord();
			neighbour_normals.push_back((n1^n2).normalized());
		}
		double minCos = GEEOMETRY_FUNCTION::getMinCos(GEEOMETRY_FUNCTION::getMostNormal(neighbour_normals), neighbour_normals);

		max_skewness = max(max_skewness, abs(acos(minCos)) * 2 / PI);
	}


	if (max_skewness > 1)
		max_skewness = 1;
	max_skewness_ = max_skewness;
	return max_skewness;
}

//#pragma optimize("",on)

int VirtualSphereMesh::isAddnode() const
{
	return add_node_count;
}

vector<VirtualSphereMesh> VirtualSphereMesh::LocalTriangulation() const
{
	if (MeshBoundaryMap::instance()->boundary_edge_result.find(*this) != MeshBoundaryMap::instance()->boundary_edge_result.end())
		return MeshBoundaryMap::instance()->boundary_edge_result[*this];
	VirtualSphereMesh input = *this;

	if (input.boundary_edges_.size() < 3)
		return vector<VirtualSphereMesh>{*this};

	int p1 = input.boundary_edges_[0][0];
	int p2 = input.boundary_edges_[0][1];


	if (input.boundary_edges_.size() == 3) {
		int p3 = input.boundary_edges_[1][1];

		input.max_skewness_=getTriSkewness(std::array<int, 3>{p1, p2, p3});
		if (input.max_skewness_ <= MeshBoundaryMap::instance()->min_max_local_skewness) {
			input.triangle_lists_.push_back(VTriangle());
			input.triangle_lists_.back().point_index_ = std::array<int, 3>{p1, p2, p3};
			MeshBoundaryMap::instance()->boundary_edge_result[*this].push_back(input);
		}
	}
	else {


		for (int i = 0; i < input.boundary_edges_.size(); i++) {
			int p3 = input.boundary_edges_[i][1];
			if (p3 == p1 || p3 == p2)
				continue;

			std::array<int, 3> my_tri{ p1,p2,p3 };

			// create new edges
			VirtualSphereMesh left = input;
			VirtualSphereMesh right = input;


			VirtualSphereMesh median;
			median.virtual_point_lists_ = this->virtual_point_lists_;
			median.boundary_edges_.resize(3);
			median.boundary_edges_[0] = std::array<int, 2>{p1, p2};
			median.boundary_edges_[1] = std::array<int, 2>{p2, p3};
			median.boundary_edges_[2] = std::array<int, 2>{p3, p1};


			// There may create invalid triangles
			


			left.boundary_edges_ = findBoundaryBetween(p1, p3);
			right.boundary_edges_ = findBoundaryBetween(p3, p2);

			vector<VirtualSphereMesh> left_triangulation   = left.LocalTriangulation();
			vector<VirtualSphereMesh> median_triangulation = median.LocalTriangulation();
			vector<VirtualSphereMesh> right_triangulation  = right.LocalTriangulation();

			VirtualSphereMesh target;
			

			for (auto m1 : left_triangulation) {
				for (auto m2 : right_triangulation) {
					for (auto m3 : median_triangulation) {
						target = mergeMesh(m1, m3);
						target = mergeMesh(m2, target);
						target.neighbour_tri_index_ = input.neighbour_tri_index_;
						target.virtual_point_lists_ = input.virtual_point_lists_;
						target.caculateMaxSkewness();
						if (target.max_skewness_ <= MeshBoundaryMap::instance()->min_max_local_skewness)
							if (target.triangle_lists_.size() == input.boundary_edges_.size() - 2) {
								if (target.isFullBoundary()) {
									target.isFullBoundary();
									MeshBoundaryMap::instance()->min_max_local_skewness=min(MeshBoundaryMap::instance()->min_max_local_skewness,target.max_skewness_);
								}
								MeshBoundaryMap::instance()->boundary_edge_result[target].push_back(target);
							}
					}
				}
			}


		}
	}
	return MeshBoundaryMap::instance()->boundary_edge_result[*this];
}

void VirtualSphereMesh::addNode()
{
	BLVector ans;
	triangle_lists_.clear();
	

	//get valid point
	vector<int> points;
	for (auto i : valid)
		if (i >= 0&&(!virtual_point_lists_[i].isFarNode()))
			points.push_back(i);
	double min_skewness = 1;
	if (points.size() == 2) {
		ans = virtual_point_lists_[points[0]].getCoord() + virtual_point_lists_[points[1]].getCoord();
		ans.normalize();
	}
	for (int i = 0; i < points.size(); i++) {
		for (int j = i+1; j < points.size(); j++) {
			for (int k = j+1; k < points.size(); k++) {
				BLVector center=GEEOMETRY_FUNCTION::solveCenterPointOfSphereCap(virtual_point_lists_[i].getCoord(), virtual_point_lists_[j].getCoord(), virtual_point_lists_[k].getCoord());	
				if (center.magnitude() < 0.9)
					continue;
				virtual_point_lists_.push_back(VPoint());
				virtual_point_lists_.back().setCoord(center);
				virtual_point_lists_.back().setIndex(virtual_point_lists_.size()-1);
				double max_skewness = 0;
				for (int l = 0; l < boundary_edges_.size(); l++) {
					std::array<int, 3> tri{boundary_edges_[l][0],boundary_edges_[l][1] ,virtual_point_lists_.size() - 1 };
					max_skewness=max(max_skewness,getTriSkewness(tri));
				}
				if (min_skewness > max_skewness) {
					min_skewness = max_skewness;
					ans = center;
				}
				virtual_point_lists_.pop_back();
			}
		}
	}
	

	if (ans.magnitude() < 0.9)
		return;
	int index = virtual_point_lists_.size() ;
	ans.normalize();
	virtual_point_lists_.push_back(VPoint());
	virtual_point_lists_.back().setCoord(ans);
	virtual_point_lists_.back().setIndex(index);

	for (int i = 0; i < boundary_edges_.size(); i++) {
		VTriangle t;
		t.point_index_[0] = boundary_edges_[i][0];
		t.point_index_[1] = boundary_edges_[i][1];
		t.point_index_[2] = index;
		triangle_lists_.push_back(t);
	}
	valid.erase(NO_INSERT);
	valid.insert(ONE_INSERT);

	max_skewness_ = 0.0;
	for (int i = 0; i < triangle_lists_.size(); i++) {
		max_skewness_ = max(max_skewness_, getTriSkewness(i));
	}
	add_node_count++;
}

void VirtualSphereMesh::addBoundaryTriangles(std::vector<std::vector<std::array<int, 2>>> manifold_triangles, map<int, BLVector> coordinate)
{
	for (auto i : coordinate) {
		virtual_point_lists_.push_back(VPoint());
		virtual_point_lists_.back().setCoord(i.second);
		virtual_point_lists_.back().setIndex(virtual_point_lists_.size() - 1);
	}
	for (int i = 0; i < manifold_triangles.size(); i++) {
		for(auto j: manifold_triangles[i])
			neighbour_tri_index_[2 * i + 1].insert(j);
	}
}
std::vector<VirtualSphereMesh> VirtualSphereMesh::getAllPossibleSubBoundary() {
	std::vector<VirtualSphereMesh> ans;
	std::set<int> valid_nodes;
	for (int i = 0; i < boundary_edges_.size(); i++) {
		valid_nodes.insert(boundary_edges_[i][0]);
		valid_nodes.insert(boundary_edges_[i][1]);
	}

	auto splittered_boundary=Splitter::instance()->getSpliiter(valid_nodes);
	for (auto s : splittered_boundary) {
		int non_far_count = 0;
		std::vector<int> nodes;
		for (auto j : s) {
			if (!virtual_point_lists_[j].isFarNode())
				non_far_count++;
			nodes.push_back(j);
		}
		if ((s.size()-non_far_count)/2+ s.size() >= 3&& non_far_count>=2) {
			VirtualSphereMesh sm = *this;
			sm.clearInner();
			sm.boundary_edges_.clear();
			for (int k = 0; k < nodes.size(); k++) {
				sm.boundary_edges_.push_back(VEDGE{nodes[k],nodes[(k+1)% nodes.size()]});
			}
			sm.caculateMaxSkewness();
			bool take_it_into_consideration = true;
			if (sm.max_skewness_ > 0.999) {
				take_it_into_consideration = false;
			}
			for (int i = 0; i < sm.boundary_edges_.size(); i++) {
				int n1 = sm.boundary_edges_[i][0]; int n2 = sm.boundary_edges_[i][1];
				if ((!sm.virtual_point_lists_[n1].isFarNode()) && (!sm.virtual_point_lists_[n2].isFarNode())) {
					if (ANGLE(sm.virtual_point_lists_[n1].getCoord(), sm.virtual_point_lists_[n2].getCoord())<90) {
						take_it_into_consideration = false;
					}
				}
			}
			if (take_it_into_consideration)
				ans.push_back(sm);
		}
	}
	return ans;
}


void VirtualSphereMesh::addNodeOpt()
{
	BLVector ans;
	for (int i = 0; i < boundary_edges_.size(); i++) {
		ans = ans + virtual_point_lists_[boundary_edges_[i][0]].getCoord();
	}

	int index = virtual_point_lists_.size();
	ans.normalize();
	virtual_point_lists_.push_back(VPoint());
	virtual_point_lists_.back().setCoord(ans);
	virtual_point_lists_.back().setIndex(index);

	for (int i = 0; i < boundary_edges_.size(); i++) {
		VTriangle t;
		t.point_index_[0] = boundary_edges_[i][0];
		t.point_index_[1] = boundary_edges_[i][1];
		t.point_index_[2] = index;
		triangle_lists_.push_back(t);
	}
	valid.erase(NO_INSERT);
	valid.insert(ONE_INSERT);
	add_node_count=1;

	int count = MAX_INTERATION;
	while (count--) {
		vector<BLVector> neighbour_normals;
		for (auto j : triangle_lists_) {
			BLVector n1 = virtual_point_lists_[j.point_index_[1]].getCoord() - virtual_point_lists_[j.point_index_[0]].getCoord();
			BLVector n2 = virtual_point_lists_[j.point_index_[2]].getCoord() - virtual_point_lists_[j.point_index_[1]].getCoord();
			neighbour_normals.push_back((n1^n2).normalized());
		}
		virtual_point_lists_[index].setCoord(GEEOMETRY_FUNCTION::getMostNormal(neighbour_normals));
	}
	

}

double VirtualSphereMesh::AddNodeSphereSpr()
{
	if (VirtualSphereMesh::best_result.find(valid) != VirtualSphereMesh::best_result.end()) {
		*this = VirtualSphereMesh::best_result[valid];
		return max_skewness_;
	}
	removeInvalidNode();
	int point_count = getValidPointCount();
	switch (point_count)
	{
	case 1:
		break;
	case 2:
		max_skewness_ = getEdgeSkewness(0);
		break;
	case 3:
		addNode();
		if(isAddnode())
		for(int k=0;k<3;k++)
			max_skewness_ =max(max_skewness_, getTriSkewness(k));
		break;
	default:
		int p1 = boundary_edges_[0][0];
		int p2 = boundary_edges_[0][1];
		VirtualSphereMesh best_one;
		double min_skewness = 1.0;
		for (int i = 0; i < virtual_point_lists_.size(); i++) {
			if (valid.find(virtual_point_lists_[i].getIndex()) == valid.end())
				continue;
			if (virtual_point_lists_[i].getIndex() == p1 || virtual_point_lists_[i].getIndex() == p2)
				continue;
			int p3 = virtual_point_lists_[i].getIndex();
			std::array<int, 3> my_tri{ p1,p2,p3 };

			// create new edges
			VirtualSphereMesh left = *this;
			VirtualSphereMesh right = *this;

		

			left.boundary_edges_ = findBoundaryBetween(p1, p3);
			right.boundary_edges_ = findBoundaryBetween(p3, p2);

			
			VirtualSphereMesh median;
			median.virtual_point_lists_ = this->virtual_point_lists_;
			median.boundary_edges_.resize(3);
			median.boundary_edges_[0] = std::array<int, 2>{p1, p2};
			median.boundary_edges_[1] = std::array<int, 2>{p2, p3};
			median.boundary_edges_[2] = std::array<int, 2>{p3, p1};

			median.SphereSpr();
			median.removeInvalidNode(); 
			double tri_skewness = getTriSkewness(my_tri);


			VirtualSphereMesh left_with_node = left;
			VirtualSphereMesh right_with_node = right;



			//left.saveBoundaryVtk("leftBoundary.vtk");
			//right.saveBoundaryVtk("rightBoundary.vtk");
			//median.saveBoundaryVtk("medianBoundary.vtk");

			double right_skewness = right.SphereSpr();
			double leaf_skewness = left.SphereSpr();

			//left.saveTriVtk("left.vtk");
			//right.saveTriVtk("right.vtk");
			//median.saveTriVtk("median.vtk");
			left_with_node.saveBoundaryVtk("leftBoundary.vtk");
			right_with_node.saveBoundaryVtk("rightBoundary.vtk");

			double left_with_node_skewness = left_with_node.AddNodeSphereSpr();
			double right_with_node_skewness = right_with_node.AddNodeSphereSpr();

			left_with_node.saveTriVtk("left.vtk");
			left_with_node.saveTriVtk("right.vtk");

			VirtualSphereMesh result[3];
			result[0] = VirtualSphereMesh::mergeMesh(left, median);
			result[0] = VirtualSphereMesh::mergeMesh(result[0], right_with_node);			
			result[0].max_skewness_ = max(leaf_skewness, tri_skewness);
			result[0].max_skewness_ = max(result[0].max_skewness_, right_with_node_skewness);
			result[0].removeInvalidNode();

			result[1] = VirtualSphereMesh::mergeMesh(left_with_node, median);
			result[1] = VirtualSphereMesh::mergeMesh(result[1], right);
			result[1].max_skewness_ = max(right_skewness, tri_skewness);
			result[1].max_skewness_ = max(result[1].max_skewness_, left_with_node_skewness);
			result[1].removeInvalidNode();

			result[2] = *this;
			result[2].addNode();


			for (int k = 0; k < 3; k++) {				
				if(result[k].isAddnode())
				if (min_skewness > result[k].max_skewness_&&result[k].triangle_lists_.size() == point_count) {
					min_skewness = result[k].max_skewness_;
					best_one = result[k];
				}
			}
			
		
		}
		*this = best_one;

		break;
	}

	removeInvalidNode();
	VirtualSphereMesh::best_result[valid] = *this;
	if (max_skewness_ < 0 || max_skewness_>2)
		throw std::logic_error("error_skewness");
	return max_skewness_;
	//*this = VirtualSphereMesh::best_result[valid];

}

double VirtualSphereMesh::SphereSpr()
{
	


	if (VirtualSphereMesh::best_result.find(valid) != VirtualSphereMesh::best_result.end()) {
		*this = VirtualSphereMesh::best_result[valid];
		return max_skewness_;
	}
	removeInvalidNode();
	int point_count = getValidPointCount();
	switch (point_count)
	{
	case 1:
		break;
	case 2:
		max_skewness_ = getEdgeSkewness(0);
		//if (skewness > 0.85)
		//	cout << "warning! low quality mesh find!" << endl;
		break;
	case 3:
		addTriangle(std::array<int, 3>{boundary_edges_[0][0], boundary_edges_[1][0], boundary_edges_[2][0]});
		max_skewness_ = getTriSkewness(0);
		//if (skewness > BAD_QUALITY)
		//	cout << "warning! low quality mesh find!" << endl;
		break;
	default:
		int p1 = boundary_edges_[0][0];
		int p2 = boundary_edges_[0][1];
		VirtualSphereMesh best_one;
		double min_skewness = 1.0;
		for (int i = 0; i < virtual_point_lists_.size(); i++) {
			if (valid.find(virtual_point_lists_[i].getIndex()) == valid.end())
				continue;
			if (virtual_point_lists_[i].getIndex() == p1 || virtual_point_lists_[i].getIndex() == p2)
				continue;
			int p3 = virtual_point_lists_[i].getIndex();
			std::array<int, 3> my_tri{ p1,p2,p3 };

			// create new edges
			VirtualSphereMesh left = *this;
			VirtualSphereMesh right = *this;


			VirtualSphereMesh median;
			//saveBoundaryVtk("test_bdry1.vtk");

			left.boundary_edges_ = findBoundaryBetween(p1, p3);
			right.boundary_edges_ = findBoundaryBetween(p3, p2);


			
			median.virtual_point_lists_ = this->virtual_point_lists_;
			median.boundary_edges_.resize(3);
			median.boundary_edges_[0] = std::array<int, 2>{p1, p2};
			median.boundary_edges_[1] = std::array<int, 2>{p2, p3};
			median.boundary_edges_[2] = std::array<int, 2>{p3, p1};


			median.SphereSpr();
			median.removeInvalidNode();


			 
			int size = valid.size();

			

			left.removeInvalidNode();
			right.removeInvalidNode();

			//left.saveBoundaryVtk("leftBoundary.vtk");
			//right.saveBoundaryVtk("rightBoundary.vtk");
			//median.saveBoundaryVtk("medianBoundary.vtk");

			double right_skewness = right.SphereSpr();
			double leaf_skewness = left.SphereSpr();

			//left.saveTriVtk("left.vtk");
			//right.saveTriVtk("right.vtk");
			//median.saveTriVtk("median.vtk");

			auto result = VirtualSphereMesh::mergeMesh(left, median);
			result = VirtualSphereMesh::mergeMesh(result, right);

			result.removeInvalidNode();
			double tri_skewness = getTriSkewness(my_tri);
			result.max_skewness_ = max(leaf_skewness, right_skewness);
			result.max_skewness_ = max(result.max_skewness_, tri_skewness);
			if (min_skewness > result.max_skewness_&&result.triangle_lists_.size() == point_count - 2) {
				min_skewness = result.max_skewness_;
				best_one = result;
			}

		}
		*this = best_one;

		break;
	}

	removeInvalidNode();
	VirtualSphereMesh::best_result[valid] = *this;
	return max_skewness_;
	//*this = VirtualSphereMesh::best_result[valid];


	

}

bool VirtualSphereMesh::isFullBoundary()
{
	set<int> sphere_nodes;
	for (int i = 0; i < virtual_point_lists_.size(); i++) {
		if(!virtual_point_lists_[i].isFarNode())
			sphere_nodes.insert(virtual_point_lists_[i].getIndex());
	}
	int count = sphere_nodes.size();
	for (int i = 0; i < boundary_edges_.size(); i++) {
		if (!virtual_point_lists_[boundary_edges_[i][0]].isFarNode()) {
			sphere_nodes.erase(virtual_point_lists_[boundary_edges_[i][0]].getIndex());
		}
		else {
			count--;
		}
	}

	return sphere_nodes.empty() && count == 0;
}

bool VirtualSphereMesh::isBoundaryValid()
{
	if (boundary_edges_.size() > 12)
		return false;
	// intersection check
	for (auto i: neighbour_tri_index_) {
		int p1 = i.first;
		for (auto j : i.second) {
			int p2 = j[0];
			int p3 = j[1];
			double line[2][3];
			double face[3][3];
			for (int k = 0; k < 3; k++) {
				face[0][k] = virtual_point_lists_[p1].getCoord()[k];
				face[1][k] = virtual_point_lists_[p2].getCoord()[k];
				face[2][k] = virtual_point_lists_[p3].getCoord()[k];
			}
			for (int l = 0; l < boundary_edges_.size(); l++) {
				int p4 = boundary_edges_[l][0];
				int p5 = boundary_edges_[l][1];
				if (p4!=p1&&p4!=p2&&p4!=p3) 
					if (p5 != p1 && p5 != p2 && p5 != p3) {
					
					for (int k = 0; k < 3; k++) {
						line[0][k] = virtual_point_lists_[p4].getCoord()[k];
						line[1][k] = virtual_point_lists_[p5].getCoord()[k];
					}
					int int_type;
					int int_code = 0;
					double int_pnt[3];
					TiGER_GEOM_FUNC::lin_tri_intersect3d(line, face, &int_type, &int_code, int_pnt, false);
					if (int_type== TiGER_GEOM_FUNC::LTI_INTERSECT_FAC)
						return false;
				}
			}
			
		}
	}

	// proximate check
	double min_distance = std::numeric_limits<double>::max();
	for (int i = 0; i < boundary_edges_.size();i++) {
		for (int j = i+1; j < boundary_edges_.size(); j++) {
			min_distance = min(min_distance, (virtual_point_lists_[i].getCoord()- virtual_point_lists_[j].getCoord()).magnitude2());
		}
	}
	if (min_distance < 1e-3)
		return false;

	if (caculateMaxSkewness() > 0.999999)
		return false;
	return true;
}

bool VirtualSphereMesh::intersectionCheck()
{
	for (int i = 0; i < triangle_lists_.size(); i++) {
		for (int j = i+1; j < triangle_lists_.size(); j++) {
			double t1[3][3], t2[3][3];
			int shared = 0;
			for (int k = 0; k < 3; k++)
				for (int l = 0; l < 3; l++)
					if (triangle_lists_[i].point_index_[k] == triangle_lists_[j].point_index_[l])
						shared = 0;
			for (int k = 0; k < 3 ; k++) {
				t1[0][k]= virtual_point_lists_[triangle_lists_[i].point_index_[0]].getCoord()[k];
				t1[1][k] = virtual_point_lists_[triangle_lists_[i].point_index_[1]].getCoord()[k];
				t1[2][k] = virtual_point_lists_[triangle_lists_[i].point_index_[2]].getCoord()[k];
				t2[0][k] = virtual_point_lists_[triangle_lists_[j].point_index_[0]].getCoord()[k];
				t2[1][k] = virtual_point_lists_[triangle_lists_[j].point_index_[1]].getCoord()[k];
				t2[2][k] = virtual_point_lists_[triangle_lists_[j].point_index_[2]].getCoord()[k];
			}
			if (shared == 0)
				if (TiGER_GEOM_FUNC::tri_tri_overlap_test_3d(t1[0], t1[1], t1[2], t2[0], t2[1], t2[2]))
					return true;
			if(shared ==1)
				if (TiGER_GEOM_FUNC::one_node_same_tri_tri_overlap_3d(t1[0], t1[1], t1[2], t2[0], t2[1], t2[2]))
					return true;
		}
	}
	
	return false;
}
