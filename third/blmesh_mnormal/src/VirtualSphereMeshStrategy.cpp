#include "VirtualSphereMeshStrategy.h"
#include "../include/MeshEvaluation.h"
#include "../include/PointOptimizer.h"
#include "VirtualSphereMeshHasher.h"
#include "common.h"
#include <ctime>

VirtualSphereMesh VirtualSphereStrategy::noInsert()
{
	double best_quality = 1;
	double first_max_skewness = 2;
	VirtualSphereMesh initial = input_;
	VirtualSphereMesh tmp = input_;
	VirtualSphereMesh output;
	int count = MAX_INTERATION;
	while (count--)
	{
		tmp = initial;
		clock_t time_start = clock();
		generateInitialTriangles(tmp);
		clock_t time_end = clock();

		double quality = MeshEvaluator::GetSingleton().GetQuality(tmp);
		if (tmp.triangle_lists_.empty())
			break;
		if (quality < best_quality) {
			best_quality = quality;
			output = tmp;
		}
		tmp.smoothBoundaryNode();
		initial.virtual_point_lists_ = tmp.virtual_point_lists_;
	}
	return output;
}
VirtualSphereMesh VirtualSphereStrategy::OneInsert()
{


	double best_max_skewness = 1;
	double best_ave_skewness = 1;
	double first_max_skewness = 2;
	VirtualSphereMesh initial = input_;
	VirtualSphereMesh tmp = input_;
	VirtualSphereMesh output;
	int count = MAX_INTERATION;
	while (count--)
	{
		tmp = initial;
		addNodeSphereSpr(tmp);
		double max_skewness = tmp.caculateMaxSkewness();
		double ave_skewness = tmp.caculateAveragySkewness();
		if (tmp.triangle_lists_.empty())
			break;
		output = tmp;
		if (first_max_skewness == 2)
			first_max_skewness = max_skewness;
		if (max_skewness > best_max_skewness)
			break;
		if (max_skewness == best_max_skewness && ave_skewness*1.01 > best_ave_skewness)
			break;
		best_max_skewness = max_skewness;
		best_ave_skewness = ave_skewness;
		//cout << "best_max_skewness="<< best_max_skewness << endl;
		tmp.smoothBoundaryNode();
		initial.virtual_point_lists_ = tmp.virtual_point_lists_;
		initial.virtual_point_lists_.pop_back();
	}

	//cout << "======================" << endl;
	return output;
}
VirtualSphereMesh VirtualSphereStrategy::generate()
{
	auto no_insert = noInsert();
	auto one_insert = OneInsert();

	if (no_insert.triangle_lists_.empty())
		return one_insert;
	if (one_insert.triangle_lists_.empty())
		return no_insert;

	if (no_insert.max_skewness_ < one_insert.max_skewness_)
		return no_insert;

	return no_insert;
}
void VirtualSphereStrategy::sphereSpr(VirtualSphereMesh& input_boundary) {
	VirtualSphereMesh::best_result.clear();
	VirtualSphereMesh test = input_;


	auto shell = input_boundary;
	auto tris = input_boundary.LocalTriangulation();
	//input_boundary.saveBoundaryVtk("BOUNDARY.vtk");
	//for (int i = 0; i < tris.size(); i++) {

	//	tris[i].saveTriVtk("TRI" + std::to_string(i) + ".vtk");
	//}

	double min_max_skewness = 1.0;
	double min_ave_skewness = 1.0;

	for (int i = 0; i < tris.size(); i++) {
		if (!tris[i].triangle_lists_.size())
			continue;
		shell.triangle_lists_ = tris[i].triangle_lists_;
		double shell_skewness = shell.caculateMaxSkewness();
		if (min_max_skewness > shell_skewness) {
			min_max_skewness = shell_skewness;
			input_boundary = shell;
			min_ave_skewness = 1.0;
		}
		else if (min_max_skewness == shell_skewness) {
			if (min_ave_skewness > shell.caculateAveragySkewness()) {
				min_ave_skewness = shell.caculateAveragySkewness();
				input_boundary = shell;
			}
		}
	}
	input_boundary.removeInvalidNode();
}
//#pragma optimize("",off)
void VirtualSphereStrategy::addNodeSphereSpr(VirtualSphereMesh & input_boundary)
{

	VirtualSphereMesh::best_result.clear();

	MeshBoundaryMap::instance()->clear();

	double max_sk_o = 0.999;
	double max_ave_sk_o = 0.999;
	auto boundaries = input_boundary.getAllPossibleSubBoundary();


	VirtualSphereMesh best_final;

	for (auto inner : boundaries) {
		auto other_boundary=VirtualSphereMesh::MeshBoundaryComplementary(inner, input_boundary.boundary_edges_.size());

		std::vector<std::vector<VirtualSphereMesh>> boundary_edge_result(other_boundary.size());
		for (int i = 0; i < other_boundary.size();i++) {
			boundary_edge_result [i] = other_boundary[i].LocalTriangulation();
		}
		
		inner.addNodeOpt();

		// There exist a compomise of efficency
		VirtualSphereMesh best_mesh = inner;
		VirtualSphereMesh middle_mesh = inner;
		for (int i = 0; i < other_boundary.size(); i++) {
			double max_sk = 0.999;
			double max_ave_sk = 0.999;
			for (int j = 0; j < boundary_edge_result[i].size(); j++) {
				VirtualSphereMesh tmp = best_mesh;
				tmp = VirtualSphereMesh::mergeMesh(tmp, boundary_edge_result[i][j]);
				double max_skewness=tmp.caculateMaxSkewness();
				double ave_skewness = tmp.caculateAveragySkewness();
				if (max_skewness < max_sk || (max_skewness == max_sk && ave_skewness < max_ave_sk)) {
					max_sk = max_skewness;
					max_ave_sk = ave_skewness;
					middle_mesh = tmp;
				}
			}
			best_mesh = middle_mesh;
		}
		double max_skewness = best_mesh.caculateMaxSkewness();
		double ave_skewness = best_mesh.caculateAveragySkewness();

		if (input_boundary.boundary_edges_.size()== best_mesh.boundary_edges_.size()&& max_sk_o > max_skewness || (max_skewness == max_sk_o && ave_skewness < max_ave_sk_o)) {
			max_sk_o = max_skewness;
			max_ave_sk_o = ave_skewness;
			best_final = best_mesh;



			//std::set<std::array<int, 2>> check;
			//for (int i = 0; i <; i++) {
			//	check.insert(std::array<int, 2>{input_boundary.boundary_edges_[i][0], input_boundary.boundary_edges_[i][1]});
			//}
			//for (int i = 0; i < best_final.boundary_edges_.size(); i++) {
			//	if (check.find(std::array<int, 2>{best_final.boundary_edges_[i][0], best_final.boundary_edges_[i][1]}) == check.end()) {

			//		inner.saveBoundaryVtk("b.vtk");
			//		input_boundary.saveBoundaryVtk("input.vtk");
			//		int count = 0;
			//		for (auto j : other_boundary) {
			//			j.saveBoundaryVtk("other" + std::to_string(count++) + ".vtk");
			//		}
			//		throw std::logic_error("wrone insert topo!");
			//	}
			//}


		}



		//inner.saveBoundaryVtk("b.vtk");
		//input_boundary.saveBoundaryVtk("input.vtk");
		//int count = 0;
		//for (auto i : other_boundary) {
		//	i.saveBoundaryVtk("other"+std::to_string(count++)+".vtk");
		//}
	}


	input_boundary = best_final;

}
//#pragma optimize("",on)
void VirtualSphereStrategy::generateInitialTriangles(VirtualSphereMesh& input_boundary) {

	auto no_insert_front = input_boundary;
	VirtualSphereMesh::best_result.clear();

	MeshBoundaryMap::instance()->clear();
	sphereSpr(no_insert_front);
	
	input_boundary= no_insert_front;

}

