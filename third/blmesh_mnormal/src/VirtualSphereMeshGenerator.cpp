#include "VirtualSphereMeshGenerator.h"
#include "spdlog/spdlog.h"
#include "../include/MeshEvaluation.h"
#include "VirtualSphereMeshHasher.h"
#include "common.h"
#include <algorithm>

VirtualSphereMeshGenerator::VirtualSphereMeshGenerator()
{
}

VirtualSphereMeshGenerator::~VirtualSphereMeshGenerator()
{
}

std::size_t VirtualSphereMeshGenerator::getNumberofProblem()
{
	return strategies_.size();
}

void VirtualSphereMeshGenerator::addStrategy(VirtualSphereStrategy vs)
{
	strategies_.push_back(vs);
}

void VirtualSphereMeshGenerator::cutStrategies(int max_number_of_problem)
{
	for (int i = 0; i < strategies_.size(); i++) {
		strategies_[i].input_.caculateAveragySkewness();
		strategies_[i].input_.caculateMaxSkewness();
	}
	vector<pair<pair<double,double>, int>> vsp;
	for (int i = 0; i < strategies_.size(); i++) {
		vsp.push_back(pair<pair<double, double>, int > {pair<double, double>{strategies_[i].input_.max_skewness_, strategies_[i].input_.ave_skewness_}, i});
	}
	std::sort(vsp.begin(), vsp.end());
	std::vector<VirtualSphereStrategy> tmp;
	for (int i = 0; i < max_number_of_problem; i++) {
		tmp.push_back(strategies_[vsp[i].second]);
	}
	strategies_ = tmp;
}



void VirtualSphereMeshGenerator::setSingleSkew(double d)
{
	single_normal_skewness_ = d;
}

double VirtualSphereMeshGenerator::getSingleSkew()
{
	return single_normal_skewness_;
}


VirtualSphereMesh VirtualSphereMeshGenerator::generateAddOneNodeTriangles()
{
	auto add_node_front = output_;
	add_node_front.valid.insert(ONE_INSERT);
	AddNodeSphereSpr(add_node_front);
	return add_node_front;

}

void VirtualSphereMeshGenerator::generate()
{
	double initial_skewness= getSingleSkew();
	double best_skewness = initial_skewness*10;
	double first_max_skewness = 2;

	for (int i = 0; i < strategies_.size();i++) {
		//cout << i << endl;
		VirtualSphereMesh mesh = strategies_[i].generate();
		if (mesh.triangle_lists_.empty())
			continue;
		double quality = MeshEvaluator::GetSingleton().GetQuality(mesh);

		if (first_max_skewness == 2)
			first_max_skewness = quality;
		/************************************************************************/
		/* At least improve %1                                                  */
		/************************************************************************/
		if (quality*1.01 < best_skewness) {
			best_skewness = quality;
		}
		else
			continue;

		

		//cout << "#########################" << endl;
		//cout << best_max_skewness << endl;
		//cout << "#########################" << endl;

		output_ = mesh;

	}
	output_.removeInvalidNode();
	static int c = 0;
	//output_.saveTriVtk("TRI"+std::to_string(c)+".vtk");
	//output_.saveBoundaryVtk("BOUNDARY" + std::to_string(c) + ".vtk");
	if (!output_.triangle_lists_.size()) {
		spdlog::warn("Concave Node Found! do nothing!");
		cout << endl;
		return;
	}
	spdlog::info("single skewness   ={:03.2f}",initial_skewness);
	spdlog::info("first skewness    ={:03.2f}",first_max_skewness);
	if(best_skewness>0.8)
		spdlog::warn("final skewness ={:03.2f}",best_skewness);
	else
		spdlog::info("final skewness    ={:03.2f}",best_skewness);
	//auto mesh2 = generateAddOneNodeTriangles();
	//if (mesh1.max_skewness_ < mesh2.max_skewness_) {
	//	input_ = mesh1;
	//}
	//else {
	////	input_ = mesh2;
	//}


}

VirtualSphereMesh & VirtualSphereMeshGenerator::getFinalMesh()
{
	return output_;
}
void VirtualSphereMeshGenerator::AddNodeSphereSpr(VirtualSphereMesh & input_boundary)
{
	VirtualSphereMesh::best_result.clear();
	VirtualSphereMesh test = input_boundary;
	test.valid.erase(NO_INSERT);
	test.valid.insert(ONE_INSERT);
	input_boundary.AddNodeSphereSpr();
	input_boundary = VirtualSphereMesh::best_result[input_boundary.valid];
	cout << "===============================================================" << endl;
	cout << "maximum skewness=" << input_boundary.max_skewness_ << endl;
	cout << "===============================================================" << endl;
	if (input_boundary.virtual_point_lists_.size() > 5) {
		static int count = 0;
		input_boundary.saveTriVtk("TRI"+ to_string(count) +".vtk");
		input_boundary.saveBoundaryVtk("BOUNDARY" + to_string(count++) + ".vtk");
	}

}


void VirtualSphereMeshGenerator::LawsonFlip()
{

}
