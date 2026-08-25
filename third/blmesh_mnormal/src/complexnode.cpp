#include "complexnode.h"
#include "spdlog/spdlog.h"
#include "geometryfunction.h"
#include "MeshSplitter.h"
#include "Splitter.h"
#include <ctime>
#include <algorithm>
#include <set>

ComplexNode::ComplexNode(){
	
}

void ComplexNode::ConfigureSplit(double plane_skewness,
	double convex_skewness, int maximum_strategy_count)
{
	plane_skewness_ = plane_skewness;
	convex_skewness_ = convex_skewness;
	maximum_strategy_count_ = maximum_strategy_count;
}

double ComplexNode::CaculateVisableAngle(BLVector normal)
{
	return GEEOMETRY_FUNCTION::getMinCos(normal, this->neighbour_front_direction_);
}

void ComplexNode::SplitNode()
{
    if (neighbour_node_.size() < 2) {
        spdlog::info("Error neighbour node id = {}", node_id_);
        throw std::runtime_error("Error neighbour node");
    }
	
	set<int> splitters;
	map<int, double> angles;
	int count = 0;
	do {
		splitters.clear();
		angles.clear();

		for (int i = 0; i < neighbour_node_.size(); i++) {
			std::size_t back = (i - 1 + neighbour_node_.size()) % neighbour_node_.size();
			std::size_t front = (i + 1 + neighbour_node_.size()) % neighbour_node_.size();
			auto back_center = (neighbour_node_[i]->coordinate + neighbour_node_[back]->coordinate) / 2;
			auto front_center = (neighbour_node_[i]->coordinate + neighbour_node_[front]->coordinate) / 2;

			double a;
			if (neighbour_front_direction_[i].normalized() * ((front_center - back_center)).normalized() > 0)
				a = 0.5 - neighbour_front_direction_[i].normalized() * neighbour_front_direction_[back].normalized() / 2;
			else
				a = neighbour_front_direction_[i].normalized() * neighbour_front_direction_[back].normalized() / 2 - 0.5;


			if (a > plane_skewness_ + count*0.01)// i is a is convex edge for this node
			{
				angles[i] = a;
				splitters.insert(i);
			}
		}
		count++;
	} while (splitters.size() > 16);/// 2^16 == 8096 

	auto combination =Splitter::instance()->getSpliiter(splitters);
	for (auto splitter : combination) {
		int num_convex = 0;
		for (auto i : splitter)
			if (angles[i] > convex_skewness_)
				num_convex++;
		if (splitter.size() - num_convex > MAX_PLAIN_RIDGE || num_convex==0)
			continue;
		//cout << splitter.size() - num_convex<<" "<<num_convex << endl;
		vector<vector<int>> classification(1);
		for (int i = 0; i < neighbour_node_.size(); i++) {
			if (splitter.find(i) == splitter.end()) {
				classification.back().push_back(i);
			}
			else {
				classification.push_back(vector<int>());
				classification.back().push_back(i);
			}
		}
		for (auto i : classification.front())
			classification.back().push_back(i);


		for (int i = 0; i < classification.size() - 1; i++) {
			classification[i] = classification[i + 1];
		}
		classification.pop_back();


		if (classification.size() != splitter.size())
			throw std::runtime_error("logic error");
		// get convex edge and its cooresponding marching direction
		vector<BLVector> convex_edge;
		for (auto i : splitter) {
			convex_edge.push_back(FARSCALE * (neighbour_node_[i]->coordinate - coordinate).normalized());
		}

		vector<BLVector> most_direction;
		for (int i = 0; i < classification.size(); i++) {
			vector<BLVector> neighbour_direction;
			for (auto j : classification[i])
				neighbour_direction.push_back(neighbour_front_direction_[j]);
			BLVector most_normal = GEEOMETRY_FUNCTION::getMostNormal(neighbour_direction).normalized();
			most_direction.push_back(most_normal);
		}

		auto classification_bak = classification;
		for (int i = 0; i < classification.size(); i++) {
			for (auto &j : classification[i]) {
				j = neighbour_front_index_[j];
			}
		}
		vector<int> splitter_id;
		for (auto i : splitter)
			splitter_id.push_back(neighbour_node_[i]->node_id_);
		generator_.setSingleSkew(original_skewness_);

		VirtualSphereStrategy vs_strategy;

		vs_strategy.input_ = VirtualSphereMesh(convex_edge, most_direction, classification, splitter_id);



		/* add manifold triangles */
		std::vector<std::vector<std::array<int, 2>>> manifold_triangles;
		std::map<int, BLVector> manifold_coordinate;

		int count = convex_edge.size() * 2;
		for (int i = 0; i < classification_bak.size(); i++) {
			for (int j = 0; j < classification_bak[i].size(); j++) {
				manifold_coordinate[count++] = FARSCALE * (neighbour_node_[classification_bak[i][j]]->coordinate - coordinate).normalized();
			}
		}
		count = convex_edge.size() * 2;
		for (int i = 0; i < classification_bak.size(); i++) {
			manifold_triangles.push_back(std::vector<std::array<int, 2>>());
			for (int j = 0; j < classification_bak[i].size(); j++) {
				manifold_triangles.back().push_back(std::array<int, 2>{count, count + 1});
				count++;
			}
		}
		manifold_triangles.back().back()[1] = convex_edge.size() * 2;


		// this funtion should only called after VirtualSphereMeshGenerator::setBoundary
		vs_strategy.input_.addBoundaryTriangles(manifold_triangles, manifold_coordinate);
		if(vs_strategy.input_.isBoundaryValid())
			generator_.addStrategy(vs_strategy);
	}
	/* generate virtual mesh */
	if (generator_.getNumberofProblem() >= maximum_strategy_count_) {
		generator_.cutStrategies(maximum_strategy_count_);
	}

	int number_of_complex_nodes=0;
	if (generator_.getNumberofProblem()) {
		spdlog::info("////////////////Node {0}//////////////////",node_id_);
		generator_.generate();
	}
	

	clock_t end_time=clock();
		

	/*try to fix topologic*/

}

VirtualSphereMesh & ComplexNode::getFinalMesh()
{
	return generator_.getFinalMesh();
}
