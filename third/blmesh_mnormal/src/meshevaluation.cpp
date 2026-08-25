#include "MeshEvaluation.h"
#include <array>
#include <set>
#include <map>
#include <cmath>



void MeshEvaluator::SetDefaultParameter(double m, double n)
{
	mu_ = m;
	nu_ = n;
}

double MeshEvaluator::GetQuality(VirtualSphereMesh &node)
{
	int virtual_node_count = 0;
	for (int i = 0; i < node.virtual_point_lists_.size(); i++) {
		if (!node.virtual_point_lists_[i].isFarNode())
			virtual_node_count++;
	}

	std::array<int, 4> order = { 0 , 0, 0 , 0 };

	for (int i = 0; i < node.triangle_lists_.size(); i++) {
		int count = 0;
		for (int j = 0; j < 3; j++) {
			if (!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[j]].isFarNode())
				count++;
		}
		order[count]++;

	}
//	cout << "Quality=" << WorstQuality(node)*(1 - mu_) + mu_ * AveragyQuality(node) + nu_ * virtual_node_count << endl;
	return WorstQuality(node)*(1 - mu_) + mu_ * AveragyQuality(node) + order[2]*ONE_ORDER_TRI_COST+order[3]*TWO_ORDER_TRI_COST;
}

void MeshEvaluator::GetWorstQualityHandler(VirtualSphereMesh & node, QualityHandler & handler)
{
	int worst_point_index=0;
	int worst_A_index=0;
	int worst_interation_index = 0;
	double point_quality = 0;//best = 0, worst = 1. 
	for (int index = 0; index < node.virtual_point_lists_.size(); index++) {
		auto neighbour_triangles = node.neighbour_tri_index_[index];
		auto inner_triangles = node.triangle_lists_;
		// far node don't need to evaluate quality
		if (node.virtual_point_lists_[index].isFarNode())
			continue;
		//quality of A e,1:
		int count = 0;
		for (auto i : neighbour_triangles) {
			int index1 = i[0];
			int index2 = i[1]; 
			// index1, index2, index from a triangle
			BLVector normal = (node.virtual_point_lists_[index1].getCoord() - node.virtual_point_lists_[index].getCoord()) ^ (node.virtual_point_lists_[index2].getCoord() - node.virtual_point_lists_[index].getCoord());
			normal.normalize();
			double value = acos(normal*node.virtual_point_lists_[index].getCoord()) * 2 / PI;
			if (value > point_quality) {
				point_quality = value;
				worst_point_index = index;
				worst_A_index = 1;
				worst_interation_index = count;
			}
			count++;
		}

		//quality of A e,2
		for (int i = 0; i < node.triangle_lists_.size(); i++) {
			for (int j = 0; j < 3; j++)
				if (node.triangle_lists_[i].point_index_[j] == index && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode()) {
					double value = asin(abs(node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord()*node.virtual_point_lists_[index].getCoord())) * 2 / PI;
					if (value > point_quality) {
						point_quality =value;
						worst_point_index = index;
						worst_A_index = 2;
						worst_interation_index = i;
					}
				}
			}

		//quality of A e,3
		for (int i = 0; i < node.triangle_lists_.size(); i++) {
			for (int j = 0; j < 3; j++)
				if (node.triangle_lists_[i].point_index_[j] == index) {
					if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						//only one far node
						BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
						BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
						far_normal.normalize(); p_jp_i.normalize();
						double value = asin(abs(far_normal*p_jp_i)) * 2 / PI;
						if (value > point_quality) {
							point_quality = value;
							worst_point_index = index;
							worst_A_index = 3;
							worst_interation_index = i;
						}
					}
					if ((node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						//only one far node
						BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
						BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
						far_normal.normalize(); p_jp_i.normalize();
						double value = asin(abs(far_normal*p_jp_i)) * 2 / PI;
						if (value> point_quality) {
							point_quality = value;
							worst_point_index = index;
							worst_A_index = 3;
							worst_interation_index = node.triangle_lists_.size() + i;
						}
					}
				}

		}

		//quality of A e,4
		for (int i = 0; i < node.triangle_lists_.size(); i++) {
			for (int j = 0; j < 3; j++) {
			
				if (node.triangle_lists_[i].point_index_[j] == index) {
					if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						//only one far node
						BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
						BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
						far_normal.normalize(); p_jp_i.normalize();
						BLVector facet_normal = -1* (p_jp_i ^ far_normal).normalized();
						double value= acos(facet_normal * node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()) * 2 / PI;
						if (value > point_quality) {
							point_quality = value;
							worst_point_index = index;
							worst_A_index = 4;
							worst_interation_index = i;
						}
					}
					if ((node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						//only one far node
						BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
						BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
						far_normal.normalize(); p_jp_i.normalize();
						BLVector facet_normal = (p_jp_i ^ far_normal).normalized();
						double value= acos(facet_normal * node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()) * 2 / PI;
						if (value > point_quality) {
							point_quality = value;
							worst_point_index = index;
							worst_A_index = 4;
							worst_interation_index = i;
						}
					}
				}
			}
		}

		//quality of A e.5
		for (int i = 0; i < node.triangle_lists_.size(); i++) {
			for (int j = 0; j < 3; j++)
				if (node.triangle_lists_[i].point_index_[j] == index) {
					if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						BLVector e1 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();
						BLVector e2 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();

						point_quality = std::max(point_quality, acos(e1*e2) * 3 / PI);
						point_quality = std::max(point_quality, (PI - acos(e1*e2)) * 3 / 2 / PI);
						double value = std::max(acos(e1*e2) * 3 / PI, (PI - acos(e1*e2)) * 3 / 2 / PI);
						if (value > point_quality) {
							point_quality = value;
							worst_point_index = index;
							worst_A_index = 5;
							worst_interation_index = i;
						}
					}
				}
		}
	}

	//quality of A e.6
	//Question: may be we don't need A e.6?


	//for (int i = 0; i < node.triangle_lists_.size(); i++) {
	//	for (int j = 0; j < 3; j++)
	//		if (node.triangle_lists_[i].point_index_[j] == index) {
	//			if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
	//				BLVector e1 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();
	//				BLVector e2 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j+1) % 3]].getCoord()).normalized();
	//				BLVector facet_normal = (e1 ^ e2).normalized();
	//				point_quality = std::max(point_quality, acos(node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()) * 3 / PI);


	//			}
	//		}
	//}
	handler = (long long)(worst_point_index * 10000 + worst_A_index * 100 + worst_interation_index);

	return ;
}

double MeshEvaluator::GetQualityOfHandler(VirtualSphereMesh & node,const QualityHandler & handler)
{
	QualityHandler tmp = handler;
	int worst_point_index = tmp / 10000; tmp -= worst_point_index * 10000;
	int worst_A_index = tmp / 100; tmp -= worst_A_index * 100;
	int worst_iteration_index = tmp;

	int index = worst_point_index;
	auto neighbour_triangles = node.neighbour_tri_index_[index];
	auto inner_triangles = node.triangle_lists_;
	int count = 0;
	int i;
	switch (worst_A_index)
	{
	case 1:
		
		for (auto i : neighbour_triangles) {
			int index1 = i[0];
			int index2 = i[1];
			// index1, index2, index from a triangle
			BLVector normal = (node.virtual_point_lists_[index1].getCoord() - node.virtual_point_lists_[index].getCoord()) ^ (node.virtual_point_lists_[index2].getCoord() - node.virtual_point_lists_[index].getCoord());
			normal.normalize();
			if (count == worst_iteration_index) {
				return acos(normal*node.virtual_point_lists_[index].getCoord()) * 2 / PI;
			}
			count++;
		}
		break;
	case 2:
		i = worst_iteration_index;
		for (int j = 0; j < 3; j++){
			if (node.triangle_lists_[i].point_index_[j] == index && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode()) {
				double value = asin(abs(node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord()*node.virtual_point_lists_[index].getCoord())) * 2 / PI;
				return value;
			}
		}
		break;
	case 3:
		i = worst_iteration_index%node.triangle_lists_.size();
		for (int j = 0; j < 3; j++) {
			if (worst_iteration_index < node.triangle_lists_.size()) {
				if (node.triangle_lists_[i].point_index_[j] == index) {
					if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						//only one far node
						BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
						BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
						far_normal.normalize(); p_jp_i.normalize();
						return asin(abs(far_normal*p_jp_i)) * 2 / PI;
					}
				}
			}
			else {
				if ((node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					//only one far node
					BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
					BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
					far_normal.normalize(); p_jp_i.normalize();
					return asin(abs(far_normal*p_jp_i)) * 2 / PI;
				}
			}
		}
		break;
	case 4:
		 i = worst_iteration_index;
			for (int j = 0; j < 3; j++){
				if (node.triangle_lists_[i].point_index_[j] == index) {
					if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						//only one far node
						BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
						BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
						far_normal.normalize(); p_jp_i.normalize();
						BLVector facet_normal = -1* (p_jp_i ^ far_normal).normalized();
						return acos(facet_normal * node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()) * 2 / PI;
					}
					if ((node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						//only one far node
						BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
						BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
						far_normal.normalize(); p_jp_i.normalize();
						BLVector facet_normal =  (p_jp_i ^ far_normal).normalized();
						return acos(facet_normal * node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()) * 2 / PI;
					}
				}
		}
		break;
	case 5:
		 i = worst_iteration_index;
			for (int j = 0; j < 3; j++){
				if (node.triangle_lists_[i].point_index_[j] == index) {
					if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						BLVector e1 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();
						BLVector e2 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();

						double value = std::max(acos(e1*e2) * 3 / PI, (PI - acos(e1*e2)) * 3 / 2 / PI);
						return value;
					}
				}
		}

		break;
	case 6:
		throw std::logic_error("wrong handler");
		break;
	default:
		throw std::logic_error("wrong handler");
		break;
	}
	throw std::logic_error("wrong handler");

}

BLVector MeshEvaluator::GetGredientQualityHandler(VirtualSphereMesh & node,const QualityHandler & handler)
{
	QualityHandler tmp = handler;
	int worst_point_index = tmp / 10000; tmp -= worst_point_index * 10000;
	int worst_A_index = tmp / 100; tmp -= worst_A_index * 100;
	int worst_iteration_index = tmp;

	int index = worst_point_index;
	auto neighbour_triangles = node.neighbour_tri_index_[index];
	auto inner_triangles = node.triangle_lists_;
	BLVector ans;
	int count = 0,i=0;
	switch (worst_A_index)
	{
	case 1:
		
		for (auto i : neighbour_triangles) {
			int index1 = i[0];
			int index2 = i[1];
			// index1, index2, index from a triangle
			BLVector normal = (node.virtual_point_lists_[index1].getCoord() - node.virtual_point_lists_[index].getCoord()) ^ (node.virtual_point_lists_[index2].getCoord() - node.virtual_point_lists_[index].getCoord());
			normal.normalize();
			if (count == worst_iteration_index) {
				BLVector e_left = normal ^ node.virtual_point_lists_[index].getCoord();
				ans = (node.virtual_point_lists_[index].getCoord() ^ e_left).normalized();
				return ans;
			}
			count++;
		}
		break;
	case 2:
		i = worst_iteration_index;
		for (int j = 0; j < 3; j++) {
			if (node.triangle_lists_[i].point_index_[j] == index && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode()) {
				BLVector e_left = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord()^node.virtual_point_lists_[index].getCoord();
				ans = (node.virtual_point_lists_[index].getCoord() ^ e_left).normalized();
				BLVector n1 = node.virtual_point_lists_[index].getCoord();
				BLVector n2 = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
				double tmp = n1 * n2;
				if (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord() * node.virtual_point_lists_[index].getCoord() < 0)
					return ans;
				return -1*ans;
			}
		}
		break;
	case 3:
		i = worst_iteration_index % node.triangle_lists_.size();
		for (int j = 0; j < 3; j++) {
			if (worst_iteration_index < node.triangle_lists_.size()) {
				if (node.triangle_lists_[i].point_index_[j] == index) {
					if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
						//only one far node
						BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
						BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
						far_normal.normalize(); p_jp_i.normalize();
						if (far_normal*p_jp_i < 0) {// too big
							ans = -1 * far_normal.normalized();
						}
						else
							ans = far_normal.normalized();
						return ans;
					}
				}
			}
			else {
				if ((node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					//only one far node
					BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
					BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
					far_normal.normalize(); p_jp_i.normalize();
					far_normal.normalize(); p_jp_i.normalize();
					if (far_normal*p_jp_i < 0) {// too big
						ans = -1 * far_normal.normalized();
					}
					else
						ans = far_normal.normalized();
					return ans;
				}
			}
		}
		break;
	case 4:
		i = worst_iteration_index;
		for (int j = 0; j < 3; j++) {
			if (node.triangle_lists_[i].point_index_[j] == index) {
				if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					//only one far node
					BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
					BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
					far_normal.normalize(); p_jp_i.normalize();
					BLVector facet_normal =  -1*(p_jp_i ^ far_normal).normalized();

					BLVector e_left = facet_normal ^ node.virtual_point_lists_[index].getCoord();
					ans = (node.virtual_point_lists_[index].getCoord() ^ e_left).normalized();
					return ans;
				}
				if ((node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					//only one far node
					BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
					BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
					far_normal.normalize(); p_jp_i.normalize();
					BLVector facet_normal =  (p_jp_i ^ far_normal).normalized();
					BLVector e_left = facet_normal ^ node.virtual_point_lists_[index].getCoord();
					ans = (node.virtual_point_lists_[index].getCoord() ^ e_left).normalized();
					return ans;
				}
			}
		}
		break;
	case 5:
		i = worst_iteration_index;
		for (int j = 0; j < 3; j++) {
			if (node.triangle_lists_[i].point_index_[j] == index) {
				if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					BLVector e1 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();
					BLVector e2 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();
					BLVector center = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() + node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j+1) % 3]].getCoord()).normalized();



					// use simple iteration method
					double length1 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).magnitude();
					double length2 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).magnitude();
					double length3 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j+1) % 3]].getCoord()).magnitude();

					BLVector vec1 = length1 * (length1 + length2)*-1 * e2+ length2 * (length1 + length2)*-1 * e1;
					return ((1 - vec1.magnitude2())*vec1).normalized();
				}
			}
		}

		break;
	case 6:
		throw std::logic_error("wrong handler");
		break;
	default:
		throw std::logic_error("wrong handler");
		break;
	}
	throw std::logic_error("wrong handler");



}

double MeshEvaluator::WorstQuality(VirtualSphereMesh &node)
{
	double worst = 0;
	for (int i = 0; i < node.virtual_point_lists_.size(); i++)
		worst=std::max(worst,getPointQuality(node, i));
	return worst;
}

double MeshEvaluator::AveragyQuality(VirtualSphereMesh & node)
{
	double sum = 0;
	for (int i = 0; i < node.virtual_point_lists_.size(); i++)
		sum+=getPointQuality(node, i);
	return sum / node.virtual_point_lists_.size();
}

double MeshEvaluator::getPointQuality(VirtualSphereMesh &node,int index)
{
	double point_quality = 0;//best = 0, worst = 1. 
	auto neighbour_triangles=node.neighbour_tri_index_[index];
	auto inner_triangles = node.triangle_lists_;
	// far node don't need to evaluate quality
	if (node.virtual_point_lists_[index].isFarNode())
		return point_quality;
	//quality of A e,1:
	for (auto i : neighbour_triangles) {
		int index1 = i[0];
		int index2 = i[1];
		// index1, index2, index from a triangle
		BLVector normal = (node.virtual_point_lists_[index1].getCoord() -node.virtual_point_lists_[index].getCoord()) ^ (node.virtual_point_lists_[index2].getCoord() - node.virtual_point_lists_[index].getCoord());
		normal.normalize();
		
		point_quality = std::max(point_quality, acos(normal*node.virtual_point_lists_[index].getCoord())*2/PI);

	}

	//quality of A e,2
	for (int i = 0; i < node.triangle_lists_.size();i++) {
		for(int j=0;j<3;j++)
		if(node.triangle_lists_[i].point_index_[j]==index&&!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j+1)%3]].isFarNode())
			point_quality = std::max(point_quality, asin(abs(node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j+1)%3]].getCoord()*node.virtual_point_lists_[index].getCoord()))*2/PI);
	}

	//quality of A e,3
	for (int i = 0; i < node.triangle_lists_.size();i++) {
		for(int j=0;j<3;j++)
			if (node.triangle_lists_[i].point_index_[j] == index) {
				if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode()))	{
					//only one far node
					BLVector far_normal= node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
					BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()-node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
					far_normal.normalize(); p_jp_i.normalize();
					point_quality = std::max(point_quality, asin(abs(far_normal*p_jp_i)) * 2 / PI);
				}
				if ((node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					//only one far node
					BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
					BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
					far_normal.normalize(); p_jp_i.normalize();
					point_quality = std::max(point_quality, asin(abs(far_normal*p_jp_i)) * 2 / PI);
				}
			}

	}

	//quality of A e,4
	for (int i = 0; i < node.triangle_lists_.size(); i++) {
		for (int j = 0; j < 3; j++)
			if (node.triangle_lists_[i].point_index_[j] == index) {
				if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					//only one far node
					BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
					BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
					far_normal.normalize(); p_jp_i.normalize();
					BLVector facet_normal = -1*(p_jp_i ^ far_normal).normalized();
					point_quality = std::max(point_quality, acos(facet_normal * node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()) * 2 / PI);
				}
				if ((node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					//only one far node
					BLVector far_normal = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord();
					BLVector p_jp_i = node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord();
					far_normal.normalize(); p_jp_i.normalize();
					BLVector facet_normal = (p_jp_i ^ far_normal).normalized();
					point_quality = std::max(point_quality, acos(facet_normal * node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()) * 2 / PI);
				}
			}

	}

	//quality of A e.5
	for (int i = 0; i < node.triangle_lists_.size(); i++) {
		for (int j = 0; j < 3; j++)
			if (node.triangle_lists_[i].point_index_[j] == index) {
				if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
					BLVector e1 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();
					BLVector e2 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();

					point_quality = std::max(point_quality, acos(e1*e2) * 3 / PI);
					point_quality = std::max(point_quality, (PI-acos(e1*e2)) * 3/2 / PI);

				
				}
			}
	}

	//quality of A e.6
	//Question: may be we don't need A e.6?


	//for (int i = 0; i < node.triangle_lists_.size(); i++) {
	//	for (int j = 0; j < 3; j++)
	//		if (node.triangle_lists_[i].point_index_[j] == index) {
	//			if ((!node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].isFarNode() && !node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].isFarNode())) {
	//				BLVector e1 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 1) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()).normalized();
	//				BLVector e2 = (node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j + 2) % 3]].getCoord() - node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j+1) % 3]].getCoord()).normalized();
	//				BLVector facet_normal = (e1 ^ e2).normalized();
	//				point_quality = std::max(point_quality, acos(node.virtual_point_lists_[node.triangle_lists_[i].point_index_[(j) % 3]].getCoord()) * 3 / PI);


	//			}
	//		}
	//}
	

	return point_quality;
}

