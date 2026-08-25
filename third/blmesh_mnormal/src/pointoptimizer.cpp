#include "../include/PointOptimizer.h"
#include "../include/MeshEvaluation.h"


 void PointOptimer::Optimize(VirtualSphereMesh & node)
 {
	 
	// cout << " # worst quality = " << MeshEvaluator::GetSingleton().GetQuality(node) << endl;;
	 int iterator= MAX_INTERATION;
	 while (iterator--)
	 {

		 double initial_quality = MeshEvaluator::GetSingleton().GetQuality(node);
		 QualityHandler qhandle;
		 MeshEvaluator::GetSingleton().GetWorstQualityHandler(node, qhandle);
		 double initial_worst_quality = MeshEvaluator::GetSingleton().GetQualityOfHandler(node, qhandle);
		 BLVector gredient = MeshEvaluator::GetSingleton().GetGredientQualityHandler(node, qhandle);

		 int worst_index = qhandle / 10000;
		 QualityHandler initial = qhandle;
		 double start = 0;
		 double end = 1;
		 while (end - start > 1e-8) {
			 double mid = (start + end) / 2;
			 VirtualSphereMesh my_mesh = node;
			 BLVector pos = my_mesh.virtual_point_lists_[worst_index].getCoord();
			 pos = (pos + mid * gredient).normalized();
			 my_mesh.virtual_point_lists_[worst_index].setCoord(pos);
			 QualityHandler tmp;
			 MeshEvaluator::GetSingleton().GetWorstQualityHandler(my_mesh, tmp);

			 if (tmp != initial)
				 end = mid;
			 else {

				 //	 if (MeshEvaluator::GetSingleton().GetQualityOfHandler(my_mesh, tmp) > initial_worst_quality)
				 //		 throw std::logic_error(" impossible case!");


				 start = mid;
			 }
		 }
		 BLVector pos = node.virtual_point_lists_[worst_index].getCoord();
		 pos = (pos + end * gredient).normalized();
		 node.virtual_point_lists_[worst_index].setCoord(pos);
	 }
	 //////////////////////////////////////////////////////////////////////////
	 /************************************************************************/
	 /* The following code only utilized for debug                           */
	 /************************************************************************/
	// cout <<" & worst quality="<< MeshEvaluator::GetSingleton().GetQuality(node) << endl;;
	 //if (MeshEvaluator::GetSingleton().GetQuality(node) > initial_quality) {
		// //cout << "error!" << initial / 10000;
		// //MeshEvaluator::GetSingleton().GetWorstQualityHandler(node, qhandle);
		// //double initial_worst_quality = MeshEvaluator::GetSingleton().GetQuality(node);
		// //BLVector gredient = MeshEvaluator::GetSingleton().GetGredientQualityHandler(node, qhandle);
		// //double q1 = MeshEvaluator::GetSingleton().GetQualityOfHandler(node, qhandle);
		// //int worst_index = qhandle / 10000;
		// //BLVector pos = node.virtual_point_lists_[worst_index].getCoord();
		// //pos = (pos + 1e-10 * gredient).normalized();
		// //node.virtual_point_lists_[worst_index].setCoord(pos);
		// //QualityHandler tmp;
		// //double value_2 = MeshEvaluator::GetSingleton().GetQuality(node);
		// //MeshEvaluator::GetSingleton().GetWorstQualityHandler(node, tmp);
		// //MeshEvaluator::GetSingleton().GetGredientQualityHandler(node, tmp);
		// //double q2 = MeshEvaluator::GetSingleton().GetQualityOfHandler(node, tmp);
		// //double q3 = q2 - q1;
		// //if (value_2 > initial_worst_quality) {
		//	// throw std::logic_error("error gredient!");
		// //}

	 //}
 }

 void PointOptimer::OptimizeOneNode(VirtualSphereMesh & node, int index)
 {
	 
 }

