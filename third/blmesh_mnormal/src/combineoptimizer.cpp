#include "../include/combineoptimizer.h"
#include "../include/common.h"
#include "../include/MeshEvaluation.h"
#include "../include/PointOptimizer.h"
#include "../include/TopologyOptimizer.h"
#include "../include/mergeoptimizer.h"
#include "../include/mostnormaloptimizer.h"
void CombineOptimer::Optimize(VirtualSphereMesh & node)
{
	//lift principle, new quality must be better than older
	double quality = MeshEvaluator::GetSingleton().GetQuality(node);
	int iteration = MAX_INTERATION/2;
	PointOptimer popt;
	MostNormalOptimer mnopt;
	TopologyOptimer topt;
	mnopt.Optimize(node);
	while (iteration--) {
		VirtualSphereMesh tmp = node;
		popt.Optimize(tmp);
	//	node = tmp;
	//	return;
	//	topt.Optimize(tmp);
		double new_quality = MeshEvaluator::GetSingleton().GetQuality(tmp);
		if (new_quality < quality) {
			node = tmp;
			quality = new_quality;
		}
		else
			break;
	}
}

