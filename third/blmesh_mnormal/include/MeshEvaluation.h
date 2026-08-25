#pragma once
#ifndef _MESHEVALUATION_H_
#define _MESHEVALUATION_H_
#include "./complexnode.h"
#include "./VirtualSphereMesh.h"
#include "./singleton.h"
/*
 * @brief Here we abstract the index of worst quality into QualityHandle, by point_index*100 + number_of_interation*10000 + the A index * 1;
*/
typedef  long long QualityHandler;
/**
 * @brief evalueate the object function of a complex mesh
 * 
 */
class MeshEvaluator:public Singleton<MeshEvaluator> {
public:
	void SetDefaultParameter(double m, double n);
	double GetQuality(VirtualSphereMesh &node);
	
	// quality handler, this three function work for point optimization
	void GetWorstQualityHandler(VirtualSphereMesh &node,QualityHandler& handler);
	double GetQualityOfHandler(VirtualSphereMesh &node, const QualityHandler& handler);
	BLVector GetGredientQualityHandler(VirtualSphereMesh &node,const QualityHandler& handler);

protected:
	double WorstQuality(VirtualSphereMesh &node);
	double AveragyQuality(VirtualSphereMesh &node);

	// objfunc = worst * (1- mu ) + avery* mu + nu* Num.normal
private:
	double getPointQuality(VirtualSphereMesh &node,int index);
protected:
	double mu_;/// balance the worst quality and average quality
	double nu_;/// balance the quantity of normal and mesh quality

};


#endif //! _ABSTRACTOPTIMIZER_H_
