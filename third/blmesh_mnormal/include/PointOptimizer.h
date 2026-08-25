#pragma once
#ifndef _POINTOPTIMIZER_H_
#define _POINTOPTIMIZER_H_
#include "./complexnode.h"
#include "AbstractOptimizer.h"
/**
 * @brief abstract class for local optimize
 *
 */
class PointOptimer :public AbstractOptimer {
public:
	void Optimize(VirtualSphereMesh &node);
private:
	void OptimizeOneNode(VirtualSphereMesh &node, int index);
};

#endif //! _ABSTRACTOPTIMIZER_H_
