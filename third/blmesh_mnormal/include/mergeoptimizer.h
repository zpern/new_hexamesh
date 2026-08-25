#pragma once
#ifndef _MERGEOPTIMIZER_H_
#define _MERGEOPTIMIZER_H_
#include "./complexnode.h"
#include "AbstractOptimizer.h"
/**
 * @brief abstract class for local optimize
 *
 */
class MergeOptimer :public AbstractOptimer {
public:
	void Optimize(VirtualSphereMesh &node);
};

#endif //! _MERGEACTOPTIMIZER_H_
