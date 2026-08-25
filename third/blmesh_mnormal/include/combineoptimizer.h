#pragma once
#ifndef _COMBINEOPTIMIZER_H_
#define _COMBINEOTIMIZER_H_
#include "./complexnode.h"
#include "AbstractOptimizer.h"
/**
 * @brief abstract class for local optimize
 *
 */
class CombineOptimer :public AbstractOptimer {
public:
	void Optimize(VirtualSphereMesh &node);
};

#endif
