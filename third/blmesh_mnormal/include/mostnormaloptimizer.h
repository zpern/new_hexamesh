#pragma once
#ifndef _MOSTNORMALOPTIMIZER_H_
#define _MOSTNORMALOPTIMIZER_H_
#include "AbstractOptimizer.h"
/**
 * @brief abstract class for local optimize
 *
 */
class MostNormalOptimer :public AbstractOptimer {
public:
	void Optimize(VirtualSphereMesh &node);
};

#endif //! _MOSTNORMALACTOPTIMIZER_H_
