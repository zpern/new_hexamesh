#pragma once
#ifndef _TOPOLOGYOPTIMIZER_H_
#define _TOPOLOGYOPTIMIZER_H_
#include "./AbstractOptimizer.h"
/**
 * @brief abstract class for local optimize
 * 
 */
class TopologyOptimer:public AbstractOptimer{
public:
    virtual void Optimize(VirtualSphereMesh &node) ;
};

#endif //! _ABSTRACTOPTIMIZER_H_
