#pragma once
#ifndef _ABSTRACTOPTIMIZER_H_
#include "./complexnode.h"
#include "./VirtualSphereMesh.h"
/**
 * @brief abstract class for local optimize
 * 
 */
class AbstractOptimer{
public:
    virtual void Optimize(VirtualSphereMesh &node)=0;
};


#define _ABSTRACTOPTIMIZER_H_
#endif //! _ABSTRACTOPTIMIZER_H_
