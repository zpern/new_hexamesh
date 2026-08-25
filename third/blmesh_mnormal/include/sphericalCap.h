
#pragma once
#ifndef _SPHER_CUP_H_
#define _SPHER_CUP_H_
#include "blvector.h"
#include <vector>
#include <array>

class VPoint;
typedef std::array<int, 2> VEDGE; // all the edge contain only two nodes 


/**
 * @brief the virtual point for complex node
 *
 */
class VPoint {
public:
	VPoint();
	~VPoint();
	BLVector getCoord() const;
	int getGlobalIndex();
	void setGlobalIndex(int index);
	int  getIndex() const;
	void setIndex(int index);

	vector<int>  getGlobalNeighbourTriIndex();
	void setGlobalNeighbourTriIndex(vector<int> index);

	void setCoord(BLVector ans);
	bool isFarNode() const;
protected:
	BLVector coordinate;
	int index_;
	int global_index_;
	vector<int> global_neighbour_tri_index_;

	

	
};




class VTriangle
{
public:
	VTriangle();
	~VTriangle();

	bool added_flag;//true : added, false: non
	std::array<int,3> point_index_;

};



#endif

