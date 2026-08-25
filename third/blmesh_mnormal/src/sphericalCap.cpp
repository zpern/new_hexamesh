#include "sphericalCap.h"
#include <cfloat>
#include <cmath>


VPoint::VPoint()
{
}

VPoint::~VPoint()
{
}

BLVector VPoint::getCoord() const
{
	return coordinate;
}


int VPoint::getGlobalIndex()
{
	return global_index_;
}

void VPoint::setGlobalIndex(int index)
{
	global_index_ = index;
}


int VPoint::getIndex() const
{
	return index_;
}

void VPoint::setIndex( int index)
{
	index_ = index;
}

vector<int> VPoint::getGlobalNeighbourTriIndex()
{
	return global_neighbour_tri_index_;
}

void VPoint::setGlobalNeighbourTriIndex(vector<int> index)
{
	global_neighbour_tri_index_ = index;
}

void VPoint::setCoord(BLVector ans)
{
	if (isnan(ans.x))
		ans = BLVector(1, 0, 0);
	coordinate=ans;
}

bool VPoint::isFarNode() const
{
	return coordinate.magnitude()>1.01;
}

VTriangle::VTriangle():added_flag(false)
{
}

VTriangle::~VTriangle()
{
}


