#ifndef  VIRTUAL_SPHERE_MESH_STRATEGY_H_
#define VIRTUAL_SPHERE_MESH_STRATEGY_H_
#include "VirtualSphereMesh.h"
/* this calss only use for non-continues manifold */
class VirtualSphereStrategy {
public:
	VirtualSphereMesh noInsert();
	VirtualSphereMesh OneInsert();

	VirtualSphereMesh generate();
	void sphereSpr(VirtualSphereMesh& input_boundary);
	void addNodeSphereSpr(VirtualSphereMesh& input_boundary);
	void generateInitialTriangles(VirtualSphereMesh& input_boundary);
	VirtualSphereMesh input_;
};
#endif // ! MESH_SPLITTER_H_


