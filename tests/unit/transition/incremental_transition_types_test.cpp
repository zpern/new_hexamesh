#include <cassert>
#include <vector>

#include <boundary_mesh/transition/incremental_transition_types.hpp>

using namespace boundary_mesh;

int main()
{
    LayerFaceSets sets;
    addInitialStop(sets, {10, 3, StopOrigin::Quality});
    addCornerSuppressedFace(
        sets, {11, 3, StopOrigin::CornerSuppression});
    addCollisionRollback(
        sets, {12, 3, StopOrigin::TransitionCollision});

    assert((sets.corner_suppression_seeds ==
            std::vector<SurfaceFaceId>{10, 12}));
    assert((sets.transition_low_faces ==
            std::vector<SurfaceFaceId>{10, 11, 12}));
    assert(sets.states.size() == 3);

    addInitialStop(sets, {10, 3, StopOrigin::Quality});
    addCornerSuppressedFace(
        sets, {11, 3, StopOrigin::CornerSuppression});
    assert(sets.corner_suppression_seeds.size() == 2);
    assert(sets.transition_low_faces.size() == 3);
    assert(sets.states.size() == 3);
    return 0;
}
