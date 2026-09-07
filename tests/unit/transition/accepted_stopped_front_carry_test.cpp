#include <algorithm>
#include <cassert>

#include <boundary_mesh/transition/accepted_stopped_front_carry.hpp>

using namespace boundary_mesh;

namespace
{
    bool contains(const std::vector<SurfaceFaceId> &ids, SurfaceFaceId id)
    {
        return std::find(ids.begin(), ids.end(), id) != ids.end();
    }
}

int main()
{
    GrowthFront accepted;
    accepted.layer = 1;
    accepted.vertices = {
        {{0,0,1},10}, {{1,0,1},11}, {{0,1,1},12},
        {{1,1,1},13}, {{2,1,1},14}};
    accepted.faces = {
        Triangle{{0,1,2}}, Triangle{{1,3,2}}, Triangle{{2,3,4}}};
    accepted.source_face_ids = {100,101,102};

    const auto carry = carryAcceptedStoppedFaces(
        accepted,
        {{0,100,2,FaceStopReason::IsotropicHeightReached}},
        {100,101,102});
    assert(carry.front.layer == 1);
    assert(carry.front.source_face_ids ==
           std::vector<SurfaceFaceId>{100});
    assert(carry.stops.size() == 1);
    assert(carry.stops.front().source_face_id == 100);
    assert(carry.stops.front().completed_layer == 1);
    assert(carry.stops.front().origin == StopOrigin::IsotropicStop);

    GrowthFront active;
    active.layer = 1;
    // Deliberately use a different compact vertex order.  Shared vertices
    // must be joined by source/branch identity, not by local VertexId.
    active.vertices = {
        {{1,1,1},13}, {{0,1,1},12}, {{1,0,1},11}, {{2,1,1},14}};
    active.faces = {Triangle{{2,0,1}}, Triangle{{1,0,3}}};
    active.source_face_ids = {101,102};

    const GrowthFront merged = mergeWithCarriedStoppedFaces(active, carry);
    assert(merged.layer == 1);
    assert(merged.source_face_ids ==
           std::vector<SurfaceFaceId>({101,102,100}));
    assert(merged.vertices.size() == 5);

    LayerFaceSets sets;
    addCarriedStops(sets, carry);
    assert(contains(sets.corner_suppression_seeds, 100));
    assert(contains(sets.transition_low_faces, 100));

    // A candidate removed by the current fixed point was never committed and
    // must not leak into the following transaction as an accepted stop.
    const auto rolled_back = carryAcceptedStoppedFaces(
        accepted,
        {{0,100,2,FaceStopReason::IsotropicHeightReached}},
        {101,102});
    assert(rolled_back.front.faces.empty());
    assert(rolled_back.stops.empty());

    // Constraint-based accepted stops do not necessarily have an explicit
    // accepted_stopped_faces event.  Detect them from the hand-off boundary:
    // a face committed in the previous candidate but absent from the next
    // compact active front.
    const auto retained_snapshot = retainAcceptedFaces(
        accepted, {}, {100,101,102});
    const auto missing = carryFacesMissingFromActive(
        retained_snapshot, active);
    assert(missing.front.source_face_ids ==
           std::vector<SurfaceFaceId>{100});
    assert(missing.stops.size() == 1);
    assert(missing.stops.front().source_face_id == 100);
    assert(missing.stops.front().completed_layer == 1);
    assert(missing.stops.front().origin == StopOrigin::AcceptedStop);
}
