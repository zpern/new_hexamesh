#include <cassert>
#include <vector>

#include <boundary_mesh/transition/transition_boundary_checker.hpp>

using namespace boundary_mesh;

namespace
{
    OwnedBoundaryTriangle triangle(
        std::array<Point3, 3> points,
        std::array<CollisionVertexKey, 3> keys,
        SurfaceFaceId source,
        std::vector<SurfaceFaceId> rollback,
        BoundaryOwnerRole role = BoundaryOwnerRole::SideTransition)
    {
        return {points, keys, {source, 2, role, std::move(rollback)}};
    }

    CollisionTriangle obstacle(
        std::array<Point3, 3> points,
        std::uint32_t id)
    {
        CollisionTriangle value;
        value.points = points;
        value.vertex_keys = {{{100,0,0}, {101,0,0}, {102,0,0}}};
        value.owner_kind = CollisionOwnerKind::OriginalSurface;
        value.owner_id = id;
        value.boundary_vertex_count = 3;
        value.boundary_points[0] = points[0];
        value.boundary_points[1] = points[1];
        value.boundary_points[2] = points[2];
        value.boundary_vertex_keys[0] = value.vertex_keys[0];
        value.boundary_vertex_keys[1] = value.vertex_keys[1];
        value.boundary_vertex_keys[2] = value.vertex_keys[2];
        return value;
    }
}

int main()
{
    const std::array<Point3, 3> flat{{
        {0,0,0}, {1,0,0}, {0,1,0}}};
    const std::array<CollisionVertexKey, 3> candidate_keys{{
        {0,2,0}, {1,2,0}, {2,2,0}}};
    const auto obstacle_index = CollisionIndex::build({obstacle(flat, 7)});
    assert(obstacle_index.hasValue());

    TransitionBoundaryChecker checker;
    TransitionBoundaryInput side_input;
    side_input.candidate_triangles.push_back(triangle(
        flat, candidate_keys, 10, {22,21}));
    side_input.original_surface = &obstacle_index.value();
    const auto side = checker.findRollbackFaces(side_input);
    assert(side.hasValue());
    assert((side.value() == std::vector<SurfaceFaceId>{21,22}));

    TransitionBoundaryInput cap_input;
    cap_input.candidate_triangles.push_back(triangle(
        flat, candidate_keys, 30, {30}, BoundaryOwnerRole::TopCap));
    cap_input.original_surface = &obstacle_index.value();
    const auto cap = checker.findRollbackFaces(cap_input);
    assert(cap.hasValue());
    assert((cap.value() == std::vector<SurfaceFaceId>{30}));

    TransitionBoundaryInput zero_layer_own_face;
    auto own_cap = triangle(
        flat, candidate_keys, 7, {21}, BoundaryOwnerRole::TopCap);
    own_cap.owner.layer = 0;
    zero_layer_own_face.candidate_triangles.push_back(own_cap);
    zero_layer_own_face.original_surface = &obstacle_index.value();
    const auto own = checker.findRollbackFaces(zero_layer_own_face);
    assert(own.hasValue());
    assert(own.value().empty());

    TransitionBoundaryInput regular_own_face;
    regular_own_face.candidate_triangles.push_back(triangle(
        flat, candidate_keys, 7, {7},
        BoundaryOwnerRole::RegularCandidate));
    regular_own_face.original_surface = &obstacle_index.value();
    const auto own_regular = checker.findRollbackFaces(regular_own_face);
    assert(own_regular.hasValue());
    assert(own_regular.value().empty());

    regular_own_face.candidate_triangles.front().owner.source_face_id = 8;
    const auto other_regular = checker.findRollbackFaces(regular_own_face);
    assert(other_regular.hasValue());
    assert((other_regular.value() == std::vector<SurfaceFaceId>{7}));

    TransitionBoundaryInput zero_layer_other_face = zero_layer_own_face;
    zero_layer_other_face.candidate_triangles.front().owner.source_face_id = 8;
    const auto other = checker.findRollbackFaces(zero_layer_other_face);
    assert(other.hasValue());
    assert((other.value() == std::vector<SurfaceFaceId>{21}));

    ExposedBoundaryTracker history;
    LayerBoundaryCandidate historical_candidate;
    historical_candidate.bottom.points = {
        {0,0,-1},{1,0,-1},{1,1,-1},{0,1,-1}};
    historical_candidate.bottom.vertex_keys = {
        {0,1,0},{1,1,0},{2,1,0},{3,1,0}};
    historical_candidate.bottom.source_face_id = 7;
    historical_candidate.top.points = {
        {0,0,0},{1,0,0},{1,1,0},{0,1,0}};
    historical_candidate.top.vertex_keys = {
        {0,2,0},{1,2,0},{2,2,0},{3,2,0}};
    historical_candidate.top.source_face_id = 7;
    historical_candidate.side_tags.resize(
        4, {SurfaceBoundaryKind::Internal,0});
    const auto history_update = history.prepare({historical_candidate});
    assert(history_update.hasValue());
    history.apply(history_update.value());

    TransitionBoundaryInput historical_own;
    historical_own.candidate_triangles.push_back(triangle(
        {{{0,0,0},{1,0,0},{0,1,0}}},
        {{{0,2,0},{1,2,0},{3,2,0}}},
        7,{21},BoundaryOwnerRole::TopCap));
    historical_own.historical_boundary = &history;
    const auto own_history = checker.findRollbackFaces(historical_own);
    assert(own_history.hasValue());
    assert(own_history.value().empty());

    historical_own.candidate_triangles.front().owner.source_face_id = 8;
    const auto other_history = checker.findRollbackFaces(historical_own);
    assert(other_history.hasValue());
    assert((other_history.value() == std::vector<SurfaceFaceId>{21}));

    TransitionBoundaryInput self_input;
    self_input.candidate_triangles = {
        triangle(flat, candidate_keys, 40, {40}),
        triangle(flat,
            {{{3,2,0}, {4,2,0}, {5,2,0}}}, 41, {41})};
    const auto self = checker.findRollbackFaces(self_input);
    assert(self.hasValue());
    assert((self.value() == std::vector<SurfaceFaceId>{40,41}));

    TransitionBoundaryInput legal_input;
    legal_input.candidate_triangles = {
        triangle(flat, candidate_keys, 50, {50}),
        triangle({{{0,0,0}, {1,0,0}, {0,0,1}}},
            {{{0,2,0}, {1,2,0}, {6,2,0}}}, 51, {51})};
    const auto legal = checker.findRollbackFaces(legal_input);
    assert(legal.hasValue());
    assert(legal.value().empty());

    TransitionBoundaryInput cancellation;
    cancellation.candidate_triangles = {
        triangle(flat, candidate_keys, 60, {60}, BoundaryOwnerRole::TopCap),
        triangle({{{0,1,0}, {1,0,0}, {0,0,0}}},
            {{{2,2,0}, {1,2,0}, {0,2,0}}}, 61, {61})};
    const auto assembled = checker.assembleExposedBoundary(cancellation);
    assert(assembled.hasValue());
    assert(assembled.value().empty());

    TransitionBoundaryInput conflict;
    conflict.diagonal_requirements = {
        {{70, 2}, QuadDiagonal::ZeroTwo},
        {{70, 2}, QuadDiagonal::OneThree}};
    const auto conflicting = checker.findRollbackFaces(conflict);
    assert(!conflicting.hasValue());
    assert(std::holds_alternative<ConflictingLayerQuadDiagonal>(
        conflicting.error()));
}
