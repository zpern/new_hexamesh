#include <cassert>
#include <memory>
#include <vector>

#include <boundary_mesh/transition/transition_boundary_checker.hpp>
#include <boundary_mesh/transition/provisional_transition_builder.hpp>
#include <boundary_mesh/spatial/sliding_intersection_index.hpp>

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
    OwnedBoundaryTriangle permission_metadata;
    permission_metadata.vertex_sliding_region_ids = {{{7}, {7}, {7}}};
    permission_metadata.physical_edge_mask = 0b101;
    permission_metadata.complete_face_exemption_regions = {7};
    auto context = std::make_shared<SlidingColumnContext>();
    context->low_points = {{0,0,0},{1,0,0},{0,1,0}};
    context->high_points = {{0,0,0},{1,0,1},{0,1,1}};
    context->low_region_ids = {{9},{},{}};
    context->high_region_ids = {{9},{},{}};
    permission_metadata.sliding_columns = context;
    if (permission_metadata.physical_edge_mask != 0b101 ||
        permission_metadata.complete_face_exemption_regions !=
            std::vector<std::uint32_t>{7} ||
        permission_metadata.sliding_columns->low_points.size() != 3 ||
        permission_metadata.sliding_columns->high_region_ids[0] !=
            std::vector<std::uint32_t>{9})
        return 20;

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

    TransitionBoundaryInput owner_input;
    owner_input.candidate_triangles.push_back(triangle(
        flat, candidate_keys, 44, {}));
    owner_input.original_surface = &obstacle_index.value();
    const auto colliding_owners = checker.findCollidingOwners(owner_input);
    assert(colliding_owners.hasValue());
    assert(colliding_owners.value().size() == 1);
    assert(colliding_owners.value().front().source_face_id == 44);
    assert(colliding_owners.value().front().rollback_high_faces.empty());

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

    SurfaceMesh sliding_mesh;
    sliding_mesh.vertices = {{0,0,0},{1,0,0},{0,1,0}};
    sliding_mesh.faces = {Triangle{{0,1,2}}};
    sliding_mesh.face_tags = {{SurfaceBoundaryKind::Symmetry, 90}};
    const auto sliding = SlidingIntersectionIndex::build(sliding_mesh);
    if (!sliding.hasValue()) return 21;
    TransitionBoundaryInput sliding_crossing;
    sliding_crossing.candidate_triangles.push_back(triangle(
        {{{0.2,0.2,-0.1},{0.7,0.2,0.1},{0.2,0.7,0.1}}},
        candidate_keys, 70, {70}));
    sliding_crossing.sliding_surface = &sliding.value();
    const auto sliding_rollback = checker.findRollbackFaces(sliding_crossing);
    if (!sliding_rollback.hasValue() ||
        sliding_rollback.value() != std::vector<SurfaceFaceId>{70})
        return 22;

    sliding_mesh.face_tags.front().kind = SurfaceBoundaryKind::Internal;
    const auto internal = SlidingIntersectionIndex::build(sliding_mesh);
    if (!internal.hasValue()) return 23;
    sliding_crossing.sliding_surface = &internal.value();
    const auto internal_rollback = checker.findRollbackFaces(sliding_crossing);
    if (!internal_rollback.hasValue() ||
        internal_rollback.value() != std::vector<SurfaceFaceId>{70})
        return 24;

    TransitionBoundaryInput attached_transition;
    auto attached = triangle(
        {{{0,0,0},{1,0,0},{0,1,0.3}}},
        candidate_keys, 71, {71});
    attached.vertex_sliding_region_ids = {{{90},{90},{}}};
    auto attached_columns = std::make_shared<SlidingColumnContext>();
    attached_columns->low_points = {
        {0,0,0},{1,0,0},{0,1,0.2}};
    attached_columns->high_points = {
        {0,0,0},{1,0,0},{0,1,0.4}};
    attached_columns->low_region_ids = {{90},{90},{}};
    attached_columns->high_region_ids = {{90},{90},{}};
    attached.sliding_columns = attached_columns;
    attached_transition.candidate_triangles.push_back(attached);
    attached_transition.sliding_surface = &internal.value();
    const auto attached_result = checker.findRollbackFaces(
        attached_transition);
    if (!attached_result.hasValue() || !attached_result.value().empty())
        return 26;

    auto crossing_columns = std::make_shared<SlidingColumnContext>(
        *attached_columns);
    crossing_columns->high_points[2].z() = -0.4;
    attached_transition.candidate_triangles.front().sliding_columns =
        crossing_columns;
    const auto upper_crossing = checker.findRollbackFaces(
        attached_transition);
    if (!upper_crossing.hasValue() ||
        upper_crossing.value() != std::vector<SurfaceFaceId>{71})
        return 27;

    SurfaceMesh two_regions = sliding_mesh;
    two_regions.vertices.insert(two_regions.vertices.end(), {
        {0.25,-0.1,-1},{0.25,1.1,-1},{0.25,-0.1,1}});
    two_regions.faces.push_back(Triangle{{3,4,5}});
    two_regions.face_tags.push_back(
        {SurfaceBoundaryKind::Internal,91});
    const auto two_region_index = SlidingIntersectionIndex::build(
        two_regions);
    if (!two_region_index.hasValue()) return 28;
    attached_transition.candidate_triangles.front().sliding_columns =
        attached_columns;
    attached_transition.sliding_surface = &two_region_index.value();
    const auto other_region = checker.findRollbackFaces(
        attached_transition);
    if (!other_region.hasValue() ||
        other_region.value() != std::vector<SurfaceFaceId>{71})
        return 29;

    GrowthFront transition_low;
    transition_low.layer = 1;
    transition_low.vertices = {
        {{0,0,0},0}, {{1,0,0},1}, {{1,1,0},2},
        {{0,1,0},3}, {{2,0,0},4}, {{2,1,0},5}};
    transition_low.faces = {
        Quad{{0,1,2,3}}, Quad{{1,4,5,2}}};
    transition_low.source_face_ids = {0,1};
    transition_low.vertices[1].boundary.sliding_region_ids = {9};
    transition_low.vertices[2].boundary.sliding_region_ids = {9};
    GrowthFront transition_high;
    transition_high.layer = 2;
    transition_high.vertices = {
        {{1,0,1},1}, {{2,0,1},4},
        {{2,1,1},5}, {{1,1,1},2}};
    transition_high.faces = {Quad{{0,1,2,3}}};
    transition_high.source_face_ids = {1};
    transition_high.vertices[0].boundary.sliding_region_ids = {9};
    transition_high.vertices[3].boundary.sliding_region_ids = {9};
    LayerFaceSets transition_sets;
    addInitialStop(transition_sets,
        {0,1,StopOrigin::Collision});
    const auto provisional = buildProvisionalTransition(
        transition_low, transition_high, {1}, transition_sets);
    if (!provisional.hasValue()) return 30;
    std::size_t side_triangles{};
    std::size_t attached_sides{};
    std::size_t associated_vertices{};
    for (const auto &owned :
         provisional.value().boundary.candidate_triangles)
        if (owned.owner.role == BoundaryOwnerRole::SideTransition)
        {
            ++side_triangles;
            if (owned.sliding_columns == nullptr) return 31;
            if (owned.sliding_columns->low_points.size() != 4) return 32;
            if (std::find(
                owned.complete_face_exemption_regions.begin(),
                owned.complete_face_exemption_regions.end(), 9) !=
                owned.complete_face_exemption_regions.end())
                ++attached_sides;
            if (owned.physical_edge_mask == 0b111) return 33;
            associated_vertices += std::count_if(
                owned.vertex_sliding_region_ids.begin(),
                owned.vertex_sliding_region_ids.end(),
                [](const auto &ids)
                { return std::find(ids.begin(), ids.end(), 9) != ids.end(); });
        }
        else if (owned.owner.role == BoundaryOwnerRole::RegularCandidate)
        {
            if (owned.sliding_columns == nullptr) return 37;
            if (owned.sliding_columns->low_points.size() != 4) return 38;
        }
    if (side_triangles != 4) return 34;
    if (attached_sides != 0) return 35;
    if (associated_vertices == 0) return 36;

    // A thin terminal Hexa prefers the external patch, but rejection of that
    // candidate must still fall back to a feasible positive-volume internal
    // split before KeepHexa is selected.
    ExternalPatchControls rejected_external;
    rejected_external.keep_hexa_faces = {0};
    const auto thin_hexa = [](SurfaceFaceId id)
        -> std::optional<HexaPoints>
    {
        if (id != 0) return std::nullopt;
        return HexaPoints{{
            Point3{0,0,-0.01}, Point3{1,0,-0.01},
            Point3{1,1,-0.01}, Point3{0,1,-0.01},
            Point3{0,0,0}, Point3{1,0,0},
            Point3{1,1,0}, Point3{0,1,0}}};
    };
    const auto internal_after_external_rejection =
        buildProvisionalTransition(
            transition_low, transition_high, {1}, transition_sets,
            thin_hexa, rejected_external);
    if (!internal_after_external_rejection.hasValue()) return 40;
    const auto terminal = std::find_if(
        internal_after_external_rejection.value().resolved_topology.begin(),
        internal_after_external_rejection.value().resolved_topology.end(),
        [](const ResolvedTransitionTopology &value)
        { return value.source_face_id == 0; });
    if (terminal ==
            internal_after_external_rejection.value().resolved_topology.end() ||
        terminal->terminal_quad_decision !=
            TerminalQuadDecision::InternalSplit ||
        !terminal->generated_point.has_value())
        return 41;

    std::vector<OwnedBoundaryTriangle> prior_transition_boundary = {
        triangle(
            {{{0,0,0},{1,0,0},{0,1,0}}},
            {{{20,10,0},{21,10,0},{22,10,0}}},
            80, {80})};
    TransitionBoundaryInput later_transition;
    later_transition.candidate_triangles.push_back(triangle(
        {{{0.25,0.25,-0.5},{0.75,0.25,0.5},{0.25,0.75,0.5}}},
        {{{30,11,0},{31,11,0},{32,11,0}}},
        81, {81}));
    later_transition.prior_transition_boundary =
        &prior_transition_boundary;
    const auto cross_prior = checker.findRollbackFaces(later_transition);
    if (!cross_prior.hasValue() ||
        cross_prior.value() != std::vector<SurfaceFaceId>{81})
        return 39;
}
