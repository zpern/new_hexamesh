#include <algorithm>
#include <cassert>

#include <boundary_mesh/transition/layer_transition_resolver.hpp>

using namespace boundary_mesh;

namespace
{
    GrowthFront front(std::vector<SurfaceFaceId> ids)
    {
        GrowthFront value;
        value.layer = 3;
        value.vertices = {
            {{0,0,0},0}, {{1,0,0},1}, {{1,1,0},2}, {{0,1,0},3},
            {{2,0,0},4}, {{3,0,0},5}, {{3,1,0},6}, {{2,1,0},7},
            {{4,0,0},8}, {{5,0,0},9}, {{5,1,0},10}, {{4,1,0},11}};
        const std::vector<Quad> faces{
            {{0,1,2,3}}, {{4,5,6,7}}, {{8,9,10,11}}};
        for (const SurfaceFaceId id : ids)
        {
            value.faces.push_back(faces[id - 30]);
            value.source_face_ids.push_back(id);
        }
        return value;
    }

    OwnedBoundaryTriangle ownedTriangle(SurfaceFaceId owner, Scalar z)
    {
        return {{{{0,0,z}, {1,0,z}, {0,1,z}}},
                 {{{0,4,0}, {1,4,0}, {2,4,0}}},
                 {owner, 4, BoundaryOwnerRole::RegularCandidate, {owner}}};
    }

    LayerTransitionInput collisionInput(std::vector<SurfaceFaceId> order)
    {
        LayerTransitionInput input;
        input.current_front = front({30,31,32});
        input.candidate_front = front(std::move(order));
        input.candidate_front.layer = 4;
        input.completed_layer = 3;
        input.build_provisional = [](
            const std::vector<SurfaceFaceId> &retained,
            const LayerFaceSets &,
            const ExternalPatchControls &)
        {
            ProvisionalLayerTransition value;
            for (const SurfaceFaceId id : retained)
                value.boundary.candidate_triangles.push_back(
                    ownedTriangle(id, id == 30 ? 0.0 : Scalar(id)));
            value.all_top_faces_are_triangles = true;
            return ProvisionalLayerTransitionResult::success(
                std::move(value));
        };
        CollisionTriangle obstacle;
        obstacle.points = {{{0,0,0}, {1,0,0}, {0,1,0}}};
        obstacle.vertex_keys = {{{100,0,0}, {101,0,0}, {102,0,0}}};
        obstacle.owner_kind = CollisionOwnerKind::OriginalSurface;
        obstacle.boundary_vertex_count = 3;
        obstacle.boundary_points[0] = obstacle.points[0];
        obstacle.boundary_points[1] = obstacle.points[1];
        obstacle.boundary_points[2] = obstacle.points[2];
        obstacle.boundary_vertex_keys[0] = obstacle.vertex_keys[0];
        obstacle.boundary_vertex_keys[1] = obstacle.vertex_keys[1];
        obstacle.boundary_vertex_keys[2] = obstacle.vertex_keys[2];
        auto index = CollisionIndex::build({obstacle});
        assert(index.hasValue());
        input.original_surface = std::move(index.value());
        return input;
    }
}

int main()
{
    const Point3 feasible_center{0.5,0.5,0.005};
    if (chooseTerminalQuadDecision(
            Scalar{0.15}, feasible_center, true) !=
            TerminalQuadDecision::InternalSplit)
        return 50;
    if (chooseTerminalQuadDecision(
            Scalar{0.15}, std::nullopt, true) !=
            TerminalQuadDecision::ExternalPatch)
        return 51;
    if (chooseTerminalQuadDecision(
            Scalar{0.01}, feasible_center, true) !=
            TerminalQuadDecision::ExternalPatch)
        return 52;
    if (chooseTerminalQuadDecision(
            Scalar{0.125}, feasible_center, true) !=
            TerminalQuadDecision::ExternalPatch)
        return 55;
    if (chooseTerminalQuadDecision(
            Scalar{0.01}, feasible_center, false) !=
            TerminalQuadDecision::InternalSplit)
        return 53;
    if (chooseTerminalQuadDecision(
            Scalar{0.01}, std::nullopt, false) !=
            TerminalQuadDecision::KeepHexa)
        return 54;

    LayerTransitionInput sliding_context;
    sliding_context.sliding_surface = nullptr;

    LayerTransitionResolver resolver;
    auto input = collisionInput({32,30,31});
    const auto result = resolver.resolve(input);
    assert(result.hasValue());
    assert(result.value().iterations == 2);
    assert(std::find(result.value().retained_high_faces.begin(),
                     result.value().retained_high_faces.end(), 30) ==
           result.value().retained_high_faces.end());
    assert(std::binary_search(
        result.value().face_sets.corner_suppression_seeds.begin(),
        result.value().face_sets.corner_suppression_seeds.end(),
        SurfaceFaceId{30}));
    assert(std::binary_search(
        result.value().face_sets.transition_low_faces.begin(),
        result.value().face_sets.transition_low_faces.end(),
        SurfaceFaceId{30}));
    assert(result.value().all_top_faces_are_triangles);

    const auto reversed = resolver.resolve(collisionInput({31,30,32}));
    assert(reversed.hasValue());
    assert(reversed.value().retained_high_faces ==
           result.value().retained_high_faces);

    LayerTransitionInput suppression;
    suppression.current_front.layer = 2;
    suppression.current_front.vertices = {
        {{0,0,0},0}, {{1,0,0},1}, {{1,1,0},2}, {{0,1,0},3},
        {{2,1,0},4}, {{2,2,0},5}, {{1,2,0},6},
        {{0,-1,0},7}, {{1,-1,0},8}};
    suppression.current_front.faces = {
        Quad{{0,1,2,3}}, Quad{{2,4,5,6}}, Quad{{7,8,1,0}}};
    suppression.current_front.source_face_ids = {10,11,20};
    suppression.candidate_front = suppression.current_front;
    suppression.candidate_front.layer = 3;
    for (auto &vertex : suppression.candidate_front.vertices)
        vertex.position.z() = 1;
    suppression.candidate_front.faces.erase(
        suppression.candidate_front.faces.begin());
    suppression.candidate_front.source_face_ids.erase(
        suppression.candidate_front.source_face_ids.begin());
    suppression.completed_layer = 2;
    addInitialStop(suppression.face_sets,
                   {10,2,StopOrigin::Quality});
    suppression.build_provisional = [](
        const std::vector<SurfaceFaceId> &,
        const LayerFaceSets &,
        const ExternalPatchControls &)
    {
        ProvisionalLayerTransition value;
        value.all_top_faces_are_triangles = true;
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    const auto suppressed = resolver.resolve(suppression);
    assert(suppressed.hasValue());
    assert(!std::binary_search(
        suppressed.value().retained_high_faces.begin(),
        suppressed.value().retained_high_faces.end(), SurfaceFaceId{11}));
    assert(!std::binary_search(
        suppressed.value().face_sets.corner_suppression_seeds.begin(),
        suppressed.value().face_sets.corner_suppression_seeds.end(),
        SurfaceFaceId{11}));
    assert(std::binary_search(
        suppressed.value().face_sets.transition_low_faces.begin(),
        suppressed.value().face_sets.transition_low_faces.end(),
        SurfaceFaceId{11}));

    LayerTransitionInput external;
    external.completed_layer = 1;
    std::size_t external_builds{};
    external.build_provisional = [&](
        const std::vector<SurfaceFaceId> &,
        const LayerFaceSets &,
        const ExternalPatchControls &controls)
    {
        ++external_builds;
        const Scalar scale = controls.distanceScale(77);
        ProvisionalLayerTransition value;
        value.boundary.candidate_triangles.push_back({
            {{{-1,-1,0},{1,-1,0},{0,1,scale}}},
            {{{200,1,0},{201,1,0},{202,1,0}}},
            {77,1,BoundaryOwnerRole::ExternalPatch,{}}});
        ResolvedTransitionTopology topology;
        topology.source_face_id = 77;
        topology.layer = 1;
        topology.template_kind = TransitionTemplateKind::QuadTopCap;
        topology.terminal_quad_decision = TerminalQuadDecision::ExternalPatch;
        topology.generated_point = Point3{0,0,scale};
        value.resolved_topology.push_back(std::move(topology));
        value.all_top_faces_are_triangles = true;
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    CollisionTriangle ceiling;
    ceiling.points = {{{-2,-2,0.5},{2,-2,0.5},{0,2,0.5}}};
    ceiling.vertex_keys = {{{300,0,0},{301,0,0},{302,0,0}}};
    ceiling.owner_kind = CollisionOwnerKind::OriginalSurface;
    ceiling.boundary_vertex_count = 3;
    std::copy(ceiling.points.begin(), ceiling.points.end(),
              ceiling.boundary_points.begin());
    std::copy(ceiling.vertex_keys.begin(), ceiling.vertex_keys.end(),
              ceiling.boundary_vertex_keys.begin());
    auto ceiling_index = CollisionIndex::build({ceiling});
    assert(ceiling_index.hasValue());
    external.original_surface = std::move(ceiling_index.value());
    const auto external_result = resolver.resolve(external);
    assert(external_result.hasValue());
    assert(external_result.value().resolved_topology.size() == 1);
    const Scalar chosen =
        external_result.value().resolved_topology.front()
            .generated_point->z();
    assert(chosen < 0.5);
    assert(chosen > 0.49);
    assert(external_builds > 4);

    LayerTransitionInput narrow_external;
    narrow_external.completed_layer = 1;
    narrow_external.build_provisional = [](
        const std::vector<SurfaceFaceId> &,
        const LayerFaceSets &,
        const ExternalPatchControls &controls)
    {
        const Scalar scale = controls.distanceScale(78);
        ProvisionalLayerTransition value;
        value.boundary.candidate_triangles.push_back({
            {{{-1,-1,0},{1,-1,0},{0,1,scale}}},
            {{{210,1,0},{211,1,0},{212,1,0}}},
            {78,1,BoundaryOwnerRole::ExternalPatch,{}}});
        ResolvedTransitionTopology topology;
        topology.source_face_id = 78;
        topology.layer = 1;
        topology.template_kind = TransitionTemplateKind::QuadTopCap;
        topology.terminal_quad_decision = controls.keepHexa(78)
            ? TerminalQuadDecision::KeepHexa
            : TerminalQuadDecision::ExternalPatch;
        if (topology.terminal_quad_decision ==
            TerminalQuadDecision::ExternalPatch)
            topology.generated_point = Point3{0,0,scale};
        value.resolved_topology.push_back(std::move(topology));
        value.all_top_faces_are_triangles = true;
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    CollisionTriangle narrow_ceiling = ceiling;
    for (Point3 &point : narrow_ceiling.points) point.z() = Scalar{5e-7};
    std::copy(narrow_ceiling.points.begin(), narrow_ceiling.points.end(),
              narrow_ceiling.boundary_points.begin());
    auto narrow_index = CollisionIndex::build({narrow_ceiling});
    if (!narrow_index.hasValue()) return 55;
    narrow_external.original_surface = std::move(narrow_index.value());
    const auto narrow_result = resolver.resolve(narrow_external);
    if (!narrow_result.hasValue() ||
        narrow_result.value().resolved_topology.size() != 1 ||
        narrow_result.value().resolved_topology.front()
                .terminal_quad_decision !=
            TerminalQuadDecision::KeepHexa ||
        narrow_result.value().resolved_topology.front()
             .generated_point.has_value())
        return 56;
}
