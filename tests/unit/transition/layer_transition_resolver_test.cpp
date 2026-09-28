#include <algorithm>
#include <cassert>

#include <boundary_mesh/transition/layer_transition_resolver.hpp>
#include <boundary_mesh/transition/provisional_transition_builder.hpp>

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
    LayerTransitionInput view_input;
    view_input.current_front = front({30});
    view_input.candidate_front = front({30});
    const GrowthFront current_view = front({31});
    const GrowthFront candidate_view = front({32});
    view_input.current_front_view = &current_view;
    view_input.candidate_front_view = &candidate_view;
    if (&view_input.currentFront() != &current_view ||
        &view_input.candidateFront() != &candidate_view)
        return 79;

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
    assert(result.value().collision_full_builds == 1);
    assert(result.value().collision_incremental_updates == 1);
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

    auto unresolvable_collision = collisionInput({30});
    unresolvable_collision.build_provisional = [](
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &,
        const ExternalPatchControls &)
    {
        ProvisionalLayerTransition value;
        for (const SurfaceFaceId id : retained)
        {
            auto triangle = ownedTriangle(id, 0.0);
            triangle.owner.rollback_high_faces.clear();
            triangle.owner.role = BoundaryOwnerRole::SideTransition;
            value.boundary.candidate_triangles.push_back(
                std::move(triangle));
        }
        return ProvisionalLayerTransitionResult::success(
            std::move(value));
    };
    const auto unresolved_result = resolver.resolve(
        unresolvable_collision);
    if (unresolved_result.hasValue()) return 87;
    const auto *unresolved_boundary = std::get_if<TransitionBoundaryError>(
        &unresolved_result.error());
    if (unresolved_boundary == nullptr) return 88;
    const auto *unresolved_collision =
        std::get_if<UnresolvedTransitionCollision>(unresolved_boundary);
    if (unresolved_collision == nullptr ||
        unresolved_collision->owners.size() != 1 ||
        unresolved_collision->owners.front().source_face_id != 30)
        return 89;

    auto local_input = collisionInput({32,30,31});
    local_input.verify_local_rebuilds = true;
    std::size_t rebuilt_faces = 0;
    local_input.affected_transition_faces = [](const auto &changed) { return changed; };
    local_input.build_transition_patches = [&rebuilt_faces](
        const auto &retained, const auto &, const auto &selected, const auto &)
    {
        rebuilt_faces += selected.size();
        ProvisionalLayerTransition value;
        value.all_top_faces_are_triangles = true;
        for (const auto id : selected)
            if (std::binary_search(retained.begin(), retained.end(), id))
                value.boundary.candidate_triangles.push_back(
                    ownedTriangle(id, id == 30 ? 0.0 : Scalar(id)));
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    const auto local_result = resolver.resolve(local_input);
    if (!local_result.hasValue() || rebuilt_faces != 1 ||
        local_result.value().retained_high_faces != result.value().retained_high_faces ||
        local_result.value().provisional_full_builds != 1 ||
        local_result.value().provisional_local_rebuilds != 1)
        return 70;

    const auto reversed = resolver.resolve(collisionInput({31,30,32}));
    assert(reversed.hasValue());
    assert(reversed.value().retained_high_faces ==
           result.value().retained_high_faces);

    auto excluded_input = collisionInput({30,31,32});
    excluded_input.excluded_candidate_faces = {31};
    const auto excluded_result = resolver.resolve(excluded_input);
    if (!excluded_result.hasValue() ||
        std::binary_search(excluded_result.value().retained_high_faces.begin(),
                           excluded_result.value().retained_high_faces.end(),
                           SurfaceFaceId{31}) ||
        !std::binary_search(excluded_result.value().retained_high_faces.begin(),
                            excluded_result.value().retained_high_faces.end(),
                            SurfaceFaceId{32}) ||
        excluded_input.candidateFront().source_face_ids.size() != 3)
        return 80;

    // Real adjacent templates: corner suppression may remove additional high
    // faces, and the selected patch closure must match a fresh full build.
    LayerTransitionInput grid;
    for (VertexId y = 0; y < 4; ++y)
        for (VertexId x = 0; x < 4; ++x)
            grid.current_front.vertices.push_back({
                Point3{Scalar(x), Scalar(y), 0}, y * 4 + x});
    for (VertexId y = 0; y < 3; ++y)
        for (VertexId x = 0; x < 3; ++x)
        {
            const VertexId v = y * 4 + x;
            grid.current_front.faces.push_back(Quad{{v, v+1, v+5, v+4}});
            grid.current_front.source_face_ids.push_back(y * 3 + x);
        }
    grid.candidate_front = grid.current_front;
    grid.candidate_front.layer = 1;
    for (auto &vertex : grid.candidate_front.vertices) vertex.position.z() = 0.2;
    ProvisionalTransitionBuildContext grid_context(grid.current_front, grid.candidate_front);
    const auto vertex_star = grid_context.affectedFaces({4});
    if (!std::binary_search(vertex_star.begin(), vertex_star.end(), SurfaceFaceId{0}) ||
        !std::binary_search(vertex_star.begin(), vertex_star.end(), SurfaceFaceId{8}))
        return 77;
    grid.build_provisional = [&](const auto &retained, const auto &sets, const auto &controls)
    {
        auto built = buildProvisionalTransition(grid_context, retained, sets, {}, controls);
        if (built.hasValue() && std::binary_search(retained.begin(), retained.end(), SurfaceFaceId{4}))
        {
            built.value().forced_rollback_high_faces.push_back(4);
            built.value().forced_rollback_by_source[4] = {4};
        }
        return built;
    };
    const auto full_grid = resolver.resolve(grid);
    if (!full_grid.hasValue()) return 71;
    grid.build_transition_patches = [&](const auto &retained, const auto &sets,
                                        const auto &selected, const auto &controls)
    { return buildProvisionalTransitionPatches(grid_context, retained, sets, selected, {}, controls); };
    grid.affected_transition_faces = [&](const auto &changed)
    { return grid_context.affectedFaces(changed); };
    grid.verify_local_rebuilds = true;
    const auto local_grid = resolver.resolve(grid);
    if (!local_grid.hasValue() || local_grid.value().provisional_full_builds != 1 ||
        local_grid.value().provisional_local_rebuilds == 0 ||
        local_grid.value().retained_high_faces != full_grid.value().retained_high_faces ||
        local_grid.value().iterations != full_grid.value().iterations)
        return 72;

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
    assert(chosen == Scalar{0.25});
    assert(external_builds == 1);

    LayerTransitionInput narrow_external;
    narrow_external.completed_layer = 1;
    narrow_external.build_provisional = [](
        const std::vector<SurfaceFaceId> &,
        const LayerFaceSets &,
        const ExternalPatchControls &controls)
    {
        const Scalar scale = controls.distanceScale(78);
        ProvisionalLayerTransition value;
        if (!controls.keepHexa(78))
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

    LayerTransitionInput multiple_external;
    multiple_external.completed_layer = 1;
    std::size_t multiple_builds{};
    std::size_t multiple_local_builds{};
    multiple_external.build_provisional = [&multiple_builds](
        const std::vector<SurfaceFaceId> &,
        const LayerFaceSets &,
        const ExternalPatchControls &controls)
    {
        ++multiple_builds;
        ProvisionalLayerTransition value;
        for (const SurfaceFaceId id : {SurfaceFaceId{80}, SurfaceFaceId{81}})
        {
            const Scalar offset = id == 80 ? Scalar{0} : Scalar{10};
            const Scalar scale = controls.distanceScale(id);
            value.boundary.candidate_triangles.push_back({
                {{{offset-1,-1,0},{offset+1,-1,0},{offset,1,scale}}},
                {{{id*10+0,1,0},{id*10+1,1,0},{id*10+2,1,0}}},
                {id,1,BoundaryOwnerRole::ExternalPatch,{}}});
            ResolvedTransitionTopology topology;
            topology.source_face_id = id;
            topology.layer = 1;
            topology.template_kind = TransitionTemplateKind::QuadTopCap;
            topology.terminal_quad_decision =
                TerminalQuadDecision::ExternalPatch;
            topology.generated_point = Point3{offset,0,scale};
            value.resolved_topology.push_back(std::move(topology));
        }
        value.all_top_faces_are_triangles = true;
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    multiple_external.build_external_patches = [&multiple_local_builds](
        const std::vector<SurfaceFaceId> &,
        const LayerFaceSets &,
        const std::vector<SurfaceFaceId> &selected,
        const ExternalPatchControls &controls)
    {
        ++multiple_local_builds;
        ProvisionalLayerTransition value;
        for (const SurfaceFaceId id : selected)
        {
            const Scalar offset = id == 80 ? Scalar{0} : Scalar{10};
            const Scalar scale = controls.distanceScale(id);
            value.boundary.candidate_triangles.push_back({
                {{{offset-1,-1,0},{offset+1,-1,0},{offset,1,scale}}},
                {{{id*10+0,1,0},{id*10+1,1,0},{id*10+2,1,0}}},
                {id,1,BoundaryOwnerRole::ExternalPatch,{}}});
            ResolvedTransitionTopology topology;
            topology.source_face_id = id;
            topology.layer = 1;
            topology.template_kind = TransitionTemplateKind::QuadTopCap;
            topology.terminal_quad_decision =
                TerminalQuadDecision::ExternalPatch;
            topology.generated_point = Point3{offset,0,scale};
            value.resolved_topology.push_back(std::move(topology));
        }
        value.all_top_faces_are_triangles = true;
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    std::vector<CollisionTriangle> multiple_ceilings;
    for (const SurfaceFaceId id : {SurfaceFaceId{80}, SurfaceFaceId{81}})
    {
        const Scalar offset = id == 80 ? Scalar{0} : Scalar{10};
        CollisionTriangle value;
        value.points = {{{offset-2,-2,0.1},
                         {offset+2,-2,0.1},
                         {offset,2,0.1}}};
        value.vertex_keys = {{{id*10+3,0,0},
                              {id*10+4,0,0},
                              {id*10+5,0,0}}};
        value.owner_kind = CollisionOwnerKind::OriginalSurface;
        value.owner_id = id;
        value.boundary_vertex_count = 3;
        std::copy(value.points.begin(), value.points.end(),
                  value.boundary_points.begin());
        std::copy(value.vertex_keys.begin(), value.vertex_keys.end(),
                  value.boundary_vertex_keys.begin());
        multiple_ceilings.push_back(std::move(value));
    }
    auto multiple_index = CollisionIndex::build(
        std::move(multiple_ceilings));
    if (!multiple_index.hasValue()) return 57;
    multiple_external.original_surface = std::move(multiple_index.value());
    const auto multiple_result = resolver.resolve(multiple_external);
    if (!multiple_result.hasValue() ||
        multiple_result.value().resolved_topology.size() != 2)
        return 58;
    for (const auto &topology : multiple_result.value().resolved_topology)
        if (!topology.generated_point.has_value() ||
            topology.generated_point->z() >= Scalar{0.1})
            return 59;
    if (multiple_builds != 1 || multiple_local_builds == 0) return 60;
    if (multiple_result.value().collision_full_builds != 1 ||
        multiple_result.value().collision_incremental_updates != 1)
        return 61;

    // A patch resolved by an apex candidate must not be pulled into the
    // distance-bisection update for another patch. In particular, looking up
    // distance_scales[id] here would insert a default zero for the resolved
    // patch and poison its final provisional build.
    LayerTransitionInput mixed_external;
    mixed_external.completed_layer = 1;
    bool saw_zero_scale_for_apex_resolved_patch = false;
    bool saw_saved_exact_apex = false;
    mixed_external.build_provisional = [&] (
        const std::vector<SurfaceFaceId> &,
        const LayerFaceSets &,
        const ExternalPatchControls &controls)
    {
        ProvisionalLayerTransition value;
        for (const SurfaceFaceId id : {SurfaceFaceId{90}, SurfaceFaceId{91}})
        {
            Scalar apex_z = controls.distanceScale(id);
            if (id == 90)
            {
                const auto saved = controls.explicit_apex_points.find(id);
                if (saved != controls.explicit_apex_points.end())
                {
                    saw_saved_exact_apex =
                        saved->second == Point3{0,0,Scalar{0.05}};
                    apex_z = saved->second.z();
                }
            }
            if (id == 90 && controls.apex_candidate_indices.count(id) != 0)
                apex_z = Scalar{0.05};
            if (id == 90 && controls.distance_scales.count(id) != 0 &&
                controls.distance_scales.at(id) == Scalar{0})
                saw_zero_scale_for_apex_resolved_patch = true;
            value.boundary.candidate_triangles.push_back({
                {{{id == 90 ? Scalar{-1} : Scalar{9},-1,0},
                  {id == 90 ? Scalar{1} : Scalar{11},-1,0},
                  {id == 90 ? Scalar{0} : Scalar{10},1,apex_z}}},
                {{{id*10+0,1,0},{id*10+1,1,0},{id*10+2,1,0}}},
                {id,1,BoundaryOwnerRole::ExternalPatch,{}}});
            ResolvedTransitionTopology topology;
            topology.source_face_id = id;
            topology.layer = 1;
            topology.template_kind = TransitionTemplateKind::QuadTopCap;
            topology.terminal_quad_decision =
                TerminalQuadDecision::ExternalPatch;
            topology.generated_point = Point3{id == 90 ? Scalar{0} : Scalar{10},
                                               0, apex_z};
            value.resolved_topology.push_back(std::move(topology));
        }
        value.all_top_faces_are_triangles = true;
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    std::vector<CollisionTriangle> mixed_ceilings;
    for (const SurfaceFaceId id : {SurfaceFaceId{90}, SurfaceFaceId{91}})
    {
        const Scalar offset = id == 90 ? Scalar{0} : Scalar{10};
        CollisionTriangle ceiling_value;
        ceiling_value.points = {{{offset-2,-2,Scalar{0.1}},
                                  {offset+2,-2,Scalar{0.1}},
                                  {offset,2,Scalar{0.1}}}};
        ceiling_value.vertex_keys = {{{id*10+3,0,0},
                                      {id*10+4,0,0},
                                      {id*10+5,0,0}}};
        ceiling_value.owner_kind = CollisionOwnerKind::OriginalSurface;
        ceiling_value.owner_id = id;
        ceiling_value.boundary_vertex_count = 3;
        std::copy(ceiling_value.points.begin(), ceiling_value.points.end(),
                  ceiling_value.boundary_points.begin());
        std::copy(ceiling_value.vertex_keys.begin(), ceiling_value.vertex_keys.end(),
                  ceiling_value.boundary_vertex_keys.begin());
        mixed_ceilings.push_back(std::move(ceiling_value));
    }
    auto mixed_index = CollisionIndex::build(std::move(mixed_ceilings));
    if (!mixed_index.hasValue()) return 63;
    mixed_external.original_surface = std::move(mixed_index.value());
    const auto mixed_result = resolver.resolve(mixed_external);
    if (!mixed_result.hasValue() ||
        saw_zero_scale_for_apex_resolved_patch ||
        !saw_saved_exact_apex ||
        mixed_result.value().resolved_topology.size() != 2)
        return 64;
    const auto resolved_apex = std::find_if(
        mixed_result.value().resolved_topology.begin(),
        mixed_result.value().resolved_topology.end(),
        [](const auto &topology) { return topology.source_face_id == 90; });
    if (resolved_apex == mixed_result.value().resolved_topology.end() ||
        !resolved_apex->generated_point.has_value() ||
        resolved_apex->generated_point->z() != Scalar{0.05})
        return 65;

    auto repeated_external = multiple_external;
    repeated_external.current_front = front({30});
    repeated_external.candidate_front = front({30});
    repeated_external.completed_layer = 3;
    const auto external_builder = repeated_external.build_provisional;
    repeated_external.build_provisional = [external_builder](const auto &retained,
        const auto &sets, const auto &controls)
    {
        auto value = external_builder(retained, sets, controls);
        if (value.hasValue() && !retained.empty())
        {
            value.value().forced_rollback_high_faces = {30};
            value.value().forced_rollback_by_source[30] = {30};
        }
        return value;
    };
    const auto repeated_result = resolver.resolve(repeated_external);
    if (!repeated_result.hasValue() || repeated_result.value().iterations != 2 ||
        repeated_result.value().search_index_full_builds != 1)
        return 73;

    LayerTransitionInput rollback_resets_external;
    rollback_resets_external.current_front = front({30,31});
    rollback_resets_external.candidate_front = front({30,31});
    rollback_resets_external.candidate_front.layer = 4;
    rollback_resets_external.completed_layer = 3;
    bool observed_preserved_controls_after_rollback = false;
    bool observed_reset_controls_after_rollback = false;
    bool expect_reset_controls = false;
    rollback_resets_external.build_provisional = [&] (
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &,
        const ExternalPatchControls &controls)
    {
        const bool temporary_high_face = std::binary_search(
            retained.begin(), retained.end(), SurfaceFaceId{30});
        if (!temporary_high_face)
        {
            if (expect_reset_controls)
                observed_reset_controls_after_rollback =
                    !controls.keepHexa(31) &&
                    controls.distance_scales.find(31) ==
                        controls.distance_scales.end() &&
                    controls.apex_candidate_indices.find(31) ==
                        controls.apex_candidate_indices.end() &&
                    controls.robust_candidate_indices.find(31) ==
                        controls.robust_candidate_indices.end() &&
                    controls.explicit_apex_points.find(31) ==
                        controls.explicit_apex_points.end();
            else
                observed_preserved_controls_after_rollback =
                    !controls.keepHexa(31) &&
                    controls.distance_scales.find(31) !=
                        controls.distance_scales.end();
        }


        ProvisionalLayerTransition value;
        if (temporary_high_face)
        {
            value.boundary.candidate_triangles.push_back({
                {{{-2,0,0.125},{2,0,0.125},{0,0,1}}},
                {{{300,4,0},{301,4,0},{302,4,0}}},
                {30,4,BoundaryOwnerRole::RegularCandidate,{30}}});
            value.forced_rollback_high_faces.push_back(30);
            value.forced_rollback_by_source[30] = {30};
        }

        ResolvedTransitionTopology topology;
        topology.source_face_id = 31;
        topology.layer = 4;
        topology.template_kind = TransitionTemplateKind::QuadTopCap;
        topology.terminal_quad_decision = controls.keepHexa(31)
            ? TerminalQuadDecision::KeepHexa
            : TerminalQuadDecision::ExternalPatch;
        if (topology.terminal_quad_decision ==
            TerminalQuadDecision::ExternalPatch)
        {
            const Scalar scale = controls.distanceScale(31);
            topology.generated_point = Point3{0,0,scale};
            value.boundary.candidate_triangles.push_back({
                {{{-1,-1,0},{1,-1,0},{0,1,scale}}},
                {{{310,4,0},{311,4,0},{312,4,0}}},
                {31,4,BoundaryOwnerRole::ExternalPatch,{}}});
        }
        value.resolved_topology.push_back(std::move(topology));
        value.all_top_faces_are_triangles = true;
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    rollback_resets_external.affected_transition_faces = [](
        const auto &) { return std::vector<SurfaceFaceId>{30}; };
    const auto rollback_reset_result =
        resolver.resolve(rollback_resets_external);
    if (!rollback_reset_result.hasValue() ||
        rollback_reset_result.value().iterations != 2 ||
        !observed_preserved_controls_after_rollback ||
        rollback_reset_result.value().resolved_topology.size() != 1 ||
        rollback_reset_result.value().resolved_topology.front()
                .terminal_quad_decision !=
            TerminalQuadDecision::ExternalPatch)
        return 62;
    // Legacy callbacks may provide only the flat forced-rollback list.
    LayerTransitionInput legacy_local;
    legacy_local.current_front = front({30,31});
    legacy_local.candidate_front = front({30,31});
    legacy_local.completed_layer = 3;
    legacy_local.build_provisional = [](const auto &retained, const auto &, const auto &)
    {
        ProvisionalLayerTransition value;
        value.all_top_faces_are_triangles = true;
        if (!retained.empty()) value.forced_rollback_high_faces = {retained.front()};
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    legacy_local.affected_transition_faces = [](const auto &ids) { return ids; };
    legacy_local.build_transition_patches = [&](const auto &retained, const auto &sets,
        const auto &, const auto &controls)
    { return legacy_local.build_provisional(retained, sets, controls); };
    const auto legacy_result = resolver.resolve(legacy_local);
    if (!legacy_result.hasValue() || !legacy_result.value().retained_high_faces.empty())
        return 75;

    legacy_local.build_provisional = [](const auto &retained, const auto &, const auto &)
    {
        ProvisionalLayerTransition value;
        value.all_top_faces_are_triangles = false;
        if (std::binary_search(retained.begin(), retained.end(), SurfaceFaceId{30}))
        {
            value.forced_rollback_high_faces = {30};
            value.forced_rollback_by_source[30] = {30};
        }
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    legacy_local.build_transition_patches = [](const auto &, const auto &, const auto &, const auto &)
    {
        ProvisionalLayerTransition value;
        value.all_top_faces_are_triangles = true;
        return ProvisionalLayerTransitionResult::success(std::move(value));
    };
    const auto aggregate_result = resolver.resolve(legacy_local);
    if (!aggregate_result.hasValue() || aggregate_result.value().all_top_faces_are_triangles)
        return 76;

    expect_reset_controls = true;
    rollback_resets_external.affected_transition_faces = [](const auto &)
    { return std::vector<SurfaceFaceId>{30,31}; };
    rollback_resets_external.build_transition_patches = [&](const auto &retained,
        const auto &sets, const auto &, const auto &controls)
    { return rollback_resets_external.build_provisional(retained, sets, controls); };
    rollback_resets_external.verify_local_rebuilds = true;
    const auto local_reset = resolver.resolve(rollback_resets_external);
    if (!local_reset.hasValue() || local_reset.value().iterations != 2 ||
        local_reset.value().provisional_local_rebuilds != 1 ||
        !observed_reset_controls_after_rollback)
        return 74;
}
