#include <algorithm>
#include <array>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/growth/termination_propagator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

using namespace boundary_mesh;

SurfaceMesh makeDelayedSelectionMesh()
{
    SurfaceMesh mesh;
    mesh.vertices = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    mesh.faces = {
        Quad{{0, 3, 2, 1}}, Quad{{4, 5, 6, 7}},
        Triangle{{0, 1, 5}}, Triangle{{0, 5, 4}},
        Quad{{1, 2, 6, 5}}, Quad{{2, 3, 7, 6}},
        Quad{{3, 0, 4, 7}}};
    mesh.face_tags.resize(
        mesh.faces.size(), {SurfaceBoundaryKind::Wall, 1});
    return mesh;
}

int main()
{
    SurfaceMesh mesh;
    mesh.vertices = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    mesh.faces = {
        Quad{{0, 3, 2, 1}}, Quad{{4, 5, 6, 7}},
        Quad{{0, 1, 5, 4}}, Quad{{1, 2, 6, 5}},
        Quad{{2, 3, 7, 6}}, Quad{{3, 0, 4, 7}}};
    mesh.face_tags.resize(6, {SurfaceBoundaryKind::Farfield, 0});
    mesh.face_tags[1] = {SurfaceBoundaryKind::Wall, 1};
    mesh.face_tags[2] = {SurfaceBoundaryKind::Wall, 1};
    mesh.face_tags[4] = {SurfaceBoundaryKind::Wall, 1};

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    assert(patch.hasValue());
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    assert(front.hasValue());

    std::vector<SourceVertexGrowthProfile> source_profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        source_profiles.push_back({vertex.source_vertex_id, {0.1, 1.0, 10}});
    }
    const auto profiles = GrowthProfileBuilder{}.build(
        patch.value(), source_profiles);
    assert(profiles.hasValue());
    const auto initial = buildFaceLayerConstraints(
        patch.value(), front.value(), profiles.value());
    assert(initial.hasValue());

    const auto propagator = TerminationPropagator::build(
        patch.value(), topology.value());
    assert(propagator.hasValue());
    assert((propagator.value().neighbors(2) ==
            std::vector<SurfaceFaceId>{1}));
    assert((propagator.value().neighbors(1) ==
            std::vector<SurfaceFaceId>{2, 4}));

    auto diff1 = initial.value();
    diff1.find(2)->allowed_layer_count = 2;
    const auto changed1 = propagator.value().propagateInitial(diff1, 1);
    assert(changed1.hasValue());
    assert(diff1.find(2)->allowed_layer_count == 2);
    assert(diff1.find(1)->allowed_layer_count == 3);
    assert(diff1.find(4)->allowed_layer_count == 4);

    auto diff0 = initial.value();
    diff0.find(2)->allowed_layer_count = 2;
    const auto changed0 = propagator.value().propagateInitial(diff0, 0);
    assert(changed0.hasValue());
    assert(diff0.find(1)->allowed_layer_count == 2);
    assert(diff0.find(4)->allowed_layer_count == 2);

    auto runtime = initial.value();
    const std::vector<FaceStopEvent> direct{{
        0, 2, 5, FaceStopReason::Collision}};
    const auto runtime_changed = propagator.value().applyDirectStops(
        runtime, direct, 1);
    assert(runtime_changed.hasValue());
    assert(runtime.find(2)->allowed_layer_count == 4);
    assert(runtime.find(2)->limit_kind == FaceLayerLimitKind::DirectStop);
    assert(runtime.find(2)->direct_reason == FaceStopReason::Collision);
    assert(runtime.find(1)->allowed_layer_count == 5);
    assert(runtime.find(4)->allowed_layer_count == 6);

    LayerStepResult actual_candidates;
    actual_candidates.layer = 5;
    actual_candidates.next_front = front.value();
    actual_candidates.next_front.layer = 5;
    actual_candidates.next_front.faces.erase(
        actual_candidates.next_front.faces.begin());
    actual_candidates.next_front.source_face_ids.erase(
        actual_candidates.next_front.source_face_ids.begin());
    actual_candidates.previous_front_face_indices = {1, 2};
    actual_candidates.previous_front_vertex_indices.resize(
        actual_candidates.next_front.vertices.size());
    for (std::size_t index = 0;
         index < actual_candidates.previous_front_vertex_indices.size();
         ++index)
        actual_candidates.previous_front_vertex_indices[index] = index;
    actual_candidates.stopped_faces = {{
        0, 1, 5, FaceStopReason::Collision}};
    auto actual_constraints = initial.value();
    actual_constraints.find(1)->allowed_layer_count = 4;
    std::vector<SurfaceFaceId> actual_pending;
    const auto actual_filtered =
        propagator.value().filterSingleHighEdgeCandidates(
            front.value(),
            actual_candidates,
            actual_constraints,
            1,
            actual_pending);
    assert(actual_filtered.hasValue());
    assert(actual_filtered.value().next_front.source_face_ids ==
           std::vector<SurfaceFaceId>{2});
    assert(actual_constraints.find(4)->allowed_layer_count == 4);
    assert((actual_pending == std::vector<SurfaceFaceId>{2}));

    const SurfaceMesh delayed_mesh = makeDelayedSelectionMesh();
    const auto delayed_topology =
        SurfaceTopologyBuilder{}.build(delayed_mesh);
    assert(delayed_topology.hasValue());
    const auto delayed_patch = GrowthPatchBuilder{}.build(
        delayed_mesh, delayed_topology.value());
    assert(delayed_patch.hasValue());
    const auto delayed_front = GrowthFrontBuilder{}.buildInitial(
        delayed_mesh, delayed_patch.value());
    assert(delayed_front.hasValue());
    std::vector<SourceVertexGrowthProfile> delayed_source_profiles;
    for (const PatchVertex &vertex : delayed_patch.value().vertices())
        delayed_source_profiles.push_back(
            {vertex.source_vertex_id, {0.1, 1.0, 10}});
    const auto delayed_profiles = GrowthProfileBuilder{}.build(
        delayed_patch.value(), delayed_source_profiles);
    assert(delayed_profiles.hasValue());
    auto delayed_constraints = buildFaceLayerConstraints(
        delayed_patch.value(), delayed_front.value(),
        delayed_profiles.value());
    assert(delayed_constraints.hasValue());
    delayed_constraints.value().find(1)->allowed_layer_count = 5;
    delayed_constraints.value().find(1)->limit_kind =
        FaceLayerLimitKind::NeighborConstraint;
    const auto delayed_propagator = TerminationPropagator::build(
        delayed_patch.value(), delayed_topology.value());
    assert(delayed_propagator.hasValue());

    LayerStepResult layer5;
    layer5.layer = 5;
    layer5.next_front = delayed_front.value();
    layer5.next_front.layer = 5;
    layer5.previous_front_face_indices = {0, 1, 2, 3, 4, 5, 6};
    layer5.previous_front_vertex_indices.resize(
        layer5.next_front.vertices.size());
    for (std::size_t index = 0;
         index < layer5.previous_front_vertex_indices.size();
         ++index)
        layer5.previous_front_vertex_indices[index] = index;

    std::vector<SurfaceFaceId> delayed_pending;
    const auto layer5_filtered =
        delayed_propagator.value().filterSingleHighEdgeCandidates(
            delayed_front.value(), layer5, delayed_constraints.value(), 1,
            delayed_pending);
    assert(layer5_filtered.hasValue());
    assert((delayed_pending == std::vector<SurfaceFaceId>{1}));
    assert(delayed_constraints.value().find(2)->allowed_layer_count == 10);
    assert(delayed_constraints.value().find(5)->allowed_layer_count == 10);

    LayerStepResult layer6;
    layer6.layer = 6;
    layer6.next_front = layer5_filtered.value().next_front;
    layer6.next_front.layer = 6;
    const SurfaceFace face2 = layer6.next_front.faces[2];
    const SurfaceFace face5 = layer6.next_front.faces[5];
    layer6.next_front.faces = {face2, face5};
    layer6.next_front.source_face_ids = {2, 5};
    layer6.previous_front_face_indices = {2, 5};
    layer6.previous_front_vertex_indices.resize(
        layer6.next_front.vertices.size());
    for (std::size_t index = 0;
         index < layer6.previous_front_vertex_indices.size();
         ++index)
        layer6.previous_front_vertex_indices[index] = index;

    const auto layer6_filtered =
        delayed_propagator.value().filterSingleHighEdgeCandidates(
            layer5_filtered.value().next_front, layer6,
            delayed_constraints.value(), 1, delayed_pending);
    assert(layer6_filtered.hasValue());
    assert((delayed_pending == std::vector<SurfaceFaceId>{5}));
    assert(delayed_constraints.value().find(2)->allowed_layer_count == 5);
    assert((layer6_filtered.value().next_front.source_face_ids ==
            std::vector<SurfaceFaceId>{5}));

    LayerStepResult empty_layer7;
    empty_layer7.layer = 7;
    empty_layer7.next_front.layer = 7;
    const auto delayed_exhausted =
        delayed_propagator.value().filterSingleHighEdgeCandidates(
            layer6_filtered.value().next_front, empty_layer7,
            delayed_constraints.value(), 1, delayed_pending);
    assert(delayed_exhausted.hasValue());
    assert(delayed_pending.empty());
    assert(delayed_exhausted.value().next_front.source_face_ids.empty());
    assert(delayed_constraints.value().find(5)->allowed_layer_count == 6);

    SurfaceMesh triangle_mesh;
    triangle_mesh.vertices = {
        {0,0,0}, {1,0,0}, {0,1,0}, {0,0,1}};
    triangle_mesh.faces = {
        Triangle{{0,2,1}}, Triangle{{0,1,3}},
        Triangle{{1,2,3}}, Triangle{{2,0,3}}};
    triangle_mesh.face_tags.resize(
        4, {SurfaceBoundaryKind::Wall, 1});
    const auto triangle_topology =
        SurfaceTopologyBuilder{}.build(triangle_mesh);
    assert(triangle_topology.hasValue());
    const auto triangle_patch = GrowthPatchBuilder{}.build(
        triangle_mesh, triangle_topology.value());
    assert(triangle_patch.hasValue());
    const auto triangle_front = GrowthFrontBuilder{}.buildInitial(
        triangle_mesh, triangle_patch.value());
    assert(triangle_front.hasValue());
    std::vector<SourceVertexGrowthProfile> triangle_source_profiles;
    for (const PatchVertex &vertex : triangle_patch.value().vertices())
        triangle_source_profiles.push_back(
            {vertex.source_vertex_id, {0.1, 1.0, 10}});
    const auto triangle_profiles = GrowthProfileBuilder{}.build(
        triangle_patch.value(), triangle_source_profiles);
    assert(triangle_profiles.hasValue());
    auto triangle_constraints = buildFaceLayerConstraints(
        triangle_patch.value(), triangle_front.value(),
        triangle_profiles.value());
    assert(triangle_constraints.hasValue());
    triangle_constraints.value().find(0)->allowed_layer_count = 5;
    triangle_constraints.value().find(0)->limit_kind =
        FaceLayerLimitKind::NeighborConstraint;
    const auto triangle_propagator = TerminationPropagator::build(
        triangle_patch.value(), triangle_topology.value());
    assert(triangle_propagator.hasValue());
    LayerStepResult triangle_candidates;
    triangle_candidates.layer = 5;
    triangle_candidates.next_front = triangle_front.value();
    triangle_candidates.next_front.layer = 5;
    triangle_candidates.previous_front_face_indices = {0,1,2,3};
    triangle_candidates.previous_front_vertex_indices.resize(
        triangle_candidates.next_front.vertices.size());
    for (std::size_t index = 0;
         index < triangle_candidates.previous_front_vertex_indices.size();
         ++index)
        triangle_candidates.previous_front_vertex_indices[index] = index;
    std::vector<SurfaceFaceId> triangle_pending;
    const auto triangle_layer5 =
        triangle_propagator.value().filterSingleHighEdgeCandidates(
            triangle_front.value(), triangle_candidates,
            triangle_constraints.value(), 1, triangle_pending);
    assert(triangle_layer5.hasValue());
    assert((triangle_pending == std::vector<SurfaceFaceId>{0}));
    assert(triangle_constraints.value().find(1)->allowed_layer_count == 10);
    assert(triangle_constraints.value().find(2)->allowed_layer_count == 10);

    LayerStepResult empty_layer6;
    empty_layer6.layer = 6;
    empty_layer6.next_front.layer = 6;
    std::vector<SurfaceFaceId> no_high_pending{0};
    const auto no_high = triangle_propagator.value()
        .filterSingleHighEdgeCandidates(
            triangle_layer5.value().next_front, empty_layer6,
            triangle_constraints.value(), 1, no_high_pending);
    assert(no_high.hasValue());
    assert(no_high_pending.empty());
    assert(triangle_constraints.value().find(3)->allowed_layer_count == 10);

    LayerStepResult triangle_layer6;
    triangle_layer6.layer = 6;
    triangle_layer6.next_front = triangle_layer5.value().next_front;
    triangle_layer6.next_front.layer = 6;
    triangle_layer6.next_front.faces.erase(
        triangle_layer6.next_front.faces.begin());
    triangle_layer6.next_front.source_face_ids.erase(
        triangle_layer6.next_front.source_face_ids.begin());
    triangle_layer6.previous_front_face_indices = {1, 2, 3};
    triangle_layer6.previous_front_vertex_indices.resize(
        triangle_layer6.next_front.vertices.size());
    for (std::size_t index = 0;
         index < triangle_layer6.previous_front_vertex_indices.size();
         ++index)
        triangle_layer6.previous_front_vertex_indices[index] = index;

    const auto triangle_confirmed =
        triangle_propagator.value().filterSingleHighEdgeCandidates(
            triangle_layer5.value().next_front, triangle_layer6,
            triangle_constraints.value(), 1, triangle_pending);
    assert(triangle_confirmed.hasValue());
    assert((triangle_pending == std::vector<SurfaceFaceId>{3}));
    assert(triangle_constraints.value().find(1)->allowed_layer_count == 5);
    assert(triangle_constraints.value().find(2)->allowed_layer_count == 5);
    assert((triangle_confirmed.value().next_front.source_face_ids ==
            std::vector<SurfaceFaceId>{3}));

    LayerStepResult empty_triangle_layer7;
    empty_triangle_layer7.layer = 7;
    empty_triangle_layer7.next_front.layer = 7;
    const auto triangle_exhausted =
        triangle_propagator.value().filterSingleHighEdgeCandidates(
            triangle_confirmed.value().next_front, empty_triangle_layer7,
            triangle_constraints.value(), 1, triangle_pending);
    assert(triangle_exhausted.hasValue());
    assert(triangle_pending.empty());
    assert(triangle_exhausted.value().next_front.source_face_ids.empty());
    assert(triangle_constraints.value().find(3)->allowed_layer_count == 6);

    auto isotropic_runtime = initial.value();
    const std::vector<FaceStopEvent> isotropic_stop{{
        0, 2, 3, FaceStopReason::IsotropicHeightReached}};
    const auto isotropic_changed =
        propagator.value().applyDirectStops(
            isotropic_runtime,
            isotropic_stop,
            1);
    if (!isotropic_changed.hasValue() ||
        isotropic_runtime.find(2)->allowed_layer_count != 2 ||
        isotropic_runtime.find(1)->allowed_layer_count != 3 ||
        isotropic_runtime.find(2)->direct_reason !=
            FaceStopReason::IsotropicHeightReached)
    {
        return 1;
    }

    auto filtering_constraints = initial.value();
    filtering_constraints.find(2)->allowed_layer_count = 4;
    filtering_constraints.find(2)->limit_kind =
        FaceLayerLimitKind::NeighborConstraint;
    LayerStepResult candidate_step;
    candidate_step.layer = 5;
    candidate_step.next_front = front.value();
    candidate_step.next_front.layer = 5;
    candidate_step.previous_front_vertex_indices.resize(
        candidate_step.next_front.vertices.size());
    for (std::size_t index = 0;
         index < candidate_step.previous_front_vertex_indices.size();
         ++index)
    {
        candidate_step.previous_front_vertex_indices[index] = index;
    }
    candidate_step.previous_front_face_indices = {0, 1, 2};
    candidate_step.accepted_stopped_faces = {
        {1, 2, 6, FaceStopReason::IsotropicHeightReached},
        {2, 4, 6, FaceStopReason::IsotropicHeightReached}};
    const auto filtered = propagator.value().filterCandidates(
        front.value(), candidate_step, filtering_constraints);
    assert(filtered.hasValue());
    assert(filtered.value().next_front.source_face_ids ==
           std::vector<SurfaceFaceId>({1, 4}));
    assert(filtered.value().stopped_faces.size() == 1);
    assert(filtered.value().stopped_faces.front().source_face_id == 2);
    assert(filtered.value().stopped_faces.front().reason ==
           FaceStopReason::NeighborLayerConstraint);
    if (filtered.value().accepted_stopped_faces.size() != 1 ||
        filtered.value().accepted_stopped_faces.front().source_face_id != 4)
    {
        return 2;
    }
}
