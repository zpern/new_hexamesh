#include <algorithm>
#include <array>
#include <cassert>

#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/growth/termination_propagator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

using namespace boundary_mesh;

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

    LayerStepResult capped_candidates;
    capped_candidates.layer = 5;
    capped_candidates.next_front = front.value();
    capped_candidates.next_front.layer = 5;
    capped_candidates.previous_front_face_indices = {0, 1, 2};
    capped_candidates.previous_front_vertex_indices.resize(
        capped_candidates.next_front.vertices.size());
    for (std::size_t index = 0;
         index < capped_candidates.previous_front_vertex_indices.size();
         ++index)
        capped_candidates.previous_front_vertex_indices[index] = index;
    auto capped_constraints = initial.value();
    capped_constraints.find(1)->allowed_layer_count = 5;
    capped_constraints.find(1)->limit_kind =
        FaceLayerLimitKind::NeighborConstraint;
    std::vector<SurfaceFaceId> capped_pending;
    const auto capped_filtered =
        propagator.value().filterSingleHighEdgeCandidates(
            front.value(), capped_candidates, capped_constraints, 1,
            capped_pending);
    assert(capped_filtered.hasValue());
    std::array<std::uint32_t, 2> capped_neighbor_limits{
        capped_constraints.find(2)->allowed_layer_count,
        capped_constraints.find(4)->allowed_layer_count};
    std::sort(capped_neighbor_limits.begin(), capped_neighbor_limits.end());
    assert((capped_neighbor_limits ==
            std::array<std::uint32_t, 2>{5, 6}));

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
    const auto triangle_filtered =
        [&]
        {
            std::vector<SurfaceFaceId> pending;
            return
        triangle_propagator.value().filterSingleHighEdgeCandidates(
            triangle_front.value(), triangle_candidates,
            triangle_constraints.value(), 1, pending);
        }();
    assert(triangle_filtered.hasValue());
    std::array<std::uint32_t, 3> triangle_neighbor_limits{
        triangle_constraints.value().find(1)->allowed_layer_count,
        triangle_constraints.value().find(2)->allowed_layer_count,
        triangle_constraints.value().find(3)->allowed_layer_count};
    std::sort(
        triangle_neighbor_limits.begin(), triangle_neighbor_limits.end());
    assert((triangle_neighbor_limits ==
            std::array<std::uint32_t, 3>{5, 5, 6}));

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
