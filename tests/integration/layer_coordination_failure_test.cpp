#include <vector>

#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/growth/termination_propagator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

int main()
{
    using namespace boundary_mesh;

    SurfaceMesh mesh;
    mesh.vertices = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    mesh.faces = {
        Quad{{0, 3, 2, 1}}, Quad{{4, 5, 6, 7}},
        Quad{{0, 1, 5, 4}}, Quad{{1, 2, 6, 5}},
        Quad{{2, 3, 7, 6}}, Quad{{3, 0, 4, 7}}};
    mesh.face_tags.resize(6, {SurfaceBoundaryKind::Farfield, 20});
    mesh.face_tags[1] = {SurfaceBoundaryKind::Wall, 10};

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue()) return 2;
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    if (!front.hasValue()) return 3;

    std::vector<SourceVertexGrowthProfile> source_profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        source_profiles.push_back(
            {vertex.source_vertex_id, {0.1, 1.0, 3}});
    }
    const auto profiles = GrowthProfileBuilder{}.build(
        patch.value(), source_profiles);
    if (!profiles.hasValue()) return 4;
    const auto constraints_result = buildFaceLayerConstraints(
        patch.value(), front.value(), profiles.value());
    if (!constraints_result.hasValue()) return 5;
    auto constraints = constraints_result.value();
    const auto before = constraints.entries();

    const auto propagator = TerminationPropagator::build(
        patch.value(), topology.value());
    if (!propagator.hasValue()) return 6;
    const std::vector<FaceStopEvent> invalid_events{
        {0, SurfaceFaceId{999}, 1, FaceStopReason::Collision}};
    const auto failure = propagator.value().applyDirectStops(
        constraints, invalid_events, 1);
    if (failure.hasValue()) return 7;
    if (failure.error().source_face_id != SurfaceFaceId{999} ||
        failure.error().layer != 1)
    {
        return 8;
    }
    const auto &after = constraints.entries();
    if (after.size() != before.size()) return 9;
    for (std::size_t index = 0; index < before.size(); ++index)
    {
        if (after[index].source_face_id != before[index].source_face_id ||
            after[index].requested_layer_count !=
                before[index].requested_layer_count ||
            after[index].allowed_layer_count !=
                before[index].allowed_layer_count ||
            after[index].limit_kind != before[index].limit_kind ||
            after[index].direct_reason != before[index].direct_reason)
        {
            return 10;
        }
    }
}
