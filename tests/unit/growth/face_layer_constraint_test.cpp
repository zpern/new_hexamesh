#include <cassert>

#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

using namespace boundary_mesh;

int main()
{
    RegularLayerGrowthOptions options;
    assert(options.max_layer_diff == 1);
    options.max_layer_diff = 0;
    assert(options.max_layer_diff == 0);

    SurfaceMesh mesh;
    mesh.vertices = {
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    mesh.faces = {
        Quad{{0, 3, 2, 1}}, Quad{{4, 5, 6, 7}},
        Quad{{0, 1, 5, 4}}, Quad{{1, 2, 6, 5}},
        Quad{{2, 3, 7, 6}}, Quad{{3, 0, 4, 7}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Farfield, 0},
        {SurfaceBoundaryKind::Wall, 1},
        {SurfaceBoundaryKind::Wall, 1},
        {SurfaceBoundaryKind::Farfield, 0},
        {SurfaceBoundaryKind::Farfield, 0},
        {SurfaceBoundaryKind::Farfield, 0}};

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    assert(patch.hasValue());
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    assert(front.hasValue());

    const std::vector<SourceVertexGrowthProfile> source_profiles{
        {0, {0.1, 1.0, 7}}, {1, {0.1, 1.0, 7}}, {4, {0.1, 1.0, 2}}, {5, {0.1, 1.0, 5}}, {6, {0.1, 1.0, 8}}, {7, {0.1, 1.0, 7}}};
    const auto profiles = GrowthProfileBuilder{}.build(
        patch.value(), source_profiles);
    assert(profiles.hasValue());

    const auto table = buildFaceLayerConstraints(
        patch.value(), front.value(), profiles.value());
    assert(table.hasValue());
    assert(table.value().entries().size() == 2);
    assert(table.value().find(1)->requested_layer_count == 2);
    assert(table.value().find(1)->allowed_layer_count == 2);
    assert(table.value().find(2)->requested_layer_count == 2);
    assert(table.value().find(2)->limit_kind ==
           FaceLayerLimitKind::Requested);
}
