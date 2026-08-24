#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

int main()
{
    using namespace boundary_mesh;

    SurfaceMesh mesh;
    mesh.vertices = {
        Point3{0, 0, 0}, Point3{1, 0, 0},
        Point3{0, 1, 0}, Point3{0, 0, 1}};
    mesh.faces = {
        Triangle{{0, 2, 1}}, Triangle{{0, 1, 3}},
        Triangle{{1, 2, 3}}, Triangle{{2, 0, 3}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Wall, 10},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Farfield, 20}};

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue()) return 2;
    const auto front = GrowthFrontBuilder{}.buildInitial(
        mesh, patch.value());
    if (!front.hasValue()) return 3;

    std::vector<SourceVertexGrowthProfile> profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        profiles.push_back({vertex.source_vertex_id, {0.1, 1.0, 1}});
    }

    RegularLayerGrowthOptions options;
    options.multi_normal.enabled = true;
    options.multi_normal.transition_height = 0.05;
    const auto result = generateRegularLayers(
        mesh, topology.value(), patch.value(), front.value(),
        profiles, options);
    if (!result.hasValue() || result.value().mesh.cells.size() != 1 ||
        !result.value().omitted_quad_transitions.empty() ||
        result.value().faces.size() != 1 ||
        result.value().faces.front().accepted_layer_count != 1)
    {
        return 4;
    }

    return 0;
}
