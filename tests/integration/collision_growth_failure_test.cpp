#include <cassert>
#include <variant>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

using namespace boundary_mesh;

int main()
{
    SurfaceMesh mesh;
    mesh.vertices = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {0, 1, 1}};
    mesh.faces = {
        Triangle{{0, 2, 1}}, Triangle{{3, 4, 5}},
        Quad{{0, 1, 4, 3}}, Quad{{1, 2, 5, 4}},
        Quad{{2, 0, 3, 5}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Farfield, 1},
        {SurfaceBoundaryKind::Wall, 2},
        {SurfaceBoundaryKind::Farfield, 1},
        {SurfaceBoundaryKind::Farfield, 1},
        {SurfaceBoundaryKind::Farfield, 1}};
    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    assert(patch.hasValue());
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    assert(front.hasValue());

    SurfaceMesh invalid = mesh;
    invalid.face_tags.pop_back();
    const std::vector<SourceVertexGrowthProfile> profiles{
        {3, {0.1, 1.0, 1}},
        {4, {0.1, 1.0, 1}},
        {5, {0.1, 1.0, 1}}};
    const auto result = generateRegularLayers(
        invalid,
        topology.value(),
        patch.value(),
        front.value(),
        profiles);
    assert(!result.hasValue());
    assert(std::get_if<CollisionInitializationFailure>(
               &result.error()) != nullptr);
}
