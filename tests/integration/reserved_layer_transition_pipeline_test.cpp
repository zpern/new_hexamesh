#include <cassert>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/transition/reserved_layer_transition.hpp>

using namespace boundary_mesh;

int main()
{
    SurfaceMesh mesh;
    mesh.vertices = {
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}};
    mesh.faces = {
        Quad{{0,3,2,1}}, Quad{{4,5,6,7}},
        Quad{{0,1,5,4}}, Quad{{1,2,6,5}},
        Quad{{2,3,7,6}}, Quad{{3,0,4,7}}};
    mesh.face_tags.resize(6, {SurfaceBoundaryKind::Farfield, 0});
    mesh.face_tags[1] = {SurfaceBoundaryKind::Wall, 1};

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    assert(patch.hasValue());
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    assert(front.hasValue());
    std::vector<SourceVertexGrowthProfile> profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
        profiles.push_back({vertex.source_vertex_id, {0.05, 1.0, 1}});

    RegularLayerGrowthOptions options;
    options.isotropic_height = 100.0;
    const auto result = generateReservedLayerTransition(
        mesh, topology.value(), patch.value(), front.value(),
        profiles, options);
    assert(result.hasValue());
    assert(result.value().trial_growth.faces[0].accepted_layer_count == 3);
    assert(result.value().mesh.cells.size() == 8);
    assert(result.value().mesh.metadata.size() == 8);
    assert(result.value().boundary_layer_top.faces.size() == 2);
    for (const SurfaceFace &face : result.value().boundary_layer_top.faces)
        assert(std::holds_alternative<Triangle>(face));
    for (const CellMetadata &metadata : result.value().mesh.metadata)
        assert(metadata.layer <= 2);
}
