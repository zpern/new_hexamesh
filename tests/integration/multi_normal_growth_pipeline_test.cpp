#include <algorithm>
#include <variant>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/multi_normal_mesh_merge.hpp>
#include <boundary_mesh/growth/multi_normal_transition_generator.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

int main()
{
    using namespace boundary_mesh;

    SurfaceMesh mesh;
    mesh.vertices = {
        Point3{0, 0, 0}, Point3{1, 0, 0},
        Point3{1, 1, 0}, Point3{0, 1, 0},
        Point3{0, 0, 1}, Point3{1, 0, 1},
        Point3{1, 1, 1}, Point3{0, 1, 1}};
    mesh.faces = {
        Quad{{0, 3, 2, 1}}, Quad{{4, 5, 6, 7}},
        Quad{{0, 1, 5, 4}}, Quad{{1, 2, 6, 5}},
        Quad{{2, 3, 7, 6}}, Quad{{3, 0, 4, 7}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Wall, 10},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Farfield, 20}};

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue()) return 2;
    const auto initial = GrowthFrontBuilder{}.buildInitial(
        mesh, patch.value());
    if (!initial.hasValue() || initial.value().faces.size() != 1 ||
        !std::holds_alternative<Quad>(initial.value().faces[0]))
    {
        return 3;
    }

    MultiNormalOptions multi_normal_options;
    const auto transition = generateMultiNormalTransition(
        initial.value(), multi_normal_options);
    if (!transition.hasValue() || transition.value().applied)
    {
        return 4;
    }

    GrowthFront transformed = transition.value().transformed_front;
    const Quad quad = std::get<Quad>(transformed.faces[0]);
    transformed.faces = {
        Triangle{{quad.vertex_ids[0], quad.vertex_ids[1], quad.vertex_ids[2]}},
        Triangle{{quad.vertex_ids[0], quad.vertex_ids[2], quad.vertex_ids[3]}}};
    transformed.source_face_ids = {
        initial.value().source_face_ids[0],
        initial.value().source_face_ids[0]};

    std::vector<SourceVertexGrowthProfile> profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        profiles.push_back({vertex.source_vertex_id, {0.1, 1.0, 0}});
    }

    RegularLayerGrowthOptions regular_options;
    const auto regular = generateRegularLayers(
        mesh, topology.value(), patch.value(), transformed,
        profiles, regular_options);
    if (!regular.hasValue()) return 5;
    if (!regular.value().mesh.cells.empty()) return 6;
    if (regular.value().faces.size() != 1) return 7;
    if (regular.value().faces[0].accepted_layer_count != 0) return 8;
    if (std::any_of(
            regular.value().mesh.metadata.begin(),
            regular.value().mesh.metadata.end(),
            [](const CellMetadata &metadata)
            {
                return metadata.role != CellRole::RegularLayer;
            })) return 9;

    const auto merged = mergeMultiNormalAndRegularMeshes(
        transition.value(), regular.value().mesh);
    if (!merged.hasValue() ||
        merged.value().vertices != regular.value().mesh.vertices ||
        merged.value().cells.size() != regular.value().mesh.cells.size())
    {
        return 10;
    }

    return 0;
}
