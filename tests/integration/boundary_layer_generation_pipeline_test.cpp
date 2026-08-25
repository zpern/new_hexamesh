#include <algorithm>
#include <variant>

#include <boundary_mesh/growth/boundary_layer_generator.hpp>
#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

int main()
{
    using namespace boundary_mesh;

    SurfaceMesh surface;
    surface.vertices = {
        Point3{0, 0, 0}, Point3{1, 0, 0},
        Point3{1, 1, 0}, Point3{0, 1, 0},
        Point3{0, 0, 1}, Point3{1, 0, 1},
        Point3{1, 1, 1}, Point3{0, 1, 1}};
    surface.faces = {
        Quad{{0, 3, 2, 1}}, Quad{{4, 5, 6, 7}},
        Quad{{0, 1, 5, 4}}, Quad{{1, 2, 6, 5}},
        Quad{{2, 3, 7, 6}}, Quad{{3, 0, 4, 7}}};
    surface.face_tags = {
        {SurfaceBoundaryKind::Wall, 10},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Wall, 10},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Farfield, 20},
        {SurfaceBoundaryKind::Wall, 10}};

    const auto topology = SurfaceTopologyBuilder{}.build(surface);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(surface, topology.value());
    if (!patch.hasValue()) return 2;
    const auto front = GrowthFrontBuilder{}.buildInitial(
        surface, patch.value());
    if (!front.hasValue()) return 3;

    std::vector<SourceVertexGrowthProfile> profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        profiles.push_back({vertex.source_vertex_id, {0.05, 1.0, 0}});
    }

    MultiNormalOptions multi_normal;
    multi_normal.enabled = true;
    multi_normal.transition_height = 0.05;
    multi_normal.split_skewness_threshold = 0.5;

    const auto generated = generateBoundaryLayers(
        surface,
        topology.value(),
        patch.value(),
        front.value(),
        profiles,
        multi_normal,
        {});
    if (!generated.hasValue()) return 4;

    const BoundaryLayerGenerationResult &result = generated.value();
    if (!result.transition.applied) return 5;
    if (!result.regular.mesh.cells.empty()) return 6;
    if (result.mesh.cells.size() !=
        result.transition.transition_cells.cells.size()) return 7;
    if (result.regular.mesh.vertices.size() <
        result.transition.transformed_front.vertices.size()) return 8;

    for (std::size_t index = 0;
         index < result.transition.transformed_front.vertices.size();
         ++index)
    {
        if ((result.regular.mesh.vertices[index] -
             result.transition.transformed_front.vertices[index].position)
                .norm() > 1e-12)
        {
            return 9;
        }
    }
    if (std::any_of(
            result.mesh.metadata.begin(),
            result.mesh.metadata.end(),
            [](const CellMetadata &metadata)
            {
                return metadata.role != CellRole::Transition;
            }))
    {
        return 10;
    }
    if (result.top_surface.faces.size() !=
        result.transition.transformed_front.faces.size())
    {
        return 11;
    }
    if (result.farfield_boundary.faces.size() !=
        std::size_t{3} + result.top_surface.faces.size())
    {
        return 12;
    }
    if (std::any_of(
            result.top_surface.face_tags.begin(),
            result.top_surface.face_tags.end(),
            [](const SurfaceBoundaryTag &tag)
            {
                return tag.kind !=
                    SurfaceBoundaryKind::BoundaryLayerInterface;
            }))
    {
        return 13;
    }
    return 0;
}
