#include <array>
#include <cmath>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/growth_profile_builder.hpp>
#include <boundary_mesh/growth/regular_layer_stepper.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceMesh makePrismWithTopWall()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0},
            Point3{0.0, 1.0, 1.0}};
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{3}, VertexId{4}, VertexId{5}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{4}, VertexId{3}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{2}, VertexId{0}, VertexId{3}, VertexId{5}}}};
        mesh.face_tags = {
            {SurfaceBoundaryKind::Farfield, 20},
            {SurfaceBoundaryKind::Wall, 10},
            {SurfaceBoundaryKind::Symmetry, 30},
            {SurfaceBoundaryKind::Symmetry, 31},
            {SurfaceBoundaryKind::Symmetry, 32}};
        return mesh;
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceMesh mesh = makePrismWithTopWall();
    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue()) return 2;
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    if (!front.hasValue()) return 3;

    const std::vector<SourceVertexGrowthProfile> input_profiles{
        {VertexId{3}, {0.25, 1.0, 2}},
        {VertexId{4}, {0.25, 1.0, 2}},
        {VertexId{5}, {0.25, 1.0, 2}}};
    const auto profiles = GrowthProfileBuilder{}.build(
        patch.value(), input_profiles);
    if (!profiles.hasValue()) return 4;

    const auto result = RegularLayerStepper{}.step(
        front.value(), profiles.value(), RegularLayerGrowthOptions{});
    if (!result.hasValue()) return 5;

    const LayerStepResult &layer = result.value();
    if (layer.layer != 1 || layer.next_front.layer != 1 ||
        layer.next_front.vertices.size() != 3 ||
        layer.next_front.faces.size() != 1 ||
        layer.previous_front_vertex_indices !=
            std::vector<std::size_t>{0, 1, 2} ||
        layer.previous_front_face_indices !=
            std::vector<std::size_t>{0} ||
        !layer.stopped_faces.empty() ||
        !layer.completed_faces.empty())
    {
        return 6;
    }

    for (std::size_t index = 0; index < 3; ++index)
    {
        const Point3 expected =
            front.value().vertices[index] + Vector3{0.0, 0.0, 0.25};
        if ((layer.next_front.vertices[index] - expected).norm() > 1e-12)
        {
            return 7;
        }
    }

    const auto *bottom = std::get_if<Triangle>(&front.value().faces[0]);
    const auto *top = std::get_if<Triangle>(&layer.next_front.faces[0]);
    if (bottom == nullptr || top == nullptr) return 8;
    const PrismPoints points{
        front.value().vertices[bottom->vertex_ids[0]],
        front.value().vertices[bottom->vertex_ids[1]],
        front.value().vertices[bottom->vertex_ids[2]],
        layer.next_front.vertices[top->vertex_ids[0]],
        layer.next_front.vertices[top->vertex_ids[1]],
        layer.next_front.vertices[top->vertex_ids[2]]};
    const auto quality = evaluatePrism(points);
    if (!quality.hasValue() || !quality.value().acceptable ||
        quality.value().validity != VolumeCellValidity::Valid)
    {
        return 9;
    }

    return 0;
}
