#include <cassert>
#include <type_traits>
#include <variant>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

using namespace boundary_mesh;

namespace
{
    void appendCube(
        SurfaceMesh &mesh,
        const Point3 &minimum,
        const Point3 &maximum,
        SurfaceBoundaryKind top_kind,
        SurfaceBoundaryKind other_kind)
    {
        const VertexId base = static_cast<VertexId>(mesh.vertices.size());
        mesh.vertices.insert(mesh.vertices.end(), {
            {minimum.x(), minimum.y(), minimum.z()},
            {maximum.x(), minimum.y(), minimum.z()},
            {maximum.x(), maximum.y(), minimum.z()},
            {minimum.x(), maximum.y(), minimum.z()},
            {minimum.x(), minimum.y(), maximum.z()},
            {maximum.x(), minimum.y(), maximum.z()},
            {maximum.x(), maximum.y(), maximum.z()},
            {minimum.x(), maximum.y(), maximum.z()}});
        const std::array<Quad, 6> faces{{
            {{base + 0, base + 3, base + 2, base + 1}},
            {{base + 4, base + 5, base + 6, base + 7}},
            {{base + 0, base + 1, base + 5, base + 4}},
            {{base + 1, base + 2, base + 6, base + 5}},
            {{base + 2, base + 3, base + 7, base + 6}},
            {{base + 3, base + 0, base + 4, base + 7}}}};
        for (std::size_t index = 0; index < faces.size(); ++index)
        {
            mesh.faces.push_back(faces[index]);
            const bool sliding =
                other_kind == SurfaceBoundaryKind::Symmetry ||
                other_kind == SurfaceBoundaryKind::Internal;
            mesh.face_tags.push_back({
                index == 1 ? top_kind : other_kind,
                index == 1 ? 10u :
                    (sliding ? 20u + static_cast<std::uint32_t>(index) : 20u)});
        }
    }
}

int main()
{
    SurfaceMesh mesh;
    appendCube(
        mesh,
        {0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0},
        SurfaceBoundaryKind::Wall,
        SurfaceBoundaryKind::Symmetry);
    appendCube(
        mesh,
        {-1.0, -1.0, -1.0},
        {2.0, 2.0, 2.0},
        SurfaceBoundaryKind::Farfield,
        SurfaceBoundaryKind::Farfield);
    const VertexId internal_base =
        static_cast<VertexId>(mesh.vertices.size());
    mesh.vertices.insert(mesh.vertices.end(), {
        {0.0, 0.0, 1.05}, {1.0, 0.0, 1.05},
        {1.0, 1.0, 1.05}, {0.0, 1.0, 1.05}});
    mesh.faces.push_back(Quad{{
        internal_base, internal_base + 1,
        internal_base + 2, internal_base + 3}});
    mesh.face_tags.push_back({SurfaceBoundaryKind::Internal, 90});

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    assert(patch.hasValue());
    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    assert(front.hasValue());

    std::vector<SourceVertexGrowthProfile> profiles;
    for (const PatchVertex &vertex : patch.value().vertices())
    {
        profiles.push_back({vertex.source_vertex_id, {0.1, 1.0, 1}});
    }

    const auto result = generateRegularLayers(
        mesh,
        topology.value(),
        patch.value(),
        front.value(),
        profiles);
    if (!result.hasValue() || !result.value().mesh.cells.empty()) return 1;
    assert(result.value().mesh.vertices.size() == front.value().vertices.size());
    assert(result.value().faces.size() == 1);
    assert(result.value().faces.front().status == FaceGrowthStatus::Stopped);
    assert(result.value().faces.front().stop_reason == FaceStopReason::Collision);
    assert(!result.value().farfield_boundary.faces.empty());
}
