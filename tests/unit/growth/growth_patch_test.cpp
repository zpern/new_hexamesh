#include <array>
#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    /// 创建封闭三棱柱：底面 Wall、顶面 Farfield，三个侧面属于不同对称区域。
    SurfaceMesh makePrismMesh()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0}, Point3{0.0, 1.0, 1.0}};
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{3}, VertexId{4}, VertexId{5}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{4}, VertexId{3}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{2}, VertexId{0}, VertexId{3}, VertexId{5}}}};
        mesh.face_tags = {
            SurfaceBoundaryTag{SurfaceBoundaryKind::Wall, 10},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Farfield, 20},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 30},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 31},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 32}};
        return mesh;
    }
}

int main()
{
    using namespace boundary_mesh;
    const SurfaceMesh mesh = makePrismMesh();
    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue())
    {
        return 1;
    }

    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue())
    {
        return 2;
    }
    if (patch.value().sourceFaceIds() !=
        std::vector<SurfaceFaceId>{SurfaceFaceId{0}})
    {
        return 3;
    }

    const auto &vertices = patch.value().vertices();
    const std::array<std::vector<std::uint32_t>, 3> expected_regions{{
        {30, 32}, {30, 31}, {31, 32}}};
    if (vertices.size() != expected_regions.size())
    {
        return 4;
    }
    for (std::size_t index = 0; index < vertices.size(); ++index)
    {
        if (vertices[index].source_vertex_id != static_cast<VertexId>(index) ||
            vertices[index].sliding_region_ids != expected_regions[index])
        {
            return 5;
        }
    }

    // 全部面都是 Wall 时，每个顶点都不携带对称约束。
    SurfaceMesh all_wall = mesh;
    for (SurfaceBoundaryTag &tag : all_wall.face_tags)
    {
        tag.kind = SurfaceBoundaryKind::Wall;
    }
    const auto all_wall_patch = GrowthPatchBuilder{}.build(
        all_wall, topology.value());
    if (!all_wall_patch.hasValue() ||
        all_wall_patch.value().vertices().size() != all_wall.vertices.size())
    {
        return 6;
    }
    for (const PatchVertex &vertex : all_wall_patch.value().vertices())
    {
        if (!vertex.sliding_region_ids.empty())
        {
            return 7;
        }
    }

    SurfaceMesh no_wall = mesh;
    for (SurfaceBoundaryTag &tag : no_wall.face_tags)
    {
        tag.kind = SurfaceBoundaryKind::Farfield;
    }
    const auto empty = GrowthPatchBuilder{}.build(no_wall, topology.value());
    const auto *empty_error = empty.hasValue()
        ? nullptr : std::get_if<EmptyGrowthPatch>(&empty.error());
    if (empty_error == nullptr)
    {
        return 8;
    }

    SurfaceMesh changed = mesh;
    changed.vertices.push_back(Point3{2.0, 2.0, 2.0});
    const auto mismatch = GrowthPatchBuilder{}.build(changed, topology.value());
    const auto *mismatch_error = mismatch.hasValue()
        ? nullptr : std::get_if<MeshTopologyMismatch>(&mismatch.error());
    if (mismatch_error == nullptr ||
        mismatch_error->mesh_vertex_count != 7 ||
        mismatch_error->topology_vertex_count != 6)
    {
        return 9;
    }

    // Internal 面只提供滑移 region，不进入 Wall patch 的源面集合。
    SurfaceMesh with_internal = mesh;
    with_internal.vertices.push_back(Point3{-1.0, 0.0, 0.0});
    with_internal.faces.push_back(
        Triangle{{VertexId{0}, VertexId{2}, VertexId{6}}});
    with_internal.face_tags.push_back(
        {SurfaceBoundaryKind::Internal, 40});

    const auto internal_topology =
        SurfaceTopologyBuilder{}.build(with_internal);
    if (!internal_topology.hasValue())
    {
        return 10;
    }
    const auto internal_patch = GrowthPatchBuilder{}.build(
        with_internal, internal_topology.value());
    if (!internal_patch.hasValue() ||
        internal_patch.value().sourceFaceIds() !=
            std::vector<SurfaceFaceId>{SurfaceFaceId{0}})
    {
        return 11;
    }

    const auto &internal_vertices = internal_patch.value().vertices();
    if (internal_vertices.size() != 3 ||
        internal_vertices[0].sliding_region_ids !=
            std::vector<std::uint32_t>{30, 32, 40} ||
        internal_vertices[1].sliding_region_ids !=
            std::vector<std::uint32_t>{30, 31} ||
        internal_vertices[2].sliding_region_ids !=
            std::vector<std::uint32_t>{31, 32, 40})
    {
        return 12;
    }
    return 0;
}
