#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/growth/growth_patch_builder.hpp>

namespace boundary_mesh
{
    Result<GrowthPatch, GrowthPatchError>
    GrowthPatchBuilder::build(
        const SurfaceMesh &mesh,
        const SurfaceTopology &topology) const
    {
        using BuildResult = Result<GrowthPatch, GrowthPatchError>;

        const std::size_t topology_vertex_count =
            topology.vertexFaces().size();
        const std::size_t topology_face_count =
            topology.faceEdges().size();

        if (mesh.vertices.size() != topology_vertex_count ||
            mesh.faces.size() != topology_face_count)
        {
            return BuildResult::failure(MeshTopologyMismatch{
                mesh.vertices.size(),
                topology_vertex_count,
                mesh.faces.size(),
                topology_face_count});
        }

        std::vector<bool> selected_vertices(mesh.vertices.size(), false);
        std::vector<SurfaceFaceId> source_face_ids;

        // 面 ID 升序扫描保证公开源面顺序确定。
        for (std::size_t face_index = 0;
             face_index < mesh.faces.size();
             ++face_index)
        {
            if (mesh.face_tags[face_index].kind != SurfaceBoundaryKind::Wall)
            {
                continue;
            }

            source_face_ids.push_back(
                static_cast<SurfaceFaceId>(face_index));
            std::visit(
                [&](const auto &face)
                {
                    for (const VertexId vertex_id : face.vertex_ids)
                    {
                        selected_vertices[static_cast<std::size_t>(vertex_id)] = true;
                    }
                },
                mesh.faces[face_index]);
        }

        if (source_face_ids.empty())
        {
            return BuildResult::failure(EmptyGrowthPatch{});
        }

        std::vector<PatchVertex> vertices;
        for (std::size_t vertex_index = 0;
             vertex_index < selected_vertices.size();
             ++vertex_index)
        {
            if (!selected_vertices[vertex_index])
            {
                continue;
            }

            std::vector<std::uint32_t> symmetry_region_ids;
            for (const SurfaceFaceId face_id : topology.vertexFaces()[vertex_index])
            {
                const SurfaceBoundaryTag &tag =
                    mesh.face_tags[static_cast<std::size_t>(face_id)];
                if (tag.kind == SurfaceBoundaryKind::Symmetry)
                {
                    symmetry_region_ids.push_back(tag.region_id);
                }
            }

            std::sort(symmetry_region_ids.begin(), symmetry_region_ids.end());
            symmetry_region_ids.erase(
                std::unique(symmetry_region_ids.begin(), symmetry_region_ids.end()),
                symmetry_region_ids.end());

            vertices.push_back(PatchVertex{
                static_cast<VertexId>(vertex_index),
                std::move(symmetry_region_ids)});
        }

        return BuildResult::success(GrowthPatch{
            std::move(vertices),
            std::move(source_face_ids)});
    }
}
