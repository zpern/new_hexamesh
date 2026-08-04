#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    /// Wall Patch 中的一个源顶点及其全部对称区域归属。
    struct PatchVertex
    {
        VertexId source_vertex_id{}; // 原始 SurfaceMesh 中的顶点编号
        std::vector<std::uint32_t> symmetry_region_ids; // 排序去重后的对称区域编号
    };

    class GrowthPatchBuilder;

    /// 从完整表面提取出的只读 Wall 生长区域。
    class GrowthPatch
    {
    public:
        GrowthPatch(const GrowthPatch &) = default;
        GrowthPatch(GrowthPatch &&) noexcept = default;
        GrowthPatch &operator=(const GrowthPatch &) = default;
        GrowthPatch &operator=(GrowthPatch &&) noexcept = default;

        const std::vector<PatchVertex> &vertices() const noexcept
        {
            return vertices_;
        }

        const std::vector<SurfaceFaceId> &sourceFaceIds() const noexcept
        {
            return source_face_ids_;
        }

    private:
        friend class GrowthPatchBuilder;

        GrowthPatch(
            std::vector<PatchVertex> vertices,
            std::vector<SurfaceFaceId> source_face_ids)
            : vertices_(std::move(vertices)),
              source_face_ids_(std::move(source_face_ids))
        {
        }

        std::vector<PatchVertex> vertices_; // 按源 VertexId 升序排列
        std::vector<SurfaceFaceId> source_face_ids_; // 按源 SurfaceFaceId 升序排列
    };
}
