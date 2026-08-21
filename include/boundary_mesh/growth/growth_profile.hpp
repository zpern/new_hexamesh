#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct VertexGrowthProfile
    {
        Scalar first_height{};       // 第一层生长步长
        Scalar growth_ratio{1};      // 相邻层生长步长的倍率
        std::uint32_t layer_count{}; // 外部请求的最大生长层数
    };

    struct SourceVertexGrowthProfile
    {
        VertexId source_vertex_id{}; // 输入 SurfaceMesh 的源顶点编号
        VertexGrowthProfile profile; // 该源顶点继承的全部生长参数
    };

    struct NonFiniteLayerHeight
    {
        VertexId source_vertex_id{}; // 无法得到有限步长的源顶点编号
        std::uint32_t layer{};       // 无法得到有限步长的目标层号
    };

    class GrowthProfileBuilder;

    /// 按源顶点编号提供不可变生长参数查询。
    class GrowthProfileTable
    {
    public:
        const VertexGrowthProfile *find(
            VertexId source_vertex_id) const noexcept;

        Result<Scalar, NonFiniteLayerHeight>
        height(
            VertexId source_vertex_id,
            std::uint32_t layer) const;

        const std::vector<SourceVertexGrowthProfile> &entries() const noexcept
        {
            return entries_;
        }

    private:
        friend class GrowthProfileBuilder;

        explicit GrowthProfileTable(
            std::vector<SourceVertexGrowthProfile> entries)
            : entries_(std::move(entries))
        {
        }

        std::vector<SourceVertexGrowthProfile> entries_; // 按源顶点编号升序保存
    };
}
