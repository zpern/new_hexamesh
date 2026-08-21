#pragma once

#include <variant>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct MissingVertexGrowthProfile
    {
        VertexId source_vertex_id{}; // GrowthPatch 中缺少参数的源顶点
    };

    struct DuplicateVertexGrowthProfile
    {
        VertexId source_vertex_id{}; // 被重复输入参数的源顶点
    };

    struct UnknownVertexGrowthProfile
    {
        VertexId source_vertex_id{}; // 不属于当前 GrowthPatch 的输入顶点
    };

    struct InvalidFirstHeight
    {
        VertexId source_vertex_id{}; // 参数非法的源顶点
        Scalar value{};              // 非有限或非正的首层高度
    };

    struct InvalidGrowthRatio
    {
        VertexId source_vertex_id{}; // 参数非法的源顶点
        Scalar value{};              // 非有限或非正的增长率
    };

    using GrowthProfileError = std::variant<
        MissingVertexGrowthProfile,
        DuplicateVertexGrowthProfile,
        UnknownVertexGrowthProfile,
        InvalidFirstHeight,
        InvalidGrowthRatio>; // 外部逐顶点生长参数的全部输入错误
}
