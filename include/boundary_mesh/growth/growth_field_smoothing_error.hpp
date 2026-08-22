#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct GrowthFieldInputMismatch
    {
        std::uint32_t front_layer{}; // 当前活动前沿层号
        std::uint32_t direction_layer{}; // 原始方向所属层号
        std::size_t vertex_count{}; // 当前活动点数量
        std::size_t direction_count{}; // 原始方向数量
        std::size_t height_count{}; // 基准步长数量
    };

    struct NonFiniteGrowthFieldInput
    {
        std::size_t front_vertex_index{}; // 非有限数据对应的活动点下标
        VertexId source_vertex_id{}; // 对应输入 Wall 顶点编号
        std::uint32_t layer{}; // 发生错误的活动层号
    };

    struct DegenerateGrowthFieldNeighbor
    {
        std::size_t front_vertex_index{}; // 当前活动点下标
        std::size_t neighbor_vertex_index{}; // 重合或越界的一环邻点下标
        VertexId source_vertex_id{}; // 当前点对应输入 Wall 顶点编号
        std::uint32_t layer{}; // 发生错误的活动层号
    };

    struct InvalidGrowthFieldBaseHeight
    {
        std::size_t front_vertex_index{}; // 非法基准步长对应的活动点下标
        VertexId source_vertex_id{}; // 对应输入 Wall 顶点编号
        Scalar base_height{}; // 非有限或非正的本层基准步长
        std::uint32_t layer{}; // 发生错误的活动层号
    };

    struct UndefinedSmoothedDirection
    {
        std::size_t front_vertex_index{}; // 无法归一化方向的活动点下标
        VertexId source_vertex_id{}; // 对应输入 Wall 顶点编号
        std::uint32_t layer{}; // 发生错误的活动层号
    };

    using GrowthFieldSmoothingError = std::variant<
        GrowthFieldInputMismatch,
        NonFiniteGrowthFieldInput,
        DegenerateGrowthFieldNeighbor,
        InvalidGrowthFieldBaseHeight,
        UndefinedSmoothedDirection>; // 活动前沿字段平滑错误
}
