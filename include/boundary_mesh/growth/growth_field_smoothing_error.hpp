#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/growth_direction_error.hpp>

namespace boundary_mesh
{
    struct GrowthFieldInputMismatch
    {
        std::uint32_t front_layer{}; // 当前活动前沿层号
        std::uint32_t direction_layer{}; // 原始方向所属层号
        std::size_t vertex_count{}; // 当前活动点数量
        std::size_t direction_count{}; // 原始方向数量
        std::size_t reference_height_count{}; // 理论基准步长数量
        std::size_t provisional_height_count{}; // 临时预测步长数量
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
        Scalar base_height{}; // 非有限或非正的理论或临时步长
        std::uint32_t layer{}; // 发生错误的活动层号
    };

    struct UndefinedSmoothedDirection
    {
        std::size_t front_vertex_index{}; // 无法归一化方向的活动点下标
        VertexId source_vertex_id{}; // 对应输入 Wall 顶点编号
        std::uint32_t layer{}; // 发生错误的活动层号
    };

    struct NonFiniteSmoothedHeight
    {
        std::size_t front_vertex_index{}; // 无法得到有限步长的活动点下标
        VertexId source_vertex_id{}; // 对应输入 Wall 顶点编号
        std::uint32_t layer{}; // 发生错误的活动层号
    };

    struct InvalidSkewnessNormalOptimizationOptions
    {
        Scalar activation_skewness{};
        Scalar first_angle_degrees{};
        Scalar second_angle_degrees{};
        std::size_t azimuth_samples{};
        std::size_t maximum_levels{};
        Scalar improvement_tolerance{};
    };

    struct SlidingGrowthFieldConstraintFailure
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::uint32_t layer{};
        GrowthDirectionError cause;
    };

    using GrowthFieldSmoothingError = std::variant<
        GrowthFieldInputMismatch,
        NonFiniteGrowthFieldInput,
        DegenerateGrowthFieldNeighbor,
        InvalidGrowthFieldBaseHeight,
        UndefinedSmoothedDirection,
        NonFiniteSmoothedHeight,
        InvalidSkewnessNormalOptimizationOptions,
        SlidingGrowthFieldConstraintFailure>; // 活动前沿字段平滑错误
}
