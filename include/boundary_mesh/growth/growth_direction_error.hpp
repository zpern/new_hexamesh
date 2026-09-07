#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/sliding/sliding_error.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    struct DirectionInputMismatch
    {
        std::uint32_t front_layer{};      // GrowthFront 声明的层号
        std::uint32_t evaluation_layer{}; // FrontEvaluation 声明的层号
    };
    struct DirectionCornerFailure
    {
        std::size_t front_vertex_index{}; // 角度计算失败的前沿顶点下标
        VertexId source_vertex_id{};      // 对应的输入表面顶点编号
        std::size_t front_face_index{};   // 角度计算失败的前沿面下标
        SurfaceFaceId source_face_id{};   // 对应的输入 Wall 面编号
        std::uint32_t layer{};            // 发生错误的前沿层号
        FaceEvaluationError cause{FaceEvaluationError::DegenerateEdge}; // 局部几何原因
    };
    struct UndefinedGrowthDirection
    {
        std::size_t front_vertex_index{}; // 无法确定方向的前沿局部顶点下标
        VertexId source_vertex_id{};      // 对应的输入表面顶点编号
        std::uint32_t layer{};            // 发生错误的前沿层号
    };
    using GrowthDirectionError = std::variant<
        DirectionInputMismatch,
        DirectionCornerFailure,
        UndefinedGrowthDirection,
        SlidingInputMismatch,
        InvalidSlidingSurface,
        OverConstrainedGrowthVertex,
        UndefinedConstrainedDirection,
        SlidingProjectionNotConverged>;
}
