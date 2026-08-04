#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    struct InvalidSurfaceEvaluationOptions
    {
        Scalar relative_length_tolerance{}; // 非有限或非正的相对容差
    };
    struct EmptyGrowthFront
    {
        std::uint32_t layer{}; // 空前沿的层号
    };
    struct FrontMappingMismatch
    {
        std::size_t vertex_count{};          // 当前前沿顶点数量
        std::size_t source_vertex_count{};   // 源顶点映射数量
        std::size_t vertex_boundary_count{}; // 顶点边界归属数量
        std::size_t face_count{};            // 当前前沿面数量
        std::size_t source_face_count{};     // 源面映射数量
        std::uint32_t layer{};               // 发生映射错配的层号
    };
    struct NonFiniteFrontVertex
    {
        std::size_t front_vertex_index{}; // 含非有限坐标的前沿顶点下标
        VertexId source_vertex_id{};      // 对应的输入表面顶点编号
        std::uint32_t layer{};            // 发生错误的前沿层号
    };
    struct InvalidFrontVertexReference
    {
        std::size_t front_face_index{}; // 引用越界顶点的前沿面下标
        SurfaceFaceId source_face_id{}; // 对应的输入 Wall 面编号
        VertexId front_vertex_id{};     // 越界的前沿局部顶点编号
        std::uint32_t layer{};          // 发生错误的前沿层号
    };
    struct DegenerateFrontScale
    {
        std::uint32_t layer{}; // 包围盒尺度退化的前沿层号
    };
    struct DegenerateFrontFace
    {
        std::size_t front_face_index{}; // 几何退化的前沿面下标
        SurfaceFaceId source_face_id{}; // 对应的输入 Wall 面编号
        std::uint32_t layer{};          // 发生错误的前沿层号
        FaceEvaluationError cause{FaceEvaluationError::DegenerateAreaVector}; // 局部几何原因
    };

    using FrontEvaluationError = std::variant<
        InvalidSurfaceEvaluationOptions,
        EmptyGrowthFront,
        FrontMappingMismatch,
        NonFiniteFrontVertex,
        InvalidFrontVertexReference,
        DegenerateFrontScale,
        DegenerateFrontFace>;
}
