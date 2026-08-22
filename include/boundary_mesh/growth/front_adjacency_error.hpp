#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct FrontAdjacencyInputMismatch
    {
        std::uint32_t layer{}; // 当前活动层号
        std::size_t face_count{}; // 当前活动面数量
        std::size_t source_face_count{}; // 源 Wall 面映射数量
    };

    struct InvalidFrontAdjacencyReference
    {
        std::uint32_t layer{}; // 当前活动层号
        std::size_t front_face_index{}; // 非法活动面下标
        SurfaceFaceId source_face_id{}; // 对应输入 Wall 面编号
        VertexId vertex_id{}; // 越界的紧凑顶点编号
    };

    using FrontAdjacencyError = std::variant<
        FrontAdjacencyInputMismatch,
        InvalidFrontAdjacencyReference>; // 活动前沿邻接构建错误
}
