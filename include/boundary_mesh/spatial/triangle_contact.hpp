#pragma once

#include <array>
#include <cstdint>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    struct CollisionTriangle;

    using TrianglePoints = std::array<Point3, 3>; // 三角形的三个空间坐标

    struct CollisionVertexKey
    {
        VertexId source_vertex_id{}; // 对应的输入表面顶点编号
        std::uint32_t layer{};        // 该顶点所属的边界层层号
        std::uint32_t branch_id{};    // 同一源点的多法向分支编号
    };

    enum class TriangleContactKind
    {
        Disjoint,        // 两个三角形完全分离
        VertexTouch,     // 仅在一个点发生零距离接触
        EdgeTouch,       // 沿边或边的一部分发生接触
        CoplanarOverlap, // 共面区域存在正面积重叠
        ProperIntersect  // 非共面穿透或交叉
    };

    Result<TriangleContactKind, SpatialError>
    classifyTriangleContact(
        const TrianglePoints &first,
        const TrianglePoints &second);

    Result<bool, SpatialError> hasIllegalTriangleContact(
        const TrianglePoints &first,
        const std::array<CollisionVertexKey, 3> &first_keys,
        const TrianglePoints &second,
        const std::array<CollisionVertexKey, 3> &second_keys);

    Result<bool, SpatialError> hasIllegalTriangleContact(
        const CollisionTriangle &first,
        const CollisionTriangle &second);
}
