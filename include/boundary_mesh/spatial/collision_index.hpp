#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology.hpp>
#include <boundary_mesh/spatial/binary_aabb_tree.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>
#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
    enum class CollisionOwnerKind
    {
        OriginalSurface, // 输入 Wall 或 Farfield 面
        ExposedBoundary, // 已提交边界层的外露面
        LayerCandidate   // 当前目标层的候选单元边界面
    };

    struct CollisionTriangle
    {
        TrianglePoints points{}; // 三角形空间坐标
        std::array<CollisionVertexKey, 3> vertex_keys{}; // 分层拓扑顶点键
        CollisionOwnerKind owner_kind{CollisionOwnerKind::OriginalSurface}; // 图元归属类别
        std::uint32_t owner_id{}; // 所属源面、外露面或候选单元编号
    };

    class CollisionIndex
    {
    public:
        static Result<CollisionIndex, SpatialError> build(
            std::vector<CollisionTriangle> triangles);

        std::vector<std::size_t> queryIllegalContacts(
            const CollisionTriangle &query) const;

        std::size_t primitiveCount() const noexcept;

        const CollisionTriangle &primitive(
            std::size_t primitive_index) const;

    private:
        std::vector<CollisionTriangle> triangles_; // 按稳定图元编号保存的三角形
        BinaryAabbTree tree_; // 对 triangles_ 的只读空间粗筛索引
    };

    Result<CollisionIndex, SpatialError>
    buildOriginalSurfaceCollisionIndex(
        const SurfaceMesh &mesh,
        const SurfaceTopology &topology);
}
