#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/spatial/aabb.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    class BinaryAabbTree
    {
    public:
        static Result<BinaryAabbTree, SpatialError> build(
            const std::vector<Aabb> &primitive_bounds);

        std::vector<std::size_t> query(
            const Aabb &bounds) const;

        std::size_t nearest(
            const Point3 &point,
            const std::function<Scalar(std::size_t)> &squared_distance,
            Scalar &best_squared_distance) const;

    private:
        struct Node
        {
            Aabb bounds{};        // 当前节点覆盖的空间范围
            std::size_t first{};  // 叶节点 primitive 区间起点
            std::size_t count{};  // 叶节点 primitive 数量
            std::size_t left{};   // 左子节点编号
            std::size_t right{};  // 右子节点编号
            bool leaf{};          // 当前节点是否为叶节点
        };

        std::size_t appendNode(
            std::size_t first,
            std::size_t last);

        std::vector<Aabb> primitive_bounds_;           // 输入图元包围盒
        std::vector<std::size_t> primitive_indices_;   // 叶节点中的稳定图元编号
        std::vector<Node> nodes_;                      // 紧凑深度优先节点数组
    };
}
