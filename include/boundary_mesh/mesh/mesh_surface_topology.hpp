#pragma once

#include <array>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    /// 负责验证 SurfaceMesh 并构造 SurfaceTopology。
    class SurfaceTopologyBuilder;

    /// 表示一条无向表面边。
    /// SurfaceTopologyBuilder 的成功结果保证端点按升序保存。
    struct Edge
    {
        std::array<VertexId, 2> vertex_ids{}; // 按升序保存的两个端点编号
    };

    using EdgeFaceIds =
        std::array<SurfaceFaceId, 2>; // 一条封闭流形边两侧的面片编号

    using TriangleEdgeIds =
        std::array<EdgeId, 3>; // 三角形按局部边顺序保存的三个边编号

    using QuadEdgeIds =
        std::array<EdgeId, 4>; // 四边形按局部边顺序保存的四个边编号

    using FaceEdgeIds =
        std::variant<
            TriangleEdgeIds,
            QuadEdgeIds>; // 与源面类型一致的局部边编号集合

    using TriangleNeighborIds =
        std::array<SurfaceFaceId, 3>; // 三角形逐条局部边对应的相邻面编号

    using QuadNeighborIds =
        std::array<SurfaceFaceId, 4>; // 四边形逐条局部边对应的相邻面编号

    using FaceNeighborIds =
        std::variant<
            TriangleNeighborIds,
            QuadNeighborIds>; // 与 FaceEdgeIds 局部顺序一致的相邻面编号集合

    /// SurfaceMesh 的只读拓扑快照。
    /// 网格发生变化后必须重新构建，不允许局部修改该对象。
    class SurfaceTopology
    {
    public:
        /// 返回全部规范化边；数组下标就是 EdgeId。
        const std::vector<Edge> &
        edges() const noexcept
        {
            return edges_;
        }

        /// 返回每条边两侧的面；数组下标就是 EdgeId。
        const std::vector<EdgeFaceIds> &
        edgeFaces() const noexcept
        {
            return edge_faces_;
        }

        /// 返回每个面的局部边；数组下标就是 SurfaceFaceId。
        const std::vector<FaceEdgeIds> &
        faceEdges() const noexcept
        {
            return face_edges_;
        }

        /// 返回每个面的逐边相邻面；局部顺序与 faceEdges() 相同。
        const std::vector<FaceNeighborIds> &
        faceNeighbors() const noexcept
        {
            return face_neighbors_;
        }

        /// 返回每个顶点关联的表面面片；数组下标就是 VertexId。
        const std::vector<
            std::vector<SurfaceFaceId>> &
        vertexFaces() const noexcept
        {
            return vertex_faces_;
        }

    private:
        // 只有构建器能创建满足全部拓扑不变量的快照。
        friend class SurfaceTopologyBuilder;

        SurfaceTopology(
            std::vector<Edge> edges,
            std::vector<EdgeFaceIds> edge_faces,
            std::vector<FaceEdgeIds> face_edges,
            std::vector<FaceNeighborIds> face_neighbors,
            std::vector<std::vector<SurfaceFaceId>> vertex_faces)
            : edges_(std::move(edges)),
              edge_faces_(std::move(edge_faces)),
              face_edges_(std::move(face_edges)),
              face_neighbors_(std::move(face_neighbors)),
              vertex_faces_(std::move(vertex_faces))
        {
        }

        std::vector<Edge> edges_;
        std::vector<EdgeFaceIds> edge_faces_;
        std::vector<FaceEdgeIds> face_edges_;
        std::vector<FaceNeighborIds> face_neighbors_;
        std::vector<std::vector<SurfaceFaceId>> vertex_faces_;
    };
}
