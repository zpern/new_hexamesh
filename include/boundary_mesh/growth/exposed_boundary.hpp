#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>
#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
    struct BoundaryFaceKey
    {
        std::array<CollisionVertexKey, 4> vertices{}; // 按拓扑键升序保存的顶点
        std::uint8_t vertex_count{}; // 当前面包含 3 或 4 个顶点
    };

    struct BoundaryFace
    {
        std::vector<Point3> points; // 按边界层外侧绕序保存的面坐标
        std::vector<CollisionVertexKey> vertex_keys; // 与 points 一一对应的分层拓扑键
        SurfaceFaceId source_face_id{}; // 产生该边界面的输入 Wall 面
        std::uint32_t region_id{}; // 对应输入 Wall 的区域编号
    };

    struct LayerBoundaryCandidate
    {
        BoundaryFace bottom; // 候选体单元与当前前沿重合的底面
        BoundaryFace top;    // 候选体单元提交后的新顶面
    };

    struct ExposedBoundaryUpdate
    {
        std::vector<BoundaryFaceKey> erase_faces; // 提交后从外露集合删除的面
        std::vector<BoundaryFace> insert_faces; // 提交后加入外露集合的面
    };

    class ExposedBoundaryTracker
    {
    public:
        Result<ExposedBoundaryUpdate, SpatialError> prepare(
            const std::vector<LayerBoundaryCandidate> &candidates) const;

        void apply(const ExposedBoundaryUpdate &update);

        std::size_t faceCount() const noexcept;

        bool contains(const BoundaryFaceKey &key) const noexcept;

        const std::vector<BoundaryFace> &faces() const noexcept;

        Result<std::vector<CollisionTriangle>, SpatialError>
        collisionTriangles() const;

    private:
        std::vector<BoundaryFace> faces_; // 按规范面键排序的当前外露面
    };

    Result<BoundaryFaceKey, SpatialError> makeBoundaryFaceKey(
        const BoundaryFace &face);
}
