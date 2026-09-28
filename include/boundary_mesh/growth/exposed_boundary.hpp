#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/spatial/incremental_collision_index.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>
#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
    struct BoundaryFaceKey
    {
        std::array<CollisionVertexKey, 4> vertices{}; // 按拓扑键升序保存的顶点
        std::uint8_t vertex_count{}; // 当前面包含 3 或 4 个顶点
    };

    struct BoundaryFaceKeyLess
    {
        bool operator()(
            const BoundaryFaceKey &left,
            const BoundaryFaceKey &right) const noexcept;
    };

    struct ExposedBoundaryApplyDiagnostics
    {
        std::size_t erased_groups{};
        std::size_t inserted_groups{};
        bool bulk_rebuild{};
        std::uint64_t preparation_nanoseconds{};
        std::uint64_t index_update_nanoseconds{};
    };

    struct BoundaryFace
    {
        std::vector<Point3> points; // 按边界层外侧绕序保存的面坐标
        std::vector<CollisionVertexKey> vertex_keys; // 与 points 一一对应的分层拓扑键
        SurfaceFaceId source_face_id{}; // 产生该边界面的输入 Wall 面
        std::uint32_t region_id{}; // 对应输入 Wall 的区域编号
        SurfaceBoundaryKind boundary_kind{
            SurfaceBoundaryKind::BoundaryLayerInterface};
        std::vector<std::vector<std::uint32_t>> vertex_sliding_region_ids;
    };

    struct LayerBoundaryCandidate
    {
        BoundaryFace bottom; // 候选体单元与当前前沿重合的底面
        BoundaryFace top;    // 候选体单元提交后的新顶面
        std::vector<SurfaceBoundaryTag> side_tags; // 按底面有向边保存侧面类别
    };

    struct ExposedBoundaryUpdate
    {
        std::vector<BoundaryFaceKey> erase_faces; // 提交后从外露集合删除的面
        std::vector<BoundaryFace> insert_faces; // 提交后加入外露集合的面
    };

    class ExposedBoundaryTracker
    {
    public:
        ExposedBoundaryTracker();

        Result<std::monostate, SpatialError> initializeWallSurface(
            const SurfaceMesh &surface);

        Result<ExposedBoundaryUpdate, SpatialError> prepare(
            const std::vector<LayerBoundaryCandidate> &candidates) const;

        Result<std::monostate, SpatialError> apply(
            const ExposedBoundaryUpdate &update);

        // Add already committed transition faces to the same historical
        // collision tree.  These groups are append-only; regular exposed
        // faces continue to use prepare/apply parity updates.
        Result<std::monostate, SpatialError> appendTransitionTriangles(
            std::vector<CollisionTriangle> triangles);

        const IncrementalCollisionIndex &collisionIndex() const noexcept;

        const ExposedBoundaryApplyDiagnostics &lastApplyDiagnostics() const noexcept;

        std::size_t faceCount() const noexcept;

        bool contains(const BoundaryFaceKey &key) const noexcept;

        const std::vector<BoundaryFace> &faces() const;

        Result<std::vector<CollisionTriangle>, SpatialError>
        collisionTriangles(
            const CollisionBoundaryPolicy &policy = {}) const;

    private:
        IncrementalCollisionIndex collision_index_;
        std::map<BoundaryFaceKey, CollisionGroupId, BoundaryFaceKeyLess>
            collision_groups_;
        std::map<CollisionGroupId, std::vector<CollisionTriangle>>
            transition_groups_;
        CollisionGroupId next_collision_group_id_{1};
        std::map<BoundaryFaceKey, BoundaryFace, BoundaryFaceKeyLess>
            faces_by_key_;
        std::set<BoundaryFaceKey, BoundaryFaceKeyLess> input_face_keys_;
        mutable std::vector<BoundaryFace> faces_cache_;
        mutable bool faces_cache_valid_{};
        ExposedBoundaryApplyDiagnostics last_apply_diagnostics_;
    };

    Result<BoundaryFaceKey, SpatialError> makeBoundaryFaceKey(
        const BoundaryFace &face);
}
