#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/spatial/aabb.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    struct CollisionOwnerTriangles
    {
        std::uint32_t owner_id{};
        Aabb bounds;
        std::vector<CollisionTriangle> triangles;
    };

    struct BatchSelfCollisionDiagnostics
    {
        std::uint64_t triangle_count{};
        std::uint64_t broad_phase_visits{};
        std::uint64_t unique_pairs{};
        std::uint64_t same_owner_skips{};
        std::uint64_t aabb_rejections{};
        std::uint64_t topology_rejections{};
        std::uint64_t exact_tests{};
        std::uint64_t illegal_owner_pairs{};
        std::uint64_t owner_pairs{};
    };

    struct BatchSelfCollisionResult
    {
        std::vector<std::uint32_t> illegal_owner_ids;
        BatchSelfCollisionDiagnostics diagnostics;
    };

    class BatchSelfCollisionDetector
    {
    public:
        static Result<BatchSelfCollisionResult, SpatialError> detect(
            const std::vector<CollisionTriangle> &triangles);

        static Result<BatchSelfCollisionResult, SpatialError> detectOwners(
            const std::vector<CollisionOwnerTriangles> &owners);

    };

    class BatchSelfCollisionIndex
    {
    public:
        static Result<BatchSelfCollisionIndex, SpatialError> build(
            const std::vector<CollisionOwnerTriangles> &owners,
            const std::vector<Aabb> &conservative_owner_bounds);

        Result<BatchSelfCollisionResult, SpatialError> detectChanged(
            const std::vector<CollisionOwnerTriangles> &active_owners,
            const std::vector<std::size_t> &changed_owner_indices) const;

    private:
        BinaryAabbTree tree_;
        std::vector<std::uint32_t> owner_ids_;
    };
}
