#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    struct BatchSelfCollisionDiagnostics
    {
        std::uint64_t triangle_count{};
        std::uint64_t sweep_pairs{};
        std::uint64_t unique_pairs{};
        std::uint64_t same_owner_skips{};
        std::uint64_t aabb_rejections{};
        std::uint64_t topology_rejections{};
        std::uint64_t exact_tests{};
        std::uint64_t illegal_owner_pairs{};
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
    };
}
