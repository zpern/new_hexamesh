#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/spatial/aabb.hpp>
#include <boundary_mesh/spatial/batch_self_collision_detector.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    using LayerBoundaryOwner = CollisionOwnerTriangles;

    struct LayerBoundaryBatchDiagnostics
    {
        std::size_t owner_count{};
        std::size_t triangle_count{};
        std::size_t omitted_shared_sides{};
    };

    class LayerBoundaryBatch
    {
    public:
        static Result<LayerBoundaryBatch, SpatialError> build(
            const std::vector<LayerBoundaryCandidate> &candidates);

        const std::vector<LayerBoundaryCandidate> &candidates() const noexcept;
        const std::vector<LayerBoundaryOwner> &owners() const noexcept;
        const LayerBoundaryBatchDiagnostics &diagnostics() const noexcept;

    private:
        std::vector<LayerBoundaryCandidate> candidates_;
        std::vector<LayerBoundaryOwner> owners_;
        LayerBoundaryBatchDiagnostics diagnostics_;
    };
}
