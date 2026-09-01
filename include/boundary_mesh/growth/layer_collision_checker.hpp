#pragma once

#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/regular_layer_growth.hpp>
#include <boundary_mesh/growth/sliding_surface.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    Result<std::vector<LayerBoundaryCandidate>, SpatialError>
    buildLayerBoundaryCandidates(
        const GrowthFront &current_front,
        const LayerStepResult &step);

    Result<std::vector<LayerBoundaryCandidate>, SpatialError>
    buildLayerBoundaryCandidates(
        const GrowthFront &current_front,
        const LayerStepResult &step,
        const SlidingSurfaceSet &sliding_surfaces);

    class LayerCollisionChecker
    {
    public:
        Result<LayerStepResult, SpatialError>
        filterAgainstObstacles(
            const CollisionIndex &original_surface,
            const ExposedBoundaryTracker &exposed_boundary,
            const GrowthFront &current_front,
            const LayerStepResult &quality_step) const;

        Result<LayerStepResult, SpatialError>
        filterSelfCollisions(
            const GrowthFront &current_front,
            const LayerStepResult &obstacle_step) const;
    };
}
