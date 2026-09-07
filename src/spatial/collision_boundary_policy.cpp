#include <boundary_mesh/spatial/collision_boundary_policy.hpp>

namespace boundary_mesh
{
    bool CollisionBoundaryPolicy::isObstacle(
        SurfaceBoundaryKind kind,
        CollisionSurfaceOrigin) const noexcept
    {
        return !isSlidingBoundary(kind);
    }
}
