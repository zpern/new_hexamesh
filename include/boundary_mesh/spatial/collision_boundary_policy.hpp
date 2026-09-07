#pragma once

#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    enum class CollisionSurfaceOrigin
    {
        InputSurface,
        GeneratedBoundary
    };

    class CollisionBoundaryPolicy
    {
    public:
        bool isObstacle(
            SurfaceBoundaryKind kind,
            CollisionSurfaceOrigin origin) const noexcept;
    };
}
