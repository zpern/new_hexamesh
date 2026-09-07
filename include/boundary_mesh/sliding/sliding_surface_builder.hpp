#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/sliding/sliding_error.hpp>
#include <boundary_mesh/sliding/sliding_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    class SlidingSurfaceBuilder
    {
    public:
        Result<SlidingSurfaceSet, SlidingError> build(
            const SurfaceMesh &mesh) const;
    };
}
