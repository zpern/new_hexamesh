#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_direction_error.hpp>
#include <boundary_mesh/growth/sliding_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    class SlidingSurfaceBuilder
    {
    public:
        Result<SlidingSurfaceSet, GrowthDirectionError> build(
            const SurfaceMesh &mesh) const;
    };
}
