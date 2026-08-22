#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    Result<SurfaceMesh, SpatialError> buildFarfieldBoundary(
        const SurfaceMesh &original_surface,
        const ExposedBoundaryTracker &exposed_boundary);
}
