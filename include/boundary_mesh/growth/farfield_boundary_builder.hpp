#pragma once

#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    Result<SurfaceMesh, SpatialError> buildFarfieldBoundary(
        const SurfaceMesh &original_surface,
        const GrowthFront &zero_layer_front,
        const ExposedBoundaryTracker &exposed_boundary,
        const std::vector<SurfaceFaceId> &zero_layer_source_face_ids,
        bool include_boundary_layer_interface = true);

    Result<SurfaceMesh, SpatialError> buildFarfieldBoundary(
        const SurfaceMesh &original_surface,
        const ExposedBoundaryTracker &exposed_boundary,
        const std::vector<SurfaceFaceId> &zero_layer_source_face_ids,
        bool include_boundary_layer_interface = true);

    Result<SurfaceMesh, SpatialError> extractBoundaryLayerTop(
        const SurfaceMesh &farfield_boundary);
}
