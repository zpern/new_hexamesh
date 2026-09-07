#pragma once

#include <boundary_mesh/boundary_layer/incremental_boundary_layer_generator.hpp>

namespace boundary_mesh
{
    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
    finalizeIncrementalLayerTopology(
        const SurfaceMesh &surface_mesh,
        const GrowthFront &initial_front,
        RegularLayerGrowthResult regular);
}
