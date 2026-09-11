#pragma once

#include <boundary_mesh/boundary_layer/incremental_boundary_layer_generator.hpp>
#include <boundary_mesh/transition/layer_transition_resolver.hpp>

namespace boundary_mesh
{
    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
    finalizeIncrementalLayerTopology(
        const SurfaceMesh &surface_mesh,
        const GrowthFront &initial_front,
        RegularLayerGrowthResult regular,
        const std::vector<ResolvedTransitionTopology> &
            resolved_topology = {});
}
