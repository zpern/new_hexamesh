#pragma once

#include <variant>

#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/transition/incremental_transition_templates.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>

namespace boundary_mesh
{
    using IncrementalLayerGrowthError = std::variant<
        RegularLayerGrowthError,
        TransitionTemplateError,
        LayerQuadDiagonalError,
        VolumeVertexIdOverflow>;

    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
    finalizeIncrementalLayerTopology(
        const SurfaceMesh &surface_mesh,
        const GrowthFront &initial_front,
        RegularLayerGrowthResult regular);

    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
    generateIncrementalBoundaryLayers(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options = {});
}
