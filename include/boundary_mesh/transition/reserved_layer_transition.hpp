#pragma once

#include <variant>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/transition/reserved_layer_growth.hpp>
#include <boundary_mesh/transition/transition_layer_coordinator.hpp>
#include <boundary_mesh/transition/transition_templates.hpp>

namespace boundary_mesh
{
    std::vector<bool> externallyExposedTopTriangles(
        const std::vector<Triangle> &candidates,
        const std::vector<VolumeCell> &cells);

    struct ReservedLayerTransitionResult
    {
        VolumeMesh mesh;
        SurfaceMesh boundary_layer_top;
        std::vector<CoordinatedTransitionFace> coordinated_faces;
        RegularLayerGrowthResult trial_growth;
    };

    using ReservedLayerTransitionError = std::variant<
        ReservedLayerCountOverflow,
        RegularLayerGrowthError,
        TransitionCoordinationError,
        TransitionTemplateError,
        VolumeVertexIdOverflow>;

    Result<ReservedLayerTransitionResult, ReservedLayerTransitionError>
    generateReservedLayerTransition(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options = {});
}
