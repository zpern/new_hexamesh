#pragma once

#include <variant>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/multi_normal/multi_normal_mesh_merge.hpp>
#include <boundary_mesh/multi_normal/multi_normal_transition_generator.hpp>
#include <boundary_mesh/transition/reserved_layer_growth.hpp>
#include <boundary_mesh/transition/transition_coordination.hpp>
#include <boundary_mesh/transition/transition_templates.hpp>

namespace boundary_mesh
{
    std::vector<bool> nonDuplicatedTopTriangles(
        const std::vector<Triangle> &candidates);

    struct ReservedLayerTransitionResult
    {
        VolumeMesh mesh;
        SurfaceMesh boundary_layer_top;
        SurfaceMesh farfield_boundary;
        std::vector<CoordinatedTransitionFace> coordinated_faces;
        RegularLayerGrowthResult trial_growth;
        MultiNormalTransitionResult multi_normal_transition;
        std::size_t reserved_transition_cell_count{};
        std::size_t regular_cell_count{};
    };

    struct ReservedMultiNormalFailure { MultiNormalError cause; };
    struct ReservedMultiNormalMergeFailure { MultiNormalMergeError cause; };

    using CombinedReservedLayerTransitionError = std::variant<
        ReservedLayerCountOverflow,
        RegularLayerGrowthError,
        TransitionCoordinationError,
        TransitionTemplateError,
        VolumeVertexIdOverflow,
        ReservedMultiNormalFailure,
        ReservedMultiNormalMergeFailure>;

    [[deprecated("use generateBoundaryLayers incremental transition path")]]
    Result<ReservedLayerTransitionResult,
           CombinedReservedLayerTransitionError>
    generateReservedLayerTransition(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const MultiNormalOptions &multi_normal_options,
        const RegularLayerGrowthOptions &options = {});
}
