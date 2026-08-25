#pragma once

#include <variant>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_patch.hpp>
#include <boundary_mesh/growth/growth_profile.hpp>
#include <boundary_mesh/growth/multi_normal_mesh_merge.hpp>
#include <boundary_mesh/growth/multi_normal_transition_generator.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology.hpp>

namespace boundary_mesh
{
    struct MultiNormalGenerationFailure
    {
        MultiNormalError cause;
    };

    struct RegularLayerGenerationFailure
    {
        RegularLayerGrowthError cause;
    };

    struct BoundaryLayerMergeFailure
    {
        MultiNormalMergeError cause;
    };

    using BoundaryLayerGenerationError = std::variant<
        MultiNormalGenerationFailure,
        RegularLayerGenerationFailure,
        BoundaryLayerMergeFailure>;

    struct BoundaryLayerGenerationResult
    {
        VolumeMesh mesh;
        MultiNormalTransitionResult transition;
        RegularLayerGrowthResult regular;
        SurfaceMesh top_surface;
        SurfaceMesh farfield_boundary;
    };

    Result<BoundaryLayerGenerationResult, BoundaryLayerGenerationError>
    generateBoundaryLayers(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const MultiNormalOptions &multi_normal_options,
        const RegularLayerGrowthOptions &regular_options = {});
}
