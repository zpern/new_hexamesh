#include <boundary_mesh/growth/boundary_layer_generator.hpp>

#include <utility>

namespace boundary_mesh
{
    Result<BoundaryLayerGenerationResult, BoundaryLayerGenerationError>
    generateBoundaryLayers(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const MultiNormalOptions &multi_normal_options,
        const RegularLayerGrowthOptions &regular_options)
    {
        using GenerationResult = Result<
            BoundaryLayerGenerationResult,
            BoundaryLayerGenerationError>;

        auto transition = generateMultiNormalTransition(
            initial_front,
            multi_normal_options);
        if (!transition.hasValue())
        {
            return GenerationResult::failure(
                MultiNormalGenerationFailure{transition.error()});
        }

        auto regular = generateRegularLayers(
            surface_mesh,
            topology,
            patch,
            transition.value().transformed_front,
            profiles,
            regular_options);
        if (!regular.hasValue())
        {
            return GenerationResult::failure(
                RegularLayerGenerationFailure{regular.error()});
        }

        auto merged = mergeMultiNormalAndRegularMeshes(
            transition.value(),
            regular.value().mesh);
        if (!merged.hasValue())
        {
            return GenerationResult::failure(
                BoundaryLayerMergeFailure{merged.error()});
        }

        SurfaceMesh top_surface = regular.value().top_surface;
        SurfaceMesh farfield_boundary =
            regular.value().farfield_boundary;
        return GenerationResult::success(
            BoundaryLayerGenerationResult{
                std::move(merged.value()),
                std::move(transition.value()),
                std::move(regular.value()),
                std::move(top_surface),
                std::move(farfield_boundary)});
    }
}
