#include <limits>
#include <vector>

#include <boundary_mesh/transition/reserved_layer_growth.hpp>

namespace boundary_mesh
{
    std::uint32_t regularLayerCount(
        std::uint32_t trial_layers) noexcept
    {
        return trial_layers > ReservedTransitionLayerCount
                   ? trial_layers - ReservedTransitionLayerCount
                   : 0;
    }

    std::uint32_t occupiedLayerCount(
        std::uint32_t trial_layers) noexcept
    {
        return trial_layers > 0 ? trial_layers - 1 : 0;
    }

    Result<std::vector<SourceVertexGrowthProfile>, ReservedLayerCountOverflow>
    makeReservedTrialProfiles(
        const std::vector<SourceVertexGrowthProfile> &requested)
    {
        using ProfileResult = Result<
            std::vector<SourceVertexGrowthProfile>,
            ReservedLayerCountOverflow>;

        std::vector<SourceVertexGrowthProfile> trial = requested;
        for (SourceVertexGrowthProfile &entry : trial)
        {
            constexpr std::uint32_t maximum =
                std::numeric_limits<std::uint32_t>::max();
            if (entry.profile.layer_count >
                maximum - ReservedTransitionLayerCount)
            {
                return ProfileResult::failure(
                    ReservedLayerCountOverflow{
                        entry.source_vertex_id,
                        entry.profile.layer_count});
            }
            entry.profile.layer_count += ReservedTransitionLayerCount;
        }
        return ProfileResult::success(std::move(trial));
    }
}
