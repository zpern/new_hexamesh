#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_profile.hpp>

namespace boundary_mesh
{
    inline constexpr std::uint32_t ReservedTransitionLayerCount{2};

    struct FaceLayerState
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t trial_layers{};
        std::uint32_t occupied_layers{};
        std::uint32_t regular_layers{};
    };

    struct ReservedLayerCountOverflow
    {
        VertexId source_vertex_id{};
        std::uint32_t requested_layers{};
    };

    std::uint32_t regularLayerCount(
        std::uint32_t trial_layers) noexcept;

    std::uint32_t occupiedLayerCount(
        std::uint32_t trial_layers) noexcept;

    Result<std::vector<SourceVertexGrowthProfile>, ReservedLayerCountOverflow>
    makeReservedTrialProfiles(
        const std::vector<SourceVertexGrowthProfile> &requested);
}
