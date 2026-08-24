#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_patch.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology.hpp>
#include <boundary_mesh/transition/reserved_layer_growth.hpp>

namespace boundary_mesh
{
    struct CoordinatedTransitionFace
    {
        FaceLayerState layers;
        std::optional<std::size_t> high_edge_local_index;
    };

    struct MissingTransitionFaceState
    {
        SurfaceFaceId source_face_id{};
    };

    struct DuplicateTransitionFaceState
    {
        SurfaceFaceId source_face_id{};
    };

    using TransitionCoordinationError = std::variant<
        MissingTransitionFaceState,
        DuplicateTransitionFaceState>;

    class TransitionLayerCoordinator
    {
    public:
        Result<std::vector<CoordinatedTransitionFace>,
               TransitionCoordinationError>
        coordinate(
            const GrowthPatch &patch,
            const SurfaceTopology &topology,
            const std::vector<
                std::pair<SurfaceFaceId, std::uint32_t>>
                &trial_layers) const;
    };
}
