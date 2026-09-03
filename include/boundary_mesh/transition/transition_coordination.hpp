#pragma once

#include <cstddef>
#include <optional>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/regular_layer_growth.hpp>
#include <boundary_mesh/transition/reserved_layer_growth.hpp>

namespace boundary_mesh
{
    struct CoordinatedTransitionFace
    {
        FaceLayerState layers;
        std::optional<std::size_t> high_edge_local_index;
        std::optional<std::size_t> second_high_edge_local_index;
        std::optional<VertexId> third_continuing_vertex_id;
        std::optional<std::size_t> continuing_edge_local_index;
    };

    struct MissingTransitionFaceState
    {
        SurfaceFaceId source_face_id{};
    };

    struct UncoordinatedTransitionLayerDifference
    {
        SurfaceFaceId first{};
        SurfaceFaceId second{};
    };

    struct MultipleTransitionHighEdges
    {
        SurfaceFaceId source_face_id{};
        std::vector<std::size_t> high_edge_local_indices;
    };

    struct TransitionCornerLayerViolation
    {
        SurfaceFaceId low_source_face_id{};
        std::size_t high_edge_local_index{};
        VertexId non_contact_vertex_id{};
        SurfaceFaceId violating_source_face_id{};
    };

    using TransitionCoordinationError = std::variant<
        MissingTransitionFaceState,
        UncoordinatedTransitionLayerDifference,
        MultipleTransitionHighEdges,
        TransitionCornerLayerViolation>;

    Result<std::vector<CoordinatedTransitionFace>,
           TransitionCoordinationError>
    coordinateTransitionFront(
        const GrowthFront &front,
        const RegularLayerGrowthResult &trial);
}
