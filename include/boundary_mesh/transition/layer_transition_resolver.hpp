#pragma once

#include <functional>
#include <optional>
#include <variant>
#include <vector>

#include <boundary_mesh/transition/corner_suppression.hpp>
#include <boundary_mesh/transition/transition_boundary_checker.hpp>

namespace boundary_mesh
{
    enum class TransitionTemplateKind
    {
        TriangleSide,
        QuadSingleHighSide,
        QuadAdjacentHighSide,
        QuadTopCap
    };

    struct ResolvedTransitionTopology
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        TransitionTemplateKind template_kind{};
        std::optional<QuadDiagonal> low_diagonal;
        std::vector<SurfaceFaceId> dependent_high_faces;
    };

    struct ProvisionalLayerTransition
    {
        TransitionBoundaryInput boundary;
        std::vector<ResolvedTransitionTopology> resolved_topology;
        bool all_top_faces_are_triangles{};
    };

    using ProvisionalLayerTransitionResult = Result<
        ProvisionalLayerTransition, TransitionBoundaryError>;

    struct LayerTransitionInput
    {
        GrowthFront current_front;
        GrowthFront candidate_front;
        LayerFaceSets face_sets;
        std::uint32_t completed_layer{};
        Scalar length_tolerance{1e-12};
        std::optional<CollisionIndex> original_surface;
        const ExposedBoundaryTracker *historical_boundary{};
        std::function<ProvisionalLayerTransitionResult(
            const std::vector<SurfaceFaceId> &,
            const LayerFaceSets &)> build_provisional;
    };

    struct StableLayerTransition
    {
        LayerFaceSets face_sets;
        std::vector<SurfaceFaceId> retained_high_faces;
        std::vector<OwnedBoundaryTriangle> exposed_boundary;
        std::vector<ResolvedTransitionTopology> resolved_topology;
        std::uint32_t iterations{};
        bool all_top_faces_are_triangles{};
    };

    using LayerTransitionError = std::variant<
        TransitionCoordinationError,
        TransitionBoundaryError>;

    class LayerTransitionResolver
    {
    public:
        Result<StableLayerTransition, LayerTransitionError>
        resolve(const LayerTransitionInput &input) const;
    };
}
