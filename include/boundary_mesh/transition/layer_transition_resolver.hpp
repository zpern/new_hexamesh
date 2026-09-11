#pragma once

#include <functional>
#include <map>
#include <optional>
#include <variant>
#include <vector>

#include <boundary_mesh/transition/corner_suppression.hpp>
#include <boundary_mesh/transition/transition_template_types.hpp>
#include <boundary_mesh/transition/transition_boundary_checker.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation.hpp>

namespace boundary_mesh
{
    enum class TransitionTemplateKind
    {
        TriangleSide,
        QuadSingleHighSide,
        QuadAdjacentHighSide,
        QuadTopCap
    };

    enum class TerminalQuadDecision
    {
        InternalSplit,
        ExternalPatch,
        KeepHexa
    };

    TerminalQuadDecision chooseTerminalQuadDecision(
        Scalar aspect_ratio,
        const std::optional<Point3> &internal_center,
        bool external_available,
        Scalar aspect_ratio_threshold = Scalar{0.15});

    struct ResolvedTransitionTopology
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        TransitionTemplateKind template_kind{};
        std::optional<QuadDiagonal> low_diagonal;
        std::vector<SurfaceFaceId> dependent_high_faces;
        TerminalQuadDecision terminal_quad_decision =
            TerminalQuadDecision::InternalSplit;
        Scalar aspect_ratio{};
        std::optional<Point3> generated_point;
        std::vector<std::size_t> retained_local_edges;
    };

    struct ProvisionalLayerTransition
    {
        TransitionBoundaryInput boundary;
        std::vector<ResolvedTransitionTopology> resolved_topology;
        std::vector<SurfaceFaceId> forced_rollback_high_faces;
        bool all_top_faces_are_triangles{};
    };

    struct ExternalPatchControls
    {
        std::map<SurfaceFaceId, Scalar> distance_scales;
        std::vector<SurfaceFaceId> keep_hexa_faces;

        Scalar distanceScale(SurfaceFaceId id) const;
        bool keepHexa(SurfaceFaceId id) const;
    };

    using LayerTransitionError = std::variant<
        TransitionCoordinationError,
        TransitionBoundaryError,
        TransitionTemplateError>;

    using ProvisionalLayerTransitionResult = Result<
        ProvisionalLayerTransition, LayerTransitionError>;

    struct LayerTransitionInput
    {
        GrowthFront current_front;
        GrowthFront candidate_front;
        LayerFaceSets face_sets;
        std::uint32_t completed_layer{};
        Scalar length_tolerance{1e-12};
        std::optional<CollisionIndex> original_surface;
        const ExposedBoundaryTracker *historical_boundary{};
        const std::vector<OwnedBoundaryTriangle>
            *prior_transition_boundary{};
        const SlidingIntersectionIndex *sliding_surface{};
        std::function<std::optional<HexaPoints>(SurfaceFaceId)>
            terminal_hexa_points;
        std::function<ProvisionalLayerTransitionResult(
            const std::vector<SurfaceFaceId> &,
            const LayerFaceSets &,
            const ExternalPatchControls &)> build_provisional;
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

    class LayerTransitionResolver
    {
    public:
        Result<StableLayerTransition, LayerTransitionError>
        resolve(const LayerTransitionInput &input) const;
    };
}
