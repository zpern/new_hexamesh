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
        // Provenance is needed to remove stale forced rollbacks on patch replacement.
        std::map<SurfaceFaceId, std::vector<SurfaceFaceId>> forced_rollback_by_source;
    };

    struct ExternalPatchControls
    {
        std::map<SurfaceFaceId, Scalar> distance_scales;
        std::map<SurfaceFaceId, std::size_t> apex_candidate_indices;
        std::map<SurfaceFaceId, std::size_t> robust_candidate_indices;
        std::map<SurfaceFaceId, Point3> explicit_apex_points;
        std::vector<SurfaceFaceId> keep_hexa_faces;
        std::vector<SurfaceFaceId> force_keep_hexa_faces;

        Scalar distanceScale(SurfaceFaceId id) const;
        std::size_t apexCandidateIndex(SurfaceFaceId id) const;
        bool keepHexa(SurfaceFaceId id) const;
        bool forceKeepHexa(SurfaceFaceId id) const;
    };

    using LayerTransitionError = std::variant<
        TransitionCoordinationError,
        TransitionBoundaryError,
        TransitionTemplateError>;

    using ProvisionalLayerTransitionResult = Result<
        ProvisionalLayerTransition, LayerTransitionError>;

    struct LayerTransitionInput
    {
        // Owning fields remain for compatibility; layer-scoped callers may
        // provide read-only views to avoid copying large fronts.
        GrowthFront current_front;
        GrowthFront candidate_front;
        const GrowthFront *current_front_view{};
        const GrowthFront *candidate_front_view{};
        // Sorted source face IDs omitted from the initial retained set.
        // The candidate front itself remains intact and can be shared.
        std::vector<SurfaceFaceId> excluded_candidate_faces;
        // Set by the incremental growth pipeline after rule-layer collision
        // filtering; standalone Resolver callers retain full validation.
        bool regular_candidate_geometry_prevalidated{};
        const GrowthFront &currentFront() const
        { return current_front_view ? *current_front_view : current_front; }
        const GrowthFront &candidateFront() const
        { return candidate_front_view ? *candidate_front_view : candidate_front; }
        LayerFaceSets face_sets;
        std::uint32_t completed_layer{};
        Scalar length_tolerance{1e-12};
        // Preferred non-owning path for layer-scoped immutable obstacle data.
        // The value field below remains for source compatibility with callers
        // that construct a self-contained resolver input.
        const CollisionIndex *original_surface_view{};
        std::optional<CollisionIndex> original_surface;
        const ExposedBoundaryTracker *historical_boundary{};
        bool historical_index_includes_transition{};
        const std::vector<OwnedBoundaryTriangle>
            *prior_transition_boundary{};
        const IncrementalCollisionIndex *prior_transition_dynamic_index{};
        const SlidingIntersectionIndex *sliding_surface{};
        std::function<std::optional<HexaPoints>(SurfaceFaceId)>
            terminal_hexa_points;
        std::function<ProvisionalLayerTransitionResult(
            const std::vector<SurfaceFaceId> &,
            const LayerFaceSets &,
            const ExternalPatchControls &)> build_provisional;
        std::function<ProvisionalLayerTransitionResult(
            const std::vector<SurfaceFaceId> &,
            const LayerFaceSets &,
            const std::vector<SurfaceFaceId> &,
            const ExternalPatchControls &)> build_external_patches;
        // Return every role for selected sources, including rollback provenance.
        // Incomplete provenance or non-triangle aggregates use the full builder.
        std::function<ProvisionalLayerTransitionResult(
            const std::vector<SurfaceFaceId> &,
            const LayerFaceSets &,
            const std::vector<SurfaceFaceId> &,
            const ExternalPatchControls &)> build_transition_patches;
        std::function<std::vector<SurfaceFaceId>(
            const std::vector<SurfaceFaceId> &)> affected_transition_faces;
        bool verify_local_rebuilds{};
    };

    struct StableLayerTransition
    {
        LayerFaceSets face_sets;
        std::vector<SurfaceFaceId> retained_high_faces;
        std::vector<OwnedBoundaryTriangle> exposed_boundary;
        std::vector<ResolvedTransitionTopology> resolved_topology;
        std::uint32_t iterations{};
        bool all_top_faces_are_triangles{};
        std::uint64_t collision_full_builds{};
        std::uint64_t collision_incremental_updates{};
        std::uint64_t provisional_full_builds{};
        std::uint64_t provisional_local_rebuilds{};
        std::uint64_t search_index_full_builds{};
    };

    class LayerTransitionResolver
    {
    public:
        Result<StableLayerTransition, LayerTransitionError>
        resolve(const LayerTransitionInput &input) const;
    };
}
