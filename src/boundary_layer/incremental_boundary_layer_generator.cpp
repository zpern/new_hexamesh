#include <algorithm>
#include <optional>
#include <unordered_set>
#include <utility>

#include <boundary_mesh/boundary_layer/incremental_boundary_layer_generator.hpp>
#include <boundary_mesh/boundary_layer/incremental_topology_finalizer.hpp>
#include <boundary_mesh/transition/accepted_stopped_front_carry.hpp>
#include <boundary_mesh/transition/layer_transition_resolver.hpp>
#include <boundary_mesh/transition/provisional_transition_builder.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool retainedFace(
            const std::vector<SurfaceFaceId> &retained, SurfaceFaceId id)
        {
            return std::binary_search(retained.begin(), retained.end(), id);
        }
    }

    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
    generateIncrementalBoundaryLayers(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options)
    {
        using GrowthResult = Result<
            RegularLayerGrowthResult, IncrementalLayerGrowthError>;
        const auto sliding_surface =
            SlidingIntersectionIndex::build(surface_mesh);
        if (!sliding_surface.hasValue())
            return GrowthResult::failure(IncrementalLayerGrowthError{
                RegularLayerGrowthError{
                    CollisionInitializationFailure{
                        sliding_surface.error()}}});
        RegularLayerGrowthOptions coordinated_options = options;
        std::optional<IncrementalLayerGrowthError> incremental_error;
        AcceptedStoppedFrontCarry previously_accepted;
        std::vector<OwnedBoundaryTriangle> prior_transition_boundary;
        const auto upstream_rejections = options.candidate_rejections;
        coordinated_options.candidate_rejections =
            [upstream_rejections, &incremental_error, &previously_accepted,
             &sliding_surface, &prior_transition_boundary](
                const GrowthFront &current,
                const LayerStepResult &candidate,
                const std::vector<VertexId> &current_global_ids,
                const CollisionIndex &original_surface,
                const ExposedBoundaryTracker &historical_boundary)
        {
            std::vector<SurfaceFaceId> rejected = upstream_rejections
                ? upstream_rejections(
                    current, candidate, current_global_ids,
                    original_surface, historical_boundary)
                : std::vector<SurfaceFaceId>{};
            LayerFaceSets face_sets;
            const AcceptedStoppedFrontCarry carried_stops =
                carryFacesMissingFromActive(previously_accepted, current);
            const GrowthFront effective_current =
                mergeWithCarriedStoppedFaces(current, carried_stops);
            addCarriedStops(face_sets, carried_stops);
            const std::unordered_set<SurfaceFaceId> continuing(
                candidate.next_front.source_face_ids.begin(),
                candidate.next_front.source_face_ids.end());
            const std::unordered_set<SurfaceFaceId> upstream_rejected_ids(
                rejected.begin(), rejected.end());
            GrowthFront filtered_candidate = candidate.next_front;
            filtered_candidate.faces.clear();
            filtered_candidate.source_face_ids.clear();
            for (std::size_t index = 0;
                 index < candidate.next_front.faces.size(); ++index)
                if (upstream_rejected_ids.find(
                        candidate.next_front.source_face_ids[index]) ==
                    upstream_rejected_ids.end())
                {
                    filtered_candidate.faces.push_back(
                        candidate.next_front.faces[index]);
                    filtered_candidate.source_face_ids.push_back(
                        candidate.next_front.source_face_ids[index]);
                }
            for (const SurfaceFaceId id : current.source_face_ids)
            {
                if (continuing.find(id) == continuing.end() ||
                    upstream_rejected_ids.find(id) !=
                        upstream_rejected_ids.end())
                    addInitialStop(face_sets, {
                        id, current.layer, StopOrigin::Quality});
            }
            const auto rememberAccepted = [&](
                const std::vector<SurfaceFaceId> &retained)
            {
                previously_accepted = retainAcceptedFaces(
                    candidate.next_front,
                    candidate.accepted_stopped_faces,
                    retained);
            };
            if (face_sets.transition_low_faces.empty())
            {
                std::vector<SurfaceFaceId> retained =
                    filtered_candidate.source_face_ids;
                std::sort(retained.begin(), retained.end());
                rememberAccepted(retained);
                return rejected;
            }
            LayerTransitionInput input;
            input.current_front = effective_current;
            input.candidate_front = std::move(filtered_candidate);
            input.face_sets = std::move(face_sets);
            input.completed_layer = current.layer;
            input.original_surface = original_surface;
            input.historical_boundary = &historical_boundary;
            input.prior_transition_boundary =
                &prior_transition_boundary;
            input.sliding_surface = &sliding_surface.value();
            input.build_provisional =
                [&effective_current, &candidate](
                    const std::vector<SurfaceFaceId> &retained,
                    const LayerFaceSets &sets)
            {
                return buildProvisionalTransition(
                    effective_current, candidate.next_front,
                    retained, sets);
            };
            const auto stable = LayerTransitionResolver{}.resolve(input);
            if (!stable.hasValue())
            {
                std::visit([&](const auto &error)
                {
                    incremental_error = IncrementalLayerGrowthError{error};
                }, stable.error());
                return rejected;
            }
            prior_transition_boundary.clear();
            for (const auto &triangle : stable.value().exposed_boundary)
                if (triangle.owner.role !=
                    BoundaryOwnerRole::RegularCandidate)
                    prior_transition_boundary.push_back(triangle);
            for (const SurfaceFaceId id :
                 candidate.next_front.source_face_ids)
                if (!retainedFace(
                        stable.value().retained_high_faces, id))
                    rejected.push_back(id);
            rememberAccepted(stable.value().retained_high_faces);
            std::sort(rejected.begin(), rejected.end());
            rejected.erase(
                std::unique(rejected.begin(), rejected.end()),
                rejected.end());
            return rejected;
        };
        auto regular = generateRegularLayers(
            surface_mesh, topology, patch, initial_front,
            profiles, coordinated_options);
        if (incremental_error.has_value())
            return GrowthResult::failure(
                *incremental_error);
        if (!regular.hasValue())
            return GrowthResult::failure(
                IncrementalLayerGrowthError{regular.error()});
        return finalizeIncrementalLayerTopology(
            surface_mesh, initial_front, std::move(regular.value()));
    }
}
