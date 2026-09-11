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

        const LayerVertexRecord *layerRecord(
            const LayerVertexTable &table,
            VertexId source_vertex_id,
            std::uint32_t branch_id)
        {
            const auto found = std::find_if(
                table.begin(), table.end(),
                [source_vertex_id, branch_id](const LayerVertexRecord &record)
                {
                    return record.source_vertex_id == source_vertex_id &&
                        record.branch_id == branch_id;
                });
            return found == table.end() ? nullptr : &*found;
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
        std::vector<ResolvedTransitionTopology> resolved_topology;
        std::unordered_map<VertexId, std::uint32_t> requested_layers;
        for (const auto &profile : profiles)
            requested_layers[profile.source_vertex_id] =
                profile.profile.layer_count;
        const auto upstream_rejections = options.candidate_rejections;
        coordinated_options.candidate_rejections =
            [upstream_rejections, &incremental_error, &previously_accepted,
             &sliding_surface, &prior_transition_boundary,
             &resolved_topology, &requested_layers](
                const GrowthFront &current,
                const LayerStepResult &candidate,
                const std::vector<VertexId> &current_global_ids,
                const VolumeMesh &committed_mesh,
                const LayerVertexTable &layer_vertices,
                const CollisionIndex &original_surface,
                const ExposedBoundaryTracker &historical_boundary)
        {
            std::vector<SurfaceFaceId> rejected = upstream_rejections
                ? upstream_rejections(
                    current, candidate, current_global_ids,
                    committed_mesh, layer_vertices,
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
            std::vector<SurfaceFaceId> terminal_candidate_faces;
            for (std::size_t index = 0;
                 index < filtered_candidate.faces.size(); ++index)
            {
                if (!std::holds_alternative<Quad>(
                        filtered_candidate.faces[index]))
                    continue;
                bool reaches_requested_limit = false;
                for (const VertexId local : std::get<Quad>(
                         filtered_candidate.faces[index]).vertex_ids)
                {
                    const auto requested = requested_layers.find(
                        filtered_candidate.vertices[local].source_vertex_id);
                    if (requested != requested_layers.end() &&
                        candidate.layer >= requested->second)
                    {
                        reaches_requested_limit = true;
                        break;
                    }
                }
                if (reaches_requested_limit)
                    terminal_candidate_faces.push_back(
                        filtered_candidate.source_face_ids[index]);
            }
            std::sort(terminal_candidate_faces.begin(),
                      terminal_candidate_faces.end());
            if (face_sets.transition_low_faces.empty() &&
                terminal_candidate_faces.empty())
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
            input.terminal_hexa_points =
                [&effective_current, &committed_mesh, &layer_vertices,
                 completed_layer = current.layer](SurfaceFaceId id)
                    -> std::optional<HexaPoints>
            {
                if (completed_layer == 0) return std::nullopt;
                const auto position = std::find(
                    effective_current.source_face_ids.begin(),
                    effective_current.source_face_ids.end(), id);
                if (position == effective_current.source_face_ids.end())
                    return std::nullopt;
                const auto index = static_cast<std::size_t>(std::distance(
                    effective_current.source_face_ids.begin(), position));
                const auto *quad = std::get_if<Quad>(
                    &effective_current.faces[index]);
                if (quad == nullptr) return std::nullopt;
                HexaPoints points{};
                for (std::size_t local = 0; local < 4; ++local)
                {
                    const auto &vertex = effective_current.vertices[
                        quad->vertex_ids[local]];
                    const auto *record = layerRecord(
                        layer_vertices, vertex.source_vertex_id,
                        vertex.branch_id);
                    if (record == nullptr ||
                        record->layer_vertex_ids.size() <= completed_layer)
                        return std::nullopt;
                    const VertexId bottom = record->layer_vertex_ids[
                        completed_layer - 1];
                    const VertexId top = record->layer_vertex_ids[
                        completed_layer];
                    if (static_cast<std::size_t>(bottom) >=
                            committed_mesh.vertices.size() ||
                        static_cast<std::size_t>(top) >=
                            committed_mesh.vertices.size())
                        return std::nullopt;
                    points[local] = committed_mesh.vertices[bottom];
                    points[4 + local] = committed_mesh.vertices[top];
                }
                return points;
            };
            input.build_provisional =
                [&effective_current, &candidate,
                 &terminal_hexa_points = input.terminal_hexa_points,
                 terminal_candidate_faces](
                    const std::vector<SurfaceFaceId> &retained,
                    const LayerFaceSets &sets,
                    const ExternalPatchControls &external_controls)
            {
                return buildProvisionalTransition(
                    effective_current, candidate.next_front,
                    retained, sets, terminal_hexa_points,
                    external_controls, terminal_candidate_faces);
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
            resolved_topology.erase(
                std::remove_if(
                    resolved_topology.begin(), resolved_topology.end(),
                    [layer = current.layer](
                        const ResolvedTransitionTopology &entry)
                    { return entry.layer == layer; }),
                resolved_topology.end());
            resolved_topology.insert(
                resolved_topology.end(),
                stable.value().resolved_topology.begin(),
                stable.value().resolved_topology.end());
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
            surface_mesh, initial_front, std::move(regular.value()),
            resolved_topology);
    }
}
