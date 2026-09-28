#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <optional>
#include <unordered_set>
#include <utility>

#include <boundary_mesh/boundary_layer/incremental_boundary_layer_generator.hpp>
#include <boundary_mesh/boundary_layer/incremental_topology_finalizer.hpp>
#include <boundary_mesh/boundary_layer/terminal_layer_lookup.hpp>
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

        bool traceTransitionFace(SurfaceFaceId id)
        {
            const char *value = std::getenv("BOUNDARY_MESH_TRACE_FACE");
            if (value == nullptr) return false;
            while (*value != '\0')
            {
                char *end = nullptr;
                const auto parsed = std::strtoull(value, &end, 10);
                if (end != value && parsed == id) return true;
                if (end == value) break;
                value = end;
                while (*value == ',' || *value == ';' || *value == ' ')
                    ++value;
            }
            return false;
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
            [upstream_rejections, verify_rebuilds = options.verify_transition_rebuilds,
             &incremental_error, &previously_accepted,
             &sliding_surface, &prior_transition_boundary,
             &resolved_topology, &requested_layers](
                const GrowthFront &current,
                const LayerStepResult &candidate,
                const std::vector<VertexId> &current_global_ids,
                const VolumeMesh &committed_mesh,
                const LayerVertexTable &layer_vertices,
                const CollisionIndex &original_surface,
                ExposedBoundaryTracker &historical_boundary)
        {
            const auto callback_started = std::chrono::steady_clock::now();
            std::vector<SurfaceFaceId> rejected = upstream_rejections
                ? upstream_rejections(
                    current, candidate, current_global_ids,
                    committed_mesh, layer_vertices,
                    original_surface, historical_boundary)
                : std::vector<SurfaceFaceId>{};
            LayerFaceSets face_sets;
            const AcceptedStoppedFrontCarry carried_stops =
                carryFacesMissingFromActive(previously_accepted, current);
            std::optional<GrowthFront> effective_current_storage;
            if (!carried_stops.front.faces.empty())
                effective_current_storage.emplace(
                    mergeWithCarriedStoppedFaces(current, carried_stops));
            const GrowthFront &effective_current =
                effective_current_storage.has_value()
                    ? *effective_current_storage : current;
            addCarriedStops(face_sets, carried_stops);
            const std::unordered_set<SurfaceFaceId> continuing(
                candidate.next_front.source_face_ids.begin(),
                candidate.next_front.source_face_ids.end());
            const std::unordered_set<SurfaceFaceId> upstream_rejected_ids(
                rejected.begin(), rejected.end());
            std::vector<SurfaceFaceId> excluded_candidate_faces = rejected;
            std::sort(excluded_candidate_faces.begin(),
                      excluded_candidate_faces.end());
            excluded_candidate_faces.erase(std::unique(
                excluded_candidate_faces.begin(),
                excluded_candidate_faces.end()),
                excluded_candidate_faces.end());
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
                 index < candidate.next_front.faces.size(); ++index)
            {
                if (upstream_rejected_ids.find(
                        candidate.next_front.source_face_ids[index]) !=
                    upstream_rejected_ids.end())
                    continue;
                if (!std::holds_alternative<Quad>(
                        candidate.next_front.faces[index]))
                    continue;
                bool reaches_requested_limit = false;
                for (const VertexId local : std::get<Quad>(
                         candidate.next_front.faces[index]).vertex_ids)
                {
                    const auto requested = requested_layers.find(
                        candidate.next_front.vertices[local].source_vertex_id);
                    if (requested != requested_layers.end() &&
                        candidate.layer >= requested->second)
                    {
                        reaches_requested_limit = true;
                        break;
                    }
                }
                if (reaches_requested_limit)
                    terminal_candidate_faces.push_back(
                        candidate.next_front.source_face_ids[index]);
            }
            std::sort(terminal_candidate_faces.begin(),
                      terminal_candidate_faces.end());
            if (std::getenv("BOUNDARY_MESH_TRACE_FACE") != nullptr)
            {
                for (const SurfaceFaceId trace_id : {SurfaceFaceId{298895},
                                                      SurfaceFaceId{445377}})
                {
                    if (!traceTransitionFace(trace_id) || current.layer != 11)
                        continue;
                    const auto contains = [trace_id](const auto &ids)
                    { return std::find(ids.begin(), ids.end(), trace_id) !=
                             ids.end(); };
                    const auto stop = std::find_if(
                        face_sets.states.begin(), face_sets.states.end(),
                        [trace_id](const LayerStopState &state)
                        { return state.source_face_id == trace_id; });
                    std::cerr << "trace generator face=" << trace_id
                              << " layer=" << current.layer
                              << " previous=" << contains(
                                     previously_accepted.front.source_face_ids)
                              << " current=" << contains(current.source_face_ids)
                              << " carried=" << contains(
                                     carried_stops.front.source_face_ids)
                              << " effective=" << contains(
                                     effective_current.source_face_ids)
                              << " candidate=" << contains(
                                     candidate.next_front.source_face_ids)
                              << " upstream_rejected=" << contains(rejected)
                              << " corner_seed=" << contains(
                                     face_sets.corner_suppression_seeds)
                              << " low=" << contains(face_sets.transition_low_faces)
                              << " terminal=" << contains(terminal_candidate_faces)
                              << " stop_origin=";
                    if (stop == face_sets.states.end())
                        std::cerr << "none";
                    else
                        std::cerr << static_cast<int>(stop->origin)
                                  << ':' << stop->completed_layer;
                    std::cerr
                              << '\n';
                }
            }
            if (face_sets.transition_low_faces.empty() &&
                terminal_candidate_faces.empty())
            {
                std::vector<SurfaceFaceId> retained =
                    candidate.next_front.source_face_ids;
                retained.erase(std::remove_if(
                    retained.begin(), retained.end(),
                    [&](SurfaceFaceId id)
                    {
                        return upstream_rejected_ids.find(id) !=
                            upstream_rejected_ids.end();
                    }), retained.end());
                std::sort(retained.begin(), retained.end());
                rememberAccepted(retained);
                return rejected;
            }
            LayerTransitionInput input;
            input.current_front_view = &effective_current;
            input.candidate_front_view = &candidate.next_front;
            input.excluded_candidate_faces =
                std::move(excluded_candidate_faces);
            input.regular_candidate_geometry_prevalidated = true;
            input.face_sets = std::move(face_sets);
            input.completed_layer = current.layer;
            input.original_surface_view = &original_surface;
            input.historical_boundary = &historical_boundary;
            input.historical_index_includes_transition = true;
            input.prior_transition_boundary =
                &prior_transition_boundary;
            input.sliding_surface = &sliding_surface.value();
            const TerminalLayerLookup terminal_lookup(
                effective_current, layer_vertices);
            input.terminal_hexa_points =
                [&effective_current, &committed_mesh, &layer_vertices,
                 &terminal_lookup,
                 completed_layer = current.layer](SurfaceFaceId id)
                    -> std::optional<HexaPoints>
            {
                return terminal_lookup.hexaPoints(
                    id, completed_layer, effective_current, layer_vertices,
                    committed_mesh);
            };
            const auto lookup_context_started =
                std::chrono::steady_clock::now();
            const ProvisionalTransitionBuildContext transition_build_context{
                effective_current, candidate.next_front};
            std::cerr
                << "temporary resolver lookup_context_builds=1"
                << " lookup_context_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() -
                       lookup_context_started).count()
                << '\n';
            input.build_provisional =
                [&transition_build_context,
                 &terminal_hexa_points = input.terminal_hexa_points,
                 terminal_candidate_faces](
                    const std::vector<SurfaceFaceId> &retained,
                    const LayerFaceSets &sets,
                    const ExternalPatchControls &external_controls)
            {
                return buildProvisionalTransition(
                    transition_build_context, retained, sets,
                    terminal_hexa_points,
                    external_controls, terminal_candidate_faces);
            };
            input.build_external_patches =
                [&transition_build_context,
                 &terminal_hexa_points = input.terminal_hexa_points,
                 terminal_candidate_faces](
                    const std::vector<SurfaceFaceId> &retained,
                    const LayerFaceSets &sets,
                    const std::vector<SurfaceFaceId> &selected,
                    const ExternalPatchControls &external_controls)
            {
                return buildProvisionalExternalPatches(
                    transition_build_context, retained, sets, selected,
                    terminal_hexa_points,
                    external_controls, terminal_candidate_faces);
            };
            input.build_transition_patches =
                [&transition_build_context,
                 &terminal_hexa_points = input.terminal_hexa_points,
                 terminal_candidate_faces](const auto &retained, const auto &sets,
                     const auto &selected, const auto &controls)
            {
                return buildProvisionalTransitionPatches(transition_build_context,
                    retained, sets, selected, terminal_hexa_points, controls,
                    terminal_candidate_faces);
            };
            input.affected_transition_faces = [&transition_build_context](const auto &changed)
            { return transition_build_context.affectedFaces(changed); };
            input.verify_local_rebuilds = verify_rebuilds;
            const auto resolver_started = std::chrono::steady_clock::now();
            const auto callback_prepare_nanoseconds =
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    resolver_started - callback_started).count();
            const auto stable = LayerTransitionResolver{}.resolve(input);
            const auto resolver_finished = std::chrono::steady_clock::now();
            if (!stable.hasValue())
            {
                std::visit([&](const auto &error)
                {
                    incremental_error = IncrementalLayerGrowthError{error};
                }, stable.error());
                return rejected;
            }
            std::vector<CollisionTriangle> accepted_transition_triangles;
            for (const auto &triangle : stable.value().exposed_boundary)
                if (triangle.owner.role !=
                    BoundaryOwnerRole::RegularCandidate)
                {
                    if (prior_transition_boundary.size() >=
                        static_cast<std::size_t>(
                            std::numeric_limits<std::uint32_t>::max()))
                    {
                        incremental_error = IncrementalLayerGrowthError{
                            TransitionBoundaryError{
                                SpatialError::PrimitiveIdOverflow}};
                        return rejected;
                    }
                    const auto owner_id = static_cast<std::uint32_t>(
                        prior_transition_boundary.size());
                    accepted_transition_triangles.push_back(
                        makeTransitionCollisionTriangle(triangle, owner_id));
                    prior_transition_boundary.push_back(triangle);
                }
            if (!accepted_transition_triangles.empty())
            {
                const auto indexed = historical_boundary.appendTransitionTriangles(
                    std::move(accepted_transition_triangles));
                if (!indexed.hasValue())
                {
                    incremental_error = IncrementalLayerGrowthError{
                        TransitionBoundaryError{indexed.error()}};
                    return rejected;
                }
            }
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
            const auto callback_finished = std::chrono::steady_clock::now();
            std::cerr << "temporary resolver callback prepare_ms="
                << callback_prepare_nanoseconds / 1000000
                << " resolve_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       resolver_finished - resolver_started).count()
                << " result_ms="
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       callback_finished - resolver_finished).count()
                << '\n';
            return rejected;
        };
        coordinated_options.defer_interface_materialization = true;
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
            resolved_topology, coordinated_options.split_failed_hexa_columns);
    }
}
