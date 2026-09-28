#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <boundary_mesh/transition/layer_transition_resolver.hpp>
#include <boundary_mesh/transition/incremental_transition_collision_state.hpp>
#include <boundary_mesh/growth/apex_solver.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar minimum_external_distance_scale{1e-6};

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

        bool sameSlidingColumns(
            const std::shared_ptr<const SlidingColumnContext> &left,
            const std::shared_ptr<const SlidingColumnContext> &right)
        {
            if (!left || !right) return left == right;
            return left->low_points == right->low_points &&
                left->high_points == right->high_points &&
                left->low_region_ids == right->low_region_ids &&
                left->high_region_ids == right->high_region_ids;
        }

        bool sameOwnedTriangle(
            const OwnedBoundaryTriangle &left,
            const OwnedBoundaryTriangle &right)
        {
            bool same_vertex_keys = true;
            for (std::size_t index = 0; index < 3; ++index)
                same_vertex_keys = same_vertex_keys &&
                    left.vertex_keys[index].source_vertex_id ==
                        right.vertex_keys[index].source_vertex_id &&
                    left.vertex_keys[index].layer ==
                        right.vertex_keys[index].layer &&
                    left.vertex_keys[index].branch_id ==
                        right.vertex_keys[index].branch_id;
            return left.points == right.points &&
                same_vertex_keys &&
                layerBoundaryOwnerKey(left.owner) ==
                    layerBoundaryOwnerKey(right.owner) &&
                left.owner.rollback_high_faces ==
                    right.owner.rollback_high_faces &&
                left.vertex_sliding_region_ids ==
                    right.vertex_sliding_region_ids &&
                left.physical_edge_mask == right.physical_edge_mask &&
                left.complete_face_exemption_regions ==
                    right.complete_face_exemption_regions &&
                sameSlidingColumns(left.sliding_columns,
                                   right.sliding_columns);
        }

        std::vector<LayerBoundaryOwnerKey> changedBoundaryOwners(
            const TransitionBoundaryInput &previous,
            const TransitionBoundaryInput &next)
        {
            using Groups = std::map<LayerBoundaryOwnerKey,
                std::vector<const OwnedBoundaryTriangle *>>;
            const auto group = [](const TransitionBoundaryInput &input)
            {
                Groups groups;
                for (const auto &triangle : input.candidate_triangles)
                    groups[layerBoundaryOwnerKey(triangle.owner)].push_back(
                        &triangle);
                return groups;
            };
            const Groups old_groups = group(previous);
            const Groups new_groups = group(next);
            std::set<LayerBoundaryOwnerKey> keys;
            for (const auto &[key, unused] : old_groups)
            { (void)unused; keys.insert(key); }
            for (const auto &[key, unused] : new_groups)
            { (void)unused; keys.insert(key); }
            std::vector<LayerBoundaryOwnerKey> changed;
            for (const auto &key : keys)
            {
                const auto old_group = old_groups.find(key);
                const auto new_group = new_groups.find(key);
                if (old_group == old_groups.end() ||
                    new_group == new_groups.end() ||
                    old_group->second.size() != new_group->second.size())
                {
                    changed.push_back(key);
                    continue;
                }
                for (std::size_t index = 0;
                     index < old_group->second.size(); ++index)
                    if (!sameOwnedTriangle(*old_group->second[index],
                                           *new_group->second[index]))
                    {
                        changed.push_back(key);
                        break;
                    }
            }
            return changed;
        }
    }

    TerminalQuadDecision chooseTerminalQuadDecision(
        Scalar aspect_ratio,
        const std::optional<Point3> &internal_center,
        bool external_available,
        Scalar aspect_ratio_threshold)
    {
        const bool prefer_external =
            aspect_ratio < aspect_ratio_threshold;
        if (!prefer_external && internal_center.has_value())
            return TerminalQuadDecision::InternalSplit;
        if (external_available)
            return TerminalQuadDecision::ExternalPatch;
        return internal_center.has_value()
            ? TerminalQuadDecision::InternalSplit
            : TerminalQuadDecision::KeepHexa;
    }

    Scalar ExternalPatchControls::distanceScale(SurfaceFaceId id) const
    {
        const auto found = distance_scales.find(id);
        return found == distance_scales.end() ? Scalar{0.25} : found->second;
    }

    std::size_t ExternalPatchControls::apexCandidateIndex(
        SurfaceFaceId id) const
    {
        const auto found = apex_candidate_indices.find(id);
        return found == apex_candidate_indices.end() ? 0 : found->second;
    }

    bool ExternalPatchControls::keepHexa(SurfaceFaceId id) const
    {
        return std::binary_search(
            keep_hexa_faces.begin(), keep_hexa_faces.end(), id);
    }

    bool ExternalPatchControls::forceKeepHexa(SurfaceFaceId id) const
    {
        return std::binary_search(
            force_keep_hexa_faces.begin(), force_keep_hexa_faces.end(), id);
    }

    namespace
    {
        void replaceExternalPatches(
            ProvisionalLayerTransition &result,
            const ProvisionalLayerTransition &replacement,
            const std::vector<SurfaceFaceId> &selected)
        {
            const std::set<SurfaceFaceId> selected_set(
                selected.begin(), selected.end());
            const auto isSelected = [&](SurfaceFaceId id)
            {
                return selected_set.count(id) != 0;
            };
            auto &triangles = result.boundary.candidate_triangles;
            triangles.erase(std::remove_if(
                triangles.begin(), triangles.end(), [&](const auto &owned)
                {
                    return owned.owner.role ==
                               BoundaryOwnerRole::ExternalPatch &&
                           isSelected(owned.owner.source_face_id);
                }), triangles.end());
            auto &topology = result.resolved_topology;
            topology.erase(std::remove_if(
                topology.begin(), topology.end(), [&](const auto &entry)
                { return isSelected(entry.source_face_id); }), topology.end());
            triangles.insert(
                triangles.end(),
                replacement.boundary.candidate_triangles.begin(),
                replacement.boundary.candidate_triangles.end());
            topology.insert(
                topology.end(), replacement.resolved_topology.begin(),
                replacement.resolved_topology.end());
            result.forced_rollback_high_faces.insert(
                result.forced_rollback_high_faces.end(),
                replacement.forced_rollback_high_faces.begin(),
                replacement.forced_rollback_high_faces.end());
            std::sort(result.forced_rollback_high_faces.begin(),
                      result.forced_rollback_high_faces.end());
            result.forced_rollback_high_faces.erase(std::unique(
                result.forced_rollback_high_faces.begin(),
                result.forced_rollback_high_faces.end()),
                result.forced_rollback_high_faces.end());
            result.all_top_faces_are_triangles =
                result.all_top_faces_are_triangles &&
                replacement.all_top_faces_are_triangles;
            for (const auto id : selected)
                result.forced_rollback_by_source.erase(id);
            result.forced_rollback_by_source.insert(
                replacement.forced_rollback_by_source.begin(),
                replacement.forced_rollback_by_source.end());
        }

        bool hasCompleteRollbackProvenance(const ProvisionalLayerTransition &value)
        {
            std::set<SurfaceFaceId> recorded;
            for (const auto &[source, dependencies] : value.forced_rollback_by_source)
            {
                (void)source;
                recorded.insert(dependencies.begin(), dependencies.end());
            }
            return recorded == std::set<SurfaceFaceId>(
                value.forced_rollback_high_faces.begin(),
                value.forced_rollback_high_faces.end());
        }

        void replaceTransitionPatches(
            ProvisionalLayerTransition &base,
            ProvisionalLayerTransition replacement,
            const std::vector<SurfaceFaceId> &selected)
        {
            const std::set<SurfaceFaceId> ids(selected.begin(), selected.end());
            auto &triangles = base.boundary.candidate_triangles;
            triangles.erase(std::remove_if(triangles.begin(), triangles.end(),
                [&](const auto &t) { return ids.count(t.owner.source_face_id) != 0; }),
                triangles.end());
            auto &topology = base.resolved_topology;
            topology.erase(std::remove_if(topology.begin(), topology.end(),
                [&](const auto &t) { return ids.count(t.source_face_id) != 0; }),
                topology.end());
            auto &diagonals = base.boundary.diagonal_requirements;
            diagonals.erase(std::remove_if(diagonals.begin(), diagonals.end(),
                [&](const auto &d) { return ids.count(d.key.source_face_id) != 0; }),
                diagonals.end());
            for (const auto id : selected) base.forced_rollback_by_source.erase(id);
            base.forced_rollback_by_source.insert(
                replacement.forced_rollback_by_source.begin(),
                replacement.forced_rollback_by_source.end());
            base.forced_rollback_high_faces.clear();
            for (const auto &[id, dependencies] : base.forced_rollback_by_source)
            {
                (void)id;
                base.forced_rollback_high_faces.insert(
                    base.forced_rollback_high_faces.end(),
                    dependencies.begin(), dependencies.end());
            }
            for (auto &triangle : replacement.boundary.candidate_triangles)
                triangles.push_back(std::move(triangle));
            for (auto &entry : replacement.resolved_topology)
                topology.push_back(std::move(entry));
            diagonals.insert(diagonals.end(),
                replacement.boundary.diagonal_requirements.begin(),
                replacement.boundary.diagonal_requirements.end());
            base.all_top_faces_are_triangles = replacement.all_top_faces_are_triangles;
        }

        bool sameProvisional(const ProvisionalLayerTransition &a,
                             const ProvisionalLayerTransition &b)
        {
            if (!changedBoundaryOwners(a.boundary, b.boundary).empty() ||
                a.all_top_faces_are_triangles != b.all_top_faces_are_triangles ||
                a.resolved_topology.size() != b.resolved_topology.size()) return false;
            auto sortedIds = [](auto ids) {
                std::sort(ids.begin(), ids.end());
                ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
                return ids;
            };
            if (a.forced_rollback_by_source != b.forced_rollback_by_source) return false;
            if (sortedIds(a.forced_rollback_high_faces) !=
                sortedIds(b.forced_rollback_high_faces)) return false;
            const auto diagonals = [](const auto &value) {
                std::map<std::pair<SurfaceFaceId, std::uint32_t>, QuadDiagonal> result;
                for (const auto &d : value.boundary.diagonal_requirements)
                    result[{d.key.source_face_id, d.key.layer}] = d.diagonal;
                return result;
            };
            if (diagonals(a) != diagonals(b)) return false;
            std::map<SurfaceFaceId, const ResolvedTransitionTopology *> lookup;
            for (const auto &t : b.resolved_topology) lookup[t.source_face_id] = &t;
            for (const auto &t : a.resolved_topology)
            {
                const auto found = lookup.find(t.source_face_id);
                if (found == lookup.end()) return false;
                const auto &u = *found->second;
                if (t.layer != u.layer || t.template_kind != u.template_kind ||
                    t.low_diagonal != u.low_diagonal ||
                    t.dependent_high_faces != u.dependent_high_faces ||
                    t.terminal_quad_decision != u.terminal_quad_decision ||
                    t.aspect_ratio != u.aspect_ratio ||
                    t.generated_point != u.generated_point ||
                    t.retained_local_edges != u.retained_local_edges) return false;
            }
            return true;
        }
    }

    Result<StableLayerTransition, LayerTransitionError>
    LayerTransitionResolver::resolve(
        const LayerTransitionInput &input) const
    {
        using ResolveResult = Result<
            StableLayerTransition, LayerTransitionError>;
        if (!input.build_provisional)
            return ResolveResult::failure(LayerTransitionError{
                TransitionCoordinationError{
                    MissingTransitionFaceState{0}}});

        LayerFaceSets face_sets = input.face_sets;
        std::vector<SurfaceFaceId> retained =
            input.candidateFront().source_face_ids;
        std::sort(retained.begin(), retained.end());
        retained.erase(std::unique(retained.begin(), retained.end()),
                       retained.end());
        if (!input.excluded_candidate_faces.empty())
        {
            retained.erase(std::remove_if(
                retained.begin(), retained.end(),
                [&](SurfaceFaceId id)
                {
                    return std::binary_search(
                        input.excluded_candidate_faces.begin(),
                        input.excluded_candidate_faces.end(), id);
                }), retained.end());
        }
        std::uint32_t iterations = 1;
        TransitionBoundaryChecker checker;
        ExternalPatchControls external_controls;
        std::uint64_t full_build_count{};
        std::uint64_t full_build_milliseconds{};
        std::uint64_t local_build_count{};
        std::uint64_t local_build_nanoseconds{};
        std::uint64_t patch_replace_milliseconds{};
        std::uint64_t exact_query_nanoseconds{};
        std::uint64_t probe_rounds{};
        std::set<SurfaceFaceId> searched_patch_ids;
        std::optional<IncrementalTransitionCollisionState> collision_state;
        std::uint64_t collision_full_builds{};
        std::uint64_t collision_incremental_updates{};
        std::uint64_t provisional_local_rebuilds{};
        std::optional<ProvisionalLayerTransition> cached_provisional;
        std::vector<SurfaceFaceId> previous_retained, previous_low, reset_control_faces;
        std::optional<IncrementalCollisionIndex> search_index;
        std::map<CollisionGroupId, std::vector<OwnedBoundaryTriangle>> search_boundary;
        std::map<SurfaceFaceId, std::set<CollisionGroupId>>
            search_source_groups;
        std::map<LayerBoundaryOwnerKey, CollisionGroupId> search_groups;
        CollisionGroupId next_search_group = CollisionGroupId{1} << 33;
        std::uint64_t search_index_full_builds{}, search_index_nanoseconds{};
        std::optional<CollisionIndex> prior_transition_index;
        std::uint64_t prior_transition_index_build_nanoseconds{};
        if (!input.historical_index_includes_transition &&
            input.prior_transition_dynamic_index == nullptr &&
            input.prior_transition_boundary != nullptr &&
            !input.prior_transition_boundary->empty())
        {
            const auto index_started = std::chrono::steady_clock::now();
            std::vector<CollisionTriangle> triangles;
            triangles.reserve(input.prior_transition_boundary->size());
            for (std::size_t index = 0;
                 index < input.prior_transition_boundary->size(); ++index)
                triangles.push_back(makeTransitionCollisionTriangle(
                    (*input.prior_transition_boundary)[index],
                    static_cast<std::uint32_t>(index)));
            auto built = CollisionIndex::build(std::move(triangles));
            if (!built.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    TransitionBoundaryError{built.error()}});
            prior_transition_index = std::move(built.value());
            prior_transition_index_build_nanoseconds +=
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() - index_started)
                        .count());
        }

        const CornerSuppressionContext corner_context(
            input.currentFront(), input.candidateFront());
        std::uint64_t corner_suppression_nanoseconds{};
        std::uint64_t external_search_nanoseconds{};
        std::uint64_t rollback_coordination_nanoseconds{};
        std::vector<OwnedBoundaryTriangle> provisional_delta_triangles;
        while (true)
        {
            provisional_delta_triangles.clear();
            std::optional<std::set<SurfaceFaceId>> rebuilt_sources;
            const auto iteration_started = std::chrono::steady_clock::now();
            const auto corner_suppression_started =
                std::chrono::steady_clock::now();
            const auto suppression = applyCornerSuppression(CornerSuppressionView{
                input.currentFront(),
                input.candidateFront(),
                retained,
                face_sets,
                input.completed_layer,
                input.length_tolerance}, corner_context);
            if (!suppression.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    suppression.error()});
            face_sets = suppression.value().face_sets;
            retained = suppression.value().retained_high_faces;
            corner_suppression_nanoseconds +=
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() -
                        corner_suppression_started).count());

            const auto configure = [&](ProvisionalLayerTransition &value)
            {
                value.boundary.regular_candidate_geometry_prevalidated =
                    input.regular_candidate_geometry_prevalidated;
                value.boundary.original_surface =
                    input.original_surface_view != nullptr
                        ? input.original_surface_view
                        : (input.original_surface.has_value()
                               ? &*input.original_surface
                               : nullptr);
                value.boundary.historical_boundary =
                    input.historical_boundary;
                value.boundary.historical_index = input.historical_boundary
                    ? &input.historical_boundary->collisionIndex()
                    : nullptr;
                value.boundary.historical_index_includes_transition =
                    input.historical_index_includes_transition;
                value.boundary.prior_transition_boundary =
                    input.prior_transition_boundary;
                value.boundary.prior_transition_index =
                    prior_transition_index.has_value()
                        ? &*prior_transition_index : nullptr;
                value.boundary.prior_transition_dynamic_index =
                    input.prior_transition_dynamic_index;
                value.boundary.sliding_surface = input.sliding_surface;
            };
            const auto build = [&]()
            {
                const auto started = std::chrono::steady_clock::now();
                auto value = input.build_provisional(
                    retained, face_sets, external_controls);
                if (value.hasValue()) configure(value.value());
                ++full_build_count;
                full_build_milliseconds +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - started)
                            .count());
                return value;
            };

            const auto provisional_build_started =
                std::chrono::steady_clock::now();
            auto provisional = [&]() -> ProvisionalLayerTransitionResult
            {
                if (!cached_provisional || !input.build_transition_patches ||
                    !input.affected_transition_faces) return build();
                // Legacy/custom builders may not provide patch-level aggregate
                // provenance. Preserve their full-build semantics in that case.
                if (!cached_provisional->all_top_faces_are_triangles ||
                    !hasCompleteRollbackProvenance(*cached_provisional)) return build();
                std::vector<SurfaceFaceId> seeds;
                std::set_symmetric_difference(previous_retained.begin(), previous_retained.end(),
                    retained.begin(), retained.end(), std::back_inserter(seeds));
                auto lows = face_sets.transition_low_faces;
                std::sort(lows.begin(), lows.end());
                std::set_symmetric_difference(previous_low.begin(), previous_low.end(),
                    lows.begin(), lows.end(), std::back_inserter(seeds));
                auto selected = input.affected_transition_faces(seeds);
                selected.insert(selected.end(), reset_control_faces.begin(), reset_control_faces.end());
                std::sort(selected.begin(), selected.end());
                selected.erase(std::unique(selected.begin(), selected.end()), selected.end());
                const auto started = std::chrono::steady_clock::now();
                auto local = input.build_transition_patches(retained, face_sets, selected,
                    external_controls);
                if (!local.hasValue()) return local;
                if (!local.value().all_top_faces_are_triangles ||
                    !hasCompleteRollbackProvenance(local.value())) return build();
                provisional_delta_triangles =
                    local.value().boundary.candidate_triangles;
                rebuilt_sources.emplace(selected.begin(), selected.end());
                replaceTransitionPatches(*cached_provisional, std::move(local.value()), selected);
                configure(*cached_provisional);
                ++provisional_local_rebuilds;
                std::cerr << "temporary resolver local_rebuild faces=" << selected.size()
                    << " ms=" << std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - started).count() << '\n';
                bool verify = input.verify_local_rebuilds;
#ifndef NDEBUG
                verify = true;
#endif
                if (verify)
                {
                    auto reference = input.build_provisional(retained, face_sets, external_controls);
                    if (!reference.hasValue()) return reference;
                    if (!sameProvisional(*cached_provisional, reference.value()))
                        throw std::logic_error("local transition rebuild differs from full rebuild");
                }
                return ProvisionalLayerTransitionResult::success(std::move(*cached_provisional));
            }();
            cached_provisional.reset();
            if (!provisional.hasValue())
                return ResolveResult::failure(provisional.error());
            std::cerr << "temporary resolver build_ms="
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() -
                             provisional_build_started).count()
                      << '\n';

            const auto buildProbe = [&](
                const std::vector<SurfaceFaceId> &selected)
                -> ProvisionalLayerTransitionResult
            {
                if (!input.build_external_patches) return build();
                const auto local_started = std::chrono::steady_clock::now();
                auto local = input.build_external_patches(
                    retained, face_sets, selected, external_controls);
                if (!local.hasValue()) return local;
                ++local_build_count;
                local_build_nanoseconds +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - local_started)
                            .count());
                const auto replace_started =
                    std::chrono::steady_clock::now();
                replaceExternalPatches(
                    provisional.value(), local.value(), selected);
                configure(provisional.value());
                patch_replace_milliseconds +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - replace_started)
                            .count());
                return ProvisionalLayerTransitionResult::success(
                    std::move(provisional.value()));
            };
            const auto buildLocalProbe = [&](
                const std::vector<SurfaceFaceId> &selected)
                -> ProvisionalLayerTransitionResult
            {
                if (!input.build_external_patches)
                    return buildProbe(selected);
                const auto local_started = std::chrono::steady_clock::now();
                auto local = input.build_external_patches(
                    retained, face_sets, selected, external_controls);
                if (local.hasValue()) configure(local.value());
                ++local_build_count;
                local_build_nanoseconds +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - local_started)
                            .count());
                return local;
            };

            std::vector<SurfaceFaceId> external_faces;
            std::set<SurfaceFaceId> no_high_external_faces;
            for (const auto &topology : provisional.value().resolved_topology)
            {
                if (topology.terminal_quad_decision ==
                    TerminalQuadDecision::ExternalPatch)
                {
                    external_faces.push_back(topology.source_face_id);
                    if (topology.retained_local_edges.empty())
                        no_high_external_faces.insert(
                            topology.source_face_id);
                }
            }
            std::sort(external_faces.begin(), external_faces.end());
            external_faces.erase(std::unique(
                external_faces.begin(), external_faces.end()),
                external_faces.end());
            for (const auto &topology : provisional.value().resolved_topology)
                if (traceTransitionFace(topology.source_face_id))
                {
                    std::cerr << "trace resolver face="
                              << topology.source_face_id
                              << " completed_layer=" << input.completed_layer
                              << " iteration=" << iterations
                              << " decision=" << static_cast<int>(
                                  topology.terminal_quad_decision)
                              << " kind=" << static_cast<int>(topology.template_kind)
                              << " edges=";
                    for (const std::size_t edge : topology.retained_local_edges)
                        std::cerr << edge << ',';
                    std::cerr << " dependencies=";
                    for (const SurfaceFaceId dependency :
                         topology.dependent_high_faces)
                        std::cerr << dependency << ',';
                    std::cerr << " apex=";
                    if (topology.generated_point.has_value())
                    {
                        const auto &point = *topology.generated_point;
                        std::cerr << '(' << point.x() << ',' << point.y()
                                  << ',' << point.z() << ')';
                    }
                    else std::cerr << "none";
                    std::cerr << '\n';
                }
            std::cerr << "temporary resolver iteration=" << iterations
                      << " retained=" << retained.size()
                      << " transition_low="
                      << face_sets.transition_low_faces.size()
                      << " external=" << external_faces.size() << '\n';

            // One global scan identifies which external patches need any
            // further work.  Safe patches must not each rebuild this index.
            const auto initial_scan_started =
                std::chrono::steady_clock::now();
            if (!collision_state.has_value())
            {
                auto built = IncrementalTransitionCollisionState::build(
                    provisional.value().boundary);
                if (!built.hasValue())
                    return ResolveResult::failure(LayerTransitionError{
                        built.error()});
                collision_state = std::move(built.value());
                ++collision_full_builds;
            }
            else
            {
                const auto changed = collision_state->ownersForSources(
                    provisional.value().boundary, rebuilt_sources,
                    rebuilt_sources ? &provisional_delta_triangles : nullptr);
                const auto changed_done = std::chrono::steady_clock::now();
                const auto updated = rebuilt_sources
                    ? collision_state->update(
                          provisional.value().boundary, changed,
                          provisional_delta_triangles)
                    : collision_state->update(
                          provisional.value().boundary, changed);
                if (!updated.hasValue())
                    return ResolveResult::failure(LayerTransitionError{
                        updated.error()});
                ++collision_incremental_updates;
                std::cerr << "temporary resolver changed_boundary_ms="
                    << std::chrono::duration_cast<std::chrono::milliseconds>(
                        changed_done-initial_scan_started).count() << '\n';
            }
            const TransitionCollisionReport &initial_report =
                collision_state->collisionReport();
            std::unordered_set<SurfaceFaceId> colliding_external_faces;
            colliding_external_faces.reserve(
                initial_report.colliding_owners.size());
            for (const LayerBoundaryOwner &owner :
                 initial_report.colliding_owners)
                if (owner.role == BoundaryOwnerRole::ExternalPatch)
                    colliding_external_faces.insert(owner.source_face_id);
            if (std::binary_search(external_faces.begin(), external_faces.end(),
                                   SurfaceFaceId{216659}) ||
                traceTransitionFace(SurfaceFaceId{216659}))
            {
                for (const auto &owner : initial_report.colliding_owners)
                    if (traceTransitionFace(owner.source_face_id))
                        std::cerr << "trace resolver collision-owner face="
                                  << owner.source_face_id
                                  << " layer=" << owner.layer
                                  << " role=" << static_cast<int>(owner.role)
                                  << '\n';
                for (const SurfaceFaceId id : external_faces)
                    if (traceTransitionFace(id))
                        std::cerr << "trace resolver external-face-listed id="
                                  << id << " initial_collision="
                                  << colliding_external_faces.count(id)
                                  << '\n';
            }
#ifndef NDEBUG
            const auto full_initial_report = checker.inspect(
                provisional.value().boundary);
            assert(full_initial_report.hasValue());
            if (!input.regular_candidate_geometry_prevalidated)
                assert(full_initial_report.value().rollback_faces ==
                       initial_report.rollback_faces);
#endif
            std::cerr << "temporary resolver initial_scan_ms="
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() -
                             initial_scan_started).count()
                      << " colliding_owners="
                      << initial_report.colliding_owners.size()
                      << '\n';
            const auto &collision_diagnostics = collision_state->diagnostics();
            std::cerr << "temporary resolver collision_work self_queries="
                      << collision_diagnostics.self_collision_queries
                      << " exact_tests="
                      << collision_diagnostics.self_collision_exact_tests
                      << " candidate_visits="
                      << collision_diagnostics.self_candidate_visits
                      << " duplicate_candidates="
                      << collision_diagnostics.self_duplicate_candidate_visits
                      << " obstacle_queries="
                      << collision_diagnostics.static_obstacle_queries
                      << " reported_primitives="
                      << collision_diagnostics.reported_colliding_primitives
                      << '\n';

            const auto external_search_started =
                std::chrono::steady_clock::now();
            bool external_changed = false;

            struct ExternalSearch
            {
                Scalar low{};
                Scalar high{};
                Scalar probe{};
                bool bracketed = false;
                bool exhausted = false;
                bool apex_candidate_resolved = false;
            };
            std::map<SurfaceFaceId, ExternalSearch> searches;
            for (const SurfaceFaceId id : external_faces)
            {
                if (colliding_external_faces.find(id) ==
                    colliding_external_faces.end())
                {
                    external_controls.distance_scales[id] =
                        external_controls.distanceScale(id);
                    continue;
                }
                const Scalar initial = external_controls.distanceScale(id);
                searches.emplace(
                    id, ExternalSearch{
                        Scalar{}, initial, initial, false, false, false});
                searched_patch_ids.insert(id);
                external_changed = true;
            }

            if (!searches.empty())
            {
                const auto started = std::chrono::steady_clock::now();
                const auto groupForOwner = [&](const LayerBoundaryOwner &owner)
                {
                    if (owner.role == BoundaryOwnerRole::ExternalPatch)
                        return static_cast<CollisionGroupId>(
                            owner.source_face_id) + 2;
                    const auto key = layerBoundaryOwnerKey(owner);
                    auto found = search_groups.find(key);
                    if (found == search_groups.end())
                        found = search_groups.emplace(
                            key, next_search_group++).first;
                    return found->second;
                };
                const auto toCollisionGroup = [](
                    CollisionGroupId group,
                    const std::vector<OwnedBoundaryTriangle> &triangles)
                {
                    CollisionPrimitiveGroup entry{group, {}};
                    entry.triangles.reserve(triangles.size());
                    for (const auto &triangle : triangles)
                        entry.triangles.push_back(makeTransitionCollisionTriangle(
                            triangle, triangle.owner.source_face_id));
                    return entry;
                };
                if (search_index && rebuilt_sources.has_value())
                {
                    std::set<CollisionGroupId> changed_groups;
                    for (const SurfaceFaceId source : *rebuilt_sources)
                    {
                        const auto groups = search_source_groups.find(source);
                        if (groups != search_source_groups.end())
                            changed_groups.insert(
                                groups->second.begin(), groups->second.end());
                    }
                    for (const CollisionGroupId group : changed_groups)
                    {
                        const auto erased = search_index->eraseGroup(group);
                        if (!erased.hasValue())
                            return ResolveResult::failure(LayerTransitionError{
                                TransitionBoundaryError{erased.error()}});
                        const auto old = search_boundary.find(group);
                        if (old != search_boundary.end())
                        {
                            for (const OwnedBoundaryTriangle &triangle :
                                 old->second)
                            {
                                const auto source_groups =
                                    search_source_groups.find(
                                        triangle.owner.source_face_id);
                                if (source_groups == search_source_groups.end())
                                    continue;
                                source_groups->second.erase(group);
                                if (source_groups->second.empty())
                                    search_source_groups.erase(source_groups);
                            }
                            search_boundary.erase(old);
                        }
                    }
                    std::map<CollisionGroupId,
                        std::vector<OwnedBoundaryTriangle>> changed;
                    for (const OwnedBoundaryTriangle &owned :
                         provisional_delta_triangles)
                        changed[groupForOwner(owned.owner)].push_back(owned);
                    for (auto &[group, triangles] : changed)
                    {
                        auto inserted = search_index->insertGroup(
                            toCollisionGroup(group, triangles));
                        if (!inserted.hasValue())
                            return ResolveResult::failure(LayerTransitionError{
                                TransitionBoundaryError{inserted.error()}});
                        for (const OwnedBoundaryTriangle &triangle : triangles)
                            search_source_groups[
                                triangle.owner.source_face_id].insert(group);
                        search_boundary[group] = std::move(triangles);
                    }
                }
                else
                {
                    std::map<CollisionGroupId,
                        std::vector<OwnedBoundaryTriangle>> grouped;
                    for (const OwnedBoundaryTriangle &owned :
                         collision_state->exposedBoundary())
                        grouped[groupForOwner(owned.owner)].push_back(owned);
                    const auto sameGroup = [](const auto &a, const auto &b) {
                        return a.size() == b.size() && std::equal(
                            a.begin(), a.end(), b.begin(),
                            [](const auto &x, const auto &y)
                            { return sameOwnedTriangle(x, y); });
                    };
                    std::vector<CollisionPrimitiveGroup> groups;
                    for (const auto &[group, triangles] : grouped)
                    {
                        const auto old = search_boundary.find(group);
                        if (old != search_boundary.end() &&
                            sameGroup(old->second, triangles))
                            continue;
                        groups.push_back(toCollisionGroup(group, triangles));
                    }
                    if (!search_index)
                    {
                        auto built = IncrementalCollisionIndex::build(
                            std::move(groups));
                        if (!built.hasValue())
                            return ResolveResult::failure(LayerTransitionError{
                                TransitionBoundaryError{built.error()}});
                        search_index = std::move(built.value());
                        ++search_index_full_builds;
                    }
                    else
                    {
                        for (const auto &[group, triangles] : search_boundary)
                        {
                            const auto next = grouped.find(group);
                            if (next != grouped.end() &&
                                sameGroup(triangles, next->second))
                                continue;
                            const auto erased = search_index->eraseGroup(group);
                            if (!erased.hasValue())
                                return ResolveResult::failure(LayerTransitionError{
                                    TransitionBoundaryError{erased.error()}});
                        }
                        for (auto &group : groups)
                        {
                            const auto inserted = search_index->insertGroup(
                                std::move(group));
                            if (!inserted.hasValue())
                                return ResolveResult::failure(LayerTransitionError{
                                    TransitionBoundaryError{inserted.error()}});
                        }
                    }
                    search_boundary = std::move(grouped);
                    search_source_groups.clear();
                    for (const auto &[group, triangles] : search_boundary)
                        for (const OwnedBoundaryTriangle &triangle : triangles)
                            search_source_groups[
                                triangle.owner.source_face_id].insert(group);
                }
                search_index_nanoseconds += std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - started).count();
            }

            using ExternalCollisionResult = Result<
                std::set<SurfaceFaceId>, LayerTransitionError>;
            std::map<SurfaceFaceId,std::vector<TrianglePoints>>
                encountered_collision_faces;
            std::map<SurfaceFaceId, ResolvedTransitionTopology>
                latest_patch_topology;
            std::map<SurfaceFaceId, std::vector<OwnedBoundaryTriangle>>
                latest_patch_triangles;
            struct ProbeCandidateCache
            {
                Aabb envelope;
                std::vector<CollisionPrimitiveId> primitives;
            };
            std::map<SurfaceFaceId, ProbeCandidateCache> probe_candidates;
            std::map<SurfaceFaceId,
                     std::vector<OwnedBoundaryTriangle>> accepted_probe_patches;
            std::optional<IncrementalCollisionIndex> accepted_probe_index;
            const auto rememberLocalProbe = [&](
                const ProvisionalLayerTransition &trial,
                const std::vector<SurfaceFaceId> &selected)
            {
                for (const SurfaceFaceId id : selected)
                {
                    latest_patch_topology.erase(id);
                    latest_patch_triangles.erase(id);
                }
                const std::set<SurfaceFaceId> selected_set(
                    selected.begin(), selected.end());
                for (const auto &entry : trial.resolved_topology)
                    if (selected_set.count(entry.source_face_id) != 0)
                        latest_patch_topology[entry.source_face_id] = entry;
                for (const auto &triangle :
                     trial.boundary.candidate_triangles)
                    if (selected_set.count(
                            triangle.owner.source_face_id) != 0)
                        latest_patch_triangles[
                            triangle.owner.source_face_id].push_back(triangle);
            };
            std::optional<TransitionStaticObstacleContext> probe_static_context;
            const auto inspectExternalChanges =
                [&](const ProvisionalLayerTransition &trial,
                    const std::vector<SurfaceFaceId> &selected,
                    bool collect_contacts = false)
                -> ExternalCollisionResult
            {
                const auto query_started = std::chrono::steady_clock::now();
                std::map<SurfaceFaceId,
                         std::vector<OwnedBoundaryTriangle>> replacements;
                for (const SurfaceFaceId id : selected)
                    if (searches.count(id) != 0)
                        replacements[id];
                for (const OwnedBoundaryTriangle &owned :
                     trial.boundary.candidate_triangles)
                    if (owned.owner.role ==
                            BoundaryOwnerRole::ExternalPatch &&
                        replacements.find(owned.owner.source_face_id) !=
                            replacements.end())
                        replacements[owned.owner.source_face_id].push_back(
                            owned);

                TransitionBoundaryInput changed;
                changed.original_surface = trial.boundary.original_surface;
                changed.historical_boundary = trial.boundary.historical_boundary;
                changed.historical_index = trial.boundary.historical_index;
                changed.historical_index_includes_transition =
                    trial.boundary.historical_index_includes_transition;
                changed.prior_transition_boundary =
                    trial.boundary.prior_transition_boundary;
                changed.prior_transition_index =
                    trial.boundary.prior_transition_index;
                changed.prior_transition_dynamic_index =
                    trial.boundary.prior_transition_dynamic_index;
                changed.sliding_surface = trial.boundary.sliding_surface;
                for (const auto &[id, triangles] : replacements)
                {
                    (void)id;
                    changed.candidate_triangles.insert(
                        changed.candidate_triangles.end(),
                        triangles.begin(), triangles.end());
                }
                std::set<SurfaceFaceId> colliding;
                if (!probe_static_context)
                {
                    auto context = TransitionStaticObstacleContext::build(changed);
                    if (!context.hasValue())
                        return ExternalCollisionResult::failure(LayerTransitionError{context.error()});
                    probe_static_context = std::move(context.value());
                }
                std::set<SurfaceFaceId> static_colliding;
                TransitionStaticObstacleQueryScratch obstacle_scratch;
                for (const auto &owned : changed.candidate_triangles)
                {
                    if (std::find(selected.begin(), selected.end(),
                                  owned.owner.source_face_id) == selected.end())
                        continue;
                    std::vector<TrianglePoints> static_contacts;
                    auto hit = probe_static_context->intersects(
                        owned,
                        collect_contacts ? &static_contacts : nullptr,
                        &obstacle_scratch);
                    if (!hit.hasValue())
                        return ExternalCollisionResult::failure(LayerTransitionError{hit.error()});
                    if (hit.value()) static_colliding.insert(owned.owner.source_face_id);
                    if (collect_contacts)
                    {
                        auto &encountered =
                            encountered_collision_faces[owned.owner.source_face_id];
                        encountered.insert(encountered.end(),
                            static_contacts.begin(), static_contacts.end());
                    }
                }
                colliding = static_colliding;

                std::map<SurfaceFaceId, std::vector<CollisionTriangle>>
                    converted_replacements;
                for (const auto &[id, triangles] : replacements)
                {
                    std::vector<CollisionTriangle> converted;
                    converted.reserve(triangles.size());
                    for (const OwnedBoundaryTriangle &owned : triangles)
                        converted.push_back(makeTransitionCollisionTriangle(
                            owned, id));
                    converted_replacements.emplace(id, std::move(converted));
                }
                std::set<CollisionGroupId> replaced_groups;
                for (const auto &[id, triangles] : accepted_probe_patches)
                {
                    (void)triangles;
                    replaced_groups.insert(
                        static_cast<CollisionGroupId>(id) + 2);
                }
                std::vector<CollisionPrimitiveGroup> trial_groups;
                trial_groups.reserve(selected.size());
                for (const auto &[id, converted] : converted_replacements)
                {
                    const CollisionGroupId group =
                        static_cast<CollisionGroupId>(id) + 2;
                    replaced_groups.insert(group);
                    trial_groups.push_back({group, converted});
                }
                for (const auto &[id, converted] : converted_replacements)
                    for (const auto &triangle : converted)
                    {
                        auto box = makeAabb(triangle.points[0], triangle.points[1], triangle.points[2]);
                        if (!box.hasValue())
                            return ExternalCollisionResult::failure(LayerTransitionError{
                                TransitionBoundaryError{box.error()}});
                        if ((triangle.points[1]-triangle.points[0]).cross(
                                triangle.points[2]-triangle.points[0]).squaredNorm() == Scalar{0})
                            return ExternalCollisionResult::failure(LayerTransitionError{
                                TransitionBoundaryError{SpatialError::DegenerateTriangle}});
                    }
                auto trial_index_result = IncrementalCollisionIndex::build(
                    std::move(trial_groups));
                if (!trial_index_result.hasValue())
                    return ExternalCollisionResult::failure(
                        LayerTransitionError{TransitionBoundaryError{
                            trial_index_result.error()}});
                IncrementalCollisionIndex &trial_index =
                    trial_index_result.value();
                for (const auto &[id, converted] : converted_replacements)
                {
                    std::optional<Aabb> patch_bounds;
                    for (const CollisionTriangle &triangle : converted)
                    {
                        const auto bounds = makeAabb(
                            triangle.points[0], triangle.points[1],
                            triangle.points[2]);
                        if (!bounds.hasValue())
                            return ExternalCollisionResult::failure(
                                LayerTransitionError{TransitionBoundaryError{
                                    bounds.error()}});
                        if (!patch_bounds)
                            patch_bounds = bounds.value();
                        else
                        {
                            patch_bounds->minimum =
                                patch_bounds->minimum.cwiseMin(
                                    bounds.value().minimum);
                            patch_bounds->maximum =
                                patch_bounds->maximum.cwiseMax(
                                    bounds.value().maximum);
                        }
                    }
                    if (!patch_bounds) continue;
                    auto cached = probe_candidates.find(id);
                    if (cached == probe_candidates.end())
                    {
                        ProbeCandidateCache entry;
                        entry.envelope = *patch_bounds;
                        entry.primitives = search_index->queryCandidates(
                            entry.envelope);
                        probe_candidates.emplace(id, std::move(entry));
                        continue;
                    }
                    const bool contained =
                        (cached->second.envelope.minimum.array() <=
                         patch_bounds->minimum.array()).all() &&
                        (cached->second.envelope.maximum.array() >=
                         patch_bounds->maximum.array()).all();
                    if (!contained)
                    {
                        cached->second.envelope.minimum =
                            cached->second.envelope.minimum.cwiseMin(
                                patch_bounds->minimum);
                        cached->second.envelope.maximum =
                            cached->second.envelope.maximum.cwiseMax(
                                patch_bounds->maximum);
                        cached->second.primitives = search_index->queryCandidates(
                            cached->second.envelope);
                    }
                }
                for (const auto &[id, converted] : converted_replacements)
                {
                    const CollisionGroupId group =
                        static_cast<CollisionGroupId>(id) + 2;
                    for (const CollisionTriangle &triangle : converted)
                    {
                        const auto candidates = probe_candidates.find(id);
                        const auto environment_contacts =
                            candidates == probe_candidates.end()
                                ? search_index->queryIllegalContacts(
                                      triangle, replaced_groups)
                                : search_index->queryIllegalContacts(
                                      triangle, candidates->second.primitives,
                                      replaced_groups);
                        const auto moving_contacts =
                            trial_index.queryIllegalContacts(triangle, group);
                        const auto accepted_contacts = accepted_probe_index
                            ? accepted_probe_index->queryIllegalContacts(
                                  triangle, group)
                            : std::vector<CollisionPrimitiveId>{};
                        if (!environment_contacts.empty() ||
                            !moving_contacts.empty() ||
                            !accepted_contacts.empty())
                        {
                            colliding.insert(id);
                        }
                        if (collect_contacts)
                        {
                            for (const auto contact : environment_contacts)
                                encountered_collision_faces[id].push_back(
                                    search_index->primitive(contact).points);
                            for (const auto contact : moving_contacts)
                                encountered_collision_faces[id].push_back(
                                    trial_index.primitive(contact).points);
                            for (const auto contact : accepted_contacts)
                                encountered_collision_faces[id].push_back(
                                    accepted_probe_index->primitive(contact).points);
                        }
                    }
                }
                exact_query_nanoseconds +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now() - query_started)
                            .count());
                return ExternalCollisionResult::success(
                    std::move(colliding));
            };
            const auto rememberAcceptedProbe = [&] (
                const ProvisionalLayerTransition &trial,
                SurfaceFaceId id)
                -> Result<std::monostate, LayerTransitionError>
            {
                std::vector<OwnedBoundaryTriangle> accepted;
                for (const OwnedBoundaryTriangle &owned :
                     trial.boundary.candidate_triangles)
                    if (owned.owner.role == BoundaryOwnerRole::ExternalPatch &&
                        owned.owner.source_face_id == id)
                        accepted.push_back(owned);
                if (accepted.empty())
                    return Result<std::monostate, LayerTransitionError>::success(
                        std::monostate{});

                std::vector<CollisionTriangle> converted;
                converted.reserve(accepted.size());
                for (const OwnedBoundaryTriangle &triangle : accepted)
                    converted.push_back(
                        makeTransitionCollisionTriangle(triangle, id));
                const CollisionGroupId group =
                    static_cast<CollisionGroupId>(id) + 2;
                if (!accepted_probe_index)
                {
                    auto built = IncrementalCollisionIndex::build(
                        {{group, std::move(converted)}});
                    if (!built.hasValue())
                        return Result<std::monostate, LayerTransitionError>::failure(
                            LayerTransitionError{TransitionBoundaryError{
                                built.error()}});
                    accepted_probe_index = std::move(built.value());
                }
                else
                {
                    const auto existing =
                        accepted_probe_patches.find(id);
                    if (existing != accepted_probe_patches.end())
                    {
                        const auto erased = accepted_probe_index->eraseGroup(group);
                        if (!erased.hasValue())
                            return Result<std::monostate, LayerTransitionError>::failure(
                                LayerTransitionError{TransitionBoundaryError{
                                    erased.error()}});
                    }
                    const auto updated = accepted_probe_index->insertGroup(
                        {group, std::move(converted)});
                    if (!updated.hasValue())
                        return Result<std::monostate, LayerTransitionError>::failure(
                            LayerTransitionError{TransitionBoundaryError{
                                updated.error()}});
                }
                accepted_probe_patches[id] = std::move(accepted);
                return Result<std::monostate, LayerTransitionError>::success(
                    std::monostate{});
            };

            constexpr std::size_t maximum_apex_candidates = 8;
            for (auto &[id, search] : searches)
            {
                bool resolved = false;
                for (std::size_t candidate = 1;
                     candidate <= maximum_apex_candidates; ++candidate)
                {
                    external_controls.apex_candidate_indices[id] = candidate;
                    const std::vector<SurfaceFaceId> selected{id};
                    auto trial = buildLocalProbe(selected);
                    ++probe_rounds;
                    if (!trial.hasValue())
                        return ResolveResult::failure(trial.error());
                    const auto topology = std::find_if(
                        trial.value().resolved_topology.begin(),
                        trial.value().resolved_topology.end(),
                        [id](const ResolvedTransitionTopology &value)
                        { return value.source_face_id == id; });
                    if (traceTransitionFace(id))
                    {
                        std::cerr << "trace resolver candidate face=" << id
                                  << " layer=" << input.completed_layer
                                  << " apex_candidate=" << candidate
                                  << " topology="
                                  << (topology != trial.value().resolved_topology.end());
                        if (topology != trial.value().resolved_topology.end())
                        {
                            std::cerr << " decision=" << static_cast<int>(
                                topology->terminal_quad_decision)
                                << " edges=";
                            for (const std::size_t edge :
                                 topology->retained_local_edges)
                                std::cerr << edge << ',';
                            std::cerr << " apex=";
                            if (topology->generated_point.has_value())
                            {
                                const auto &point = *topology->generated_point;
                                std::cerr << '(' << point.x() << ','
                                          << point.y() << ',' << point.z() << ')';
                            }
                            else std::cerr << "none";
                        }
                        std::cerr << '\n';
                    }
                    if (topology == trial.value().resolved_topology.end() ||
                        topology->terminal_quad_decision !=
                            TerminalQuadDecision::ExternalPatch ||
                        !topology->generated_point.has_value())
                    {
                        continue;
                    }

                    const auto collisions = inspectExternalChanges(trial.value(), selected);
                    if (!collisions.hasValue())
                        return ResolveResult::failure(collisions.error());
                    rememberLocalProbe(trial.value(), selected);
                    if (traceTransitionFace(id))
                        std::cerr << "trace resolver candidate face=" << id
                                  << " layer=" << input.completed_layer
                                  << " apex_candidate=" << candidate
                                  << " colliding="
                                  << (collisions.value().count(id) != 0)
                                  << '\n';
                    if (collisions.value().find(id) == collisions.value().end())
                    {
                        const auto remembered =
                            rememberAcceptedProbe(trial.value(), id);
                        if (!remembered.hasValue())
                            return ResolveResult::failure(remembered.error());
                        if (no_high_external_faces.count(id) != 0)
                        {
                            external_controls.explicit_apex_points[id] =
                                *topology->generated_point;
                            external_controls.apex_candidate_indices.erase(id);
                        }
                        search.apex_candidate_resolved = true;
                        search.bracketed = true;
                        search.exhausted = true;
                        search.low = search.high = search.probe =
                            external_controls.distanceScale(id);
                        resolved = true;
                        break;
                    }
                }
                if (!resolved)
                    external_controls.apex_candidate_indices.erase(id);
                if (!resolved && no_high_external_faces.count(id) != 0)
                {
                    for (std::size_t candidate = 1;
                         candidate <= maximum_apex_candidates; ++candidate)
                    {
                        external_controls.robust_candidate_indices[id] =
                            candidate;
                        const std::vector<SurfaceFaceId> selected{id};
                        auto trial = buildLocalProbe(selected);
                        ++probe_rounds;
                        if (!trial.hasValue())
                            return ResolveResult::failure(trial.error());
                        const auto topology = std::find_if(
                            trial.value().resolved_topology.begin(),
                            trial.value().resolved_topology.end(),
                            [id](const ResolvedTransitionTopology &value)
                            { return value.source_face_id == id; });
                        if (topology == trial.value().resolved_topology.end() ||
                            topology->terminal_quad_decision !=
                                TerminalQuadDecision::ExternalPatch ||
                            !topology->generated_point.has_value())
                            continue;
                        const auto collisions = inspectExternalChanges(
                            trial.value(), selected);
                        if (!collisions.hasValue())
                            return ResolveResult::failure(collisions.error());
                        rememberLocalProbe(trial.value(), selected);
                        if (collisions.value().count(id) != 0) continue;
                        const auto remembered =
                            rememberAcceptedProbe(trial.value(), id);
                        if (!remembered.hasValue())
                            return ResolveResult::failure(remembered.error());
                        external_controls.explicit_apex_points[id] =
                            *topology->generated_point;
                        external_controls.robust_candidate_indices.erase(id);
                        search.apex_candidate_resolved = true;
                        search.bracketed = true;
                        search.exhausted = true;
                        search.low = search.high = search.probe =
                            external_controls.distanceScale(id);
                        resolved = true;
                        break;
                    }
                    if (!resolved)
                        external_controls.robust_candidate_indices.erase(id);
                }
            }

            while (std::any_of(
                searches.begin(), searches.end(), [](const auto &entry)
                { return !entry.second.bracketed && !entry.second.exhausted; }))
            {
                std::vector<SurfaceFaceId> active_ids;
                for (auto &[id, search] : searches)
                {
                    if (search.bracketed || search.exhausted) continue;
                    search.probe = std::max(
                        search.probe * Scalar{0.5},
                        minimum_external_distance_scale);
                    external_controls.distance_scales[id] = search.probe;
                    active_ids.push_back(id);
                    if (traceTransitionFace(id))
                        std::cerr << "trace resolver distance-probe face=" << id
                                  << " layer=" << input.completed_layer
                                  << " scale=" << search.probe << '\n';
                }
                auto trial = buildLocalProbe(active_ids);
                ++probe_rounds;
                if (!trial.hasValue())
                    return ResolveResult::failure(trial.error());
                const auto collisions = inspectExternalChanges(trial.value(), active_ids);
                if (!collisions.hasValue())
                    return ResolveResult::failure(collisions.error());
                for (auto &[id, search] : searches)
                {
                    if (search.bracketed || search.exhausted) continue;
                    if (collisions.value().find(id) ==
                        collisions.value().end())
                    {
                        search.low = search.probe;
                        search.bracketed = true;
                        if (traceTransitionFace(id))
                            std::cerr << "trace resolver distance-result face="
                                      << id << " layer=" << input.completed_layer
                                      << " scale=" << search.probe
                                      << " collision=0 bracketed=1\n";
                    }
                    else
                    {
                        search.high = search.probe;
                        search.exhausted =
                            search.probe == minimum_external_distance_scale;
                        if (traceTransitionFace(id))
                            std::cerr << "trace resolver distance-result face="
                                      << id << " layer=" << input.completed_layer
                                      << " scale=" << search.probe
                                      << " collision=1 exhausted="
                                      << search.exhausted << '\n';
                    }
                }
                rememberLocalProbe(trial.value(), active_ids);
            }

            for (auto &[id,search] : searches)
            {
                if (search.bracketed || !no_high_external_faces.count(id) ||
                    search.apex_candidate_resolved)
                    continue;
                const auto topology = latest_patch_topology.find(id);
                if (topology == latest_patch_topology.end() ||
                    !topology->second.generated_point.has_value())
                    continue;
                const Point3 anchor = *topology->second.generated_point;
                const auto previous_contacts =
                    encountered_collision_faces.find(id);
                if (previous_contacts == encountered_collision_faces.end() ||
                    previous_contacts->second.empty())
                {
                    const std::vector<SurfaceFaceId> selected{id};
                    auto trial = buildLocalProbe(selected);
                    ++probe_rounds;
                    if (!trial.hasValue())
                        return ResolveResult::failure(trial.error());
                    const auto collision = inspectExternalChanges(
                        trial.value(), selected, true);
                    if (!collision.hasValue())
                        return ResolveResult::failure(collision.error());
                    rememberLocalProbe(trial.value(), selected);
                }
                std::array<Point3,4> quad{};
                std::size_t quad_count = 0;
                for (const auto &owned : latest_patch_triangles[id])
                {
                    if (owned.owner.role != BoundaryOwnerRole::ExternalPatch ||
                        owned.owner.source_face_id != id) continue;
                    for (const Point3 &point : owned.points)
                    {
                        if ((point-anchor).norm() <= Scalar{1e-9}) continue;
                        bool duplicate = false;
                        for (std::size_t i = 0; i < quad_count; ++i)
                            duplicate = duplicate ||
                                (quad[i]-point).norm() <= Scalar{1e-9};
                        if (!duplicate && quad_count < 4)
                            quad[quad_count++] = point;
                    }
                }
                if (quad_count != 4) continue;
                ApexSolverInput solver_input;
                solver_input.quad = quad;
                solver_input.target = anchor;
                const auto planes = encountered_collision_faces.find(id);
                if (planes == encountered_collision_faces.end()) continue;
                struct PlaneSearchNode
                {
                    ApexSolverInput constraints;
                    TrianglePoints plane{};
                };
                const auto samePlane = [](const TrianglePoints &left,
                                           const TrianglePoints &right)
                {
                    Vector3 a = (left[1]-left[0]).cross(left[2]-left[0]);
                    Vector3 b = (right[1]-right[0]).cross(right[2]-right[0]);
                    if (a.norm() <= 1e-14 || b.norm() <= 1e-14) return false;
                    a.normalize(); b.normalize();
                    const Scalar sign = a.dot(b) < 0 ? Scalar{-1} : Scalar{1};
                    return (a-sign*b).norm() <= Scalar{1e-8} &&
                        std::abs(a.dot(left[0])-sign*b.dot(right[0])) <= 1e-8;
                };
                std::vector<PlaneSearchNode> pending;
                std::vector<TrianglePoints> unique_planes;
                for (const auto &plane : planes->second)
                    if (std::none_of(unique_planes.begin(),unique_planes.end(),
                        [&](const TrianglePoints &other)
                        { return samePlane(plane,other); }))
                        unique_planes.push_back(plane);
                for (const auto &plane : unique_planes)
                    pending.push_back({solver_input,plane});
                std::size_t attempts = 0;
                while (!pending.empty() && attempts < 128 &&
                       !search.apex_candidate_resolved)
                {
                    PlaneSearchNode node = std::move(pending.back());
                    pending.pop_back();
                const auto candidates = projectPyramidApexToPlaneSides(
                        node.constraints,anchor,node.plane,14);
                    for (const Point3 &candidate : candidates)
                    {
                        if (++attempts > 128) break;
                        const std::size_t old_plane_count =
                            encountered_collision_faces[id].size();
                        external_controls.explicit_apex_points[id] = candidate;
                        const std::vector<SurfaceFaceId> selected{id};
                        auto trial = buildLocalProbe(selected);
                        ++probe_rounds;
                        if (!trial.hasValue())
                            return ResolveResult::failure(trial.error());
                        const auto collision = inspectExternalChanges(
                            trial.value(), selected, true);
                        if (!collision.hasValue())
                            return ResolveResult::failure(collision.error());
                        rememberLocalProbe(trial.value(), selected);
                        if (collision.value().count(id) == 0)
                        {
                            const auto remembered =
                                rememberAcceptedProbe(trial.value(), id);
                            if (!remembered.hasValue())
                                return ResolveResult::failure(
                                    remembered.error());
                            search.apex_candidate_resolved = true;
                            search.bracketed = true;
                            search.exhausted = true;
                            search.low = search.high = search.probe =
                                external_controls.distanceScale(id);
                            break;
                        }
                        ApexSolverInput child = node.constraints;
                        Vector3 normal = (node.plane[1]-node.plane[0]).cross(
                            node.plane[2]-node.plane[0]).normalized();
                        const Scalar side = normal.dot(candidate-node.plane[0]) >= 0
                            ? Scalar{1} : Scalar{-1};
                        const Scalar epsilon = std::max(
                            (quad[1]-quad[0]).norm(),Scalar{1})*1e-9;
                        child.additional_constraints.push_back({
                            side*normal,side*normal.dot(node.plane[0])+epsilon});
                        const auto &updated_planes = encountered_collision_faces[id];
                        for (std::size_t index = old_plane_count;
                             index < updated_planes.size(); ++index)
                        {
                            bool already_constrained = samePlane(
                                updated_planes[index],node.plane);
                            const Vector3 new_normal =
                                (updated_planes[index][1]-updated_planes[index][0])
                                .cross(updated_planes[index][2]-updated_planes[index][0])
                                .normalized();
                            for (const ApexConstraint &constraint :
                                 child.additional_constraints)
                            {
                                const Scalar aligned = constraint.normal.dot(new_normal);
                                if (std::abs(std::abs(aligned)-1) <= Scalar{1e-8} &&
                                    std::abs(constraint.normal.dot(
                                        updated_planes[index][0])-constraint.offset) <=
                                        Scalar{1e-7})
                                    already_constrained = true;
                            }
                            if (!already_constrained)
                                pending.push_back({child,updated_planes[index]});
                        }
                    }
                }
                if (!search.apex_candidate_resolved)
                    external_controls.explicit_apex_points.erase(id);
            }

            bool keep_hexa_changed = false;
            for (const auto &[id, search] : searches)
            {
                if (search.bracketed) continue;
                const auto topology = latest_patch_topology.find(id);
                if (topology != latest_patch_topology.end() &&
                    !topology->second.dependent_high_faces.empty())
                    continue;
                external_controls.keep_hexa_faces.push_back(id);
                keep_hexa_changed = true;
                if (traceTransitionFace(id))
                    std::cerr << "trace resolver keep-hexa face=" << id
                              << " layer=" << input.completed_layer
                              << " bracketed=" << search.bracketed
                              << " apex_resolved="
                              << search.apex_candidate_resolved
                              << " dependency_count="
                              << (topology == latest_patch_topology.end()
                                      ? 0
                                      : topology->second.dependent_high_faces.size())
                              << '\n';
            }
            if (keep_hexa_changed)
            {
                std::sort(external_controls.keep_hexa_faces.begin(),
                          external_controls.keep_hexa_faces.end());
                external_controls.keep_hexa_faces.erase(std::unique(
                    external_controls.keep_hexa_faces.begin(),
                    external_controls.keep_hexa_faces.end()),
                    external_controls.keep_hexa_faces.end());
            }

            for (std::uint32_t step = 0; step < 12; ++step)
            {
                bool active = false;
                std::vector<SurfaceFaceId> active_ids;
                for (auto &[id, search] : searches)
                {
                    if (!search.bracketed || search.apex_candidate_resolved)
                        continue;
                    active = true;
                    external_controls.distance_scales[id] =
                        (search.low + search.high) * Scalar{0.5};
                    active_ids.push_back(id);
                }
                if (!active) break;
                auto trial = buildLocalProbe(active_ids);
                ++probe_rounds;
                if (!trial.hasValue())
                    return ResolveResult::failure(trial.error());
                const auto collisions = inspectExternalChanges(trial.value(), active_ids);
                if (!collisions.hasValue())
                    return ResolveResult::failure(collisions.error());
                for (auto &[id, search] : searches)
                {
                    if (!search.bracketed || search.apex_candidate_resolved)
                        continue;
                    const auto scale = external_controls.distance_scales.find(id);
                    if (scale == external_controls.distance_scales.end())
                        continue;
                    const Scalar middle = scale->second;
                    if (collisions.value().find(id) !=
                        collisions.value().end())
                        search.high = middle;
                    else
                        search.low = middle;
                }
                rememberLocalProbe(trial.value(), active_ids);
            }
            for (const auto &[id, search] : searches)
                if (search.bracketed && !search.apex_candidate_resolved)
                    external_controls.distance_scales[id] = search.low;
            if (!searches.empty() || keep_hexa_changed)
            {
                std::vector<SurfaceFaceId> selected;
                for (const auto &[id, search] : searches)
                    selected.push_back(id);
                provisional = buildProbe(selected);
                if (!provisional.hasValue())
                    return ResolveResult::failure(provisional.error());
            }

            external_search_nanoseconds +=
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() -
                        external_search_started).count());
            const auto rollback_started = std::chrono::steady_clock::now();
            std::vector<SurfaceFaceId> rollback;
            if (external_changed)
            {
                std::optional<std::set<SurfaceFaceId>> changed_sources;
                if (input.build_external_patches)
                {
                    changed_sources.emplace();
                    for (const auto &[id, search] : searches) changed_sources->insert(id);
                }
                std::vector<OwnedBoundaryTriangle> external_delta_triangles;
                if (changed_sources)
                    for (const SurfaceFaceId source : *changed_sources)
                    {
                        const auto patch = latest_patch_triangles.find(source);
                        if (patch != latest_patch_triangles.end())
                            external_delta_triangles.insert(
                                external_delta_triangles.end(),
                                patch->second.begin(), patch->second.end());
                    }
                const auto changed = collision_state->ownersForSources(
                    provisional.value().boundary, changed_sources,
                    changed_sources ? &external_delta_triangles : nullptr);
                const auto updated = changed_sources
                    ? collision_state->update(
                          provisional.value().boundary, changed,
                          external_delta_triangles)
                    : collision_state->update(
                          provisional.value().boundary, changed);
                if (!updated.hasValue())
                    return ResolveResult::failure(LayerTransitionError{
                        updated.error()});
                ++collision_incremental_updates;
                rollback = collision_state->collisionReport().rollback_faces;
#ifndef NDEBUG
                const auto full_final_report = checker.inspect(
                    provisional.value().boundary);
                assert(full_final_report.hasValue());
                assert(full_final_report.value().rollback_faces == rollback);
#endif
            }
            else
                rollback = initial_report.rollback_faces;
            rollback.insert(rollback.end(),
                provisional.value().forced_rollback_high_faces.begin(),
                provisional.value().forced_rollback_high_faces.end());
            std::sort(rollback.begin(),rollback.end());
            rollback.erase(std::unique(rollback.begin(),rollback.end()),
                           rollback.end());
            for (const SurfaceFaceId watched : {SurfaceFaceId{298895},
                                                 SurfaceFaceId{445377}})
                if (traceTransitionFace(watched))
                {
                    const auto is_present = [watched](const auto &values)
                    { return std::binary_search(values.begin(), values.end(), watched); };
                    std::cerr << "trace resolver rollback layer="
                              << input.completed_layer
                              << " iteration=" << iterations
                              << " source=" << watched
                              << " collision_report="
                              << is_present(collision_state->collisionReport().rollback_faces)
                              << " forced_rollback="
                              << (std::find(
                                      provisional.value().forced_rollback_high_faces.begin(),
                                      provisional.value().forced_rollback_high_faces.end(),
                                      watched) != provisional.value().forced_rollback_high_faces.end())
                              << " final=" << is_present(rollback)
                              << '\n';
                }
            rollback_coordination_nanoseconds +=
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now() -
                        rollback_started).count());
            if (rollback.empty())
            {
                const TransitionCollisionReport &final_report =
                    collision_state->collisionReport();
                if (!final_report.colliding_owners.empty())
                {
                    std::vector<SurfaceFaceId> fallback_faces;
                    for (const LayerBoundaryOwner &owner :
                         final_report.colliding_owners)
                    {
                        const SurfaceFaceId id = owner.source_face_id;
                        if (owner.role == BoundaryOwnerRole::ExternalPatch &&
                            !external_controls.keepHexa(id))
                        {
                            external_controls.keep_hexa_faces.insert(
                                std::lower_bound(
                                    external_controls.keep_hexa_faces.begin(),
                                    external_controls.keep_hexa_faces.end(), id),
                                id);
                            external_controls.explicit_apex_points.erase(id);
                            external_controls.apex_candidate_indices.erase(id);
                            external_controls.robust_candidate_indices.erase(id);
                            fallback_faces.push_back(id);
                        }
                        else if (owner.role == BoundaryOwnerRole::ExternalPatch &&
                                 !external_controls.forceKeepHexa(id))
                        {
                            external_controls.force_keep_hexa_faces.insert(
                                std::lower_bound(
                                    external_controls.force_keep_hexa_faces.begin(),
                                    external_controls.force_keep_hexa_faces.end(), id),
                                id);
                            fallback_faces.push_back(id);
                        }
                        else if (owner.role == BoundaryOwnerRole::TopCap &&
                                 !external_controls.forceKeepHexa(id))
                        {
                            external_controls.force_keep_hexa_faces.insert(
                                std::lower_bound(
                                    external_controls.force_keep_hexa_faces.begin(),
                                    external_controls.force_keep_hexa_faces.end(), id),
                                id);
                            if (!external_controls.keepHexa(id))
                                external_controls.keep_hexa_faces.insert(
                                    std::lower_bound(
                                        external_controls.keep_hexa_faces.begin(),
                                        external_controls.keep_hexa_faces.end(), id),
                                    id);
                            external_controls.explicit_apex_points.erase(id);
                            external_controls.apex_candidate_indices.erase(id);
                            external_controls.robust_candidate_indices.erase(id);
                            fallback_faces.push_back(id);
                        }
                    }
                    if (!fallback_faces.empty())
                    {
                        std::sort(fallback_faces.begin(), fallback_faces.end());
                        fallback_faces.erase(std::unique(
                            fallback_faces.begin(), fallback_faces.end()),
                            fallback_faces.end());
                        previous_retained = retained;
                        previous_low = face_sets.transition_low_faces;
                        std::sort(previous_low.begin(), previous_low.end());
                        reset_control_faces = fallback_faces;
                        cached_provisional = std::move(provisional.value());
                        std::cerr << "temporary resolver collision fallback layer="
                                  << input.completed_layer
                                  << " faces=" << fallback_faces.size()
                                  << " next=internal-or-keep-hexa\n";
                        ++iterations;
                        continue;
                    }
                    const auto failure =
                        collision_state->failureDiagnostics();
                    std::cerr << "temporary resolver unresolved layer="
                        << input.completed_layer
                        << " colliding_owners="
                        << final_report.colliding_owners.size()
                        << " role_counts="
                        << failure.owners_by_role[0] << ','
                        << failure.owners_by_role[1] << ','
                        << failure.owners_by_role[2] << ','
                        << failure.owners_by_role[3]
                        << " static_hit_owners="
                        << failure.owners_with_static_contacts
                        << " self_contact_owners="
                        << failure.owners_with_self_contacts
                        << " self_contact_owner_pairs="
                        << failure.self_contact_owner_pairs << '\n';
                    for (const LayerBoundaryOwner &owner :
                         failure.owner_samples)
                    {
                        std::cerr << "temporary resolver unresolved_owner"
                            << " source_face=" << owner.source_face_id
                            << " layer=" << owner.layer
                            << " role=" << static_cast<int>(owner.role)
                            << " rollback_dependencies="
                            << owner.rollback_high_faces.size() << '\n';
                    }
                    for (const auto &[left, right] :
                         failure.self_contact_samples)
                    {
                        std::cerr << "temporary resolver unresolved_pair"
                            << " left=" << left.source_face_id << ':'
                            << left.layer << ':'
                            << static_cast<int>(left.role)
                            << " right=" << right.source_face_id << ':'
                            << right.layer << ':'
                            << static_cast<int>(right.role) << '\n';
                    }
                    UnresolvedTransitionCollision unresolved;
                    for (const LayerBoundaryOwner &owner :
                         final_report.colliding_owners)
                        unresolved.owners.push_back(
                            layerBoundaryOwnerKey(owner));
                    return ResolveResult::failure(LayerTransitionError{
                        TransitionBoundaryError{std::move(unresolved)}});
                }
                std::cerr
                    << "temporary resolver external full_builds="
                    << full_build_count
                    << " full_build_ms=" << full_build_milliseconds
                    << " local_builds=" << local_build_count
                    << " local_build_ms=" << local_build_nanoseconds / 1000000
                    << " replace_ms=" << patch_replace_milliseconds
                    << " query_ms=" << exact_query_nanoseconds / 1000000
                    << " searched_patches=" << searched_patch_ids.size()
                    << " probe_rounds=" << probe_rounds << '\n';
                std::cerr << "temporary resolver coordination corner_suppression_ms="
                    << corner_suppression_nanoseconds / 1000000
                    << " prior_index_build_ms="
                    << prior_transition_index_build_nanoseconds / 1000000
                    << " external_search_ms="
                    << external_search_nanoseconds / 1000000
                    << " rollback_ms="
                    << rollback_coordination_nanoseconds / 1000000
                    << '\n';
                std::cerr << "temporary resolver search_index full_builds="
                    << search_index_full_builds << " update_ms="
                    << search_index_nanoseconds / 1000000
                    << " transition_local_rebuilds=" << provisional_local_rebuilds << '\n';
                StableLayerTransition stable;
                stable.face_sets = std::move(face_sets);
                stable.retained_high_faces = std::move(retained);
                stable.exposed_boundary = collision_state->exposedBoundary();
                stable.resolved_topology = std::move(
                    provisional.value().resolved_topology);
                stable.iterations = iterations;
                stable.all_top_faces_are_triangles =
                    provisional.value().all_top_faces_are_triangles;
                stable.collision_full_builds = collision_full_builds;
                stable.collision_incremental_updates =
                    collision_incremental_updates;
                stable.provisional_full_builds = full_build_count;
                stable.provisional_local_rebuilds = provisional_local_rebuilds;
                stable.search_index_full_builds = search_index_full_builds;
                return ResolveResult::success(std::move(stable));
            }

            previous_retained = retained;
            previous_low = face_sets.transition_low_faces;
            std::sort(previous_low.begin(), previous_low.end());
            reset_control_faces.clear();
            for (const auto &[id, scale] : external_controls.distance_scales)
                if (scale != Scalar{0.25}) reset_control_faces.push_back(id);
            for (const auto &[id, value] : external_controls.apex_candidate_indices)
                if (value != 0) reset_control_faces.push_back(id);
            for (const auto &[id, value] : external_controls.robust_candidate_indices)
                if (value != 0) reset_control_faces.push_back(id);
            for (const auto &[id, value] : external_controls.explicit_apex_points)
            { (void)value; reset_control_faces.push_back(id); }
            reset_control_faces.insert(reset_control_faces.end(),
                external_controls.keep_hexa_faces.begin(), external_controls.keep_hexa_faces.end());
            cached_provisional = std::move(provisional.value());
            bool changed = false;
            for (const SurfaceFaceId id : rollback)
            {
                const auto position = std::lower_bound(
                    retained.begin(), retained.end(), id);
                if (position == retained.end() || *position != id)
                    continue;
                retained.erase(position);
                addCollisionRollback(
                    face_sets,
                    {id, input.completed_layer,
                     StopOrigin::TransitionCollision});
                changed = true;
            }
            if (!changed)
                return ResolveResult::failure(LayerTransitionError{
                    TransitionCoordinationError{
                        MissingTransitionFaceState{
                            rollback.front()}}});
            std::set<SurfaceFaceId> invalidated_control_faces;
            if (input.affected_transition_faces)
            {
                const auto affected =
                    input.affected_transition_faces(rollback);
                invalidated_control_faces.insert(
                    affected.begin(), affected.end());
            }
            else
            {
                for (const auto &[id, unused] :
                     external_controls.distance_scales)
                { (void)unused; invalidated_control_faces.insert(id); }
                for (const auto &[id, unused] :
                     external_controls.apex_candidate_indices)
                { (void)unused; invalidated_control_faces.insert(id); }
                for (const auto &[id, unused] :
                     external_controls.robust_candidate_indices)
                { (void)unused; invalidated_control_faces.insert(id); }
                for (const auto &[id, unused] :
                     external_controls.explicit_apex_points)
                { (void)unused; invalidated_control_faces.insert(id); }
                invalidated_control_faces.insert(
                    external_controls.keep_hexa_faces.begin(),
                    external_controls.keep_hexa_faces.end());
            }
            for (const auto &topology : provisional.value().resolved_topology)
                if (std::any_of(
                        topology.dependent_high_faces.begin(),
                        topology.dependent_high_faces.end(),
                        [&](SurfaceFaceId id)
                        { return std::binary_search(
                            rollback.begin(), rollback.end(), id); }))
                    invalidated_control_faces.insert(
                        topology.source_face_id);
            for (const auto &[source, dependencies] :
                 provisional.value().forced_rollback_by_source)
                if (std::any_of(
                        dependencies.begin(), dependencies.end(),
                        [&](SurfaceFaceId id)
                        { return std::binary_search(
                            rollback.begin(), rollback.end(), id); }))
                    invalidated_control_faces.insert(source);
            invalidated_control_faces.insert(
                rollback.begin(), rollback.end());

            reset_control_faces.clear();
            const auto eraseInvalidated = [&](auto &values)
            {
                for (auto iterator = values.begin();
                     iterator != values.end();)
                {
                    if (invalidated_control_faces.count(iterator->first) == 0)
                    {
                        ++iterator;
                        continue;
                    }
                    reset_control_faces.push_back(iterator->first);
                    iterator = values.erase(iterator);
                }
            };
            eraseInvalidated(external_controls.distance_scales);
            eraseInvalidated(external_controls.apex_candidate_indices);
            eraseInvalidated(external_controls.robust_candidate_indices);
            eraseInvalidated(external_controls.explicit_apex_points);
            external_controls.keep_hexa_faces.erase(
                std::remove_if(
                    external_controls.keep_hexa_faces.begin(),
                    external_controls.keep_hexa_faces.end(),
                    [&](SurfaceFaceId id)
                    {
                        const bool invalidated =
                            invalidated_control_faces.count(id) != 0;
                        if (invalidated)
                            reset_control_faces.push_back(id);
                        return invalidated;
                    }),
                external_controls.keep_hexa_faces.end());
            std::sort(reset_control_faces.begin(), reset_control_faces.end());
            reset_control_faces.erase(std::unique(
                reset_control_faces.begin(), reset_control_faces.end()),
                reset_control_faces.end());
            ++iterations;
        }
    }
}
