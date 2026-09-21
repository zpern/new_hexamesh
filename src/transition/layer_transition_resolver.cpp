#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <set>
#include <utility>

#include <boundary_mesh/transition/layer_transition_resolver.hpp>
#include <boundary_mesh/transition/incremental_transition_collision_state.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar minimum_external_distance_scale{1e-6};

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

    bool ExternalPatchControls::keepHexa(SurfaceFaceId id) const
    {
        return std::binary_search(
            keep_hexa_faces.begin(), keep_hexa_faces.end(), id);
    }

    namespace
    {
        GrowthFront retainFaces(
            const GrowthFront &front,
            const std::vector<SurfaceFaceId> &retained)
        {
            GrowthFront result = front;
            result.faces.clear();
            result.source_face_ids.clear();
            for (std::size_t index = 0;
                 index < front.source_face_ids.size(); ++index)
            {
                if (!std::binary_search(
                        retained.begin(), retained.end(),
                        front.source_face_ids[index]))
                    continue;
                result.faces.push_back(front.faces[index]);
                result.source_face_ids.push_back(
                    front.source_face_ids[index]);
            }
            return result;
        }

        ProvisionalLayerTransition replaceExternalPatches(
            const ProvisionalLayerTransition &base,
            const ProvisionalLayerTransition &replacement,
            const std::vector<SurfaceFaceId> &selected)
        {
            const auto isSelected = [&](SurfaceFaceId id)
            {
                return std::find(selected.begin(), selected.end(), id) !=
                    selected.end();
            };
            ProvisionalLayerTransition result = base;
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
                base.all_top_faces_are_triangles &&
                replacement.all_top_faces_are_triangles;
            return result;
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
            input.candidate_front.source_face_ids;
        std::sort(retained.begin(), retained.end());
        retained.erase(std::unique(retained.begin(), retained.end()),
                       retained.end());
        std::uint32_t iterations = 1;
        TransitionBoundaryChecker checker;
        ExternalPatchControls external_controls;
        std::uint64_t full_build_count{};
        std::uint64_t full_build_milliseconds{};
        std::uint64_t local_build_count{};
        std::uint64_t local_build_milliseconds{};
        std::uint64_t patch_replace_milliseconds{};
        std::uint64_t exact_query_milliseconds{};
        std::uint64_t probe_rounds{};
        std::set<SurfaceFaceId> searched_patch_ids;
        std::optional<IncrementalTransitionCollisionState> collision_state;
        std::optional<TransitionBoundaryInput> collision_boundary;
        std::uint64_t collision_full_builds{};
        std::uint64_t collision_incremental_updates{};

        while (true)
        {
            const auto iteration_started = std::chrono::steady_clock::now();
            const auto suppression = applyCornerSuppression({
                input.current_front,
                retainFaces(input.candidate_front, retained),
                face_sets,
                input.completed_layer,
                input.length_tolerance});
            if (!suppression.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    suppression.error()});
            face_sets = suppression.value().face_sets;
            retained = suppression.value().retained_high_faces;

            const auto configure = [&](ProvisionalLayerTransition &value)
            {
                value.boundary.original_surface =
                    input.original_surface.has_value()
                        ? &*input.original_surface
                        : nullptr;
                value.boundary.historical_boundary =
                    input.historical_boundary;
                value.boundary.historical_index = input.historical_boundary
                    ? &input.historical_boundary->collisionIndex()
                    : nullptr;
                value.boundary.prior_transition_boundary =
                    input.prior_transition_boundary;
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
            auto provisional = build();
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
                local_build_milliseconds +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - local_started)
                            .count());
                const auto replace_started =
                    std::chrono::steady_clock::now();
                auto merged = replaceExternalPatches(
                    provisional.value(), local.value(), selected);
                configure(merged);
                patch_replace_milliseconds +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - replace_started)
                            .count());
                return ProvisionalLayerTransitionResult::success(
                    std::move(merged));
            };

            std::vector<SurfaceFaceId> external_faces;
            for (const auto &topology : provisional.value().resolved_topology)
                if (topology.terminal_quad_decision ==
                    TerminalQuadDecision::ExternalPatch)
                    external_faces.push_back(topology.source_face_id);
            std::sort(external_faces.begin(), external_faces.end());
            external_faces.erase(std::unique(
                external_faces.begin(), external_faces.end()),
                external_faces.end());
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
                const auto changed = changedBoundaryOwners(
                    *collision_boundary, provisional.value().boundary);
                const auto updated = collision_state->update(
                    provisional.value().boundary, changed);
                if (!updated.hasValue())
                    return ResolveResult::failure(LayerTransitionError{
                        updated.error()});
                ++collision_incremental_updates;
            }
            collision_boundary = provisional.value().boundary;
            const TransitionCollisionReport initial_report =
                collision_state->collisionReport();
#ifndef NDEBUG
            const auto full_initial_report = checker.inspect(
                provisional.value().boundary);
            assert(full_initial_report.hasValue());
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

            bool external_changed = false;

            struct ExternalSearch
            {
                Scalar low{};
                Scalar high{};
                Scalar probe{};
                bool bracketed = false;
                bool exhausted = false;
            };
            std::map<SurfaceFaceId, ExternalSearch> searches;
            const auto isColliding = [](const auto &owners, SurfaceFaceId id)
            {
                return std::any_of(
                    owners.begin(), owners.end(),
                    [&](const LayerBoundaryOwner &owner)
                    {
                        return owner.role == BoundaryOwnerRole::ExternalPatch &&
                               owner.source_face_id == id;
                    });
            };
            for (const SurfaceFaceId id : external_faces)
            {
                if (!isColliding(
                    initial_report.colliding_owners,
                    id))
                {
                    external_controls.distance_scales[id] =
                        external_controls.distanceScale(id);
                    continue;
                }
                const Scalar initial = external_controls.distanceScale(id);
                searches.emplace(
                    id, ExternalSearch{Scalar{}, initial, initial, false, false});
                searched_patch_ids.insert(id);
                external_changed = true;
            }

            std::optional<IncrementalCollisionIndex> search_index;
            if (!searches.empty())
            {
                const auto assembled = checker.assembleExposedBoundary(
                    provisional.value().boundary);
                if (!assembled.hasValue())
                    return ResolveResult::failure(
                        LayerTransitionError{assembled.error()});
                std::map<CollisionGroupId, std::vector<CollisionTriangle>>
                    grouped;
                for (const OwnedBoundaryTriangle &owned : assembled.value())
                {
                    const CollisionGroupId group =
                        owned.owner.role == BoundaryOwnerRole::ExternalPatch
                        ? static_cast<CollisionGroupId>(
                              owned.owner.source_face_id) + 2
                        : CollisionGroupId{1};
                    grouped[group].push_back(makeTransitionCollisionTriangle(
                        owned, owned.owner.source_face_id));
                }
                std::vector<CollisionPrimitiveGroup> groups;
                for (auto &[group, triangles] : grouped)
                    groups.push_back({group, std::move(triangles)});
                auto built = IncrementalCollisionIndex::build(
                    std::move(groups));
                if (!built.hasValue())
                    return ResolveResult::failure(LayerTransitionError{
                        TransitionBoundaryError{built.error()}});
                search_index = std::move(built.value());
            }

            using ExternalCollisionResult = Result<
                std::set<SurfaceFaceId>, LayerTransitionError>;
            const auto inspectExternalChanges =
                [&](const ProvisionalLayerTransition &trial)
                -> ExternalCollisionResult
            {
                const auto query_started = std::chrono::steady_clock::now();
                std::map<SurfaceFaceId,
                         std::vector<OwnedBoundaryTriangle>> replacements;
                for (const OwnedBoundaryTriangle &owned :
                     trial.boundary.candidate_triangles)
                    if (owned.owner.role ==
                            BoundaryOwnerRole::ExternalPatch &&
                        searches.find(owned.owner.source_face_id) !=
                            searches.end())
                        replacements[owned.owner.source_face_id].push_back(
                            owned);

                TransitionBoundaryInput changed = trial.boundary;
                changed.diagonal_requirements.clear();
                changed.candidate_triangles.clear();
                for (const auto &[id, triangles] : replacements)
                    changed.candidate_triangles.insert(
                        changed.candidate_triangles.end(),
                        triangles.begin(), triangles.end());
                const auto obstacle_report = checker.inspect(changed);
                if (!obstacle_report.hasValue())
                    return ExternalCollisionResult::failure(
                        LayerTransitionError{obstacle_report.error()});

                std::set<SurfaceFaceId> colliding;
                for (const LayerBoundaryOwner &owner :
                     obstacle_report.value().colliding_owners)
                    if (owner.role == BoundaryOwnerRole::ExternalPatch)
                        colliding.insert(owner.source_face_id);

                IncrementalCollisionIndex working_index = *search_index;
                for (const auto &[id, triangles] : replacements)
                {
                    const CollisionGroupId group =
                        static_cast<CollisionGroupId>(id) + 2;
                    const auto erased = working_index.eraseGroup(group);
                    if (!erased.hasValue())
                        return ExternalCollisionResult::failure(
                            LayerTransitionError{
                                TransitionBoundaryError{erased.error()}});
                }
                std::map<SurfaceFaceId, std::vector<CollisionTriangle>>
                    converted_replacements;
                for (const auto &[id, triangles] : replacements)
                {
                    const CollisionGroupId group =
                        static_cast<CollisionGroupId>(id) + 2;
                    std::vector<CollisionTriangle> converted;
                    converted.reserve(triangles.size());
                    for (const OwnedBoundaryTriangle &owned : triangles)
                        converted.push_back(makeTransitionCollisionTriangle(
                            owned, id));
                    const auto inserted = working_index.insertGroup(
                        {group, converted});
                    if (!inserted.hasValue())
                        return ExternalCollisionResult::failure(
                            LayerTransitionError{
                                TransitionBoundaryError{inserted.error()}});
                    converted_replacements.emplace(id, std::move(converted));
                }
                for (const auto &[id, converted] : converted_replacements)
                {
                    const CollisionGroupId group =
                        static_cast<CollisionGroupId>(id) + 2;
                    for (const CollisionTriangle &triangle : converted)
                        if (!working_index.queryIllegalContacts(
                                triangle, group).empty())
                        {
                            colliding.insert(id);
                            break;
                        }
                }
                search_index = std::move(working_index);
                exact_query_milliseconds +=
                    static_cast<std::uint64_t>(
                        std::chrono::duration_cast<
                            std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - query_started)
                            .count());
                return ExternalCollisionResult::success(
                    std::move(colliding));
            };

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
                }
                auto trial = buildProbe(active_ids);
                ++probe_rounds;
                if (!trial.hasValue())
                    return ResolveResult::failure(trial.error());
                const auto collisions = inspectExternalChanges(trial.value());
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
                    }
                    else
                    {
                        search.high = search.probe;
                        search.exhausted =
                            search.probe == minimum_external_distance_scale;
                    }
                }
                provisional = std::move(trial);
            }

            bool keep_hexa_changed = false;
            for (const auto &[id, search] : searches)
            {
                if (search.bracketed) continue;
                const auto topology = std::find_if(
                    provisional.value().resolved_topology.begin(),
                    provisional.value().resolved_topology.end(),
                    [&](const ResolvedTransitionTopology &value)
                    { return value.source_face_id == id; });
                if (topology != provisional.value().resolved_topology.end() &&
                    !topology->dependent_high_faces.empty())
                    continue;
                external_controls.keep_hexa_faces.push_back(id);
                keep_hexa_changed = true;
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
                    if (!search.bracketed) continue;
                    active = true;
                    external_controls.distance_scales[id] =
                        (search.low + search.high) * Scalar{0.5};
                    active_ids.push_back(id);
                }
                if (!active) break;
                auto trial = buildProbe(active_ids);
                ++probe_rounds;
                if (!trial.hasValue())
                    return ResolveResult::failure(trial.error());
                const auto collisions = inspectExternalChanges(trial.value());
                if (!collisions.hasValue())
                    return ResolveResult::failure(collisions.error());
                for (auto &[id, search] : searches)
                {
                    if (!search.bracketed) continue;
                    const Scalar middle =
                        external_controls.distance_scales[id];
                    if (collisions.value().find(id) !=
                        collisions.value().end())
                        search.high = middle;
                    else
                        search.low = middle;
                }
                provisional = std::move(trial);
            }
            for (const auto &[id, search] : searches)
                if (search.bracketed)
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

            std::vector<SurfaceFaceId> rollback;
            if (external_changed)
            {
                const auto final_report = checker.inspect(
                    provisional.value().boundary);
                if (!final_report.hasValue())
                    return ResolveResult::failure(LayerTransitionError{
                        final_report.error()});
                rollback = final_report.value().rollback_faces;
            }
            else
                rollback = initial_report.rollback_faces;
            rollback.insert(rollback.end(),
                provisional.value().forced_rollback_high_faces.begin(),
                provisional.value().forced_rollback_high_faces.end());
            std::sort(rollback.begin(),rollback.end());
            rollback.erase(std::unique(rollback.begin(),rollback.end()),
                           rollback.end());
            if (rollback.empty())
            {
                std::cerr
                    << "temporary resolver external full_builds="
                    << full_build_count
                    << " full_build_ms=" << full_build_milliseconds
                    << " local_builds=" << local_build_count
                    << " local_build_ms=" << local_build_milliseconds
                    << " replace_ms=" << patch_replace_milliseconds
                    << " query_ms=" << exact_query_milliseconds
                    << " searched_patches=" << searched_patch_ids.size()
                    << " probe_rounds=" << probe_rounds << '\n';
                StableLayerTransition stable;
                stable.face_sets = std::move(face_sets);
                stable.retained_high_faces = std::move(retained);
                if (!external_changed)
                    stable.exposed_boundary = collision_state->exposedBoundary();
                else
                {
                    const auto exposed = checker.assembleExposedBoundary(
                        provisional.value().boundary);
                    if (!exposed.hasValue())
                        return ResolveResult::failure(LayerTransitionError{
                            exposed.error()});
                    stable.exposed_boundary = exposed.value();
                }
                stable.resolved_topology = std::move(
                    provisional.value().resolved_topology);
                stable.iterations = iterations;
                stable.all_top_faces_are_triangles =
                    provisional.value().all_top_faces_are_triangles;
                stable.collision_full_builds = collision_full_builds;
                stable.collision_incremental_updates =
                    collision_incremental_updates;
                return ResolveResult::success(std::move(stable));
            }

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
            ++iterations;
        }
    }
}
