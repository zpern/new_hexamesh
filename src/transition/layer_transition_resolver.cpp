#include <algorithm>
#include <chrono>
#include <iostream>
#include <set>
#include <utility>

#include <boundary_mesh/transition/layer_transition_resolver.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar minimum_external_distance_scale{1e-6};
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
                auto value = input.build_provisional(
                    retained, face_sets, external_controls);
                if (value.hasValue()) configure(value.value());
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
            const auto initial_report =
                checker.inspect(provisional.value().boundary);
            if (!initial_report.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    initial_report.error()});
            std::cerr << "temporary resolver initial_scan_ms="
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() -
                             initial_scan_started).count()
                      << " colliding_owners="
                      << initial_report.value().colliding_owners.size()
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
                    initial_report.value().colliding_owners,
                    id))
                {
                    external_controls.distance_scales[id] =
                        external_controls.distanceScale(id);
                    continue;
                }
                const Scalar initial = external_controls.distanceScale(id);
                searches.emplace(
                    id, ExternalSearch{Scalar{}, initial, initial, false, false});
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

                for (const auto &[id, triangles] : replacements)
                {
                    const CollisionGroupId group =
                        static_cast<CollisionGroupId>(id) + 2;
                    const auto erased = search_index->eraseGroup(group);
                    if (!erased.hasValue())
                        return ExternalCollisionResult::failure(
                            LayerTransitionError{
                                TransitionBoundaryError{erased.error()}});
                    std::vector<CollisionTriangle> converted;
                    converted.reserve(triangles.size());
                    for (const OwnedBoundaryTriangle &owned : triangles)
                        converted.push_back(makeTransitionCollisionTriangle(
                            owned, id));
                    const auto inserted = search_index->insertGroup(
                        {group, converted});
                    if (!inserted.hasValue())
                        return ExternalCollisionResult::failure(
                            LayerTransitionError{
                                TransitionBoundaryError{inserted.error()}});
                    for (const CollisionTriangle &triangle : converted)
                        if (!search_index->queryIllegalContacts(
                                triangle, group).empty())
                        {
                            colliding.insert(id);
                            break;
                        }
                }
                return ExternalCollisionResult::success(
                    std::move(colliding));
            };

            while (std::any_of(
                searches.begin(), searches.end(), [](const auto &entry)
                { return !entry.second.bracketed && !entry.second.exhausted; }))
            {
                for (auto &[id, search] : searches)
                {
                    if (search.bracketed || search.exhausted) continue;
                    search.probe = std::max(
                        search.probe * Scalar{0.5},
                        minimum_external_distance_scale);
                    external_controls.distance_scales[id] = search.probe;
                }
                auto trial = build();
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
                for (auto &[id, search] : searches)
                {
                    if (!search.bracketed) continue;
                    active = true;
                    external_controls.distance_scales[id] =
                        (search.low + search.high) * Scalar{0.5};
                }
                if (!active) break;
                auto trial = build();
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
                provisional = build();
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
                rollback = initial_report.value().rollback_faces;
            rollback.insert(rollback.end(),
                provisional.value().forced_rollback_high_faces.begin(),
                provisional.value().forced_rollback_high_faces.end());
            std::sort(rollback.begin(),rollback.end());
            rollback.erase(std::unique(rollback.begin(),rollback.end()),
                           rollback.end());
            if (rollback.empty())
            {
                const auto exposed = checker.assembleExposedBoundary(
                    provisional.value().boundary);
                if (!exposed.hasValue())
                    return ResolveResult::failure(LayerTransitionError{
                        exposed.error()});
                StableLayerTransition stable;
                stable.face_sets = std::move(face_sets);
                stable.retained_high_faces = std::move(retained);
                stable.exposed_boundary = exposed.value();
                stable.resolved_topology = std::move(
                    provisional.value().resolved_topology);
                stable.iterations = iterations;
                stable.all_top_faces_are_triangles =
                    provisional.value().all_top_faces_are_triangles;
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
