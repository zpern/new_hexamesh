#include <algorithm>
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

            auto provisional = build();
            if (!provisional.hasValue())
                return ResolveResult::failure(provisional.error());

            std::vector<SurfaceFaceId> external_faces;
            for (const auto &topology : provisional.value().resolved_topology)
                if (topology.terminal_quad_decision ==
                    TerminalQuadDecision::ExternalPatch)
                    external_faces.push_back(topology.source_face_id);
            std::sort(external_faces.begin(), external_faces.end());
            external_faces.erase(std::unique(
                external_faces.begin(), external_faces.end()),
                external_faces.end());

            const auto collides = [&](const ProvisionalLayerTransition &value,
                                      SurfaceFaceId id)
                -> Result<bool, TransitionBoundaryError>
            {
                const auto owners = checker.findCollidingOwners(value.boundary);
                if (!owners.hasValue())
                    return Result<bool, TransitionBoundaryError>::failure(
                        owners.error());
                return Result<bool, TransitionBoundaryError>::success(
                    std::any_of(owners.value().begin(), owners.value().end(),
                        [&](const LayerBoundaryOwner &owner)
                        {
                            return owner.role ==
                                       BoundaryOwnerRole::ExternalPatch &&
                                   owner.source_face_id == id;
                        }));
            };

            // One global scan identifies which external patches need any
            // further work.  Safe patches must not each rebuild this index.
            const auto initial_owners =
                checker.findCollidingOwners(provisional.value().boundary);
            if (!initial_owners.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    initial_owners.error()});

            for (const SurfaceFaceId id : external_faces)
            {
                Scalar low{};
                Scalar high{};
                bool bracketed = false;
                const Scalar initial = external_controls.distanceScale(id);
                const bool initial_collision = std::any_of(
                    initial_owners.value().begin(), initial_owners.value().end(),
                    [&](const LayerBoundaryOwner &owner)
                    {
                        return owner.role == BoundaryOwnerRole::ExternalPatch &&
                               owner.source_face_id == id;
                    });
                if (initial_collision)
                {
                    high = initial;
                    Scalar probe = initial;
                    std::optional<ProvisionalLayerTransition> safe;
                    while (probe > minimum_external_distance_scale)
                    {
                        probe = std::max(
                            probe * Scalar{0.5},
                            minimum_external_distance_scale);
                        external_controls.distance_scales[id] = probe;
                        auto trial = build();
                        if (!trial.hasValue())
                            return ResolveResult::failure(trial.error());
                        const auto hit = collides(trial.value(), id);
                        if (!hit.hasValue())
                            return ResolveResult::failure(
                                LayerTransitionError{hit.error()});
                        if (!hit.value())
                        {
                            low = probe;
                            safe = std::move(trial.value());
                            bracketed = true;
                            break;
                        }
                        high = probe;
                        provisional = std::move(trial);
                        if (probe == minimum_external_distance_scale) break;
                    }
                    if (!safe.has_value())
                    {
                        auto topology = std::find_if(
                            provisional.value().resolved_topology.begin(),
                            provisional.value().resolved_topology.end(),
                            [&](const ResolvedTransitionTopology &value)
                            { return value.source_face_id == id; });
                        if (topology !=
                                provisional.value().resolved_topology.end() &&
                            !topology->dependent_high_faces.empty())
                            break;
                        external_controls.keep_hexa_faces.push_back(id);
                        std::sort(external_controls.keep_hexa_faces.begin(),
                                  external_controls.keep_hexa_faces.end());
                        external_controls.keep_hexa_faces.erase(std::unique(
                            external_controls.keep_hexa_faces.begin(),
                            external_controls.keep_hexa_faces.end()),
                            external_controls.keep_hexa_faces.end());
                        provisional = build();
                        if (!provisional.hasValue())
                            return ResolveResult::failure(provisional.error());
                        continue;
                    }
                    provisional = ProvisionalLayerTransitionResult::success(
                        std::move(*safe));
                }
                else
                {
                    // The default outward distance is already a valid
                    // non-intersecting candidate.  Do not spend a global
                    // boundary rebuild expanding every safe patch; the
                    // binary search below is reserved for candidates that
                    // actually intersect at their initial distance.
                    low = initial;
                    external_controls.distance_scales[id] = low;
                    continue;
                }
                if (!bracketed)
                {
                    external_controls.distance_scales[id] = low;
                    continue;
                }
                for (std::uint32_t step = 0; step < 12; ++step)
                {
                    const Scalar middle = (low + high) * Scalar{0.5};
                    external_controls.distance_scales[id] = middle;
                    auto trial = build();
                    if (!trial.hasValue())
                        return ResolveResult::failure(trial.error());
                    const auto hit = collides(trial.value(), id);
                    if (!hit.hasValue())
                        return ResolveResult::failure(LayerTransitionError{
                            hit.error()});
                    if (hit.value()) high = middle;
                    else
                    {
                        low = middle;
                        provisional = std::move(trial);
                    }
                }
                external_controls.distance_scales[id] = low;
                provisional = build();
                if (!provisional.hasValue())
                    return ResolveResult::failure(provisional.error());
            }

            const auto checked_rollback = checker.findRollbackFaces(
                provisional.value().boundary);
            if (!checked_rollback.hasValue())
                return ResolveResult::failure(LayerTransitionError{
                    checked_rollback.error()});
            std::vector<SurfaceFaceId> rollback =
                checked_rollback.value();
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
