#include <algorithm>
#include <array>
#include <set>
#include <tuple>
#include <utility>

#include <boundary_mesh/transition/transition_boundary_checker.hpp>

namespace boundary_mesh
{
    LayerBoundaryOwnerKey layerBoundaryOwnerKey(
        const LayerBoundaryOwner &owner)
    {
        return {owner.source_face_id, owner.layer, owner.role};
    }

    TransitionTriangleKey transitionTriangleKey(
        const OwnedBoundaryTriangle &triangle)
    {
        const auto tuple = [](const CollisionVertexKey &value)
        {
            return TransitionVertexTuple{
                value.source_vertex_id, value.layer, value.branch_id};
        };
        TransitionTriangleKey result{{
            tuple(triangle.vertex_keys[0]),
            tuple(triangle.vertex_keys[1]),
            tuple(triangle.vertex_keys[2])}};
        std::sort(result.begin(), result.end());
        return result;
    }

    CollisionTriangle makeTransitionCollisionTriangle(
        const OwnedBoundaryTriangle &owned,
        std::uint32_t owner_id)
    {
        CollisionTriangle result;
        result.points = owned.points;
        result.vertex_keys = owned.vertex_keys;
        result.owner_kind = CollisionOwnerKind::LayerCandidate;
        result.owner_id = owner_id;
        result.boundary_vertex_count = 3;
        for (std::size_t index = 0; index < 3; ++index)
        {
            result.boundary_points[index] = owned.points[index];
            result.boundary_vertex_keys[index] = owned.vertex_keys[index];
        }
        return result;
    }

    namespace
    {
        void appendOwner(
            std::vector<SurfaceFaceId> &rollback,
            const LayerBoundaryOwner &owner,
            std::vector<LayerBoundaryOwner> *colliding_owners)
        {
            appendLayerBoundaryRollbackFaces(owner, rollback);
            if (colliding_owners == nullptr) return;
            const LayerBoundaryOwnerKey owner_key =
                layerBoundaryOwnerKey(owner);
            const auto same = [&](const LayerBoundaryOwner &other)
            { return layerBoundaryOwnerKey(other) == owner_key; };
            if (std::none_of(
                    colliding_owners->begin(), colliding_owners->end(), same))
                colliding_owners->push_back(owner);
        }

        bool sameKey(
            const LayerQuadFaceKey &left,
            const LayerQuadFaceKey &right)
        {
            return left.source_face_id == right.source_face_id &&
                   left.layer == right.layer;
        }

        bool sameVertexKey(
            const CollisionVertexKey &left,
            const CollisionVertexKey &right)
        {
            return left.source_vertex_id == right.source_vertex_id &&
                   left.layer == right.layer &&
                   left.branch_id == right.branch_id;
        }

        bool belongsToHistoricalTop(
            const OwnedBoundaryTriangle &candidate,
            const BoundaryFace &historical)
        {
            if (candidate.owner.role != BoundaryOwnerRole::TopCap ||
                candidate.owner.source_face_id !=
                    historical.source_face_id)
                return false;
            for (const CollisionVertexKey &key : candidate.vertex_keys)
                if (std::none_of(
                        historical.vertex_keys.begin(),
                        historical.vertex_keys.end(),
                        [&](const CollisionVertexKey &other)
                        { return sameVertexKey(key, other); }))
                    return false;
            return true;
        }

        bool belongsToHistoricalTop(
            const OwnedBoundaryTriangle &candidate,
            const CollisionTriangle &historical,
            const std::vector<OwnedBoundaryTriangle> *prior_transition = nullptr)
        {
            if (historical.owner_kind == CollisionOwnerKind::LayerCandidate)
            {
                if (prior_transition == nullptr ||
                    historical.owner_id >= prior_transition->size())
                    return false;
                return transitionTriangleKey(candidate) ==
                    transitionTriangleKey((*prior_transition)[historical.owner_id]);
            }
            if (candidate.owner.role != BoundaryOwnerRole::TopCap ||
                candidate.owner.source_face_id != historical.owner_id)
                return false;
            for (const CollisionVertexKey &key : candidate.vertex_keys)
            {
                bool found = false;
                for (std::size_t index = 0;
                     index < historical.boundary_vertex_count; ++index)
                    found = found || sameVertexKey(
                        key, historical.boundary_vertex_keys[index]);
                if (!found) return false;
            }
            return true;
        }

        bool containsRegion(
            const std::vector<std::uint32_t> &ids, std::uint32_t region)
        {
            return std::find(ids.begin(), ids.end(), region) != ids.end();
        }

        Result<bool, SpatialError> canIgnoreOwnSlidingRegion(
            const OwnedBoundaryTriangle &owned,
            const SlidingIntersectionIndex &index,
            std::uint32_t region)
        {
            using IgnoreResult = Result<bool, SpatialError>;
            if (!owned.sliding_columns)
                return IgnoreResult::success(false);
            const auto &columns = *owned.sliding_columns;
            const std::size_t count = columns.low_points.size();
            if (count == 0 || columns.high_points.size() != count ||
                columns.low_region_ids.size() != count ||
                columns.high_region_ids.size() != count)
                return IgnoreResult::failure(
                    SpatialError::InvalidTopologyReference);

            std::vector<std::size_t> associated;
            for (std::size_t column = 0; column < count; ++column)
                if (containsRegion(
                        columns.low_region_ids[column], region) &&
                    containsRegion(
                        columns.high_region_ids[column], region))
                    associated.push_back(column);
            if (associated.empty() || associated.size() == count)
                return IgnoreResult::success(false);

            const auto reference = index.faceNormalAtPoint(
                region, columns.high_points[associated.front()]);
            if (!reference.hasValue())
                return IgnoreResult::failure(reference.error());
            std::vector<Scalar> sides;
            for (std::size_t column = 0; column < count; ++column)
            {
                if (std::find(associated.begin(), associated.end(), column) !=
                    associated.end())
                    continue;
                for (const Point3 *point : {
                         &columns.low_points[column],
                         &columns.high_points[column]})
                {
                    const auto side = index.signedSideToRegion(
                        region, *point, reference.value());
                    if (!side.hasValue())
                        return IgnoreResult::failure(side.error());
                    sides.push_back(side.value());
                }
            }
            return IgnoreResult::success(
                slidingSideValuesStayOnOneSide(sides, Scalar{1e-10}));
        }
    }

    TransitionStaticObstacleContext::BuildResult
    TransitionStaticObstacleContext::build(
        const TransitionBoundaryInput &input)
    {
        TransitionStaticObstacleContext context;
        context.original_surface_ = input.original_surface;
        context.historical_boundary_ = input.historical_boundary;
        context.historical_index_ = input.historical_index;
        context.historical_index_includes_transition_ =
            input.historical_index_includes_transition;
        context.prior_transition_boundary_ =
            input.prior_transition_boundary;
        context.shared_prior_transition_index_ =
            input.prior_transition_index;
        context.shared_prior_transition_dynamic_index_ =
            input.prior_transition_dynamic_index;
        context.sliding_surface_ = input.sliding_surface;
        if (input.historical_index == nullptr &&
            input.historical_boundary != nullptr)
        {
            const auto triangles =
                input.historical_boundary->collisionTriangles();
            if (!triangles.hasValue())
                return BuildResult::failure(
                    TransitionBoundaryError{triangles.error()});
            if (!triangles.value().empty())
            {
                auto built = CollisionIndex::build(triangles.value());
                if (!built.hasValue())
                    return BuildResult::failure(
                        TransitionBoundaryError{built.error()});
                context.immutable_historical_index_ =
                    std::move(built.value());
            }
        }
        if (!input.historical_index_includes_transition &&
            input.prior_transition_index == nullptr &&
            input.prior_transition_dynamic_index == nullptr &&
            input.prior_transition_boundary != nullptr &&
            !input.prior_transition_boundary->empty())
        {
            std::vector<CollisionTriangle> prior;
            prior.reserve(input.prior_transition_boundary->size());
            for (std::size_t index = 0;
                 index < input.prior_transition_boundary->size(); ++index)
                prior.push_back(makeTransitionCollisionTriangle(
                    (*input.prior_transition_boundary)[index],
                    static_cast<std::uint32_t>(index)));
            auto built = CollisionIndex::build(std::move(prior));
            if (!built.hasValue())
                return BuildResult::failure(
                    TransitionBoundaryError{built.error()});
            context.prior_transition_index_ = std::move(built.value());
        }
        return BuildResult::success(std::move(context));
    }

    Result<bool, TransitionBoundaryError>
    TransitionStaticObstacleContext::intersects(
        const OwnedBoundaryTriangle &owned,
        std::vector<TrianglePoints> *collided_faces,
        TransitionStaticObstacleQueryScratch *scratch) const
    {
        using HitResult = Result<bool, TransitionBoundaryError>;
        TransitionStaticObstacleQueryScratch local_scratch;
        TransitionStaticObstacleQueryScratch &workspace =
            scratch != nullptr ? *scratch : local_scratch;
        bool intersected = false;
        const CollisionTriangle collision =
            makeTransitionCollisionTriangle(owned, 0);
        if (original_surface_ != nullptr)
        {
            original_surface_->queryIllegalContacts(
                collision, workspace.candidate_ids,
                workspace.traversal_nodes, workspace.contact_ids);
            for (const std::size_t contact : workspace.contact_ids)
            {
                const CollisionTriangle &obstacle =
                    original_surface_->primitive(contact);
                const bool own_zero_layer_cap =
                    owned.owner.role == BoundaryOwnerRole::TopCap &&
                    owned.owner.layer == 0 &&
                    obstacle.owner_kind ==
                        CollisionOwnerKind::OriginalSurface &&
                    obstacle.owner_id == owned.owner.source_face_id;
                const bool own_regular_source =
                    owned.owner.role ==
                        BoundaryOwnerRole::RegularCandidate &&
                    obstacle.owner_kind ==
                        CollisionOwnerKind::OriginalSurface &&
                    obstacle.owner_id == owned.owner.source_face_id;
                if (!own_zero_layer_cap && !own_regular_source)
                {
                    if (collided_faces == nullptr)
                        return HitResult::success(true);
                    collided_faces->push_back(obstacle.points);
                    intersected = true;
                }
            }
        }
        if (historical_index_ != nullptr)
        {
            historical_index_->queryIllegalContacts(
                collision, workspace.incremental_candidate_ids,
                workspace.incremental_contact_ids);
            for (const CollisionPrimitiveId contact :
                 workspace.incremental_contact_ids)
            {
                const auto &obstacle = historical_index_->primitive(contact);
                if (!belongsToHistoricalTop(
                        owned, obstacle,
                        historical_index_includes_transition_
                            ? prior_transition_boundary_ : nullptr))
                {
                    if (collided_faces == nullptr)
                        return HitResult::success(true);
                    collided_faces->push_back(obstacle.points);
                    intersected = true;
                }
            }
        }
        if (immutable_historical_index_.has_value())
        {
            immutable_historical_index_->queryIllegalContacts(
                collision, workspace.candidate_ids,
                workspace.traversal_nodes, workspace.contact_ids);
            for (const std::size_t contact : workspace.contact_ids)
            {
                const CollisionTriangle &obstacle =
                    immutable_historical_index_->primitive(contact);
                const auto &faces = historical_boundary_->faces();
                const bool own_top =
                    obstacle.owner_kind ==
                        CollisionOwnerKind::ExposedBoundary &&
                    obstacle.owner_id < faces.size() &&
                    belongsToHistoricalTop(owned, faces[obstacle.owner_id]);
                if (!own_top)
                {
                    if (collided_faces == nullptr)
                        return HitResult::success(true);
                    collided_faces->push_back(obstacle.points);
                    intersected = true;
                }
            }
        }
        const CollisionIndex *prior_index =
            shared_prior_transition_index_ != nullptr
                ? shared_prior_transition_index_
                : (prior_transition_index_.has_value()
                    ? &*prior_transition_index_ : nullptr);
        if (prior_index != nullptr)
        {
            prior_index->queryIllegalContacts(
                collision, workspace.candidate_ids,
                workspace.traversal_nodes, workspace.contact_ids);
            for (const std::size_t contact : workspace.contact_ids)
            {
                const auto &prior = (*prior_transition_boundary_)[
                    prior_index->primitive(contact).owner_id];
                if (transitionTriangleKey(owned) !=
                    transitionTriangleKey(prior))
                {
                    if (collided_faces == nullptr)
                        return HitResult::success(true);
                    collided_faces->push_back(prior.points);
                    intersected = true;
                }
            }
        }
        if (shared_prior_transition_dynamic_index_ != nullptr)
        {
            shared_prior_transition_dynamic_index_->queryIllegalContacts(
                collision, workspace.incremental_candidate_ids,
                workspace.incremental_contact_ids);
            for (const CollisionPrimitiveId contact :
                 workspace.incremental_contact_ids)
            {
                const auto &obstacle =
                    shared_prior_transition_dynamic_index_->primitive(contact);
                if (obstacle.owner_id >= prior_transition_boundary_->size())
                    return HitResult::failure(TransitionBoundaryError{
                        SpatialError::InvalidTopologyReference});
                const auto &prior =
                    (*prior_transition_boundary_)[obstacle.owner_id];
                if (transitionTriangleKey(owned) !=
                    transitionTriangleKey(prior))
                {
                    if (collided_faces == nullptr)
                        return HitResult::success(true);
                    collided_faces->push_back(prior.points);
                    intersected = true;
                }
            }
        }
        if (sliding_surface_ != nullptr)
        {
            const auto permissions = buildSlidingContactPermissions(
                owned.vertex_sliding_region_ids,
                owned.physical_edge_mask,
                owned.complete_face_exemption_regions);
            std::set<std::uint32_t> ignored;
            while (true)
            {
                const auto hit = sliding_surface_->query(
                    owned.points, permissions, ignored);
                if (!hit.hasValue())
                    return HitResult::failure(
                        TransitionBoundaryError{hit.error()});
                if (!hit.value().intersected) break;
                const auto legal = canIgnoreOwnSlidingRegion(
                    owned, *sliding_surface_, hit.value().region_id);
                if (!legal.hasValue())
                    return HitResult::failure(
                        TransitionBoundaryError{legal.error()});
                if (!legal.value())
                {
                    if (collided_faces == nullptr)
                        return HitResult::success(true);
                    intersected = true;
                    break;
                }
                ignored.insert(hit.value().region_id);
            }
        }
        return HitResult::success(intersected);
    }

    Result<std::vector<OwnedBoundaryTriangle>, TransitionBoundaryError>
    TransitionBoundaryChecker::assembleExposedBoundary(
        const TransitionBoundaryInput &input) const
    {
        using AssemblyResult = Result<
            std::vector<OwnedBoundaryTriangle>, TransitionBoundaryError>;
        std::map<std::pair<SurfaceFaceId, std::uint32_t>, QuadDiagonal> seen;
        for (const auto &requirement : input.diagonal_requirements)
        {
                const auto [found, inserted] = seen.emplace(
                    std::make_pair(requirement.key.source_face_id, requirement.key.layer),
                    requirement.diagonal);
                if (!inserted && found->second != requirement.diagonal)
                    return AssemblyResult::failure(
                        TransitionBoundaryError{
                            ConflictingLayerQuadDiagonal{
                                requirement.key, found->second, requirement.diagonal}});
        }

        std::vector<std::pair<
            TransitionTriangleKey, OwnedBoundaryTriangle>> sorted;
        sorted.reserve(input.candidate_triangles.size());
        for (const auto &triangle : input.candidate_triangles)
            sorted.push_back({transitionTriangleKey(triangle), triangle});
        std::stable_sort(sorted.begin(), sorted.end(),
            [](const auto &left, const auto &right)
            { return left.first < right.first; });

        std::vector<OwnedBoundaryTriangle> exposed;
        for (std::size_t index = 0; index < sorted.size();)
        {
            std::size_t end = index + 1;
            while (end < sorted.size() &&
                   sorted[end].first == sorted[index].first)
                ++end;
            if ((end - index) % 2 == 1)
                exposed.push_back(std::move(sorted[index].second));
            index = end;
        }
        return AssemblyResult::success(std::move(exposed));
    }

    Result<std::vector<SurfaceFaceId>, TransitionBoundaryError>
    TransitionBoundaryChecker::findRollbackFaces(
        const TransitionBoundaryInput &input) const
    {
        const auto report = inspect(input);
        if (!report.hasValue())
            return Result<std::vector<SurfaceFaceId>, TransitionBoundaryError>::failure(
                report.error());
        return Result<std::vector<SurfaceFaceId>, TransitionBoundaryError>::success(
            report.value().rollback_faces);
    }

    Result<std::vector<LayerBoundaryOwner>, TransitionBoundaryError>
    TransitionBoundaryChecker::findCollidingOwners(
        const TransitionBoundaryInput &input) const
    {
        using OwnerResult = Result<
            std::vector<LayerBoundaryOwner>, TransitionBoundaryError>;
        const auto report = inspect(input);
        if (!report.hasValue())
            return OwnerResult::failure(report.error());
        return OwnerResult::success(report.value().colliding_owners);
    }

    Result<TransitionCollisionReport, TransitionBoundaryError>
    TransitionBoundaryChecker::inspect(
        const TransitionBoundaryInput &input) const
    {
        using ReportResult = Result<
            TransitionCollisionReport, TransitionBoundaryError>;
        TransitionCollisionReport report;
        const auto rollback = findRollbackFacesImpl(
            input, &report.colliding_owners);
        if (!rollback.hasValue())
            return ReportResult::failure(rollback.error());
        report.rollback_faces = rollback.value();
        return ReportResult::success(std::move(report));
    }

    Result<std::vector<SurfaceFaceId>, TransitionBoundaryError>
    TransitionBoundaryChecker::findRollbackFacesImpl(
        const TransitionBoundaryInput &input,
        std::vector<LayerBoundaryOwner> *colliding_owners) const
    {
        using RollbackResult = Result<
            std::vector<SurfaceFaceId>, TransitionBoundaryError>;
        const auto assembly = assembleExposedBoundary(input);
        if (!assembly.hasValue())
            return RollbackResult::failure(assembly.error());
        const auto &owned = assembly.value();
        std::vector<CollisionTriangle> collisions;
        collisions.reserve(owned.size());
        for (std::size_t index = 0; index < owned.size(); ++index)
            collisions.push_back(makeTransitionCollisionTriangle(
                owned[index], static_cast<std::uint32_t>(index)));

        std::optional<CollisionIndex> immutable_historical_index;
        if (input.historical_index == nullptr &&
            input.historical_boundary != nullptr)
        {
            const auto history_triangles =
                input.historical_boundary->collisionTriangles();
            if (!history_triangles.hasValue())
                return RollbackResult::failure(
                    TransitionBoundaryError{history_triangles.error()});
            if (!history_triangles.value().empty())
            {
                auto history = CollisionIndex::build(
                    history_triangles.value());
                if (!history.hasValue())
                    return RollbackResult::failure(
                        TransitionBoundaryError{history.error()});
                immutable_historical_index = std::move(history.value());
            }
        }

        std::optional<CollisionIndex> owned_prior_transition_index;
        if (!input.historical_index_includes_transition &&
            input.prior_transition_index == nullptr &&
            input.prior_transition_dynamic_index == nullptr &&
            input.prior_transition_boundary != nullptr &&
            !input.prior_transition_boundary->empty())
        {
            std::vector<CollisionTriangle> prior;
            prior.reserve(input.prior_transition_boundary->size());
            for (std::size_t index = 0;
                 index < input.prior_transition_boundary->size(); ++index)
                prior.push_back(makeTransitionCollisionTriangle(
                    (*input.prior_transition_boundary)[index],
                    static_cast<std::uint32_t>(index)));
            auto built = CollisionIndex::build(prior);
            if (!built.hasValue())
                return RollbackResult::failure(
                    TransitionBoundaryError{built.error()});
            owned_prior_transition_index = std::move(built.value());
        }
        const CollisionIndex *prior_transition_index =
            input.prior_transition_index != nullptr
                ? input.prior_transition_index
                : (owned_prior_transition_index.has_value()
                    ? &*owned_prior_transition_index : nullptr);

        std::vector<SurfaceFaceId> rollback;
        for (std::size_t index = 0; index < collisions.size(); ++index)
        {
            bool hit = false;
            if (input.original_surface != nullptr)
                for (const std::size_t contact :
                     input.original_surface->queryIllegalContacts(
                         collisions[index]))
                {
                    const CollisionTriangle &obstacle =
                        input.original_surface->primitive(contact);
                    const bool own_zero_layer_cap =
                        owned[index].owner.role ==
                            BoundaryOwnerRole::TopCap &&
                        owned[index].owner.layer == 0 &&
                        obstacle.owner_kind ==
                            CollisionOwnerKind::OriginalSurface &&
                        obstacle.owner_id ==
                            owned[index].owner.source_face_id;
                    const bool own_regular_source =
                        owned[index].owner.role ==
                            BoundaryOwnerRole::RegularCandidate &&
                        obstacle.owner_kind ==
                            CollisionOwnerKind::OriginalSurface &&
                        obstacle.owner_id ==
                            owned[index].owner.source_face_id;
                    if (!own_zero_layer_cap && !own_regular_source)
                    {
                        hit = true;
                        break;
                    }
                }
            if (input.historical_index != nullptr)
                for (const CollisionPrimitiveId contact :
                     input.historical_index->queryIllegalContacts(
                         collisions[index]))
                {
                    const CollisionTriangle &obstacle =
                        input.historical_index->primitive(contact);
                    if (!belongsToHistoricalTop(
                            owned[index], obstacle,
                            input.historical_index_includes_transition
                                ? input.prior_transition_boundary : nullptr))
                    {
                        hit = true;
                        break;
                    }
                }
            if (immutable_historical_index.has_value())
                for (const std::size_t contact :
                     immutable_historical_index->queryIllegalContacts(
                         collisions[index]))
                {
                    const CollisionTriangle &obstacle =
                        immutable_historical_index->primitive(contact);
                    const auto &history_faces =
                        input.historical_boundary->faces();
                    const bool own_historical_top =
                        obstacle.owner_kind ==
                            CollisionOwnerKind::ExposedBoundary &&
                        obstacle.owner_id < history_faces.size() &&
                        belongsToHistoricalTop(
                            owned[index],
                            history_faces[obstacle.owner_id]);
                    if (!own_historical_top)
                    {
                        hit = true;
                        break;
                    }
                }
            if (input.prior_transition_dynamic_index != nullptr)
            {
                std::vector<CollisionPrimitiveId> candidates, contacts;
                input.prior_transition_dynamic_index->queryIllegalContacts(
                    collisions[index], candidates, contacts);
                for (const CollisionPrimitiveId contact : contacts)
                {
                    const auto &obstacle =
                        input.prior_transition_dynamic_index->primitive(contact);
                    if (obstacle.owner_id >=
                        input.prior_transition_boundary->size())
                        return RollbackResult::failure(
                            TransitionBoundaryError{
                                SpatialError::InvalidTopologyReference});
                    const auto &prior = (*input.prior_transition_boundary)[
                        obstacle.owner_id];
                    if (transitionTriangleKey(owned[index]) !=
                        transitionTriangleKey(prior))
                    {
                        hit = true;
                        break;
                    }
                }
            }
            else if (prior_transition_index != nullptr)
                for (const std::size_t contact :
                     prior_transition_index->queryIllegalContacts(
                         collisions[index]))
                {
                    const auto &prior = (*input.prior_transition_boundary)[
                        prior_transition_index->primitive(contact).owner_id];
                    if (transitionTriangleKey(owned[index]) !=
                        transitionTriangleKey(prior))
                    {
                        hit = true;
                        break;
                    }
                }
            if (input.sliding_surface != nullptr)
            {
                const auto permissions = buildSlidingContactPermissions(
                    owned[index].vertex_sliding_region_ids,
                    owned[index].physical_edge_mask,
                    owned[index].complete_face_exemption_regions);
                std::set<std::uint32_t> ignored;
                while (true)
                {
                    const auto sliding_hit = input.sliding_surface->query(
                        owned[index].points, permissions, ignored);
                    if (!sliding_hit.hasValue())
                        return RollbackResult::failure(
                            TransitionBoundaryError{sliding_hit.error()});
                    if (!sliding_hit.value().intersected) break;
                    const auto legal = canIgnoreOwnSlidingRegion(
                        owned[index], *input.sliding_surface,
                        sliding_hit.value().region_id);
                    if (!legal.hasValue())
                        return RollbackResult::failure(
                            TransitionBoundaryError{legal.error()});
                    if (!legal.value())
                    {
                        hit = true;
                        break;
                    }
                    ignored.insert(sliding_hit.value().region_id);
                }
            }
            if (hit) appendOwner(
                rollback, owned[index].owner, colliding_owners);
        }

        if (!collisions.empty())
        {
            const auto candidate_index = IncrementalCollisionIndex::build(
                {{CollisionGroupId{1}, collisions}});
            if (!candidate_index.hasValue())
                return RollbackResult::failure(
                    TransitionBoundaryError{candidate_index.error()});
            for (std::size_t first = 0; first < collisions.size(); ++first)
                for (const CollisionPrimitiveId second :
                     candidate_index.value().queryIllegalContacts(
                         collisions[first]))
                    if (first < static_cast<std::size_t>(second))
                    {
                        appendOwner(
                            rollback, owned[first].owner, colliding_owners);
                        appendOwner(
                            rollback,
                            owned[static_cast<std::size_t>(second)].owner,
                            colliding_owners);
                    }
        }
        std::sort(rollback.begin(), rollback.end());
        rollback.erase(std::unique(rollback.begin(), rollback.end()),
                       rollback.end());
        return RollbackResult::success(std::move(rollback));
    }
}
