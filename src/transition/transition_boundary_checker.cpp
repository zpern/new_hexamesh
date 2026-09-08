#include <algorithm>
#include <array>
#include <set>
#include <tuple>
#include <utility>

#include <boundary_mesh/transition/transition_boundary_checker.hpp>

namespace boundary_mesh
{
    namespace
    {
        using VertexTuple = std::tuple<VertexId, std::uint32_t, std::uint32_t>;
        using TriangleKey = std::array<VertexTuple, 3>;

        VertexTuple tuple(const CollisionVertexKey &key)
        {
            return {key.source_vertex_id, key.layer, key.branch_id};
        }

        TriangleKey key(const OwnedBoundaryTriangle &triangle)
        {
            TriangleKey result{{tuple(triangle.vertex_keys[0]),
                                tuple(triangle.vertex_keys[1]),
                                tuple(triangle.vertex_keys[2])}};
            std::sort(result.begin(), result.end());
            return result;
        }

        CollisionTriangle collisionTriangle(
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

        void appendOwner(
            std::vector<SurfaceFaceId> &rollback,
            const LayerBoundaryOwner &owner)
        {
            rollback.insert(rollback.end(),
                owner.rollback_high_faces.begin(),
                owner.rollback_high_faces.end());
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

    Result<std::vector<OwnedBoundaryTriangle>, TransitionBoundaryError>
    TransitionBoundaryChecker::assembleExposedBoundary(
        const TransitionBoundaryInput &input) const
    {
        using AssemblyResult = Result<
            std::vector<OwnedBoundaryTriangle>, TransitionBoundaryError>;
        for (std::size_t first = 0;
             first < input.diagonal_requirements.size(); ++first)
            for (std::size_t second = first + 1;
                 second < input.diagonal_requirements.size(); ++second)
                if (sameKey(input.diagonal_requirements[first].key,
                            input.diagonal_requirements[second].key) &&
                    input.diagonal_requirements[first].diagonal !=
                        input.diagonal_requirements[second].diagonal)
                    return AssemblyResult::failure(
                        TransitionBoundaryError{
                            ConflictingLayerQuadDiagonal{
                                input.diagonal_requirements[first].key,
                                input.diagonal_requirements[first].diagonal,
                                input.diagonal_requirements[second].diagonal}});

        std::vector<std::pair<TriangleKey, OwnedBoundaryTriangle>> sorted;
        sorted.reserve(input.candidate_triangles.size());
        for (const auto &triangle : input.candidate_triangles)
            sorted.push_back({key(triangle), triangle});
        std::sort(sorted.begin(), sorted.end(),
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
        using RollbackResult = Result<
            std::vector<SurfaceFaceId>, TransitionBoundaryError>;
        const auto assembly = assembleExposedBoundary(input);
        if (!assembly.hasValue())
            return RollbackResult::failure(assembly.error());
        const auto &owned = assembly.value();
        std::vector<CollisionTriangle> collisions;
        collisions.reserve(owned.size());
        for (std::size_t index = 0; index < owned.size(); ++index)
            collisions.push_back(collisionTriangle(
                owned[index], static_cast<std::uint32_t>(index)));

        std::optional<CollisionIndex> historical_index;
        if (input.historical_boundary != nullptr)
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
                historical_index = std::move(history.value());
            }
        }

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
            if (historical_index.has_value())
                for (const std::size_t contact :
                     historical_index->queryIllegalContacts(
                         collisions[index]))
                {
                    const CollisionTriangle &obstacle =
                        historical_index->primitive(contact);
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
            if (hit) appendOwner(rollback, owned[index].owner);
        }

        if (!collisions.empty())
        {
            const auto candidate_index = CollisionIndex::build(collisions);
            if (!candidate_index.hasValue())
                return RollbackResult::failure(
                    TransitionBoundaryError{candidate_index.error()});
            for (std::size_t first = 0; first < collisions.size(); ++first)
                for (const std::size_t second :
                     candidate_index.value().queryIllegalContacts(
                         collisions[first]))
                    if (first < second)
                    {
                        appendOwner(rollback, owned[first].owner);
                        appendOwner(rollback, owned[second].owner);
                    }
        }
        std::sort(rollback.begin(), rollback.end());
        rollback.erase(std::unique(rollback.begin(), rollback.end()),
                       rollback.end());
        return RollbackResult::success(std::move(rollback));
    }
}
