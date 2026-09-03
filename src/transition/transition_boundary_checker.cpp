#include <algorithm>
#include <array>
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
            bool hit = input.original_surface != nullptr &&
                !input.original_surface->queryIllegalContacts(
                    collisions[index]).empty();
            if (historical_index.has_value())
                hit = hit || !historical_index->queryIllegalContacts(
                    collisions[index]).empty();
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
