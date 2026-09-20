#include <boundary_mesh/growth/layer_boundary_batch.hpp>

#include <algorithm>
#include <array>
#include <map>
#include <utility>

namespace boundary_mesh
{
    namespace
    {
        bool vertexLess(
            const CollisionVertexKey &left,
            const CollisionVertexKey &right) noexcept
        {
            return left.source_vertex_id < right.source_vertex_id ||
                (left.source_vertex_id == right.source_vertex_id &&
                 (left.layer < right.layer ||
                  (left.layer == right.layer &&
                   left.branch_id < right.branch_id)));
        }

        struct EdgeKey
        {
            CollisionVertexKey first;
            CollisionVertexKey second;
        };

        struct EdgeKeyLess
        {
            bool operator()(const EdgeKey &left, const EdgeKey &right) const noexcept
            {
                if (vertexLess(left.first, right.first)) return true;
                if (vertexLess(right.first, left.first)) return false;
                return vertexLess(left.second, right.second);
            }
        };

        EdgeKey edgeKey(CollisionVertexKey first, CollisionVertexKey second)
        {
            if (vertexLess(second, first)) std::swap(first, second);
            return {first, second};
        }

        bool validCandidate(const LayerBoundaryCandidate &candidate)
        {
            const std::size_t count = candidate.bottom.points.size();
            return (count == 3 || count == 4) &&
                candidate.bottom.vertex_keys.size() == count &&
                candidate.top.points.size() == count &&
                candidate.top.vertex_keys.size() == count;
        }

        Result<std::vector<CollisionTriangle>, SpatialError> splitFace(
            const BoundaryFace &face,
            std::uint32_t owner_id)
        {
            const std::array<std::array<std::size_t, 3>, 2> splits{{
                {{0, 1, 2}}, {{0, 2, 3}}}};
            const std::size_t split_count = face.points.size() == 3 ? 1 : 2;
            std::vector<CollisionTriangle> result;
            result.reserve(split_count);
            for (std::size_t split = 0; split < split_count; ++split)
            {
                CollisionTriangle triangle;
                triangle.owner_kind = CollisionOwnerKind::LayerCandidate;
                triangle.owner_id = owner_id;
                triangle.boundary_vertex_count =
                    static_cast<std::uint8_t>(face.points.size());
                for (std::size_t boundary = 0;
                     boundary < face.points.size(); ++boundary)
                {
                    triangle.boundary_points[boundary] = face.points[boundary];
                    triangle.boundary_vertex_keys[boundary] =
                        face.vertex_keys[boundary];
                }
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const std::size_t index = splits[split][corner];
                    triangle.points[corner] = face.points[index];
                    triangle.vertex_keys[corner] = face.vertex_keys[index];
                }
                const auto bounds = makeAabb(
                    triangle.points[0], triangle.points[1], triangle.points[2]);
                if (!bounds.hasValue())
                    return Result<std::vector<CollisionTriangle>, SpatialError>::failure(
                        bounds.error());
                result.push_back(std::move(triangle));
            }
            return Result<std::vector<CollisionTriangle>, SpatialError>::success(
                std::move(result));
        }

        BoundaryFace sideFace(
            const LayerBoundaryCandidate &candidate,
            std::size_t side)
        {
            const std::size_t next =
                (side + 1) % candidate.bottom.points.size();
            return BoundaryFace{
                {candidate.bottom.points[side], candidate.bottom.points[next],
                 candidate.top.points[next], candidate.top.points[side]},
                {candidate.bottom.vertex_keys[side],
                 candidate.bottom.vertex_keys[next],
                 candidate.top.vertex_keys[next],
                 candidate.top.vertex_keys[side]},
                candidate.top.source_face_id,
                candidate.top.region_id};
        }

        void extend(Aabb &target, const Aabb &source)
        {
            target.minimum = target.minimum.cwiseMin(source.minimum);
            target.maximum = target.maximum.cwiseMax(source.maximum);
        }
    }

    Result<LayerBoundaryBatch, SpatialError> LayerBoundaryBatch::build(
        const std::vector<LayerBoundaryCandidate> &candidates)
    {
        using BuildResult = Result<LayerBoundaryBatch, SpatialError>;
        std::map<EdgeKey, std::size_t, EdgeKeyLess> incidence;
        for (const LayerBoundaryCandidate &candidate : candidates)
        {
            if (!validCandidate(candidate))
                return BuildResult::failure(
                    SpatialError::InvalidTopologyReference);
            for (std::size_t side = 0;
                 side < candidate.bottom.vertex_keys.size(); ++side)
            {
                const std::size_t next =
                    (side + 1) % candidate.bottom.vertex_keys.size();
                const std::size_t count = ++incidence[edgeKey(
                    candidate.bottom.vertex_keys[side],
                    candidate.bottom.vertex_keys[next])];
                if (count > 2)
                    return BuildResult::failure(
                        SpatialError::InvalidTopologyReference);
            }
        }

        LayerBoundaryBatch batch;
        batch.candidates_ = candidates;
        batch.owners_.reserve(candidates.size());
        for (std::size_t owner_index = 0;
             owner_index < candidates.size(); ++owner_index)
        {
            const LayerBoundaryCandidate &candidate = candidates[owner_index];
            LayerBoundaryOwner owner;
            owner.owner_id = static_cast<std::uint32_t>(owner_index);
            const auto top = splitFace(candidate.top, owner.owner_id);
            if (!top.hasValue()) return BuildResult::failure(top.error());
            owner.triangles = top.value();
            for (std::size_t side = 0;
                 side < candidate.bottom.vertex_keys.size(); ++side)
            {
                const std::size_t next =
                    (side + 1) % candidate.bottom.vertex_keys.size();
                if (incidence.at(edgeKey(
                        candidate.bottom.vertex_keys[side],
                        candidate.bottom.vertex_keys[next])) == 2)
                {
                    ++batch.diagnostics_.omitted_shared_sides;
                    continue;
                }
                const auto triangles = splitFace(
                    sideFace(candidate, side), owner.owner_id);
                if (!triangles.hasValue())
                    return BuildResult::failure(triangles.error());
                owner.triangles.insert(
                    owner.triangles.end(),
                    triangles.value().begin(), triangles.value().end());
            }
            const auto first_bounds = makeAabb(
                owner.triangles.front().points[0],
                owner.triangles.front().points[1],
                owner.triangles.front().points[2]);
            if (!first_bounds.hasValue())
                return BuildResult::failure(first_bounds.error());
            owner.bounds = first_bounds.value();
            for (std::size_t triangle = 1;
                 triangle < owner.triangles.size(); ++triangle)
            {
                const auto bounds = makeAabb(
                    owner.triangles[triangle].points[0],
                    owner.triangles[triangle].points[1],
                    owner.triangles[triangle].points[2]);
                if (!bounds.hasValue())
                    return BuildResult::failure(bounds.error());
                extend(owner.bounds, bounds.value());
            }
            batch.diagnostics_.triangle_count += owner.triangles.size();
            batch.owners_.push_back(std::move(owner));
        }
        batch.diagnostics_.owner_count = batch.owners_.size();
        return BuildResult::success(std::move(batch));
    }

    const std::vector<LayerBoundaryCandidate> &
    LayerBoundaryBatch::candidates() const noexcept
    {
        return candidates_;
    }

    const std::vector<LayerBoundaryOwner> &
    LayerBoundaryBatch::owners() const noexcept
    {
        return owners_;
    }

    const LayerBoundaryBatchDiagnostics &
    LayerBoundaryBatch::diagnostics() const noexcept
    {
        return diagnostics_;
    }
}
