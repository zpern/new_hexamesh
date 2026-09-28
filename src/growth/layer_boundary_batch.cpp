#include <boundary_mesh/growth/layer_boundary_batch.hpp>

#include <algorithm>
#include <array>
#include <limits>
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

        Result<LayerBoundaryOwner, SpatialError> buildOwner(
            const LayerBoundaryCandidate &candidate,
            const std::vector<std::size_t> &adjacent_owners,
            const std::vector<bool> &active_owners)
        {
            using OwnerResult = Result<LayerBoundaryOwner, SpatialError>;
            constexpr std::size_t no_owner =
                std::numeric_limits<std::size_t>::max();
            LayerBoundaryOwner owner;
            owner.owner_id = candidate.top.source_face_id;
            const auto top = splitFace(candidate.top, owner.owner_id);
            if (!top.hasValue()) return OwnerResult::failure(top.error());
            owner.triangles = top.value();
            for (std::size_t side = 0;
                 side < candidate.bottom.vertex_keys.size(); ++side)
            {
                const std::size_t neighbor = adjacent_owners[side];
                if (neighbor != no_owner && active_owners[neighbor])
                    continue;
                const auto triangles = splitFace(
                    sideFace(candidate, side), owner.owner_id);
                if (!triangles.hasValue())
                    return OwnerResult::failure(triangles.error());
                owner.triangles.insert(owner.triangles.end(),
                    triangles.value().begin(), triangles.value().end());
            }
            const auto first_bounds = makeAabb(
                owner.triangles.front().points[0],
                owner.triangles.front().points[1],
                owner.triangles.front().points[2]);
            if (!first_bounds.hasValue())
                return OwnerResult::failure(first_bounds.error());
            owner.bounds = first_bounds.value();
            for (std::size_t triangle = 1;
                 triangle < owner.triangles.size(); ++triangle)
            {
                const auto bounds = makeAabb(
                    owner.triangles[triangle].points[0],
                    owner.triangles[triangle].points[1],
                    owner.triangles[triangle].points[2]);
                if (!bounds.hasValue())
                    return OwnerResult::failure(bounds.error());
                extend(owner.bounds, bounds.value());
            }
            return OwnerResult::success(std::move(owner));
        }
    }

    Result<LayerBoundaryBatch, SpatialError> LayerBoundaryBatch::build(
        const std::vector<LayerBoundaryCandidate> &candidates)
    {
        using BuildResult = Result<LayerBoundaryBatch, SpatialError>;
        struct EdgeIncidence
        {
            std::size_t count{};
            std::size_t first_owner{};
            std::size_t first_side{};
        };
        std::map<EdgeKey, EdgeIncidence, EdgeKeyLess> incidence;
        LayerBoundaryBatch batch;
        batch.adjacent_source_face_ids_.resize(candidates.size());
        batch.adjacent_owner_indices_.resize(candidates.size());
        batch.active_owners_.assign(candidates.size(), true);
        for (std::size_t owner_index = 0;
             owner_index < candidates.size(); ++owner_index)
            batch.adjacent_owner_indices_[owner_index].assign(
                candidates[owner_index].bottom.vertex_keys.size(),
                std::numeric_limits<std::size_t>::max());
        for (std::size_t owner_index = 0;
             owner_index < candidates.size();
             ++owner_index)
        {
            const LayerBoundaryCandidate &candidate = candidates[owner_index];
            if (!validCandidate(candidate))
                return BuildResult::failure(
                    SpatialError::InvalidTopologyReference);
            for (std::size_t side = 0;
                 side < candidate.bottom.vertex_keys.size(); ++side)
            {
                const std::size_t next =
                    (side + 1) % candidate.bottom.vertex_keys.size();
                EdgeIncidence &edge = incidence[edgeKey(
                    candidate.bottom.vertex_keys[side],
                    candidate.bottom.vertex_keys[next])];
                if (edge.count == 0)
                {
                    edge.first_owner = owner_index;
                    edge.first_side = side;
                }
                else if (edge.count == 1)
                {
                    batch.adjacent_source_face_ids_[edge.first_owner]
                        .push_back(
                            candidate.top.source_face_id);
                    batch.adjacent_source_face_ids_[owner_index]
                        .push_back(candidates[edge.first_owner]
                                       .top.source_face_id);
                    batch.adjacent_owner_indices_[owner_index][side] =
                        edge.first_owner;
                    batch.adjacent_owner_indices_[edge.first_owner]
                        [edge.first_side] = owner_index;
                }
                if (++edge.count > 2)
                    return BuildResult::failure(
                        SpatialError::InvalidTopologyReference);
            }
        }

        batch.candidates_ = candidates;
        batch.owners_.reserve(candidates.size());
        for (std::size_t owner_index = 0;
             owner_index < candidates.size(); ++owner_index)
        {
            const auto owner = buildOwner(
                candidates[owner_index],
                batch.adjacent_owner_indices_[owner_index],
                batch.active_owners_);
            if (!owner.hasValue())
                return BuildResult::failure(owner.error());
            batch.diagnostics_.omitted_shared_sides +=
                static_cast<std::size_t>(std::count_if(
                    batch.adjacent_owner_indices_[owner_index].begin(),
                    batch.adjacent_owner_indices_[owner_index].end(),
                    [](std::size_t neighbor)
                    { return neighbor != std::numeric_limits<std::size_t>::max(); }));
            batch.diagnostics_.triangle_count += owner.value().triangles.size();
            batch.owners_.push_back(std::move(owner.value()));
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

    const std::vector<std::vector<SurfaceFaceId>> &
    LayerBoundaryBatch::adjacentSourceFaceIds() const noexcept
    {
        return adjacent_source_face_ids_;
    }

    const std::vector<bool> &LayerBoundaryBatch::activeOwners() const noexcept
    {
        return active_owners_;
    }

    Result<std::vector<std::size_t>, SpatialError>
    LayerBoundaryBatch::deactivateOwners(
        const std::vector<std::size_t> &owner_indices)
    {
        using UpdateResult = Result<std::vector<std::size_t>, SpatialError>;
        constexpr std::size_t no_owner =
            std::numeric_limits<std::size_t>::max();
        std::vector<bool> pending(owners_.size(), false);
        for (const std::size_t owner : owner_indices)
        {
            if (owner >= owners_.size())
                return UpdateResult::failure(
                    SpatialError::InvalidTopologyReference);
            pending[owner] = true;
        }

        std::size_t removed_shared_sides = 0;
        std::vector<bool> deactivated(owners_.size(), false);
        for (std::size_t owner = 0; owner < pending.size(); ++owner)
        {
            if (!pending[owner] || !active_owners_[owner]) continue;
            for (const std::size_t neighbor : adjacent_owner_indices_[owner])
                if (neighbor != no_owner && active_owners_[neighbor])
                    ++removed_shared_sides;
        }
        for (std::size_t owner = 0; owner < pending.size(); ++owner)
        {
            if (!pending[owner] || !active_owners_[owner]) continue;
            diagnostics_.triangle_count -= owners_[owner].triangles.size();
            owners_[owner].triangles.clear();
            active_owners_[owner] = false;
            deactivated[owner] = true;
            --diagnostics_.owner_count;
        }
        diagnostics_.omitted_shared_sides -= removed_shared_sides;

        std::vector<std::size_t> affected;
        for (std::size_t owner = 0; owner < pending.size(); ++owner)
        {
            if (!deactivated[owner]) continue;
            for (const std::size_t neighbor : adjacent_owner_indices_[owner])
                if (neighbor != no_owner && active_owners_[neighbor])
                    affected.push_back(neighbor);
        }
        std::sort(affected.begin(), affected.end());
        affected.erase(std::unique(affected.begin(), affected.end()),
                       affected.end());
        for (const std::size_t owner : affected)
        {
            diagnostics_.triangle_count -= owners_[owner].triangles.size();
            const auto rebuilt = buildOwner(
                candidates_[owner], adjacent_owner_indices_[owner],
                active_owners_);
            if (!rebuilt.hasValue())
                return UpdateResult::failure(rebuilt.error());
            owners_[owner] = std::move(rebuilt.value());
            diagnostics_.triangle_count += owners_[owner].triangles.size();
            diagnostics_.omitted_shared_sides -= static_cast<std::size_t>(
                std::count_if(adjacent_owner_indices_[owner].begin(),
                              adjacent_owner_indices_[owner].end(),
                              [&](std::size_t neighbor)
                              {
                                  return neighbor != no_owner &&
                                      !active_owners_[neighbor];
                              }));
        }
        return UpdateResult::success(std::move(affected));
    }

    const LayerBoundaryBatchDiagnostics &
    LayerBoundaryBatch::diagnostics() const noexcept
    {
        return diagnostics_;
    }
}
