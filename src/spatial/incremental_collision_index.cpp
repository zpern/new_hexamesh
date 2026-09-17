#include <boundary_mesh/spatial/incremental_collision_index.hpp>

#include <limits>
#include <utility>

#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
    Result<IncrementalCollisionIndex, SpatialError>
    IncrementalCollisionIndex::build(
        std::vector<CollisionPrimitiveGroup> groups,
        IncrementalCollisionIndexOptions options)
    {
        IncrementalCollisionIndex index;
        index.options_ = options;
        for (CollisionPrimitiveGroup &group : groups)
        {
            const auto inserted = index.insertGroup(std::move(group));
            if (!inserted.hasValue())
                return Result<IncrementalCollisionIndex, SpatialError>::failure(
                    inserted.error());
        }
        return Result<IncrementalCollisionIndex, SpatialError>::success(
            std::move(index));
    }

    Result<std::monostate, SpatialError>
    IncrementalCollisionIndex::insertGroup(CollisionPrimitiveGroup group)
    {
        if (groups_.find(group.id) != groups_.end())
            return Result<std::monostate, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        std::vector<Aabb> bounds;
        bounds.reserve(group.triangles.size());
        for (const CollisionTriangle &triangle : group.triangles)
        {
            const auto box = makeAabb(
                triangle.points[0], triangle.points[1], triangle.points[2]);
            if (!box.hasValue())
                return Result<std::monostate, SpatialError>::failure(box.error());
            if ((triangle.points[1] - triangle.points[0])
                    .cross(triangle.points[2] - triangle.points[0])
                    .squaredNorm() == Scalar{0})
                return Result<std::monostate, SpatialError>::failure(
                    SpatialError::DegenerateTriangle);
            bounds.push_back(box.value());
        }
        if (primitives_.size() + group.triangles.size() >
            static_cast<std::size_t>(
                std::numeric_limits<CollisionPrimitiveId>::max()))
            return Result<std::monostate, SpatialError>::failure(
                SpatialError::PrimitiveIdOverflow);

        std::vector<CollisionPrimitiveId> ids;
        ids.reserve(group.triangles.size());
        for (std::size_t index = 0; index < group.triangles.size(); ++index)
        {
            const auto id = static_cast<CollisionPrimitiveId>(primitives_.size());
            ids.push_back(id);
            primitives_.push_back(StoredPrimitive{
                std::move(group.triangles[index]), bounds[index], group.id, true});
        }
        const std::size_t count = ids.size();
        groups_.emplace(group.id, std::move(ids));
        diagnostics_.inserts += count;
        diagnostics_.active_primitives += count;
        diagnostics_.maximum_leaf_load = diagnostics_.active_primitives;
        return Result<std::monostate, SpatialError>::success({});
    }

    Result<std::monostate, SpatialError>
    IncrementalCollisionIndex::eraseGroup(CollisionGroupId group)
    {
        const auto found = groups_.find(group);
        if (found == groups_.end())
            return Result<std::monostate, SpatialError>::failure(
                SpatialError::MissingPrimitiveGroup);
        for (const CollisionPrimitiveId id : found->second)
        {
            primitives_.at(static_cast<std::size_t>(id)).active = false;
            ++diagnostics_.erases;
            --diagnostics_.active_primitives;
            ++diagnostics_.inactive_primitives;
        }
        groups_.erase(found);
        diagnostics_.maximum_leaf_load = diagnostics_.active_primitives;
        return Result<std::monostate, SpatialError>::success({});
    }

    std::vector<CollisionPrimitiveId>
    IncrementalCollisionIndex::queryCandidates(
        const Aabb &bounds,
        std::optional<CollisionGroupId> ignored_group) const
    {
        ++diagnostics_.queries;
        std::vector<CollisionPrimitiveId> result;
        for (std::size_t index = 0; index < primitives_.size(); ++index)
        {
            const StoredPrimitive &primitive = primitives_[index];
            if (!primitive.active ||
                (ignored_group.has_value() && primitive.group == *ignored_group) ||
                !overlaps(bounds, primitive.bounds))
                continue;
            result.push_back(static_cast<CollisionPrimitiveId>(index));
        }
        diagnostics_.broad_phase_candidates += result.size();
        return result;
    }

    std::vector<CollisionPrimitiveId>
    IncrementalCollisionIndex::queryIllegalContacts(
        const CollisionTriangle &triangle,
        std::optional<CollisionGroupId> ignored_group) const
    {
        const auto bounds = makeAabb(
            triangle.points[0], triangle.points[1], triangle.points[2]);
        if (!bounds.hasValue()) return {};
        std::vector<CollisionPrimitiveId> result;
        for (const CollisionPrimitiveId id :
             queryCandidates(bounds.value(), ignored_group))
        {
            ++diagnostics_.exact_tests;
            const auto illegal = hasIllegalTriangleContact(triangle, primitive(id));
            if (!illegal.hasValue() || illegal.value()) result.push_back(id);
        }
        return result;
    }

    const CollisionTriangle &IncrementalCollisionIndex::primitive(
        CollisionPrimitiveId id) const
    {
        return primitives_.at(static_cast<std::size_t>(id)).triangle;
    }

    const CollisionIndexDiagnostics &
    IncrementalCollisionIndex::diagnostics() const noexcept
    {
        return diagnostics_;
    }

    Result<bool, SpatialError>
    IncrementalCollisionIndex::rebuildIfDegraded()
    {
        return Result<bool, SpatialError>::success(false);
    }
}
