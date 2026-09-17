#include <boundary_mesh/spatial/incremental_collision_index.hpp>

#include <algorithm>
#include <limits>
#include <numeric>
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
        index.rebuildTree();
        index.diagnostics_.rebuilds = 0;
        index.diagnostics_.root_expansions = 0;
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

        const bool had_root = !nodes_.empty();
        const bool expands_root = had_root && std::any_of(
            bounds.begin(), bounds.end(),
            [&](const Aabb &box) { return !rootContains(box); });
        std::vector<CollisionPrimitiveId> ids;
        ids.reserve(group.triangles.size());
        for (std::size_t index = 0; index < group.triangles.size(); ++index)
        {
            const auto id = static_cast<CollisionPrimitiveId>(primitives_.size());
            ids.push_back(id);
            primitives_.push_back(StoredPrimitive{
                std::move(group.triangles[index]), bounds[index], group.id, true});
            if (had_root && !expands_root)
                insertIntoNode(0, id);
        }
        const std::size_t count = ids.size();
        groups_.emplace(group.id, std::move(ids));
        diagnostics_.inserts += count;
        diagnostics_.active_primitives += count;
        if (!had_root || expands_root)
        {
            if (expands_root) ++diagnostics_.root_expansions;
            rebuildTree();
        }
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
        return Result<std::monostate, SpatialError>::success({});
    }

    std::vector<CollisionPrimitiveId>
    IncrementalCollisionIndex::queryCandidates(
        const Aabb &bounds,
        std::optional<CollisionGroupId> ignored_group) const
    {
        ++diagnostics_.queries;
        std::vector<CollisionPrimitiveId> candidates;
        if (!nodes_.empty()) queryNode(0, bounds, candidates);
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()),
                         candidates.end());
        std::vector<CollisionPrimitiveId> result;
        for (const CollisionPrimitiveId id : candidates)
        {
            const StoredPrimitive &primitive = primitives_[static_cast<std::size_t>(id)];
            if (!primitive.active ||
                (ignored_group.has_value() && primitive.group == *ignored_group))
                continue;
            result.push_back(id);
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

    void IncrementalCollisionIndex::compactInactive()
    {
        for (Node &node : nodes_)
        {
            if (!node.leaf) continue;
            node.primitives.erase(
                std::remove_if(
                    node.primitives.begin(), node.primitives.end(),
                    [&](CollisionPrimitiveId id)
                    {
                        return !primitives_[static_cast<std::size_t>(id)].active;
                    }),
                node.primitives.end());
        }
        diagnostics_.inactive_primitives = 0;
        diagnostics_.maximum_leaf_load = 0;
        for (const Node &node : nodes_)
            if (node.leaf)
                diagnostics_.maximum_leaf_load = std::max(
                    diagnostics_.maximum_leaf_load, node.primitives.size());
    }

    Result<bool, SpatialError>
    IncrementalCollisionIndex::rebuildIfDegraded()
    {
        const std::size_t total = diagnostics_.active_primitives +
                                  diagnostics_.inactive_primitives;
        const bool inactive = total != 0 &&
            static_cast<Scalar>(diagnostics_.inactive_primitives) /
                static_cast<Scalar>(total) >= options_.rebuild_inactive_ratio;
        if (diagnostics_.maximum_leaf_load <= options_.rebuild_leaf_capacity &&
            !inactive)
            return Result<bool, SpatialError>::success(false);
        rebuildTree();
        ++diagnostics_.rebuilds;
        diagnostics_.inactive_primitives = 0;
        return Result<bool, SpatialError>::success(true);
    }

    bool IncrementalCollisionIndex::rootContains(const Aabb &bounds) const
    {
        if (nodes_.empty()) return false;
        return (nodes_[0].bounds.minimum.array() <= bounds.minimum.array()).all() &&
               (nodes_[0].bounds.maximum.array() >= bounds.maximum.array()).all();
    }

    void IncrementalCollisionIndex::rebuildTree()
    {
        nodes_.clear();
        std::vector<CollisionPrimitiveId> ids;
        for (std::size_t index = 0; index < primitives_.size(); ++index)
            if (primitives_[index].active)
                ids.push_back(static_cast<CollisionPrimitiveId>(index));
        diagnostics_.maximum_leaf_load = 0;
        if (ids.empty()) return;
        Aabb root = primitives_[static_cast<std::size_t>(ids.front())].bounds;
        for (const CollisionPrimitiveId id : ids)
        {
            const Aabb &box = primitives_[static_cast<std::size_t>(id)].bounds;
            root.minimum = root.minimum.cwiseMin(box.minimum);
            root.maximum = root.maximum.cwiseMax(box.maximum);
        }
        const Vector3 span = root.maximum - root.minimum;
        const Scalar padding = std::max(span.maxCoeff() * options_.root_padding,
                                        Scalar{1e-9});
        root.minimum.array() -= padding;
        root.maximum.array() += padding;
        appendNode(root, ids, 0);
    }

    std::size_t IncrementalCollisionIndex::appendNode(
        const Aabb &bounds,
        const std::vector<CollisionPrimitiveId> &ids,
        std::size_t depth)
    {
        const std::size_t node_index = nodes_.size();
        nodes_.push_back(Node{});
        nodes_[node_index].bounds = bounds;
        nodes_[node_index].children.fill(std::numeric_limits<std::size_t>::max());
        if (ids.size() <= options_.target_leaf_capacity ||
            depth >= options_.maximum_depth)
        {
            nodes_[node_index].primitives = ids;
            diagnostics_.maximum_leaf_load = std::max(
                diagnostics_.maximum_leaf_load, ids.size());
            return node_index;
        }

        const Point3 middle = (bounds.minimum + bounds.maximum) * Scalar{0.5};
        std::array<Aabb, 8> child_bounds;
        std::array<std::vector<CollisionPrimitiveId>, 8> child_ids;
        for (std::size_t child = 0; child < 8; ++child)
        {
            Aabb box = bounds;
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                if ((child & (std::size_t{1} << axis)) != 0)
                    box.minimum[static_cast<Eigen::Index>(axis)] = middle[static_cast<Eigen::Index>(axis)];
                else
                    box.maximum[static_cast<Eigen::Index>(axis)] = middle[static_cast<Eigen::Index>(axis)];
            }
            child_bounds[child] = box;
            for (const CollisionPrimitiveId id : ids)
                if (overlaps(primitives_[static_cast<std::size_t>(id)].bounds, box))
                    child_ids[child].push_back(id);
        }
        const bool useful = std::any_of(
            child_ids.begin(), child_ids.end(),
            [&](const auto &values) { return !values.empty() && values.size() < ids.size(); });
        if (!useful)
        {
            nodes_[node_index].primitives = ids;
            diagnostics_.maximum_leaf_load = std::max(
                diagnostics_.maximum_leaf_load, ids.size());
            return node_index;
        }
        nodes_[node_index].leaf = false;
        for (std::size_t child = 0; child < 8; ++child)
            nodes_[node_index].children[child] = appendNode(
                child_bounds[child], child_ids[child], depth + 1);
        return node_index;
    }

    void IncrementalCollisionIndex::insertIntoNode(
        std::size_t node_index,
        CollisionPrimitiveId id)
    {
        Node &node = nodes_[node_index];
        if (node.leaf)
        {
            node.primitives.push_back(id);
            diagnostics_.maximum_leaf_load = std::max(
                diagnostics_.maximum_leaf_load, node.primitives.size());
            return;
        }
        const Aabb bounds = primitives_[static_cast<std::size_t>(id)].bounds;
        const auto children = node.children;
        for (const std::size_t child : children)
            if (child != std::numeric_limits<std::size_t>::max() &&
                overlaps(nodes_[child].bounds, bounds))
                insertIntoNode(child, id);
    }

    void IncrementalCollisionIndex::queryNode(
        std::size_t node_index,
        const Aabb &bounds,
        std::vector<CollisionPrimitiveId> &result) const
    {
        const Node &node = nodes_[node_index];
        if (!overlaps(node.bounds, bounds)) return;
        if (node.leaf)
        {
            for (const CollisionPrimitiveId id : node.primitives)
                if (overlaps(primitives_[static_cast<std::size_t>(id)].bounds, bounds))
                    result.push_back(id);
            return;
        }
        for (const std::size_t child : node.children)
            if (child != std::numeric_limits<std::size_t>::max())
                queryNode(child, bounds, result);
    }
}
