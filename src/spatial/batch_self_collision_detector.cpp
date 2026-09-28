#include <boundary_mesh/spatial/batch_self_collision_detector.hpp>

#include <set>
#include <limits>
#include <map>
#include <utility>

#include <boundary_mesh/spatial/aabb.hpp>
#include <boundary_mesh/spatial/binary_aabb_tree.hpp>
#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool validBounds(const Aabb &bounds)
        {
            return bounds.minimum.allFinite() && bounds.maximum.allFinite() &&
                (bounds.minimum.array() <= bounds.maximum.array()).all();
        }
    }

    Result<BatchSelfCollisionResult, SpatialError>
    BatchSelfCollisionDetector::detect(
        const std::vector<CollisionTriangle> &triangles)
    {
        BatchSelfCollisionResult result;
        result.diagnostics.triangle_count = triangles.size();
        if (triangles.empty())
        {
            return Result<BatchSelfCollisionResult, SpatialError>::success(
                std::move(result));
        }

        std::vector<Aabb> bounds;
        bounds.reserve(triangles.size());
        for (std::size_t primitive = 0;
             primitive < triangles.size();
             ++primitive)
        {
            const CollisionTriangle &triangle = triangles[primitive];
            const auto box = makeAabb(
                triangle.points[0],
                triangle.points[1],
                triangle.points[2]);
            if (!box.hasValue())
            {
                return Result<BatchSelfCollisionResult, SpatialError>::failure(
                    box.error());
            }
            if ((triangle.points[1] - triangle.points[0])
                    .cross(triangle.points[2] - triangle.points[0])
                    .squaredNorm() == Scalar{0})
            {
                return Result<BatchSelfCollisionResult, SpatialError>::failure(
                    SpatialError::DegenerateTriangle);
            }
            bounds.push_back(box.value());
        }

        const auto tree = BinaryAabbTree::build(bounds);
        if (!tree.hasValue())
        {
            return Result<BatchSelfCollisionResult, SpatialError>::failure(
                tree.error());
        }

        std::set<std::uint32_t> illegal_owners;
        std::set<std::pair<std::uint32_t, std::uint32_t>> illegal_owner_pairs;
        for (std::size_t first = 0; first < triangles.size(); ++first)
        {
            for (const std::size_t second : tree.value().query(bounds[first]))
            {
                ++result.diagnostics.broad_phase_visits;
                if (second <= first)
                {
                    continue;
                }
                ++result.diagnostics.unique_pairs;

                const CollisionTriangle &left = triangles[first];
                const CollisionTriangle &right = triangles[second];
                if (left.owner_id == right.owner_id)
                {
                    ++result.diagnostics.same_owner_skips;
                    continue;
                }
                ++result.diagnostics.exact_tests;
                const auto illegal = hasIllegalTriangleContact(left, right);
                if (!illegal.hasValue())
                {
                    return Result<BatchSelfCollisionResult, SpatialError>::failure(
                        illegal.error());
                }
                if (!illegal.value())
                {
                    continue;
                }

                illegal_owners.insert(left.owner_id);
                illegal_owners.insert(right.owner_id);
                illegal_owner_pairs.insert(std::minmax(
                    left.owner_id,
                    right.owner_id));
            }
        }

        result.illegal_owner_ids.assign(
            illegal_owners.begin(),
            illegal_owners.end());
        result.diagnostics.illegal_owner_pairs = illegal_owner_pairs.size();
        return Result<BatchSelfCollisionResult, SpatialError>::success(
            std::move(result));
    }

    namespace
    {
        using DetectResult = Result<BatchSelfCollisionResult, SpatialError>;
        DetectResult detectOwnersImpl(
            const std::vector<CollisionOwnerTriangles> &owners,
            const std::vector<std::size_t> *changed_owner_indices)
        {
            BatchSelfCollisionResult result;
            if (owners.empty())
            {
                if (changed_owner_indices != nullptr &&
                    !changed_owner_indices->empty())
                    return DetectResult::failure(
                        SpatialError::InvalidTopologyReference);
                return DetectResult::success(std::move(result));
            }

            std::set<std::uint32_t> seen_owner_ids;
            std::vector<Aabb> owner_bounds;
            std::vector<std::vector<Aabb>> triangle_bounds;
            owner_bounds.reserve(owners.size());
            triangle_bounds.reserve(owners.size());
            for (const CollisionOwnerTriangles &owner : owners)
            {
                if (!validBounds(owner.bounds))
                    return DetectResult::failure(SpatialError::InvalidAabb);
                if (!seen_owner_ids.insert(owner.owner_id).second)
                    return DetectResult::failure(
                        SpatialError::InvalidTopologyReference);
                owner_bounds.push_back(owner.bounds);
                std::vector<Aabb> local_bounds;
                local_bounds.reserve(owner.triangles.size());
                for (const CollisionTriangle &triangle : owner.triangles)
                {
                    const auto box = makeAabb(
                        triangle.points[0], triangle.points[1],
                        triangle.points[2]);
                    if (!box.hasValue())
                        return DetectResult::failure(box.error());
                    if ((triangle.points[1] - triangle.points[0])
                            .cross(triangle.points[2] - triangle.points[0])
                            .squaredNorm() == Scalar{0})
                        return DetectResult::failure(
                            SpatialError::DegenerateTriangle);
                    local_bounds.push_back(box.value());
                    ++result.diagnostics.triangle_count;
                }
                triangle_bounds.push_back(std::move(local_bounds));
            }

            std::vector<bool> changed;
            if (changed_owner_indices != nullptr)
            {
                changed.assign(owners.size(), false);
                for (const std::size_t index : *changed_owner_indices)
                {
                    if (index >= owners.size())
                        return DetectResult::failure(
                            SpatialError::InvalidTopologyReference);
                    changed[index] = true;
                }
            }

            const auto tree = BinaryAabbTree::build(owner_bounds);
            if (!tree.hasValue()) return DetectResult::failure(tree.error());

            std::set<std::uint32_t> illegal_owners;
            for (std::size_t first = 0; first < owners.size(); ++first)
            {
                if (changed_owner_indices != nullptr && !changed[first])
                    continue;
                for (const std::size_t second :
                     tree.value().query(owner_bounds[first]))
                {
                    ++result.diagnostics.broad_phase_visits;
                    if (second == first) continue;
                    if (changed_owner_indices != nullptr && changed[second] &&
                        second < first)
                        continue;
                    ++result.diagnostics.owner_pairs;
                    bool pair_is_illegal = false;
                    for (std::size_t left = 0;
                         left < owners[first].triangles.size() &&
                             !pair_is_illegal;
                         ++left)
                    {
                        for (std::size_t right = 0;
                             right < owners[second].triangles.size(); ++right)
                        {
                            ++result.diagnostics.unique_pairs;
                            if (!overlaps(
                                    triangle_bounds[first][left],
                                    triangle_bounds[second][right]))
                            {
                                ++result.diagnostics.aabb_rejections;
                                continue;
                            }
                            ++result.diagnostics.exact_tests;
                            const auto illegal = hasIllegalTriangleContact(
                                owners[first].triangles[left],
                                owners[second].triangles[right]);
                            if (!illegal.hasValue())
                                return DetectResult::failure(illegal.error());
                            if (illegal.value())
                            {
                                pair_is_illegal = true;
                                break;
                            }
                        }
                    }
                    if (!pair_is_illegal) continue;
                    illegal_owners.insert(owners[first].owner_id);
                    illegal_owners.insert(owners[second].owner_id);
                    ++result.diagnostics.illegal_owner_pairs;
                }
            }
            result.illegal_owner_ids.assign(
                illegal_owners.begin(), illegal_owners.end());
            return DetectResult::success(std::move(result));
        }
    }

    Result<BatchSelfCollisionResult, SpatialError>
    BatchSelfCollisionDetector::detectOwners(
        const std::vector<CollisionOwnerTriangles> &owners)
    {
        return detectOwnersImpl(owners, nullptr);
    }

    Result<BatchSelfCollisionIndex, SpatialError>
    BatchSelfCollisionIndex::build(
        const std::vector<CollisionOwnerTriangles> &owners,
        const std::vector<Aabb> &conservative_owner_bounds)
    {
        using BuildResult = Result<BatchSelfCollisionIndex, SpatialError>;
        if (owners.size() != conservative_owner_bounds.size())
            return BuildResult::failure(
                SpatialError::InvalidTopologyReference);

        BatchSelfCollisionIndex index;
        std::set<std::uint32_t> seen;
        for (const auto &owner : owners)
        {
            if (!seen.insert(owner.owner_id).second)
                return BuildResult::failure(
                    SpatialError::InvalidTopologyReference);
            index.owner_ids_.push_back(owner.owner_id);
        }
        for (std::size_t index = 0;
             index < conservative_owner_bounds.size();
             ++index)
        {
            const Aabb &bounds = conservative_owner_bounds[index];
            if (!validBounds(bounds))
                return BuildResult::failure(SpatialError::InvalidAabb);
            if ((bounds.minimum.array() > owners[index].bounds.minimum.array())
                    .any() ||
                (bounds.maximum.array() < owners[index].bounds.maximum.array())
                    .any())
                return BuildResult::failure(SpatialError::InvalidAabb);
        }

        auto tree = BinaryAabbTree::build(conservative_owner_bounds);
        if (!tree.hasValue()) return BuildResult::failure(tree.error());
        index.tree_ = std::move(tree.value());
        return BuildResult::success(std::move(index));
    }

    Result<BatchSelfCollisionResult, SpatialError>
    BatchSelfCollisionIndex::detectChanged(
        const std::vector<CollisionOwnerTriangles> &active_owners,
        const std::vector<std::size_t> &changed_owner_indices) const
    {
        using DetectResult = Result<BatchSelfCollisionResult, SpatialError>;
        BatchSelfCollisionResult result;
        if (active_owners.size() != owner_ids_.size())
            return DetectResult::failure(
                SpatialError::InvalidTopologyReference);
        std::set<std::size_t> changed;
        for (const std::size_t index : changed_owner_indices)
        {
            if (index >= active_owners.size())
                return DetectResult::failure(
                    SpatialError::InvalidTopologyReference);
            if (active_owners[index].owner_id != owner_ids_[index] ||
                !validBounds(active_owners[index].bounds))
                return DetectResult::failure(
                    SpatialError::InvalidTopologyReference);
            changed.insert(index);
        }

        std::map<std::size_t, std::vector<Aabb>> triangle_bounds;
        const auto boundsFor = [&](std::size_t owner_index)
            -> Result<const std::vector<Aabb> *, SpatialError>
        {
            const auto found = triangle_bounds.find(owner_index);
            if (found != triangle_bounds.end())
                return Result<const std::vector<Aabb> *, SpatialError>::success(
                    &found->second);
            std::vector<Aabb> bounds;
            const auto &triangles = active_owners[owner_index].triangles;
            bounds.reserve(triangles.size());
            for (const CollisionTriangle &triangle : triangles)
            {
                const auto box = makeAabb(
                    triangle.points[0], triangle.points[1],
                    triangle.points[2]);
                if (!box.hasValue())
                    return Result<const std::vector<Aabb> *, SpatialError>::failure(
                        box.error());
                if ((triangle.points[1] - triangle.points[0])
                        .cross(triangle.points[2] - triangle.points[0])
                        .squaredNorm() == Scalar{0})
                    return Result<const std::vector<Aabb> *, SpatialError>::failure(
                        SpatialError::DegenerateTriangle);
                bounds.push_back(box.value());
            }
            result.diagnostics.triangle_count += triangles.size();
            auto inserted = triangle_bounds.emplace(
                owner_index, std::move(bounds));
            return Result<const std::vector<Aabb> *, SpatialError>::success(
                &inserted.first->second);
        };

        std::set<std::uint32_t> illegal_owners;
        std::set<std::pair<std::uint32_t, std::uint32_t>> illegal_pairs;
        std::vector<std::size_t> candidate_indices;
        std::vector<std::size_t> traversal_scratch;
        for (const std::size_t first : changed)
        {
            const auto left_bounds_result = boundsFor(first);
            if (!left_bounds_result.hasValue())
                return DetectResult::failure(left_bounds_result.error());
            const auto &left_bounds = *left_bounds_result.value();
            tree_.query(active_owners[first].bounds,
                        candidate_indices, traversal_scratch);
            for (const std::size_t tree_index : candidate_indices)
            {
                if (tree_index >= active_owners.size())
                    return DetectResult::failure(
                        SpatialError::InvalidTopologyReference);
                const std::size_t second = tree_index;
                if (active_owners[second].owner_id != owner_ids_[second])
                    return DetectResult::failure(
                        SpatialError::InvalidTopologyReference);
                if (second == first || active_owners[second].triangles.empty())
                    continue;
                if (changed.find(second) != changed.end() &&
                    active_owners[second].owner_id <
                        active_owners[first].owner_id)
                    continue;
                ++result.diagnostics.broad_phase_visits;
                ++result.diagnostics.owner_pairs;
                const auto right_bounds_result = boundsFor(second);
                if (!right_bounds_result.hasValue())
                    return DetectResult::failure(right_bounds_result.error());
                const auto &right_bounds = *right_bounds_result.value();
                bool pair_is_illegal = false;
                for (std::size_t left = 0;
                     left < left_bounds.size() && !pair_is_illegal;
                     ++left)
                {
                    for (std::size_t right = 0;
                         right < right_bounds.size();
                         ++right)
                    {
                        ++result.diagnostics.unique_pairs;
                        if (!overlaps(left_bounds[left], right_bounds[right]))
                        {
                            ++result.diagnostics.aabb_rejections;
                            continue;
                        }
                        ++result.diagnostics.exact_tests;
                        const auto illegal = hasIllegalTriangleContact(
                            active_owners[first].triangles[left],
                            active_owners[second].triangles[right]);
                        if (!illegal.hasValue())
                            return DetectResult::failure(illegal.error());
                        if (illegal.value())
                        {
                            pair_is_illegal = true;
                            break;
                        }
                    }
                }
                if (!pair_is_illegal) continue;
                illegal_owners.insert(active_owners[first].owner_id);
                illegal_owners.insert(active_owners[second].owner_id);
                illegal_pairs.insert(std::minmax(
                    active_owners[first].owner_id,
                    active_owners[second].owner_id));
            }
        }
        result.illegal_owner_ids.assign(
            illegal_owners.begin(), illegal_owners.end());
        result.diagnostics.illegal_owner_pairs = illegal_pairs.size();
        return DetectResult::success(std::move(result));
    }
}
