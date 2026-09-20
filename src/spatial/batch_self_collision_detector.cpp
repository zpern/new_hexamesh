#include <boundary_mesh/spatial/batch_self_collision_detector.hpp>

#include <set>
#include <limits>
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

    Result<BatchSelfCollisionResult, SpatialError>
    BatchSelfCollisionDetector::detectOwners(
        const std::vector<CollisionOwnerTriangles> &owners)
    {
        using DetectResult = Result<BatchSelfCollisionResult, SpatialError>;
        BatchSelfCollisionResult result;
        if (owners.empty()) return DetectResult::success(std::move(result));

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
                    triangle.points[0], triangle.points[1], triangle.points[2]);
                if (!box.hasValue()) return DetectResult::failure(box.error());
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

        const auto tree = BinaryAabbTree::build(owner_bounds);
        if (!tree.hasValue()) return DetectResult::failure(tree.error());

        std::set<std::uint32_t> illegal_owners;
        for (std::size_t first = 0; first < owners.size(); ++first)
        {
            for (const std::size_t second : tree.value().query(owner_bounds[first]))
            {
                ++result.diagnostics.broad_phase_visits;
                if (second <= first) continue;
                ++result.diagnostics.owner_pairs;
                bool pair_is_illegal = false;
                for (std::size_t left = 0;
                     left < owners[first].triangles.size() && !pair_is_illegal;
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
