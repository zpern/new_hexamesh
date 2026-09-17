#include <boundary_mesh/spatial/batch_self_collision_detector.hpp>

#include <set>
#include <utility>

#include <boundary_mesh/spatial/aabb.hpp>
#include <boundary_mesh/spatial/binary_aabb_tree.hpp>
#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
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
}
