#include <boundary_mesh/spatial/batch_self_collision_detector.hpp>

#include <algorithm>
#include <array>
#include <set>
#include <utility>

#include <boundary_mesh/spatial/aabb.hpp>
#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct BoundedPrimitive
        {
            std::size_t primitive{};
            Aabb bounds{};
        };

        std::size_t longestAxis(const Aabb &bounds) noexcept
        {
            const Point3 extent = bounds.maximum - bounds.minimum;
            if (extent.y() > extent.x() && extent.y() >= extent.z())
            {
                return 1;
            }
            if (extent.z() > extent.x() && extent.z() > extent.y())
            {
                return 2;
            }
            return 0;
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

        std::vector<BoundedPrimitive> bounded;
        bounded.reserve(triangles.size());
        Aabb overall{};
        for (std::size_t primitive = 0;
             primitive < triangles.size();
             ++primitive)
        {
            const CollisionTriangle &triangle = triangles[primitive];
            const auto bounds = makeAabb(
                triangle.points[0],
                triangle.points[1],
                triangle.points[2]);
            if (!bounds.hasValue())
            {
                return Result<BatchSelfCollisionResult, SpatialError>::failure(
                    bounds.error());
            }
            if ((triangle.points[1] - triangle.points[0])
                    .cross(triangle.points[2] - triangle.points[0])
                    .squaredNorm() == Scalar{0})
            {
                return Result<BatchSelfCollisionResult, SpatialError>::failure(
                    SpatialError::DegenerateTriangle);
            }
            if (bounded.empty())
            {
                overall = bounds.value();
            }
            else
            {
                overall.minimum = overall.minimum.cwiseMin(
                    bounds.value().minimum);
                overall.maximum = overall.maximum.cwiseMax(
                    bounds.value().maximum);
            }
            bounded.push_back({primitive, bounds.value()});
        }

        const std::size_t axis = longestAxis(overall);
        std::stable_sort(
            bounded.begin(),
            bounded.end(),
            [axis](
                const BoundedPrimitive &left,
                const BoundedPrimitive &right)
            {
                if (left.bounds.minimum[axis] != right.bounds.minimum[axis])
                {
                    return left.bounds.minimum[axis] <
                           right.bounds.minimum[axis];
                }
                return left.primitive < right.primitive;
            });

        std::set<std::uint32_t> illegal_owners;
        std::set<std::pair<std::uint32_t, std::uint32_t>> illegal_owner_pairs;
        for (std::size_t first = 0; first < bounded.size(); ++first)
        {
            for (std::size_t second = first + 1;
                 second < bounded.size();
                 ++second)
            {
                if (bounded[second].bounds.minimum[axis] >
                    bounded[first].bounds.maximum[axis])
                {
                    break;
                }
                ++result.diagnostics.sweep_pairs;
                ++result.diagnostics.unique_pairs;

                const CollisionTriangle &left =
                    triangles[bounded[first].primitive];
                const CollisionTriangle &right =
                    triangles[bounded[second].primitive];
                if (left.owner_id == right.owner_id)
                {
                    ++result.diagnostics.same_owner_skips;
                    continue;
                }
                if (!overlaps(
                        bounded[first].bounds,
                        bounded[second].bounds))
                {
                    ++result.diagnostics.aabb_rejections;
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
