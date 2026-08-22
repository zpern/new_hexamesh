#include <boundary_mesh/spatial/aabb.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace boundary_mesh
{
    namespace
    {
        template <std::size_t Size>
        Result<Aabb, SpatialError> makeAabbFromPoints(
            const std::array<const Point3 *, Size> &points)
        {
            for (const Point3 *point : points)
            {
                if (!point->allFinite())
                {
                    return Result<Aabb, SpatialError>::failure(
                        SpatialError::NonFiniteCoordinate);
                }
            }

            Aabb box{*points.front(), *points.front()};
            for (const Point3 *point : points)
            {
                box.minimum = box.minimum.cwiseMin(*point);
                box.maximum = box.maximum.cwiseMax(*point);
            }

            if ((box.minimum.array() > box.maximum.array()).any())
            {
                return Result<Aabb, SpatialError>::failure(
                    SpatialError::InvalidAabb);
            }

            return Result<Aabb, SpatialError>::success(box);
        }
    }

    bool overlaps(
        const Aabb &left,
        const Aabb &right) noexcept
    {
        return left.minimum.x() <= right.maximum.x() &&
               left.maximum.x() >= right.minimum.x() &&
               left.minimum.y() <= right.maximum.y() &&
               left.maximum.y() >= right.minimum.y() &&
               left.minimum.z() <= right.maximum.z() &&
               left.maximum.z() >= right.minimum.z();
    }

    Result<Aabb, SpatialError> makeAabb(
        const Point3 &first,
        const Point3 &second,
        const Point3 &third)
    {
        return makeAabbFromPoints<3>({&first, &second, &third});
    }

    Result<Aabb, SpatialError> makeAabb(
        const Point3 &first,
        const Point3 &second,
        const Point3 &third,
        const Point3 &fourth)
    {
        return makeAabbFromPoints<4>(
            {&first, &second, &third, &fourth});
    }
}
