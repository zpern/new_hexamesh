#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    struct Aabb
    {
        Point3 minimum{}; // 三个坐标分量的下界
        Point3 maximum{}; // 三个坐标分量的上界
    };

    bool overlaps(
        const Aabb &left,
        const Aabb &right) noexcept;

    Result<Aabb, SpatialError> makeAabb(
        const Point3 &first,
        const Point3 &second,
        const Point3 &third);

    Result<Aabb, SpatialError> makeAabb(
        const Point3 &first,
        const Point3 &second,
        const Point3 &third,
        const Point3 &fourth);
}
