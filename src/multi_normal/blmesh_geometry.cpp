#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <boundary_mesh/multi_normal/detail/blmesh_geometry.hpp>

namespace boundary_mesh::detail
{
    Vector3 blmeshCircleCenter(
        const Vector3 &first,
        const Vector3 &second,
        const Vector3 &third)
    {
        const Scalar x1 = first.x(), y1 = first.y(), z1 = first.z();
        const Scalar x2 = second.x(), y2 = second.y(), z2 = second.z();
        const Scalar x3 = third.x(), y3 = third.y(), z3 = third.z();

        const Scalar a1 = y1*z2-y2*z1-y1*z3+y3*z1+y2*z3-y3*z2;
        const Scalar b1 = -(x1*z2-x2*z1-x1*z3+x3*z1+x2*z3-x3*z2);
        const Scalar c1 = x1*y2-x2*y1-x1*y3+x3*y1+x2*y3-x3*y2;
        const Scalar d1 = -(x1*y2*z3-x1*y3*z2-x2*y1*z3+x2*y3*z1+x3*y1*z2-x3*y2*z1);
        const Scalar a2 = Scalar{2} * (x2-x1);
        const Scalar b2 = Scalar{2} * (y2-y1);
        const Scalar c2 = Scalar{2} * (z2-z1);
        const Scalar d2 = first.squaredNorm()-second.squaredNorm();
        const Scalar a3 = Scalar{2} * (x3-x1);
        const Scalar b3 = Scalar{2} * (y3-y1);
        const Scalar c3 = Scalar{2} * (z3-z1);
        const Scalar d3 = first.squaredNorm()-third.squaredNorm();
        const Scalar denominator =
            a1*b2*c3-a1*b3*c2-a2*b1*c3+a2*b3*c1+a3*b1*c2-a3*b2*c1;

        Vector3 center;
        center.x() = -(b1*c2*d3-b1*c3*d2-b2*c1*d3+b2*c3*d1+b3*c1*d2-b3*c2*d1)/denominator;
        center.y() = (a1*c2*d3-a1*c3*d2-a2*c1*d3+a2*c3*d1+a3*c1*d2-a3*c2*d1)/denominator;
        center.z() = -(a1*b2*d3-a1*b3*d2-a2*b1*d3+a2*b3*d1+a3*b1*d2-a3*b2*d1)/denominator;
        return center;
    }

    Scalar blmeshMinCos(
        const Vector3 &direction,
        const std::vector<Vector3> &neighbor_normals)
    {
        Scalar minimum = Scalar{1};
        for (const Vector3 &normal : neighbor_normals)
        {
            minimum = std::min(minimum, normal.dot(direction));
        }
        return minimum;
    }

    Vector3 blmeshMostNormal(
        const std::vector<Vector3> &neighbor_normals)
    {
        Vector3 result = Vector3::Zero();
        if (neighbor_normals.size() == 1) return neighbor_normals[0];
        if (neighbor_normals.size() == 2)
        {
            return (neighbor_normals[0] + neighbor_normals[1]).normalized();
        }

        Scalar best_visibility = Scalar{-10};
        const std::size_t count = neighbor_normals.size();
        for (std::size_t first = 0; first < count; ++first)
        {
            for (std::size_t second = first + 1; second < count; ++second)
            {
                for (std::size_t third = second + 1; third < count; ++third)
                {
                    const Vector3 normal = blmeshCircleCenter(
                        neighbor_normals[first], neighbor_normals[second],
                        neighbor_normals[third]).normalized();
                    if (std::isnan(normal.x())) continue;
                    const Scalar visibility =
                        blmeshMinCos(normal, neighbor_normals);
                    if (best_visibility < visibility)
                    {
                        best_visibility = visibility;
                        result = normal;
                    }
                }
            }
        }
        for (std::size_t first = 0; first < count; ++first)
        {
            for (std::size_t second = first + 1; second < count; ++second)
            {
                const Vector3 normal =
                    (neighbor_normals[first] + neighbor_normals[second]).normalized();
                if (std::isnan(normal.x())) continue;
                const Scalar visibility = blmeshMinCos(normal, neighbor_normals);
                if (best_visibility < visibility)
                {
                    best_visibility = visibility;
                    result = normal;
                }
            }
        }
        return result;
    }
}
