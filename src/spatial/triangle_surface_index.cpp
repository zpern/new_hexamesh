#include <boundary_mesh/spatial/triangle_surface_index.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace boundary_mesh
{
    namespace
    {
        Point3 closestPointOnTriangle(
            const Point3 &point,
            const std::array<Point3, 3> &triangle)
        {
            const Point3 &a = triangle[0];
            const Point3 &b = triangle[1];
            const Point3 &c = triangle[2];
            const Vector3 ab = b - a;
            const Vector3 ac = c - a;
            const Vector3 ap = point - a;
            const Scalar d1 = ab.dot(ap);
            const Scalar d2 = ac.dot(ap);
            if (d1 <= 0 && d2 <= 0) return a;

            const Vector3 bp = point - b;
            const Scalar d3 = ab.dot(bp);
            const Scalar d4 = ac.dot(bp);
            if (d3 >= 0 && d4 <= d3) return b;

            const Scalar vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0)
                return a + (d1 / (d1 - d3)) * ab;

            const Vector3 cp = point - c;
            const Scalar d5 = ab.dot(cp);
            const Scalar d6 = ac.dot(cp);
            if (d6 >= 0 && d5 <= d6) return c;

            const Scalar vb = d5 * d2 - d1 * d6;
            if (vb <= 0 && d2 >= 0 && d6 <= 0)
                return a + (d2 / (d2 - d6)) * ac;

            const Scalar va = d3 * d6 - d5 * d4;
            if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0)
                return b + ((d4 - d3) /
                    ((d4 - d3) + (d5 - d6))) * (c - b);

            const Scalar denominator = Scalar{1} / (va + vb + vc);
            return a + ab * (vb * denominator) +
                ac * (vc * denominator);
        }
    }

    Result<TriangleSurfaceIndex, SpatialError>
    TriangleSurfaceIndex::build(std::vector<SurfaceTriangle> triangles)
    {
        std::stable_sort(
            triangles.begin(), triangles.end(),
            [](const SurfaceTriangle &left, const SurfaceTriangle &right)
            {
                return std::tie(left.source_face_id, left.local_triangle_id) <
                    std::tie(right.source_face_id, right.local_triangle_id);
            });
        std::vector<Aabb> bounds;
        bounds.reserve(triangles.size());
        for (const SurfaceTriangle &triangle : triangles)
        {
            for (const Point3 &point : triangle.points)
                if (!point.allFinite())
                    return Result<TriangleSurfaceIndex, SpatialError>::failure(
                        SpatialError::NonFiniteCoordinate);
            const Vector3 normal =
                (triangle.points[1] - triangle.points[0]).cross(
                    triangle.points[2] - triangle.points[0]);
            if (!normal.allFinite() || normal.squaredNorm() == Scalar{0})
                return Result<TriangleSurfaceIndex, SpatialError>::failure(
                    SpatialError::DegenerateTriangle);
            const auto box = makeAabb(
                triangle.points[0], triangle.points[1], triangle.points[2]);
            if (!box.hasValue())
                return Result<TriangleSurfaceIndex, SpatialError>::failure(
                    box.error());
            bounds.push_back(box.value());
        }
        const auto tree = BinaryAabbTree::build(bounds);
        if (!tree.hasValue())
            return Result<TriangleSurfaceIndex, SpatialError>::failure(
                tree.error());
        TriangleSurfaceIndex output;
        output.triangles_ = std::move(triangles);
        output.tree_ = tree.value();
        return Result<TriangleSurfaceIndex, SpatialError>::success(
            std::move(output));
    }

    Result<ClosestSurfacePoint, SpatialError>
    TriangleSurfaceIndex::closestPoint(const Point3 &query) const
    {
        if (!query.allFinite())
            return Result<ClosestSurfacePoint, SpatialError>::failure(
                SpatialError::NonFiniteCoordinate);
        if (triangles_.empty())
            return Result<ClosestSurfacePoint, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);

        Scalar distance{};
        const std::size_t primitive = tree_.nearest(
            query,
            [&](std::size_t index)
            {
                return (closestPointOnTriangle(query, triangles_[index].points) -
                        query).squaredNorm();
            },
            distance);
        const SurfaceTriangle &triangle = triangles_[primitive];
        const Point3 candidate = closestPointOnTriangle(query, triangle.points);
        Vector3 normal =
            (triangle.points[1] - triangle.points[0]).cross(
                triangle.points[2] - triangle.points[0]);
        normal.normalize();
        const ClosestSurfacePoint best{
            candidate, normal, distance,
            triangle.source_face_id, triangle.local_triangle_id};
        return Result<ClosestSurfacePoint, SpatialError>::success(best);
    }
}
