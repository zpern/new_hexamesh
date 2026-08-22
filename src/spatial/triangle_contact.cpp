#include <boundary_mesh/spatial/triangle_contact.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <geom_func.h>

namespace boundary_mesh
{
    namespace
    {
        struct Point2
        {
            Scalar x{};
            Scalar y{};
        };

        bool sameKey(
            const CollisionVertexKey &left,
            const CollisionVertexKey &right)
        {
            return left.source_vertex_id == right.source_vertex_id &&
                   left.layer == right.layer;
        }

        bool validTriangle(const TrianglePoints &triangle)
        {
            return triangle[0].allFinite() &&
                   triangle[1].allFinite() &&
                   triangle[2].allFinite();
        }

        Vector3 normal(const TrianglePoints &triangle)
        {
            return (triangle[1] - triangle[0]).cross(
                triangle[2] - triangle[0]);
        }

        std::array<std::array<double, 3>, 3> mutablePoints(
            const TrianglePoints &triangle)
        {
            return {{{triangle[0].x(), triangle[0].y(), triangle[0].z()},
                     {triangle[1].x(), triangle[1].y(), triangle[1].z()},
                     {triangle[2].x(), triangle[2].y(), triangle[2].z()}}};
        }

        int dominantAxis(const Vector3 &value)
        {
            const Vector3 absolute = value.cwiseAbs();
            if (absolute.x() >= absolute.y() &&
                absolute.x() >= absolute.z())
            {
                return 0;
            }
            return absolute.y() >= absolute.z() ? 1 : 2;
        }

        Point2 project(const Point3 &point, int axis)
        {
            if (axis == 0)
            {
                return {point.y(), point.z()};
            }
            if (axis == 1)
            {
                return {point.x(), point.z()};
            }
            return {point.x(), point.y()};
        }

        Scalar cross2(
            const Point2 &first,
            const Point2 &second,
            const Point2 &third)
        {
            return (second.x - first.x) * (third.y - first.y) -
                   (second.y - first.y) * (third.x - first.x);
        }

        std::vector<Point2> clipPolygon(
            std::vector<Point2> polygon,
            std::array<Point2, 3> clip)
        {
            const Scalar orientation = cross2(clip[0], clip[1], clip[2]);
            if (orientation < Scalar{0})
            {
                std::swap(clip[1], clip[2]);
            }

            for (std::size_t edge = 0; edge < 3 && !polygon.empty(); ++edge)
            {
                const Point2 a = clip[edge];
                const Point2 b = clip[(edge + 1) % 3];
                std::vector<Point2> output;
                Point2 previous = polygon.back();
                Scalar previous_side = cross2(a, b, previous);
                for (const Point2 current : polygon)
                {
                    const Scalar current_side = cross2(a, b, current);
                    const bool previous_inside = previous_side >= Scalar{0};
                    const bool current_inside = current_side >= Scalar{0};
                    if (previous_inside != current_inside)
                    {
                        const Scalar denominator =
                            previous_side - current_side;
                        const Scalar ratio = previous_side / denominator;
                        output.push_back({
                            previous.x + ratio * (current.x - previous.x),
                            previous.y + ratio * (current.y - previous.y)});
                    }
                    if (current_inside)
                    {
                        output.push_back(current);
                    }
                    previous = current;
                    previous_side = current_side;
                }
                polygon = std::move(output);
            }
            return polygon;
        }

        Scalar polygonArea(const std::vector<Point2> &polygon)
        {
            Scalar twice_area = Scalar{0};
            for (std::size_t index = 0; index < polygon.size(); ++index)
            {
                const Point2 &first = polygon[index];
                const Point2 &second = polygon[(index + 1) % polygon.size()];
                twice_area += first.x * second.y - first.y * second.x;
            }
            return std::abs(twice_area) * Scalar{0.5};
        }

        std::size_t distinctPointCount(
            const std::vector<Point2> &points)
        {
            std::vector<Point2> distinct;
            for (const Point2 point : points)
            {
                const bool exists = std::any_of(
                    distinct.begin(),
                    distinct.end(),
                    [&](const Point2 &value)
                    {
                        return value.x == point.x && value.y == point.y;
                    });
                if (!exists)
                {
                    distinct.push_back(point);
                }
            }
            return distinct.size();
        }

        bool pointInTriangle(
            const Point3 &point,
            const TrianglePoints &triangle,
            const Vector3 &triangle_normal)
        {
            const Scalar first = triangle_normal.dot(
                (triangle[1] - triangle[0]).cross(point - triangle[0]));
            const Scalar second = triangle_normal.dot(
                (triangle[2] - triangle[1]).cross(point - triangle[1]));
            const Scalar third = triangle_normal.dot(
                (triangle[0] - triangle[2]).cross(point - triangle[2]));
            return (first >= 0 && second >= 0 && third >= 0) ||
                   (first <= 0 && second <= 0 && third <= 0);
        }

        void appendUnique(
            std::vector<Point3> &points,
            const Point3 &point)
        {
            const auto found = std::find_if(
                points.begin(),
                points.end(),
                [&](const Point3 &existing)
                {
                    return (existing.array() == point.array()).all();
                });
            if (found == points.end())
            {
                points.push_back(point);
            }
        }

        void appendPlaneIntersections(
            const TrianglePoints &source,
            const TrianglePoints &target,
            const Vector3 &target_normal,
            std::vector<Point3> &points)
        {
            for (std::size_t edge = 0; edge < 3; ++edge)
            {
                const Point3 &first = source[edge];
                const Point3 &second = source[(edge + 1) % 3];
                const Scalar first_distance =
                    target_normal.dot(first - target[0]);
                const Scalar second_distance =
                    target_normal.dot(second - target[0]);

                if (first_distance == Scalar{0} &&
                    pointInTriangle(first, target, target_normal))
                {
                    appendUnique(points, first);
                }
                if (second_distance == Scalar{0} &&
                    pointInTriangle(second, target, target_normal))
                {
                    appendUnique(points, second);
                }
                if ((first_distance < Scalar{0} &&
                     second_distance > Scalar{0}) ||
                    (first_distance > Scalar{0} &&
                     second_distance < Scalar{0}))
                {
                    const Scalar ratio = first_distance /
                        (first_distance - second_distance);
                    const Point3 point = first + ratio * (second - first);
                    if (pointInTriangle(point, target, target_normal))
                    {
                        appendUnique(points, point);
                    }
                }
            }
        }

        bool pointsLieOnOneEdge(
            const std::vector<Point3> &points,
            const TrianglePoints &triangle)
        {
            for (std::size_t edge = 0; edge < 3; ++edge)
            {
                const Point3 &first = triangle[edge];
                const Point3 &second = triangle[(edge + 1) % 3];
                const Vector3 direction = second - first;
                bool all_on_edge = true;
                for (const Point3 &point : points)
                {
                    if (direction.cross(point - first).squaredNorm() != Scalar{0} ||
                        (point - first).dot(point - second) > Scalar{0})
                    {
                        all_on_edge = false;
                        break;
                    }
                }
                if (all_on_edge)
                {
                    return true;
                }
            }
            return false;
        }
    }

    Result<TriangleContactKind, SpatialError>
    classifyTriangleContact(
        const TrianglePoints &first,
        const TrianglePoints &second)
    {
        if (!validTriangle(first) || !validTriangle(second))
        {
            return Result<TriangleContactKind, SpatialError>::failure(
                SpatialError::NonFiniteCoordinate);
        }

        const Vector3 first_normal = normal(first);
        const Vector3 second_normal = normal(second);
        if (first_normal.squaredNorm() == Scalar{0} ||
            second_normal.squaredNorm() == Scalar{0})
        {
            return Result<TriangleContactKind, SpatialError>::failure(
                SpatialError::DegenerateTriangle);
        }

        auto first_values = mutablePoints(first);
        auto second_values = mutablePoints(second);
        const int contact = TiGER_GEOM_FUNC::tri_tri_overlap_test_3d(
            first_values[0].data(),
            first_values[1].data(),
            first_values[2].data(),
            second_values[0].data(),
            second_values[1].data(),
            second_values[2].data());
        if (contact == 0)
        {
            return Result<TriangleContactKind, SpatialError>::success(
                TriangleContactKind::Disjoint);
        }

        const bool coplanar =
            first_normal.dot(second[0] - first[0]) == Scalar{0} &&
            first_normal.dot(second[1] - first[0]) == Scalar{0} &&
            first_normal.dot(second[2] - first[0]) == Scalar{0};
        if (coplanar)
        {
            const int axis = dominantAxis(first_normal);
            std::vector<Point2> polygon{
                project(first[0], axis),
                project(first[1], axis),
                project(first[2], axis)};
            const std::array<Point2, 3> clip{{
                project(second[0], axis),
                project(second[1], axis),
                project(second[2], axis)}};
            polygon = clipPolygon(std::move(polygon), clip);
            if (polygon.size() >= 3 && polygonArea(polygon) > Scalar{0})
            {
                return Result<TriangleContactKind, SpatialError>::success(
                    TriangleContactKind::CoplanarOverlap);
            }
            return Result<TriangleContactKind, SpatialError>::success(
                distinctPointCount(polygon) >= 2
                    ? TriangleContactKind::EdgeTouch
                    : TriangleContactKind::VertexTouch);
        }

        std::vector<Point3> points;
        appendPlaneIntersections(first, second, second_normal, points);
        appendPlaneIntersections(second, first, first_normal, points);
        if (points.size() <= 1)
        {
            return Result<TriangleContactKind, SpatialError>::success(
                TriangleContactKind::VertexTouch);
        }
        return Result<TriangleContactKind, SpatialError>::success(
            pointsLieOnOneEdge(points, first) &&
                    pointsLieOnOneEdge(points, second)
                ? TriangleContactKind::EdgeTouch
                : TriangleContactKind::ProperIntersect);
    }

    Result<bool, SpatialError> hasIllegalTriangleContact(
        const TrianglePoints &first,
        const std::array<CollisionVertexKey, 3> &first_keys,
        const TrianglePoints &second,
        const std::array<CollisionVertexKey, 3> &second_keys)
    {
        const auto classification =
            classifyTriangleContact(first, second);
        if (!classification.hasValue())
        {
            return Result<bool, SpatialError>::failure(
                classification.error());
        }
        if (classification.value() == TriangleContactKind::Disjoint)
        {
            return Result<bool, SpatialError>::success(false);
        }

        std::size_t shared_count = 0;
        for (const CollisionVertexKey &first_key : first_keys)
        {
            for (const CollisionVertexKey &second_key : second_keys)
            {
                if (sameKey(first_key, second_key))
                {
                    ++shared_count;
                    break;
                }
            }
        }

        bool legal = false;
        if (shared_count == 1)
        {
            legal = classification.value() ==
                TriangleContactKind::VertexTouch;
        }
        else if (shared_count == 2)
        {
            legal = classification.value() ==
                TriangleContactKind::EdgeTouch;
        }
        else if (shared_count == 3)
        {
            legal = classification.value() ==
                TriangleContactKind::CoplanarOverlap;
        }
        return Result<bool, SpatialError>::success(!legal);
    }
}
