#include <boundary_mesh/spatial/sliding_intersection.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace boundary_mesh
{
    namespace
    {
        struct Point2 { Scalar x{}; Scalar y{}; };

        bool finite(const TrianglePoints &triangle)
        {
            return std::all_of(triangle.begin(), triangle.end(),
                [](const Point3 &point) { return point.allFinite(); });
        }

        Scalar distanceSquared(const Point3 &a, const Point3 &b)
        {
            return (a - b).squaredNorm();
        }

        Vector3 normal(const TrianglePoints &triangle)
        {
            return (triangle[1] - triangle[0]).cross(
                triangle[2] - triangle[0]);
        }

        Scalar toleranceFor(
            const TrianglePoints &first,
            const TrianglePoints &second)
        {
            Scalar scale{1};
            for (const TrianglePoints *triangle : {&first, &second})
            {
                for (const Point3 &point : *triangle)
                    scale = std::max(scale, point.cwiseAbs().maxCoeff());
                for (std::size_t i = 0; i < 3; ++i)
                    scale = std::max(scale,
                        ((*triangle)[(i + 1) % 3] - (*triangle)[i]).norm());
            }
            return std::max(
                Scalar{1e-10},
                Scalar{64} * std::numeric_limits<Scalar>::epsilon() * scale);
        }

        bool degenerate(const TrianglePoints &triangle, Scalar tolerance)
        {
            const Scalar longest_squared = std::max({
                distanceSquared(triangle[0], triangle[1]),
                distanceSquared(triangle[1], triangle[2]),
                distanceSquared(triangle[2], triangle[0])});
            return normal(triangle).squaredNorm() <=
                tolerance * tolerance * std::max(Scalar{1}, longest_squared);
        }

        bool boxesOverlap(
            const TrianglePoints &first,
            const TrianglePoints &second,
            Scalar tolerance)
        {
            for (Eigen::Index axis = 0; axis < 3; ++axis)
            {
                Scalar first_min = first[0][axis];
                Scalar first_max = first_min;
                Scalar second_min = second[0][axis];
                Scalar second_max = second_min;
                for (std::size_t i = 1; i < 3; ++i)
                {
                    first_min = std::min(first_min, first[i][axis]);
                    first_max = std::max(first_max, first[i][axis]);
                    second_min = std::min(second_min, second[i][axis]);
                    second_max = std::max(second_max, second[i][axis]);
                }
                if (first_max + tolerance < second_min ||
                    second_max + tolerance < first_min)
                    return false;
            }
            return true;
        }

        bool pointInTriangle(
            const Point3 &point,
            const TrianglePoints &triangle,
            Scalar tolerance)
        {
            const Vector3 v0 = triangle[1] - triangle[0];
            const Vector3 v1 = triangle[2] - triangle[0];
            const Vector3 v2 = point - triangle[0];
            const Scalar d00 = v0.dot(v0);
            const Scalar d01 = v0.dot(v1);
            const Scalar d11 = v1.dot(v1);
            const Scalar d20 = v2.dot(v0);
            const Scalar d21 = v2.dot(v1);
            const Scalar denominator = d00 * d11 - d01 * d01;
            if (std::abs(denominator) <=
                std::numeric_limits<Scalar>::epsilon())
                return false;
            const Scalar v = (d11 * d20 - d01 * d21) / denominator;
            const Scalar w = (d00 * d21 - d01 * d20) / denominator;
            const Scalar u = Scalar{1} - v - w;
            const Scalar scale = std::max({
                Scalar{1}, std::sqrt(d00), std::sqrt(d11)});
            const Scalar barycentric_tolerance = tolerance / scale;
            return u >= -barycentric_tolerance &&
                   v >= -barycentric_tolerance &&
                   w >= -barycentric_tolerance;
        }

        void addUnique(
            std::vector<Point3> &points,
            const Point3 &point,
            Scalar tolerance)
        {
            const Scalar squared = tolerance * tolerance;
            if (std::none_of(points.begin(), points.end(),
                    [&](const Point3 &existing) {
                        return distanceSquared(existing, point) <= squared;
                    }))
                points.push_back(point);
        }

        void appendPlaneIntersections(
            const TrianglePoints &source,
            const TrianglePoints &target,
            const Vector3 &target_normal,
            Scalar target_normal_length,
            Scalar tolerance,
            std::vector<Point3> &points)
        {
            std::array<Scalar, 3> distances{};
            for (std::size_t i = 0; i < 3; ++i)
                distances[i] = target_normal.dot(source[i] - target[0]) /
                    target_normal_length;
            for (std::size_t i = 0; i < 3; ++i)
            {
                const std::size_t j = (i + 1) % 3;
                if (std::abs(distances[i]) <= tolerance &&
                    pointInTriangle(source[i], target, tolerance))
                    addUnique(points, source[i], tolerance);
                if ((distances[i] < -tolerance && distances[j] > tolerance) ||
                    (distances[i] > tolerance && distances[j] < -tolerance))
                {
                    const Scalar fraction = distances[i] /
                        (distances[i] - distances[j]);
                    const Point3 point = source[i] +
                        fraction * (source[j] - source[i]);
                    if (pointInTriangle(point, target, tolerance))
                        addUnique(points, point, tolerance);
                }
            }
        }

        Eigen::Index dominantAxis(const Vector3 &value)
        {
            Eigen::Index axis{};
            value.cwiseAbs().maxCoeff(&axis);
            return axis;
        }

        Point2 project(const Point3 &point, Eigen::Index dropped_axis)
        {
            if (dropped_axis == 0) return {point.y(), point.z()};
            if (dropped_axis == 1) return {point.x(), point.z()};
            return {point.x(), point.y()};
        }

        Scalar cross2(const Point2 &a, const Point2 &b, const Point2 &c)
        {
            return (b.x - a.x) * (c.y - a.y) -
                   (b.y - a.y) * (c.x - a.x);
        }

        Scalar polygonArea2(const std::vector<Point2> &polygon)
        {
            Scalar area{};
            for (std::size_t i = 0; i < polygon.size(); ++i)
            {
                const Point2 &a = polygon[i];
                const Point2 &b = polygon[(i + 1) % polygon.size()];
                area += a.x * b.y - a.y * b.x;
            }
            return area;
        }

        Point2 lineIntersection(
            const Point2 &start, const Point2 &end,
            const Point2 &a, const Point2 &b)
        {
            const Scalar sx = end.x - start.x;
            const Scalar sy = end.y - start.y;
            const Scalar ax = b.x - a.x;
            const Scalar ay = b.y - a.y;
            const Scalar denominator = sx * ay - sy * ax;
            if (std::abs(denominator) <=
                std::numeric_limits<Scalar>::epsilon())
                return end;
            const Scalar fraction =
                ((a.x - start.x) * ay - (a.y - start.y) * ax) /
                denominator;
            return {start.x + fraction * sx, start.y + fraction * sy};
        }

        std::vector<Point2> clipCoplanar(
            const TrianglePoints &candidate,
            const TrianglePoints &surface,
            Eigen::Index dropped_axis,
            Scalar tolerance)
        {
            std::vector<Point2> polygon;
            for (const Point3 &point : candidate)
                polygon.push_back(project(point, dropped_axis));
            std::array<Point2, 3> clip{};
            for (std::size_t i = 0; i < 3; ++i)
                clip[i] = project(surface[i], dropped_axis);
            const Scalar orientation =
                cross2(clip[0], clip[1], clip[2]) >= 0 ? 1 : -1;
            for (std::size_t edge = 0; edge < 3 && !polygon.empty(); ++edge)
            {
                const Point2 a = clip[edge];
                const Point2 b = clip[(edge + 1) % 3];
                const Scalar cross_tolerance = tolerance *
                    std::hypot(b.x - a.x, b.y - a.y);
                std::vector<Point2> output;
                Point2 previous = polygon.back();
                bool previous_inside = orientation *
                    cross2(a, b, previous) >= -cross_tolerance;
                for (const Point2 &current : polygon)
                {
                    const bool current_inside = orientation *
                        cross2(a, b, current) >= -cross_tolerance;
                    if (current_inside != previous_inside)
                        output.push_back(lineIntersection(
                            previous, current, a, b));
                    if (current_inside) output.push_back(current);
                    previous = current;
                    previous_inside = current_inside;
                }
                polygon.swap(output);
            }
            return polygon;
        }

        std::array<std::size_t, 2> edgeVertices(std::size_t edge)
        {
            static const std::array<std::array<std::size_t, 2>, 3> edges{{
                {{0, 1}}, {{1, 2}}, {{2, 0}}}};
            return edges[edge];
        }

        Scalar pointSegmentDistanceSquared(
            const Point3 &point,
            const Point3 &first,
            const Point3 &second)
        {
            const Vector3 edge = second - first;
            const Scalar length_squared = edge.squaredNorm();
            if (length_squared == Scalar{0})
                return distanceSquared(point, first);
            const Scalar fraction = std::clamp(
                (point - first).dot(edge) / length_squared,
                Scalar{0}, Scalar{1});
            return distanceSquared(point, first + fraction * edge);
        }

        bool pointPermitted(
            const Point3 &point,
            const TrianglePoints &candidate,
            const SlidingContactPermission &permission,
            Scalar tolerance)
        {
            const Scalar squared = tolerance * tolerance;
            for (std::size_t vertex = 0; vertex < 3; ++vertex)
                if ((permission.vertex_mask & (1u << vertex)) != 0 &&
                    distanceSquared(point, candidate[vertex]) <= squared)
                    return true;
            for (std::size_t edge = 0; edge < 3; ++edge)
            {
                if ((permission.edge_mask & (1u << edge)) == 0) continue;
                const auto vertices = edgeVertices(edge);
                if (pointSegmentDistanceSquared(
                        point, candidate[vertices[0]], candidate[vertices[1]]) <=
                    squared)
                    return true;
            }
            return false;
        }

        bool segmentPermitted(
            const std::array<Point3, 2> &segment,
            const TrianglePoints &candidate,
            const SlidingContactPermission &permission,
            Scalar tolerance)
        {
            if (distanceSquared(segment[0], segment[1]) <=
                tolerance * tolerance)
                return pointPermitted(
                    segment[0], candidate, permission, tolerance);
            for (std::size_t edge = 0; edge < 3; ++edge)
            {
                if ((permission.edge_mask & (1u << edge)) == 0) continue;
                const auto vertices = edgeVertices(edge);
                if (pointSegmentDistanceSquared(segment[0],
                        candidate[vertices[0]], candidate[vertices[1]]) <=
                        tolerance * tolerance &&
                    pointSegmentDistanceSquared(segment[1],
                        candidate[vertices[0]], candidate[vertices[1]]) <=
                        tolerance * tolerance)
                    return true;
            }
            return false;
        }

        SlidingIntersectionGeometry classify(
            const TrianglePoints &candidate,
            const TrianglePoints &surface,
            Scalar tolerance)
        {
            SlidingIntersectionGeometry result;
            if (!boxesOverlap(candidate, surface, tolerance)) return result;
            const Vector3 candidate_normal = normal(candidate);
            const Vector3 surface_normal = normal(surface);
            const Scalar candidate_length = candidate_normal.norm();
            const Scalar surface_length = surface_normal.norm();
            if (candidate_length == 0 || surface_length == 0) return result;

            bool coplanar = true;
            for (const Point3 &point : candidate)
                coplanar = coplanar && std::abs(surface_normal.dot(
                    point - surface[0]) / surface_length) <= tolerance;
            for (const Point3 &point : surface)
                coplanar = coplanar && std::abs(candidate_normal.dot(
                    point - candidate[0]) / candidate_length) <= tolerance;
            if (coplanar)
            {
                const auto polygon = clipCoplanar(candidate, surface,
                    dominantAxis(surface_normal), tolerance);
                if (polygon.empty()) return result;
                if (polygon.size() >= 3 &&
                    std::abs(polygonArea2(polygon)) > tolerance * tolerance)
                {
                    result.kind = SlidingIntersectionKind::CoplanarArea;
                    return result;
                }
                for (const Point3 &point : candidate)
                    if (pointInTriangle(point, surface, tolerance))
                        addUnique(result.points, point, tolerance);
                for (const Point3 &point : surface)
                    if (pointInTriangle(point, candidate, tolerance))
                        addUnique(result.points, point, tolerance);
            }
            else
            {
                appendPlaneIntersections(candidate, surface, surface_normal,
                    surface_length, tolerance, result.points);
                appendPlaneIntersections(surface, candidate, candidate_normal,
                    candidate_length, tolerance, result.points);
            }
            if (result.points.empty()) return result;
            if (result.points.size() == 1)
            {
                result.kind = SlidingIntersectionKind::Points;
                return result;
            }
            std::size_t first{};
            std::size_t second{1};
            Scalar maximum = distanceSquared(result.points[0], result.points[1]);
            for (std::size_t i = 0; i < result.points.size(); ++i)
                for (std::size_t j = i + 1; j < result.points.size(); ++j)
                    if (distanceSquared(result.points[i], result.points[j]) > maximum)
                    {
                        maximum = distanceSquared(result.points[i], result.points[j]);
                        first = i;
                        second = j;
                    }
            if (maximum <= tolerance * tolerance)
                result.kind = SlidingIntersectionKind::Points;
            else
            {
                result.kind = SlidingIntersectionKind::Segments;
                result.segments.push_back(
                    {result.points[first], result.points[second]});
            }
            return result;
        }
    }

    SlidingIntersectionGeometry classifySlidingIntersection(
        const TrianglePoints &candidate,
        const TrianglePoints &surface)
    {
        if (!finite(candidate) || !finite(surface)) return {};
        return classify(candidate, surface, toleranceFor(candidate, surface));
    }

    Result<bool, SpatialError> hasInvalidSlidingIntersection(
        const TrianglePoints &candidate,
        const TrianglePoints &surface,
        const SlidingContactPermission &permission)
    {
        using CheckResult = Result<bool, SpatialError>;
        if (!finite(candidate) || !finite(surface))
            return CheckResult::failure(SpatialError::NonFiniteCoordinate);
        if (permission.complete_face_exemption)
            return CheckResult::success(false);
        const Scalar tolerance = toleranceFor(candidate, surface);
        if (!boxesOverlap(candidate, surface, tolerance))
            return CheckResult::success(false);
        if (degenerate(candidate, tolerance) || degenerate(surface, tolerance))
            return CheckResult::failure(SpatialError::DegenerateTriangle);
        const SlidingIntersectionGeometry hit =
            classify(candidate, surface, tolerance);
        if (hit.kind == SlidingIntersectionKind::Empty)
            return CheckResult::success(false);
        if (hit.kind == SlidingIntersectionKind::CoplanarArea)
            return CheckResult::success(true);
        for (const Point3 &point : hit.points)
            if (!pointPermitted(point, candidate, permission, tolerance))
                return CheckResult::success(true);
        for (const auto &segment : hit.segments)
            if (!segmentPermitted(segment, candidate, permission, tolerance))
                return CheckResult::success(true);
        return CheckResult::success(false);
    }

    std::map<std::uint32_t, SlidingContactPermission>
    buildSlidingContactPermissions(
        const std::array<std::vector<std::uint32_t>, 3> &node_associations,
        std::uint8_t physical_edge_mask,
        const std::vector<std::uint32_t> &complete_exemptions)
    {
        std::map<std::uint32_t, SlidingContactPermission> result;
        for (std::size_t vertex = 0; vertex < 3; ++vertex)
            for (const std::uint32_t region : node_associations[vertex])
                result[region].vertex_mask |=
                    static_cast<std::uint8_t>(1u << vertex);
        for (std::size_t edge = 0; edge < 3; ++edge)
        {
            if ((physical_edge_mask & (1u << edge)) == 0) continue;
            const auto vertices = edgeVertices(edge);
            for (auto &[region, permission] : result)
            {
                const auto contains = [region](const auto &regions) {
                    return std::find(regions.begin(), regions.end(), region) !=
                        regions.end();
                };
                if (contains(node_associations[vertices[0]]) &&
                    contains(node_associations[vertices[1]]))
                    permission.edge_mask |=
                        static_cast<std::uint8_t>(1u << edge);
            }
        }
        for (const std::uint32_t region : complete_exemptions)
            result[region].complete_face_exemption = true;
        return result;
    }

    bool slidingSideValuesStayOnOneSide(
        const std::vector<Scalar> &values,
        Scalar tolerance)
    {
        bool negative{};
        bool positive{};
        for (const Scalar value : values)
        {
            if (!std::isfinite(value)) return false;
            negative = negative || value < -tolerance;
            positive = positive || value > tolerance;
        }
        return !(negative && positive);
    }
}
