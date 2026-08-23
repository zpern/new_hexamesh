#include <boundary_mesh/spatial/triangle_contact.hpp>

#include <boundary_mesh/spatial/collision_index.hpp>

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

        struct TriangleContactEvidence
        {
            TriangleContactKind kind{TriangleContactKind::Disjoint};
            std::vector<Point3> points;
        };

        enum class SharedFeatureKind
        {
            None,
            Points,
            Segment,
            Face
        };

        struct SharedFeature
        {
            SharedFeatureKind kind{SharedFeatureKind::None};
            std::vector<Point3> points;
            std::array<Point3, 4> face_points{};
            std::uint8_t face_point_count{};
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

        Point3 lift(
            const Point2 &point,
            int axis,
            const TrianglePoints &plane,
            const Vector3 &plane_normal)
        {
            Point3 result;
            if (axis == 0)
            {
                result.y() = point.x;
                result.z() = point.y;
                result.x() = plane[0].x() -
                    (plane_normal.y() * (result.y() - plane[0].y()) +
                     plane_normal.z() * (result.z() - plane[0].z())) /
                        plane_normal.x();
            }
            else if (axis == 1)
            {
                result.x() = point.x;
                result.z() = point.y;
                result.y() = plane[0].y() -
                    (plane_normal.x() * (result.x() - plane[0].x()) +
                     plane_normal.z() * (result.z() - plane[0].z())) /
                        plane_normal.y();
            }
            else
            {
                result.x() = point.x;
                result.y() = point.y;
                result.z() = plane[0].z() -
                    (plane_normal.x() * (result.x() - plane[0].x()) +
                     plane_normal.y() * (result.y() - plane[0].y())) /
                        plane_normal.z();
            }
            return result;
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

        Result<TriangleContactEvidence, SpatialError> contactEvidence(
            const TrianglePoints &first,
            const TrianglePoints &second)
        {
            if (!validTriangle(first) || !validTriangle(second))
            {
                return Result<TriangleContactEvidence, SpatialError>::failure(
                    SpatialError::NonFiniteCoordinate);
            }

            const Vector3 first_normal = normal(first);
            const Vector3 second_normal = normal(second);
            if (first_normal.squaredNorm() == Scalar{0} ||
                second_normal.squaredNorm() == Scalar{0})
            {
                return Result<TriangleContactEvidence, SpatialError>::failure(
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
                return Result<TriangleContactEvidence, SpatialError>::success(
                    {});
            }

            TriangleContactEvidence evidence;
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
                evidence.kind =
                    polygon.size() >= 3 && polygonArea(polygon) > Scalar{0}
                    ? TriangleContactKind::CoplanarOverlap
                    : (distinctPointCount(polygon) >= 2
                           ? TriangleContactKind::EdgeTouch
                           : TriangleContactKind::VertexTouch);
                for (const Point2 &point : polygon)
                {
                    appendUnique(
                        evidence.points,
                        lift(point, axis, first, first_normal));
                }
                return Result<TriangleContactEvidence, SpatialError>::success(
                    std::move(evidence));
            }

            appendPlaneIntersections(
                first,
                second,
                second_normal,
                evidence.points);
            appendPlaneIntersections(
                second,
                first,
                first_normal,
                evidence.points);
            if (evidence.points.size() <= 1)
            {
                evidence.kind = TriangleContactKind::VertexTouch;
            }
            else
            {
                evidence.kind =
                    pointsLieOnOneEdge(evidence.points, first) &&
                            pointsLieOnOneEdge(evidence.points, second)
                        ? TriangleContactKind::EdgeTouch
                        : TriangleContactKind::ProperIntersect;
            }
            return Result<TriangleContactEvidence, SpatialError>::success(
                std::move(evidence));
        }

        std::size_t boundaryVertexCount(const CollisionTriangle &triangle)
        {
            return triangle.boundary_vertex_count == 0
                ? std::size_t{3}
                : static_cast<std::size_t>(triangle.boundary_vertex_count);
        }

        const Point3 &boundaryPoint(
            const CollisionTriangle &triangle,
            std::size_t index)
        {
            return triangle.boundary_vertex_count == 0
                ? triangle.points[index]
                : triangle.boundary_points[index];
        }

        const CollisionVertexKey &boundaryKey(
            const CollisionTriangle &triangle,
            std::size_t index)
        {
            return triangle.boundary_vertex_count == 0
                ? triangle.vertex_keys[index]
                : triangle.boundary_vertex_keys[index];
        }

        bool adjacentBoundaryKeys(
            const CollisionTriangle &triangle,
            const CollisionVertexKey &first,
            const CollisionVertexKey &second)
        {
            const std::size_t count = boundaryVertexCount(triangle);
            for (std::size_t index = 0; index < count; ++index)
            {
                if (!sameKey(boundaryKey(triangle, index), first))
                {
                    continue;
                }
                return sameKey(
                           boundaryKey(triangle, (index + 1) % count),
                           second) ||
                       sameKey(
                           boundaryKey(
                               triangle,
                               (index + count - 1) % count),
                           second);
            }
            return false;
        }

        SharedFeature makeSharedFeature(
            const CollisionTriangle &first,
            const CollisionTriangle &second)
        {
            SharedFeature feature;
            const std::size_t first_count = boundaryVertexCount(first);
            const std::size_t second_count = boundaryVertexCount(second);
            std::vector<CollisionVertexKey> shared_keys;
            for (std::size_t index = 0; index < first_count; ++index)
            {
                const CollisionVertexKey &key = boundaryKey(first, index);
                for (std::size_t second_index = 0;
                     second_index < second_count;
                     ++second_index)
                {
                    if (!sameKey(boundaryKey(second, second_index), key))
                    {
                        continue;
                    }
                    if ((boundaryPoint(first, index).array() !=
                         boundaryPoint(second, second_index).array()).any())
                    {
                        return {};
                    }
                    shared_keys.push_back(key);
                    feature.points.push_back(boundaryPoint(first, index));
                    break;
                }
            }

            if (first_count == second_count &&
                shared_keys.size() == first_count)
            {
                feature.kind = SharedFeatureKind::Face;
                feature.face_point_count =
                    static_cast<std::uint8_t>(first_count);
                for (std::size_t index = 0; index < first_count; ++index)
                {
                    feature.face_points[index] = boundaryPoint(first, index);
                }
            }
            else if (shared_keys.size() == 2 &&
                     adjacentBoundaryKeys(
                         first, shared_keys[0], shared_keys[1]) &&
                     adjacentBoundaryKeys(
                         second, shared_keys[0], shared_keys[1]))
            {
                feature.kind = SharedFeatureKind::Segment;
            }
            else if (!shared_keys.empty())
            {
                feature.kind = SharedFeatureKind::Points;
            }
            return feature;
        }

        bool samePoint(const Point3 &left, const Point3 &right)
        {
            return (left.array() == right.array()).all();
        }

        bool pointOnSegment(
            const Point3 &point,
            const Point3 &first,
            const Point3 &second)
        {
            return (second - first).cross(point - first).squaredNorm() ==
                       Scalar{0} &&
                   (point - first).dot(point - second) <= Scalar{0};
        }

        bool pointInBoundaryTriangle(
            const Point3 &point,
            const Point3 &first,
            const Point3 &second,
            const Point3 &third)
        {
            const TrianglePoints triangle{{first, second, third}};
            const Vector3 triangle_normal = normal(triangle);
            if (triangle_normal.squaredNorm() == Scalar{0})
            {
                return false;
            }
            const int axis = dominantAxis(triangle_normal);
            const Point2 projected_point = project(point, axis);
            const Point2 projected_first = project(first, axis);
            const Point2 projected_second = project(second, axis);
            const Point2 projected_third = project(third, axis);
            const Scalar first_side = cross2(
                projected_first,
                projected_second,
                projected_point);
            const Scalar second_side = cross2(
                projected_second,
                projected_third,
                projected_point);
            const Scalar third_side = cross2(
                projected_third,
                projected_first,
                projected_point);
            return (first_side >= Scalar{0} &&
                    second_side >= Scalar{0} &&
                    third_side >= Scalar{0}) ||
                   (first_side <= Scalar{0} &&
                    second_side <= Scalar{0} &&
                    third_side <= Scalar{0});
        }

        bool pointInBoundaryFace(
            const Point3 &point,
            const SharedFeature &feature)
        {
            if (feature.face_point_count == 3)
            {
                return pointInBoundaryTriangle(
                    point,
                    feature.face_points[0],
                    feature.face_points[1],
                    feature.face_points[2]);
            }
            return feature.face_point_count == 4 &&
                   (pointInBoundaryTriangle(
                        point,
                        feature.face_points[0],
                        feature.face_points[1],
                        feature.face_points[2]) ||
                    pointInBoundaryTriangle(
                        point,
                        feature.face_points[0],
                        feature.face_points[2],
                        feature.face_points[3]));
        }

        bool containsEvidencePoint(
            const SharedFeature &feature,
            const Point3 &point)
        {
            if (feature.kind == SharedFeatureKind::Points)
            {
                return std::any_of(
                    feature.points.begin(),
                    feature.points.end(),
                    [&](const Point3 &allowed)
                    {
                        return samePoint(point, allowed);
                    });
            }
            if (feature.kind == SharedFeatureKind::Segment)
            {
                return pointOnSegment(
                    point,
                    feature.points[0],
                    feature.points[1]);
            }
            if (feature.kind == SharedFeatureKind::Face)
            {
                return pointInBoundaryFace(point, feature);
            }
            return false;
        }

        bool featureContainsEvidence(
            const SharedFeature &feature,
            const TriangleContactEvidence &evidence)
        {
            if (evidence.kind == TriangleContactKind::Disjoint)
            {
                return true;
            }
            if (feature.kind == SharedFeatureKind::None ||
                evidence.points.empty())
            {
                return false;
            }
            if (feature.kind == SharedFeatureKind::Points &&
                evidence.points.size() != 1)
            {
                return false;
            }
            for (std::size_t index = 0;
                 index < evidence.points.size();
                 ++index)
            {
                if (!containsEvidencePoint(feature, evidence.points[index]))
                {
                    return false;
                }
                if (feature.kind == SharedFeatureKind::Face &&
                    evidence.points.size() > 1)
                {
                    const Point3 midpoint = Scalar{0.5} *
                        (evidence.points[index] +
                         evidence.points[(index + 1) % evidence.points.size()]);
                    if (!containsEvidencePoint(feature, midpoint))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        struct LocalSharedKeys
        {
            std::size_t count{};
            std::array<std::size_t, 3> first_indices{};
            std::array<std::size_t, 3> second_indices{};
        };

        LocalSharedKeys localSharedKeys(
            const CollisionTriangle &first,
            const CollisionTriangle &second)
        {
            LocalSharedKeys result;
            for (std::size_t first_index = 0;
                 first_index < first.vertex_keys.size();
                 ++first_index)
            {
                for (std::size_t second_index = 0;
                     second_index < second.vertex_keys.size();
                     ++second_index)
                {
                    if (sameKey(
                            first.vertex_keys[first_index],
                            second.vertex_keys[second_index]))
                    {
                        result.first_indices[result.count] = first_index;
                        result.second_indices[result.count] = second_index;
                        ++result.count;
                        break;
                    }
                }
            }
            return result;
        }

        bool oppositeEdgeIntersectsTriangle(
            const CollisionTriangle &edge_source,
            std::size_t shared_vertex,
            const CollisionTriangle &target)
        {
            double line[2][3];
            double face[3][3];
            const std::size_t first = (shared_vertex + 1) % 3;
            const std::size_t second = (shared_vertex + 2) % 3;
            for (std::size_t axis = 0; axis < 3; ++axis)
            {
                line[0][axis] = edge_source.points[first][axis];
                line[1][axis] = edge_source.points[second][axis];
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    face[corner][axis] = target.points[corner][axis];
                }
            }

            int intersection_type{};
            int intersection_code{};
            double intersection_point[3]{};
            bool epsilon = false;
            return TiGER_GEOM_FUNC::lin_tri_intersect3d(
                       line,
                       face,
                       &intersection_type,
                       &intersection_code,
                       intersection_point,
                       epsilon) != 0;
        }

    }

    Result<TriangleContactKind, SpatialError>
    classifyTriangleContact(
        const TrianglePoints &first,
        const TrianglePoints &second)
    {
        const auto evidence = contactEvidence(first, second);
        if (!evidence.hasValue())
        {
            return Result<TriangleContactKind, SpatialError>::failure(
                evidence.error());
        }
        return Result<TriangleContactKind, SpatialError>::success(
            evidence.value().kind);
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

    Result<bool, SpatialError> hasIllegalTriangleContact(
        const CollisionTriangle &first,
        const CollisionTriangle &second)
    {
        const std::size_t first_count = boundaryVertexCount(first);
        const std::size_t second_count = boundaryVertexCount(second);
        if ((first_count != 3 && first_count != 4) ||
            (second_count != 3 && second_count != 4))
        {
            return Result<bool, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        }

        const auto evidence = contactEvidence(first.points, second.points);
        if (!evidence.hasValue())
        {
            return Result<bool, SpatialError>::failure(evidence.error());
        }
        const SharedFeature feature = makeSharedFeature(first, second);
        if (evidence.value().kind == TriangleContactKind::Disjoint)
        {
            return Result<bool, SpatialError>::success(false);
        }

        const LocalSharedKeys shared = localSharedKeys(first, second);
        if (feature.kind == SharedFeatureKind::Face)
        {
            return Result<bool, SpatialError>::success(false);
        }
        if (shared.count == 1 &&
            feature.kind != SharedFeatureKind::None)
        {
            return Result<bool, SpatialError>::success(
                oppositeEdgeIntersectsTriangle(
                    first,
                    shared.first_indices[0],
                    second) ||
                oppositeEdgeIntersectsTriangle(
                    second,
                    shared.second_indices[0],
                    first));
        }
        if (shared.count == 2 &&
            feature.kind == SharedFeatureKind::Segment)
        {
            return Result<bool, SpatialError>::success(
                evidence.value().kind ==
                TriangleContactKind::CoplanarOverlap);
        }
        return Result<bool, SpatialError>::success(
            !featureContainsEvidence(feature, evidence.value()));
    }
}
