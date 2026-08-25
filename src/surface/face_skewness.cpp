#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>

#include <boundary_mesh/surface/face_skewness.hpp>

namespace boundary_mesh
{
    namespace
    {
        /// 圆周率常量，避免为理想角额外调用 acos。
        constexpr Scalar pi =
            3.141592653589793238462643383279502884;

        /// 判断一个三维点的所有坐标分量是否都是有限数。
        bool finite(
            const Point3 &point)
        {
            return std::isfinite(point.x()) &&
                   std::isfinite(point.y()) &&
                   std::isfinite(point.z());
        }

        template <std::size_t VertexCount>
        Result<Vector3, FaceEvaluationError>
        orderedUnitNormal(
            const std::array<Point3, VertexCount> &points,
            Scalar length_tolerance)
        {
            using NormalResult =
                Result<Vector3, FaceEvaluationError>;

            Vector3 area_vector = Vector3::Zero();
            const Point3 &origin = points[0];
            for (std::size_t index = 0;
                 index < VertexCount;
                 ++index)
            {
                area_vector += (points[index] - origin).cross(
                    points[(index + 1) % VertexCount] - origin);
            }

            const Scalar area_vector_length = area_vector.norm();
            const Scalar area_tolerance =
                length_tolerance * length_tolerance;
            if (!area_vector.allFinite() ||
                !std::isfinite(area_vector_length) ||
                area_vector_length <= area_tolerance)
            {
                return NormalResult::failure(
                    FaceEvaluationError::DegenerateAreaVector);
            }

            return NormalResult::success(
                area_vector / area_vector_length);
        }

        template <std::size_t VertexCount>
        Result<Scalar, FaceEvaluationError>
        equiangularSkewness(
            const std::array<Point3, VertexCount> &points,
            Scalar ideal_angle,
            Scalar length_tolerance)
        {
            using SkewnessResult =
                Result<
                    Scalar,
                    FaceEvaluationError>;

            for (const Point3 &point : points)
            {
                if (!finite(point))
                {
                    return SkewnessResult::failure(
                        FaceEvaluationError::
                            NonFiniteCoordinate);
                }
            }

            const auto normal_result = orderedUnitNormal(
                points,
                length_tolerance);
            if (!normal_result.hasValue())
            {
                return SkewnessResult::failure(
                    normal_result.error());
            }
            const Vector3 &unit_normal = normal_result.value();

            Scalar minimum_angle =
                std::numeric_limits<Scalar>::infinity();
            Scalar maximum_angle =
                -std::numeric_limits<Scalar>::infinity();
            bool has_reflex_corner = false;

            for (std::size_t index = 0;
                 index < VertexCount;
                 ++index)
            {
                const Point3 &previous =
                    points[(index + VertexCount - 1) % VertexCount];
                const Point3 &center = points[index];
                const Point3 &next =
                    points[(index + 1) % VertexCount];

                const Vector3 first = previous - center;
                const Vector3 second = next - center;
                const Scalar first_length = first.norm();
                const Scalar second_length = second.norm();

                if (!std::isfinite(first_length) ||
                    !std::isfinite(second_length) ||
                    first_length <= length_tolerance ||
                    second_length <= length_tolerance)
                {
                    return SkewnessResult::failure(
                        FaceEvaluationError::
                            DegenerateEdge);
                }

                const Vector3 first_unit = first / first_length;
                const Vector3 second_unit = second / second_length;

                if (!first_unit.allFinite() ||
                    !second_unit.allFinite())
                {
                    return SkewnessResult::failure(
                        FaceEvaluationError::
                            DegenerateEdge);
                }

                const Vector3 corner_cross =
                    first_unit.cross(second_unit);
                const Scalar cosine = std::clamp(
                    first_unit.dot(second_unit),
                    Scalar{-1},
                    Scalar{1});
                const Scalar minor_angle = std::atan2(
                    corner_cross.norm(),
                    cosine);
                const Scalar orientation =
                    corner_cross.dot(unit_normal);

                if (!corner_cross.allFinite() ||
                    !std::isfinite(cosine) ||
                    !std::isfinite(minor_angle) ||
                    !std::isfinite(orientation))
                {
                    return SkewnessResult::failure(
                        FaceEvaluationError::
                            DegenerateEdge);
                }

                minimum_angle = std::min(
                    minimum_angle,
                    minor_angle);
                maximum_angle = std::max(
                    maximum_angle,
                    minor_angle);
                if constexpr (VertexCount == 4)
                {
                    has_reflex_corner =
                        has_reflex_corner ||
                        orientation > Scalar{0};
                }
            }

            // Verdict 的 quad_minimum_maximum_angle 使用完整三维
            // 边夹角；任一角点有向面积为负时，将最大角转换为反角。
            if constexpr (VertexCount == 4)
            {
                if (has_reflex_corner)
                {
                    maximum_angle =
                        Scalar{2} * pi - maximum_angle;
                }
            }

            if (!std::isfinite(maximum_angle) ||
                !std::isfinite(minimum_angle))
            {
                return SkewnessResult::failure(
                    FaceEvaluationError::
                        DegenerateEdge);
            }

            const Scalar skewness = std::max(
                (maximum_angle - ideal_angle) /
                    (pi - ideal_angle),
                (ideal_angle - minimum_angle) /
                    ideal_angle);

            if (!std::isfinite(skewness))
            {
                return SkewnessResult::failure(
                    FaceEvaluationError::
                        DegenerateEdge);
            }

            return SkewnessResult::success(skewness);
        }
    }

    Result<Scalar, FaceEvaluationError>
    triangleEquiangularSkewness(
        const std::array<Point3, 3> &points,
        Scalar length_tolerance)
    {
        return equiangularSkewness(
            points,
            pi / Scalar{3},
            length_tolerance);
    }

    Result<Scalar, FaceEvaluationError>
    quadEquiangularSkewness(
        const std::array<Point3, 4> &points,
        Scalar length_tolerance)
    {
        return equiangularSkewness(
            points,
            pi / Scalar{2},
            length_tolerance);
    }
}
