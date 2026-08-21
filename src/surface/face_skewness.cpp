#include <algorithm>
#include <cmath>
#include <limits>

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

            Scalar minimum_cosine =
                std::numeric_limits<Scalar>::infinity();
            Scalar maximum_cosine =
                -std::numeric_limits<Scalar>::infinity();

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

                const Scalar cosine = first_unit.dot(second_unit);

                if (!std::isfinite(cosine))
                {
                    return SkewnessResult::failure(
                        FaceEvaluationError::
                            DegenerateEdge);
                }

                minimum_cosine = std::min(
                    minimum_cosine,
                    cosine);
                maximum_cosine = std::max(
                    maximum_cosine,
                    cosine);
            }

            const Scalar maximum_angle = std::acos(
                std::clamp(
                    minimum_cosine,
                    Scalar{-1},
                    Scalar{1}));
            const Scalar minimum_angle = std::acos(
                std::clamp(
                    maximum_cosine,
                    Scalar{-1},
                    Scalar{1}));

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

            return SkewnessResult::success(
                std::clamp(
                    skewness,
                    Scalar{0},
                    Scalar{1}));
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
