#include <array>
#include <cmath>

#include <boundary_mesh/surface/face_skewness.hpp>

namespace
{
    using namespace boundary_mesh;

    bool near(
        Scalar first,
        Scalar second,
        Scalar tolerance = 1e-12)
    {
        return std::abs(first - second) <= tolerance;
    }
}

int main()
{
    using namespace boundary_mesh;

    const Scalar length_tolerance = 1e-12;
    const Scalar root_three = std::sqrt(Scalar{3});

    const std::array<Point3, 3> equilateral_triangle{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{0.5, root_three / 2.0, 0.0}};

    const auto triangle_result =
        triangleEquiangularSkewness(
            equilateral_triangle,
            length_tolerance);

    if (!triangle_result.hasValue() ||
        !near(triangle_result.value(), 0.0))
    {
        return 1;
    }

    const std::array<Point3, 4> square{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0}};

    const auto square_result =
        quadEquiangularSkewness(
            square,
            length_tolerance);

    if (!square_result.hasValue() ||
        !near(square_result.value(), 0.0))
    {
        return 2;
    }

    // 直角等腰三角形的角为 90°、45°、45°，
    // 按 equiangular 公式得到 skewness = 0.25。
    const std::array<Point3, 3> right_triangle{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{0.0, 1.0, 0.0}};

    const auto distorted_result =
        triangleEquiangularSkewness(
            right_triangle,
            length_tolerance);

    if (!distorted_result.hasValue() ||
        !near(distorted_result.value(), 0.25))
    {
        return 3;
    }

    if (distorted_result.value() < 0.0 ||
        distorted_result.value() > 1.0)
    {
        return 4;
    }

    const std::array<Point3, 4> zero_edge_quad{
        Point3{0.0, 0.0, 0.0},
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0}};

    const auto zero_edge_result =
        quadEquiangularSkewness(
            zero_edge_quad,
            length_tolerance);

    if (zero_edge_result.hasValue() ||
        zero_edge_result.error() !=
            FaceEvaluationError::DegenerateEdge)
    {
        return 5;
    }

    const std::array<Point3, 4> concave_quad{
        Point3{0.0, 0.0, 0.0},
        Point3{2.0, 0.0, 0.0},
        Point3{1.0, 0.5, 0.0},
        Point3{2.0, 1.0, 0.0}};

    const auto concave_result =
        quadEquiangularSkewness(
            concave_quad,
            length_tolerance);

    if (!concave_result.hasValue() ||
        !(concave_result.value() > Scalar{1}))
    {
        return 6;
    }

    const Scalar pi = std::acos(Scalar{-1});
    const Scalar concave_minor_angle =
        std::atan2(Scalar{0.8}, Scalar{0.6});
    const Scalar expected_concave_skewness =
        (Scalar{2} * pi - concave_minor_angle - pi / Scalar{2}) /
        (pi / Scalar{2});
    if (!near(
            concave_result.value(),
            expected_concave_skewness))
    {
        return 7;
    }

    const std::array<Point3, 4> reversed_concave_quad{
        concave_quad[3],
        concave_quad[2],
        concave_quad[1],
        concave_quad[0]};

    const auto reversed_concave_result =
        quadEquiangularSkewness(
            reversed_concave_quad,
            length_tolerance);

    if (!reversed_concave_result.hasValue() ||
        !near(
            reversed_concave_result.value(),
            concave_result.value()))
    {
        return 8;
    }

    const std::array<Point3, 4> warped_quad{
        Point3{0.0, 0.0, 0.0},
        Point3{2.0, 0.0, 0.2},
        Point3{2.0, 1.0, 0.0},
        Point3{0.0, 1.0, -0.1}};
    const std::array<Point3, 4> reversed_warped_quad{
        warped_quad[3],
        warped_quad[2],
        warped_quad[1],
        warped_quad[0]};
    const auto warped_result = quadEquiangularSkewness(
        warped_quad,
        length_tolerance);
    const auto reversed_warped_result = quadEquiangularSkewness(
        reversed_warped_quad,
        length_tolerance);
    if (!warped_result.hasValue() ||
        !reversed_warped_result.hasValue() ||
        !near(
            warped_result.value(),
            reversed_warped_result.value()))
    {
        return 9;
    }

    std::array<Point3, 4> transformed_warped_quad = warped_quad;
    for (Point3 &point : transformed_warped_quad)
    {
        point = Scalar{3} * point + Point3{4.0, -2.0, 7.0};
    }
    const auto transformed_warped_result = quadEquiangularSkewness(
        transformed_warped_quad,
        length_tolerance);
    if (!transformed_warped_result.hasValue() ||
        !near(
            warped_result.value(),
            transformed_warped_result.value()))
    {
        return 10;
    }

    const std::array<Point3, 4> cancelling_quad{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        Point3{1.0, 0.0, 0.0}};
    const auto cancelling_result = quadEquiangularSkewness(
        cancelling_quad,
        length_tolerance);
    if (cancelling_result.hasValue() ||
        cancelling_result.error() !=
            FaceEvaluationError::DegenerateAreaVector)
    {
        return 11;
    }

    return 0;
}
