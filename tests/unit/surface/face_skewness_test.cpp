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
        return 7;
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
        return 8;
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
        return 9;
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
        return 10;
    }

    // Verdict quad_simple2 reference fixture.
    const std::array<Point3, 4> verdict_warped_quad{
        Point3{2.0, 0.0, 0.0},
        Point3{1.0, 1.0, 2.0},
        Point3{0.0, 1.0, 0.0},
        Point3{0.0, 0.0, 0.0}};
    const auto verdict_warped_result = quadEquiangularSkewness(
        verdict_warped_quad,
        length_tolerance);
    if (!verdict_warped_result.hasValue() ||
        !near(
            verdict_warped_result.value(),
            Scalar{0.36901011957},
            Scalar{1e-10}))
    {
        return 11;
    }

    // Verdict quad_chevron reference fixture with a reflex angle.
    const std::array<Point3, 4> verdict_chevron_quad{
        Point3{1.0, 1.0, 1.0},
        Point3{3.0, 1.0, 2.0},
        Point3{5.0, 0.8, 1.3},
        Point3{2.0, 1.1, 3.7}};
    const auto verdict_chevron_result = quadEquiangularSkewness(
        verdict_chevron_quad,
        length_tolerance);
    if (!verdict_chevron_result.hasValue() ||
        !near(
            verdict_chevron_result.value(),
            Scalar{1.5122294809},
            Scalar{1e-10}))
    {
        return 12;
    }

    const std::array<Point3, 4> verdict_bowtie_quad{
        Point3{-1.0, -1.0, -1.0},
        Point3{6.0, 2.0, -1.1},
        Point3{3.0, -2.5, -1.15},
        Point3{4.5, 4.3, -0.9}};
    const auto verdict_bowtie_result = quadEquiangularSkewness(
        verdict_bowtie_quad,
        length_tolerance);
    if (!verdict_bowtie_result.hasValue() ||
        !near(
            verdict_bowtie_result.value(),
            Scalar{2.6262710934},
            Scalar{1e-10}))
    {
        return 13;
    }

    return 0;
}
