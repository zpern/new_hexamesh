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

    return 0;
}
