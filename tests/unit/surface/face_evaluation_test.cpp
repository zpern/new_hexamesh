#include <cmath>
#include <limits>

#include <boundary_mesh/surface/face_evaluation.hpp>

namespace
{
    using namespace boundary_mesh;

    bool near(
        Scalar first,
        Scalar second,
        Scalar tolerance = 1e-12)
    {
        return std::abs(first - second) <=
               tolerance;
    }

    bool nearVector(
        const Vector3 &first,
        const Vector3 &second,
        Scalar tolerance = 1e-12)
    {
        return (first - second).norm() <=
               tolerance;
    }
}

int main()
{
    using namespace boundary_mesh;

    const Scalar tolerance = 1e-12;

    // 单位直角三角形应得到确定的面积、中心和法向。
    const auto result = evaluateTriangle(
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        tolerance);

    if (!result.hasValue())
    {
        return 1;
    }

    const FaceEvaluation &evaluation =
        result.value();

    if (!near(evaluation.area, 0.5))
    {
        return 2;
    }

    if (!nearVector(
            evaluation.area_vector,
            Vector3{0.0, 0.0, 0.5}))
    {
        return 3;
    }

    if (!nearVector(
            evaluation.unit_normal,
            Vector3{0.0, 0.0, 1.0}))
    {
        return 4;
    }

    if (!nearVector(
            evaluation.centroid,
            Point3{
                1.0 / 3.0,
                1.0 / 3.0,
                0.0}))
    {
        return 5;
    }

    if (!near(evaluation.warpage_angle, 0.0))
    {
        return 6;
    }

    // 平面单位四边形应得到单位面积、+Z 法向和零翘曲角。
    const auto quad_result = evaluateQuad(
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        tolerance);

    if (!quad_result.hasValue())
    {
        return 7;
    }

    const FaceEvaluation &quad =
        quad_result.value();

    if (!near(quad.area, 1.0))
    {
        return 8;
    }

    if (!nearVector(
            quad.area_vector,
            Vector3{0.0, 0.0, 1.0}))
    {
        return 9;
    }

    if (!nearVector(
            quad.unit_normal,
            Vector3{0.0, 0.0, 1.0}))
    {
        return 10;
    }

    if (!nearVector(
            quad.centroid,
            Point3{0.5, 0.5, 0.0}))
    {
        return 11;
    }

    if (!near(quad.warpage_angle, 0.0))
    {
        return 12;
    }

    // 两条互相垂直的角边应形成 π/2 内角。
    const auto angle_result = cornerAngle(
        Point3{1.0, 0.0, 0.0},
        Point3{0.0, 0.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        tolerance);

    if (!angle_result.hasValue())
    {
        return 13;
    }

    if (!near(
            angle_result.value(),
            std::acos(-1.0) / 2.0))
    {
        return 14;
    }

    // 非有限坐标不得进入面积和法向计算。
    const auto non_finite_result =
        evaluateTriangle(
            Point3{
                std::numeric_limits<Scalar>::
                    quiet_NaN(),
                0.0,
                0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            tolerance);

    if (non_finite_result.hasValue() ||
        non_finite_result.error() !=
            FaceEvaluationError::
                NonFiniteCoordinate)
    {
        return 15;
    }

    // 两个端点重合会形成零长度边。
    const auto zero_edge_result =
        evaluateTriangle(
            Point3{0.0, 0.0, 0.0},
            Point3{0.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            tolerance);

    if (zero_edge_result.hasValue() ||
        zero_edge_result.error() !=
            FaceEvaluationError::
                DegenerateEdge)
    {
        return 16;
    }

    // 三个不同顶点共线时边长有效，但面积向量为零。
    const auto collinear_result =
        evaluateTriangle(
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{2.0, 0.0, 0.0},
            tolerance);

    if (collinear_result.hasValue() ||
        collinear_result.error() !=
            FaceEvaluationError::
                DegenerateAreaVector)
    {
        return 17;
    }

    // 两个四边形子三角形方向相反且面积相等时，
    // 总面积向量会完全抵消，无法产生统一法向。
    const auto cancelled_quad_result =
        evaluateQuad(
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            tolerance);

    if (cancelled_quad_result.hasValue() ||
        cancelled_quad_result.error() !=
            FaceEvaluationError::
                DegenerateAreaVector)
    {
        return 18;
    }

    // 非共面四边形的翘曲角等于两个固定子三角形
    // 单位法向之间的夹角。
    const auto warped_quad_result =
        evaluateQuad(
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{1.0, 1.0, 0.0},
            Point3{0.0, 1.0, 1.0},
            tolerance);

    if (!warped_quad_result.hasValue())
    {
        return 19;
    }

    const Scalar expected_warpage =
        std::acos(
            Scalar{1} /
            std::sqrt(Scalar{3}));

    if (!near(
            warped_quad_result.value().warpage_angle,
            expected_warpage))
    {
        return 20;
    }

    // 同一个小尺度三角形在精细容差下应保持有效。
    const auto small_scale_result =
        evaluateTriangle(
            Point3{0.0, 0.0, 0.0},
            Point3{1e-6, 0.0, 0.0},
            Point3{0.0, 1e-6, 0.0},
            Scalar{1e-9});

    if (!small_scale_result.hasValue())
    {
        return 21;
    }

    if (std::abs(
            small_scale_result.value().area -
            Scalar{5e-13}) >
        Scalar{1e-24})
    {
        return 22;
    }

    // 对同一几何使用大于边长的容差时，应判定为退化边。
    const auto coarse_tolerance_result =
        evaluateTriangle(
            Point3{0.0, 0.0, 0.0},
            Point3{1e-6, 0.0, 0.0},
            Point3{0.0, 1e-6, 0.0},
            Scalar{1e-5});

    if (coarse_tolerance_result.hasValue() ||
        coarse_tolerance_result.error() !=
            FaceEvaluationError::
                DegenerateEdge)
    {
        return 23;
    }

    // cornerAngle 的任一角边退化时也必须返回明确错误。
    const auto degenerate_angle_result =
        cornerAngle(
            Point3{0.0, 0.0, 0.0},
            Point3{0.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            tolerance);

    if (degenerate_angle_result.hasValue() ||
        degenerate_angle_result.error() !=
            FaceEvaluationError::
                DegenerateEdge)
    {
        return 24;
    }
    // 输入坐标虽然有限，但叉积溢出时不能成功返回
    // 包含 Inf/NaN 的面积和法向。
    const auto overflow_triangle_result =
        evaluateTriangle(
            Point3{0.0, 0.0, 0.0},
            Point3{1e200, 0.0, 0.0},
            Point3{0.0, 1e200, 0.0},
            Scalar{1.0});

    if (overflow_triangle_result.hasValue() ||
        overflow_triangle_result.error() !=
            FaceEvaluationError::
                DegenerateAreaVector)
    {
        return 25;
    }

    // 角边坐标有限，但 norm 和 dot 溢出时，
    // cornerAngle 不能成功返回 NaN。
    const auto overflow_angle_result =
        cornerAngle(
            Point3{1e200, 0.0, 0.0},
            Point3{0.0, 0.0, 0.0},
            Point3{1e200, 1e200, 0.0},
            Scalar{1.0});

    if (overflow_angle_result.hasValue() ||
        overflow_angle_result.error() !=
            FaceEvaluationError::
                DegenerateEdge)
    {
        return 26;
    }

    // 四边形顶点坐标有限，但两个子三角形的叉积
    // 都可能溢出，因此不能成功返回非有限几何量。
    const auto overflow_quad_result =
        evaluateQuad(
            Point3{0.0, 0.0, 0.0},
            Point3{1e200, 0.0, 0.0},
            Point3{1e200, 1e200, 0.0},
            Point3{0.0, 1e200, 0.0},
            Scalar{1.0});

    if (overflow_quad_result.hasValue() ||
        overflow_quad_result.error() !=
            FaceEvaluationError::
                DegenerateAreaVector)
    {
        return 27;
    }

    // 位于极大平移坐标处的有效四边形，其局部几何仍然可计算；
    // 中心点求和不得因为四个大坐标直接相加而溢出。
    const auto translated_quad_result =
        evaluateQuad(
            Point3{1e308, 0.0, 0.0},
            Point3{1e308, 1.0, 0.0},
            Point3{1e308, 1.0, 1.0},
            Point3{1e308, 0.0, 1.0},
            tolerance);

    if (!translated_quad_result.hasValue() ||
        !translated_quad_result.value().centroid.allFinite() ||
        !near(
            translated_quad_result.value().centroid.x() /
                Scalar{1e308},
            Scalar{1}))
    {
        return 28;
    }

    // 反向顶点绕序必须保留定向信息并得到 -Z 法向。
    const auto reversed_triangle_result =
        evaluateTriangle(
            Point3{0.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            tolerance);

    if (!reversed_triangle_result.hasValue() ||
        !nearVector(
            reversed_triangle_result.value().unit_normal,
            Vector3{0.0, 0.0, -1.0}))
    {
        return 29;
    }

    // 四边形中的任一非有限坐标必须被明确拒绝。
    const auto non_finite_quad_result =
        evaluateQuad(
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{
                1.0,
                std::numeric_limits<Scalar>::infinity(),
                0.0},
            Point3{0.0, 1.0, 0.0},
            tolerance);

    if (non_finite_quad_result.hasValue() ||
        non_finite_quad_result.error() !=
            FaceEvaluationError::
                NonFiniteCoordinate)
    {
        return 30;
    }

    // 四边形相邻顶点重合时必须报告退化边。
    const auto zero_edge_quad_result =
        evaluateQuad(
            Point3{0.0, 0.0, 0.0},
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 1.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            tolerance);

    if (zero_edge_quad_result.hasValue() ||
        zero_edge_quad_result.error() !=
            FaceEvaluationError::
                DegenerateEdge)
    {
        return 31;
    }

    // 固定拆分后的任一子三角形退化时，四边形不可用。
    const auto degenerate_subtriangle_result =
        evaluateQuad(
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{2.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            tolerance);

    if (degenerate_subtriangle_result.hasValue() ||
        degenerate_subtriangle_result.error() !=
            FaceEvaluationError::
                DegenerateAreaVector)
    {
        return 32;
    }

    // cornerAngle 与面计算采用相同的非有限坐标规则。
    const auto non_finite_angle_result =
        cornerAngle(
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 0.0, 0.0},
            Point3{
                0.0,
                std::numeric_limits<Scalar>::quiet_NaN(),
                0.0},
            tolerance);

    if (non_finite_angle_result.hasValue() ||
        non_finite_angle_result.error() !=
            FaceEvaluationError::
                NonFiniteCoordinate)
    {
        return 33;
    }

    return 0;
}
