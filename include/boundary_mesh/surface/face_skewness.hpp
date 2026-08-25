#pragma once

#include <array>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    /// 计算三角形的等角偏斜度。
    ///
    /// 顶点按面片绕序传入；length_tolerance 用于判定退化边。
    /// 反角或严重无效几何可能返回大于 1 的有限值。
    Result<Scalar, FaceEvaluationError>
    triangleEquiangularSkewness(
        const std::array<Point3, 3> &points,
        Scalar length_tolerance);

    /// 计算四边形的等角偏斜度。
    ///
    /// 顶点按面片绕序传入；length_tolerance 用于判定退化边。
    /// 反角或严重无效几何可能返回大于 1 的有限值。
    Result<Scalar, FaceEvaluationError>
    quadEquiangularSkewness(
        const std::array<Point3, 4> &points,
        Scalar length_tolerance);
}
