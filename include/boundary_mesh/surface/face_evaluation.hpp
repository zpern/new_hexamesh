#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    /// 单个三角形或四边形的一次无状态表面计算结果。
    ///
    /// area_vector 和 unit_normal 的方向由输入顶点绕序决定。
    /// 三角形的 warpage_angle 固定为零。
    struct FaceEvaluation
    {
        Point3 centroid{Point3::Zero()};      // 面片顶点的几何中心
        Vector3 area_vector{Vector3::Zero()}; // 方向由顶点绕序决定的面积向量
        Vector3 unit_normal{Vector3::Zero()}; // 面积向量归一化后的单位法向
        Scalar area{};                        // 面片面积
        Scalar warpage_angle{};               // 四边形子三角形法向夹角，三角形固定为零
    };

    /// 单面算法可能返回的局部几何错误。
    /// 实体编号由遍历整个前沿的上层调用方补充。
    enum class FaceEvaluationError
    {
        NonFiniteCoordinate, // 至少一个输入坐标分量不是有限数
        DegenerateEdge,      // 至少一条局部边不大于有效长度容差
        DegenerateAreaVector // 面积向量无法稳定归一化
    };

    /// 计算三角形面积、中心以及定向法向。
    /// length_tolerance 必须由调用方根据当前模型尺度计算。
    Result<FaceEvaluation, FaceEvaluationError>
    evaluateTriangle(
        const Point3 &v0,
        const Point3 &v1,
        const Point3 &v2,
        Scalar length_tolerance);

    /// 计算四边形面积、中心、定向法向和翘曲角。
    ///
    /// 四边形固定沿 v0-v2 对角线拆分为两个三角形：
    /// (v0, v1, v2) 和 (v0, v2, v3)。
    /// length_tolerance 必须由调用方根据当前模型尺度计算。
    Result<FaceEvaluation, FaceEvaluationError>
    evaluateQuad(
        const Point3 &v0,
        const Point3 &v1,
        const Point3 &v2,
        const Point3 &v3,
        Scalar length_tolerance);

    /// 计算 previous-center-next 形成的顶点内角。
    ///
    /// 返回值范围为 [0, π]。任一角边不大于有效长度
    /// 容差时返回 DegenerateEdge。
    Result<Scalar, FaceEvaluationError>
    cornerAngle(
        const Point3 &previous,
        const Point3 &center,
        const Point3 &next,
        Scalar length_tolerance);
}
