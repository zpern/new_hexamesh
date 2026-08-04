#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    namespace
    {
        /// 判断一个三维点的所有坐标分量是否都是有限数。
        bool finite(
            const Point3 &point)
        {
            return std::isfinite(point.x()) &&
                   std::isfinite(point.y()) &&
                   std::isfinite(point.z());
        }
    }

    Result<FaceEvaluation, FaceEvaluationError>
    evaluateTriangle(
        const Point3 &v0,
        const Point3 &v1,
        const Point3 &v2,
        Scalar length_tolerance)
    {
        using EvaluationResult =
            Result<
                FaceEvaluation,
                FaceEvaluationError>;

        if (!finite(v0) ||
            !finite(v1) ||
            !finite(v2))
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    NonFiniteCoordinate);
        }

        const Vector3 edge01 = v1 - v0;
        const Vector3 edge12 = v2 - v1;
        const Vector3 edge20 = v0 - v2;

        if (edge01.norm() <= length_tolerance ||
            edge12.norm() <= length_tolerance ||
            edge20.norm() <= length_tolerance)
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    DegenerateEdge);
        }

        const Vector3 area_vector =
            Scalar{0.5} *
            edge01.cross(v2 - v0);

        // 输入坐标有限不代表叉积结果一定有限；极大坐标可能使中间结果溢出。
        if (!area_vector.allFinite())
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    DegenerateAreaVector);
        }

        const Scalar area =
            area_vector.norm();

        const Scalar area_tolerance =
            length_tolerance *
            length_tolerance;

        if (!std::isfinite(area) ||
            area <= area_tolerance)
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    DegenerateAreaVector);
        }

        const Vector3 unit_normal =
            area_vector / area;

        if (!unit_normal.allFinite())
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    DegenerateAreaVector);
        }

        // 先分别除以 3，避免三个有限的大坐标直接相加时溢出。
        const Point3 centroid =
            v0 / Scalar{3} +
            v1 / Scalar{3} +
            v2 / Scalar{3};

        if (!centroid.allFinite())
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    NonFiniteCoordinate);
        }

        return EvaluationResult::success(
            FaceEvaluation{
                centroid,
                area_vector,
                unit_normal,
                area,
                Scalar{0}});
    }

    Result<FaceEvaluation, FaceEvaluationError>
    evaluateQuad(
        const Point3 &v0,
        const Point3 &v1,
        const Point3 &v2,
        const Point3 &v3,
        Scalar length_tolerance)
    {
        using EvaluationResult =
            Result<
                FaceEvaluation,
                FaceEvaluationError>;

        if (!finite(v0) ||
            !finite(v1) ||
            !finite(v2) ||
            !finite(v3))
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    NonFiniteCoordinate);
        }

        const Vector3 boundary_edges[] = {
            v1 - v0,
            v2 - v1,
            v3 - v2,
            v0 - v3};

        for (const Vector3 &edge :
             boundary_edges)
        {
            if (edge.norm() <=
                length_tolerance)
            {
                return EvaluationResult::failure(
                    FaceEvaluationError::
                        DegenerateEdge);
            }
        }

        // 固定沿 v0-v2 将四边形拆成两个三角形。
        const Vector3 first_area_vector =
            Scalar{0.5} *
            (v1 - v0).cross(v2 - v0);

        const Vector3 second_area_vector =
            Scalar{0.5} *
            (v2 - v0).cross(v3 - v0);

        // 有限坐标的叉积仍可能因数值范围过大而溢出。
        if (!first_area_vector.allFinite() ||
            !second_area_vector.allFinite())
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    DegenerateAreaVector);
        }

        const Scalar first_area =
            first_area_vector.norm();

        const Scalar second_area =
            second_area_vector.norm();

        const Scalar area_tolerance =
            length_tolerance *
            length_tolerance;

        if (!std::isfinite(first_area) ||
            !std::isfinite(second_area) ||
            first_area <= area_tolerance ||
            second_area <= area_tolerance)
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    DegenerateAreaVector);
        }

        const Vector3 area_vector =
            first_area_vector +
            second_area_vector;

        const Scalar area_vector_norm =
            area_vector.norm();

        // 两个子三角形的面积向量可能方向相反并相互抵消。
        if (area_vector_norm <=
            area_tolerance)
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    DegenerateAreaVector);
        }

        const Vector3 first_normal =
            first_area_vector /
            first_area;

        const Vector3 second_normal =
            second_area_vector /
            second_area;

        const Scalar normal_cosine =
            std::clamp(
                first_normal.dot(
                    second_normal),
                Scalar{-1},
                Scalar{1});

        // 先分别除以 4，避免四个有限的大坐标直接相加时溢出。
        const Point3 centroid =
            v0 / Scalar{4} +
            v1 / Scalar{4} +
            v2 / Scalar{4} +
            v3 / Scalar{4};

        if (!centroid.allFinite())
        {
            return EvaluationResult::failure(
                FaceEvaluationError::
                    NonFiniteCoordinate);
        }

        return EvaluationResult::success(
            FaceEvaluation{
                centroid,
                area_vector,
                area_vector /
                    area_vector_norm,
                first_area +
                    second_area,
                std::acos(
                    normal_cosine)});
    }

    Result<Scalar, FaceEvaluationError>
    cornerAngle(
        const Point3 &previous,
        const Point3 &center,
        const Point3 &next,
        Scalar length_tolerance)
    {
        using AngleResult =
            Result<
                Scalar,
                FaceEvaluationError>;

        if (!finite(previous) ||
            !finite(center) ||
            !finite(next))
        {
            return AngleResult::failure(
                FaceEvaluationError::
                    NonFiniteCoordinate);
        }

        const Vector3 first =
            previous - center;

        const Vector3 second =
            next - center;

        const Scalar first_length =
            first.norm();

        const Scalar second_length =
            second.norm();

        if (!std::isfinite(first_length) ||
            !std::isfinite(second_length) ||
            first_length <=
                length_tolerance ||
            second_length <=
                length_tolerance)
        {
            return AngleResult::failure(
                FaceEvaluationError::
                    DegenerateEdge);
        }

        const Vector3 first_unit =
            first / first_length;

        const Vector3 second_unit =
            second / second_length;

        if (!first_unit.allFinite() ||
            !second_unit.allFinite())
        {
            return AngleResult::failure(
                FaceEvaluationError::
                    DegenerateEdge);
        }

        const Scalar dot =
            first_unit.dot(second_unit);

        if (!std::isfinite(dot))
        {
            return AngleResult::failure(
                FaceEvaluationError::
                    DegenerateEdge);
        }

        const Scalar cosine =
            std::clamp(
                dot,
                Scalar{-1},
                Scalar{1});

        const Scalar angle =
            std::acos(cosine);

        if (!std::isfinite(angle))
        {
            return AngleResult::failure(
                FaceEvaluationError::
                    DegenerateEdge);
        }

        return AngleResult::success(
            angle);
    }
}
