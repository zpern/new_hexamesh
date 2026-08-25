#include <algorithm>
#include <array>
#include <utility>

#include <boundary_mesh/surface/face_skewness.hpp>
#include <boundary_mesh/transition/quad_diagonal.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct Candidate
        {
            QuadDiagonal diagonal{};
            std::array<Triangle, 2> triangles{};
            std::array<std::array<std::size_t, 3>, 2> local{};
            std::array<VertexId, 2> endpoint_ids{};
        };

        Result<Scalar, FaceEvaluationError> worstSkewness(
            const OrientedQuad &quad,
            const Candidate &candidate,
            Scalar tolerance)
        {
            std::array<Scalar, 2> values{};
            for (std::size_t index = 0; index < 2; ++index)
            {
                const auto &local = candidate.local[index];
                const std::array<Point3, 3> points{
                    quad.points[local[0]],
                    quad.points[local[1]],
                    quad.points[local[2]]};
                const auto value = triangleEquiangularSkewness(
                    points, tolerance);
                if (!value.hasValue())
                {
                    return Result<Scalar, FaceEvaluationError>::failure(
                        value.error());
                }
                values[index] = value.value();
            }
            return Result<Scalar, FaceEvaluationError>::success(
                std::max(values[0], values[1]));
        }
    }

    Result<QuadDiagonalSelection, FaceEvaluationError>
    chooseQuadDiagonal(
        const OrientedQuad &quad,
        Scalar length_tolerance)
    {
        using SelectionResult = Result<
            QuadDiagonalSelection, FaceEvaluationError>;

        Candidate zero_two;
        zero_two.diagonal = QuadDiagonal::ZeroTwo;
        zero_two.triangles = {
            Triangle{{quad.vertex_ids[0], quad.vertex_ids[1],
                      quad.vertex_ids[2]}},
            Triangle{{quad.vertex_ids[0], quad.vertex_ids[2],
                      quad.vertex_ids[3]}}};
        zero_two.local[0] = {0, 1, 2};
        zero_two.local[1] = {0, 2, 3};
        zero_two.endpoint_ids = {
            quad.vertex_ids[0], quad.vertex_ids[2]};

        Candidate one_three;
        one_three.diagonal = QuadDiagonal::OneThree;
        one_three.triangles = {
            Triangle{{quad.vertex_ids[1], quad.vertex_ids[2],
                      quad.vertex_ids[3]}},
            Triangle{{quad.vertex_ids[1], quad.vertex_ids[3],
                      quad.vertex_ids[0]}}};
        one_three.local[0] = {1, 2, 3};
        one_three.local[1] = {1, 3, 0};
        one_three.endpoint_ids = {
            quad.vertex_ids[1], quad.vertex_ids[3]};
        std::sort(zero_two.endpoint_ids.begin(), zero_two.endpoint_ids.end());
        std::sort(one_three.endpoint_ids.begin(), one_three.endpoint_ids.end());

        const auto score_zero = worstSkewness(
            quad, zero_two, length_tolerance);
        if (!score_zero.hasValue())
        {
            return SelectionResult::failure(score_zero.error());
        }
        const auto score_one = worstSkewness(
            quad, one_three, length_tolerance);
        if (!score_one.hasValue())
        {
            return SelectionResult::failure(score_one.error());
        }

        const Candidate *selected = &zero_two;
        Scalar score = score_zero.value();
        if (score_one.value() < score ||
            (score_one.value() == score &&
             one_three.endpoint_ids < zero_two.endpoint_ids))
        {
            selected = &one_three;
            score = score_one.value();
        }
        return SelectionResult::success(
            QuadDiagonalSelection{
                selected->diagonal,
                selected->triangles,
                score});
    }
}
