#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

#include <boundary_mesh/quality/volume_cell_evaluator.hpp>
#include <boundary_mesh/surface/face_skewness.hpp>

#include "volume_cell_evaluator_internal.hpp"

namespace boundary_mesh
{
    namespace
    {
        using Subtet = std::array<std::size_t, 4>;

        constexpr std::array<Subtet, 3> prism_subtets{{
            {0, 1, 2, 3},
            {1, 2, 3, 4},
            {2, 3, 4, 5}}}; // Prism 固定子四面体按公共诊断下标排列

        template <std::size_t VertexCount>
        bool checkFaceEdges(
            const std::array<Point3, VertexCount> &face,
            bool &degenerate)
        {
            for (std::size_t index = 0;
                 index < VertexCount;
                 ++index)
            {
                const Vector3 edge =
                    face[(index + 1) % VertexCount] - face[index];
                if (!edge.allFinite())
                {
                    return false;
                }

                const Scalar length = edge.norm();
                if (!std::isfinite(length))
                {
                    return false;
                }
                if (length == Scalar{0})
                {
                    degenerate = true;
                }
            }

            return true;
        }

        std::optional<Scalar> prismSkewness(
            const PrismPoints &points,
            bool &degenerate)
        {
            Scalar skewness = Scalar{0};

            const std::array<std::array<Point3, 3>, 2> triangles{{
                {points[0], points[1], points[2]},
                {points[3], points[4], points[5]}}};
            for (const auto &triangle : triangles)
            {
                bool face_degenerate = false;
                if (!checkFaceEdges(triangle, face_degenerate))
                {
                    return std::nullopt;
                }
                if (face_degenerate)
                {
                    degenerate = true;
                    continue;
                }

                const auto result = triangleEquiangularSkewness(
                    triangle,
                    Scalar{0});
                if (!result.hasValue())
                {
                    if (result.error() ==
                        FaceEvaluationError::DegenerateAreaVector)
                    {
                        degenerate = true;
                        continue;
                    }
                    return std::nullopt;
                }
                skewness = std::max(skewness, result.value());
            }

            const std::array<std::array<Point3, 4>, 3> quads{{
                {points[0], points[1], points[4], points[3]},
                {points[1], points[2], points[5], points[4]},
                {points[2], points[0], points[3], points[5]}}};
            for (const auto &quad : quads)
            {
                bool face_degenerate = false;
                if (!checkFaceEdges(quad, face_degenerate))
                {
                    return std::nullopt;
                }
                if (face_degenerate)
                {
                    degenerate = true;
                    continue;
                }

                const auto result = quadEquiangularSkewness(
                    quad,
                    Scalar{0});
                if (!result.hasValue())
                {
                    if (result.error() ==
                        FaceEvaluationError::DegenerateAreaVector)
                    {
                        degenerate = true;
                        continue;
                    }
                    return std::nullopt;
                }
                skewness = std::max(skewness, result.value());
            }

            return degenerate ? std::optional<Scalar>{
                                    std::max(skewness, Scalar{1})}
                              : std::optional<Scalar>{skewness};
        }

        VolumeCellEvaluationError intermediateError(
            std::optional<std::size_t> subtet_index = std::nullopt)
        {
            return VolumeCellEvaluationError{
                VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult,
                VolumeCellKind::Prism,
                Scalar{0},
                std::nullopt,
                subtet_index};
        }
    }

    Result<VolumeCellEvaluation, VolumeCellEvaluationError>
    evaluatePrism(
        const PrismPoints &points,
        const VolumeCellQualityOptions &options)
    {
        using EvaluationResult =
            Result<VolumeCellEvaluation, VolumeCellEvaluationError>;

        if (const auto error = quality_internal::validateQualityInput(
                points.data(),
                points.size(),
                VolumeCellKind::Prism,
                options))
        {
            return EvaluationResult::failure(*error);
        }

        quality_internal::SubtetVolumeAccumulator accumulator;
        for (std::size_t index = 0;
             index < prism_subtets.size();
             ++index)
        {
            const Subtet &subtet = prism_subtets[index];
            if (!accumulator.add(
                    points[subtet[0]],
                    points[subtet[1]],
                    points[subtet[2]],
                    points[subtet[3]],
                    index))
            {
                return EvaluationResult::failure(
                    intermediateError(index));
            }
        }

        bool face_degenerate = false;
        const auto skewness = prismSkewness(
            points,
            face_degenerate);
        if (!skewness || !std::isfinite(*skewness))
        {
            return EvaluationResult::failure(intermediateError());
        }

        VolumeCellEvaluation evaluation;
        evaluation.validity = accumulator.validity();
        evaluation.signed_volume = accumulator.signed_volume;
        evaluation.minimum_subtet_signed_volume =
            accumulator.minimum_signed_volume;
        evaluation.maximum_subtet_signed_volume =
            accumulator.maximum_signed_volume;
        evaluation.worst_subtet_index =
            accumulator.worst_subtet_index;
        evaluation.skewness = *skewness;
        evaluation.acceptable =
            evaluation.validity == VolumeCellValidity::Valid &&
            !face_degenerate &&
            evaluation.skewness <= options.maximum_skewness;

        return EvaluationResult::success(evaluation);
    }
}
