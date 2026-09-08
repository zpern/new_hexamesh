#include <array>
#include <cstddef>

#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

#include "volume_cell_evaluator_internal.hpp"

namespace boundary_mesh
{
    namespace
    {
        using Subtet = std::array<std::size_t, 4>;

        template <std::size_t PointCount, std::size_t SubtetCount>
        Result<VolumeCellEvaluation, VolumeCellEvaluationError> evaluate(
            const std::array<Point3, PointCount> &points,
            const std::array<Subtet, SubtetCount> &subtets,
            VolumeCellKind kind,
            const VolumeCellQualityOptions &options)
        {
            using EvaluationResult =
                Result<VolumeCellEvaluation, VolumeCellEvaluationError>;
            if (const auto error = quality_internal::validateQualityInput(
                    points.data(), points.size(), kind, options))
                return EvaluationResult::failure(*error);

            quality_internal::SubtetVolumeAccumulator accumulator;
            for (std::size_t index = 0; index < subtets.size(); ++index)
            {
                const auto &subtet = subtets[index];
                if (!accumulator.add(
                        points[subtet[0]], points[subtet[1]],
                        points[subtet[2]], points[subtet[3]], index))
                    return EvaluationResult::failure({
                        VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult,
                        kind, Scalar{0}, std::nullopt, index});
            }

            VolumeCellEvaluation result;
            result.validity = accumulator.validity();
            result.signed_volume = accumulator.signed_volume;
            result.minimum_subtet_signed_volume =
                accumulator.minimum_signed_volume;
            result.maximum_subtet_signed_volume =
                accumulator.maximum_signed_volume;
            result.worst_subtet_index = accumulator.worst_subtet_index;
            result.acceptable = result.validity == VolumeCellValidity::Valid;
            return EvaluationResult::success(result);
        }
    }

    Result<VolumeCellEvaluation, VolumeCellEvaluationError> evaluateTetra(
        const TetraPoints &points,
        const VolumeCellQualityOptions &options)
    {
        constexpr std::array<Subtet, 1> subtets{{{0, 1, 2, 3}}};
        return evaluate(points, subtets, VolumeCellKind::Tetra, options);
    }

    Result<VolumeCellEvaluation, VolumeCellEvaluationError> evaluatePyramid(
        const PyramidPoints &points,
        const VolumeCellQualityOptions &options)
    {
        constexpr std::array<Subtet, 2> subtets{{
            {0, 1, 2, 4}, {0, 2, 3, 4}}};
        return evaluate(points, subtets, VolumeCellKind::Pyramid, options);
    }
}
