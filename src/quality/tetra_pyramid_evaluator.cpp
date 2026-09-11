#include <algorithm>
#include <array>
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

        std::optional<Scalar> tetraSkewness(const TetraPoints &p)
        {
            const std::array<std::array<Point3,3>,4> faces{{
                {p[0],p[2],p[1]}, {p[0],p[1],p[3]},
                {p[1],p[2],p[3]}, {p[2],p[0],p[3]}}};
            Scalar maximum{};
            for (const auto &face : faces)
            {
                const auto value = triangleEquiangularSkewness(face, Scalar{0});
                if (!value.hasValue()) return std::nullopt;
                maximum = std::max(maximum, value.value());
            }
            return maximum;
        }

        std::optional<Scalar> pyramidSkewness(const PyramidPoints &p)
        {
            const auto base = quadEquiangularSkewness(
                {p[0],p[1],p[2],p[3]}, Scalar{0});
            if (!base.hasValue()) return std::nullopt;
            Scalar maximum = base.value();
            const std::array<std::array<Point3,3>,4> sides{{
                {p[0],p[1],p[4]}, {p[1],p[2],p[4]},
                {p[2],p[3],p[4]}, {p[3],p[0],p[4]}}};
            for (const auto &face : sides)
            {
                const auto value = triangleEquiangularSkewness(face, Scalar{0});
                if (!value.hasValue()) return std::nullopt;
                maximum = std::max(maximum, value.value());
            }
            return maximum;
        }

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
            const auto local = quality_internal::minimumLocalJacobian(points);
            if (!local)
                return EvaluationResult::failure({
                    VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult,
                    kind, Scalar{0}, std::nullopt, std::nullopt});
            result.minimum_local_jacobian = *local;
            if (result.validity == VolumeCellValidity::Valid &&
                *local < Scalar{0})
                result.validity = VolumeCellValidity::LocallyInverted;
            else if (result.validity == VolumeCellValidity::Valid &&
                     *local == Scalar{0})
                result.validity = VolumeCellValidity::Degenerate;
            result.acceptable = result.validity == VolumeCellValidity::Valid;
            return EvaluationResult::success(result);
        }
    }

    Result<VolumeCellEvaluation, VolumeCellEvaluationError> evaluateTetra(
        const TetraPoints &points,
        const VolumeCellQualityOptions &options)
    {
        constexpr std::array<Subtet, 1> subtets{{{0, 1, 2, 3}}};
        auto result = evaluate(points, subtets, VolumeCellKind::Tetra, options);
        if (!result.hasValue()) return result;
        const auto skewness = tetraSkewness(points);
        if (!skewness) return Result<VolumeCellEvaluation,
            VolumeCellEvaluationError>::failure({
                VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult,
                VolumeCellKind::Tetra,Scalar{0},std::nullopt,std::nullopt});
        result.value().skewness = *skewness;
        return result;
    }

    Result<VolumeCellEvaluation, VolumeCellEvaluationError> evaluatePyramid(
        const PyramidPoints &points,
        const VolumeCellQualityOptions &options)
    {
        constexpr std::array<Subtet, 2> subtets{{
            {0, 1, 2, 4}, {0, 2, 3, 4}}};
        auto result = evaluate(
            points, subtets, VolumeCellKind::Pyramid, options);
        if (!result.hasValue()) return result;
        const auto integrated =
            quality_internal::integratedJacobianVolume(points);
        if (!integrated.has_value())
            return Result<VolumeCellEvaluation,
                VolumeCellEvaluationError>::failure({
                    VolumeCellEvaluationErrorCategory::
                        NonFiniteIntermediateResult,
                    VolumeCellKind::Pyramid,Scalar{0},
                    std::nullopt,std::nullopt});
        result.value().validity = quality_internal::combinedValidity(
            result.value().validity,*integrated);
        const auto local = quality_internal::minimumLocalJacobian(points);
        if (!local) return Result<VolumeCellEvaluation,
            VolumeCellEvaluationError>::failure({
                VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult,
                VolumeCellKind::Pyramid,Scalar{0},std::nullopt,std::nullopt});
        result.value().minimum_local_jacobian = *local;
        if (result.value().validity == VolumeCellValidity::Valid &&
            *local < Scalar{0})
            result.value().validity = VolumeCellValidity::LocallyInverted;
        else if (result.value().validity == VolumeCellValidity::Valid &&
                 *local == Scalar{0})
            result.value().validity = VolumeCellValidity::Degenerate;
        result.value().signed_volume = *integrated;
        const auto skewness = pyramidSkewness(points);
        if (!skewness) return Result<VolumeCellEvaluation,
            VolumeCellEvaluationError>::failure({
                VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult,
                VolumeCellKind::Pyramid,Scalar{0},std::nullopt,std::nullopt});
        result.value().skewness = *skewness;
        result.value().acceptable =
            result.value().validity == VolumeCellValidity::Valid;
        return result;
    }
}
