#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

#include <Eigen/Core>
#include <Eigen/LU>

#include <boundary_mesh/quality/volume_cell_evaluator.hpp>
#include <boundary_mesh/surface/face_skewness.hpp>

#include "volume_cell_evaluator_internal.hpp"

namespace boundary_mesh
{
    namespace
    {
        struct ReferenceSample
        {
            Scalar r;
            Scalar s;
            Scalar t;
            JacobianSampleLocation location;
            Scalar volume_weight;
        };

        constexpr Scalar inverse_root_three =
            0.577350269189625764509148780501957456;

        constexpr std::array<ReferenceSample, 13> samples{{
            {0.0, 0.0, -1.0, {JacobianSampleKind::Vertex, 0}, 0.0},
            {1.0, 0.0, -1.0, {JacobianSampleKind::Vertex, 1}, 0.0},
            {0.0, 1.0, -1.0, {JacobianSampleKind::Vertex, 2}, 0.0},
            {0.0, 0.0, 1.0, {JacobianSampleKind::Vertex, 3}, 0.0},
            {1.0, 0.0, 1.0, {JacobianSampleKind::Vertex, 4}, 0.0},
            {0.0, 1.0, 1.0, {JacobianSampleKind::Vertex, 5}, 0.0},
            {1.0 / 3.0, 1.0 / 3.0, 0.0, {JacobianSampleKind::Center, 0}, 0.0},
            {1.0 / 6.0, 1.0 / 6.0, -inverse_root_three, {JacobianSampleKind::IntegrationPoint, 0}, 1.0 / 6.0},
            {1.0 / 6.0, 1.0 / 6.0, inverse_root_three, {JacobianSampleKind::IntegrationPoint, 1}, 1.0 / 6.0},
            {2.0 / 3.0, 1.0 / 6.0, -inverse_root_three, {JacobianSampleKind::IntegrationPoint, 2}, 1.0 / 6.0},
            {2.0 / 3.0, 1.0 / 6.0, inverse_root_three, {JacobianSampleKind::IntegrationPoint, 3}, 1.0 / 6.0},
            {1.0 / 6.0, 2.0 / 3.0, -inverse_root_three, {JacobianSampleKind::IntegrationPoint, 4}, 1.0 / 6.0},
            {1.0 / 6.0, 2.0 / 3.0, inverse_root_three, {JacobianSampleKind::IntegrationPoint, 5}, 1.0 / 6.0}}};

        Eigen::Matrix<Scalar, 3, 3> prismJacobian(
            const PrismPoints &points,
            Scalar r,
            Scalar s,
            Scalar t)
        {
            const Scalar lower = (Scalar{1} - t) / Scalar{2};
            const Scalar upper = (Scalar{1} + t) / Scalar{2};
            const Scalar first_triangle_coordinate =
                Scalar{1} - r - s;

            Eigen::Matrix<Scalar, 3, 3> jacobian;
            jacobian.col(0) =
                lower * (points[1] - points[0]) +
                upper * (points[4] - points[3]);
            jacobian.col(1) =
                lower * (points[2] - points[0]) +
                upper * (points[5] - points[3]);
            jacobian.col(2) = Scalar{0.5} * (
                first_triangle_coordinate * (points[3] - points[0]) +
                r * (points[4] - points[1]) +
                s * (points[5] - points[2]));
            return jacobian;
        }

        std::optional<Scalar> prismSkewness(
            const PrismPoints &points,
            Scalar length_tolerance,
            bool &degenerate)
        {
            Scalar skewness = 0.0;

            const std::array<std::array<Point3, 3>, 2> triangles{{
                {points[0], points[1], points[2]},
                {points[3], points[4], points[5]}}};
            for (const auto &triangle : triangles)
            {
                const auto result = triangleEquiangularSkewness(
                    triangle,
                    length_tolerance);
                if (!result.hasValue())
                {
                    degenerate = true;
                    continue;
                }
                skewness = std::max(skewness, result.value());
            }

            const std::array<std::array<Point3, 4>, 3> quads{{
                {points[0], points[1], points[4], points[3]},
                {points[1], points[2], points[5], points[4]},
                {points[2], points[0], points[3], points[5]}}};
            for (const auto &quad : quads)
            {
                const auto result = quadEquiangularSkewness(
                    quad,
                    length_tolerance);
                if (!result.hasValue())
                {
                    degenerate = true;
                    continue;
                }
                skewness = std::max(skewness, result.value());
            }

            return degenerate ? std::optional<Scalar>{Scalar{1}}
                              : std::optional<Scalar>{skewness};
        }

        VolumeCellEvaluationError intermediateError(
            JacobianSampleLocation location)
        {
            return VolumeCellEvaluationError{
                VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult,
                VolumeCellKind::Prism,
                0.0,
                std::nullopt,
                location};
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

        const auto characteristic_length =
            quality_internal::characteristicLength(
                points.data(), points.size());
        if (!characteristic_length)
        {
            return EvaluationResult::failure(intermediateError(
                {JacobianSampleKind::Center, 0}));
        }

        quality_internal::JacobianAccumulator accumulator{
            options.relative_jacobian_tolerance};
        Scalar signed_volume = 0.0;

        for (const ReferenceSample &sample : samples)
        {
            const Eigen::Matrix<Scalar, 3, 3> jacobian =
                prismJacobian(points, sample.r, sample.s, sample.t);
            const Scalar determinant = jacobian.determinant();
            const Scalar local_scale =
                jacobian.col(0).norm() *
                jacobian.col(1).norm() *
                jacobian.col(2).norm();

            if (!accumulator.add(
                    determinant,
                    local_scale,
                    sample.location))
            {
                return EvaluationResult::failure(
                    intermediateError(sample.location));
            }

            signed_volume += sample.volume_weight * determinant;
            if (!std::isfinite(signed_volume))
            {
                return EvaluationResult::failure(
                    intermediateError(sample.location));
            }
        }

        bool face_degenerate = *characteristic_length == 0.0;
        const auto skewness = prismSkewness(
            points,
            *characteristic_length * options.relative_length_tolerance,
            face_degenerate);
        if (!skewness || !std::isfinite(*skewness))
        {
            return EvaluationResult::failure(intermediateError(
                {JacobianSampleKind::Center, 0}));
        }

        VolumeCellEvaluation evaluation;
        evaluation.validity = accumulator.validity(face_degenerate);
        evaluation.signed_volume = signed_volume;
        evaluation.minimum_jacobian = accumulator.minimum_jacobian;
        evaluation.maximum_jacobian = accumulator.maximum_jacobian;
        evaluation.minimum_normalized_jacobian =
            accumulator.minimum_normalized_jacobian;
        evaluation.maximum_normalized_jacobian =
            accumulator.maximum_normalized_jacobian;
        evaluation.skewness = *skewness;
        evaluation.worst_jacobian_location = accumulator.worst_location;
        evaluation.acceptable =
            evaluation.validity == VolumeCellValidity::Valid &&
            evaluation.skewness <= options.maximum_skewness;

        return EvaluationResult::success(evaluation);
    }
}
