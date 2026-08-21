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

        constexpr std::array<ReferenceSample, 17> samples{{
            {-1.0, -1.0, -1.0, {JacobianSampleKind::Vertex, 0}, 0.0},
            {1.0, -1.0, -1.0, {JacobianSampleKind::Vertex, 1}, 0.0},
            {1.0, 1.0, -1.0, {JacobianSampleKind::Vertex, 2}, 0.0},
            {-1.0, 1.0, -1.0, {JacobianSampleKind::Vertex, 3}, 0.0},
            {-1.0, -1.0, 1.0, {JacobianSampleKind::Vertex, 4}, 0.0},
            {1.0, -1.0, 1.0, {JacobianSampleKind::Vertex, 5}, 0.0},
            {1.0, 1.0, 1.0, {JacobianSampleKind::Vertex, 6}, 0.0},
            {-1.0, 1.0, 1.0, {JacobianSampleKind::Vertex, 7}, 0.0},
            {0.0, 0.0, 0.0, {JacobianSampleKind::Center, 0}, 0.0},
            {-inverse_root_three, -inverse_root_three, -inverse_root_three, {JacobianSampleKind::IntegrationPoint, 0}, 1.0},
            {inverse_root_three, -inverse_root_three, -inverse_root_three, {JacobianSampleKind::IntegrationPoint, 1}, 1.0},
            {inverse_root_three, inverse_root_three, -inverse_root_three, {JacobianSampleKind::IntegrationPoint, 2}, 1.0},
            {-inverse_root_three, inverse_root_three, -inverse_root_three, {JacobianSampleKind::IntegrationPoint, 3}, 1.0},
            {-inverse_root_three, -inverse_root_three, inverse_root_three, {JacobianSampleKind::IntegrationPoint, 4}, 1.0},
            {inverse_root_three, -inverse_root_three, inverse_root_three, {JacobianSampleKind::IntegrationPoint, 5}, 1.0},
            {inverse_root_three, inverse_root_three, inverse_root_three, {JacobianSampleKind::IntegrationPoint, 6}, 1.0},
            {-inverse_root_three, inverse_root_three, inverse_root_three, {JacobianSampleKind::IntegrationPoint, 7}, 1.0}}};

        constexpr std::array<Scalar, 8> reference_r{
            -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0};
        constexpr std::array<Scalar, 8> reference_s{
            -1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0};
        constexpr std::array<Scalar, 8> reference_t{
            -1.0, -1.0, -1.0, -1.0, 1.0, 1.0, 1.0, 1.0};

        Eigen::Matrix<Scalar, 3, 3> hexaJacobian(
            const HexaPoints &points,
            Scalar r,
            Scalar s,
            Scalar t)
        {
            Eigen::Matrix<Scalar, 3, 3> jacobian =
                Eigen::Matrix<Scalar, 3, 3>::Zero();

            for (std::size_t index = 0; index < points.size(); ++index)
            {
                const Scalar derivative_r = Scalar{0.125} *
                    reference_r[index] *
                    (Scalar{1} + s * reference_s[index]) *
                    (Scalar{1} + t * reference_t[index]);
                const Scalar derivative_s = Scalar{0.125} *
                    reference_s[index] *
                    (Scalar{1} + r * reference_r[index]) *
                    (Scalar{1} + t * reference_t[index]);
                const Scalar derivative_t = Scalar{0.125} *
                    reference_t[index] *
                    (Scalar{1} + r * reference_r[index]) *
                    (Scalar{1} + s * reference_s[index]);

                jacobian.col(0) += derivative_r * points[index];
                jacobian.col(1) += derivative_s * points[index];
                jacobian.col(2) += derivative_t * points[index];
            }

            return jacobian;
        }

        std::optional<Scalar> hexaSkewness(
            const HexaPoints &points,
            Scalar length_tolerance,
            bool &degenerate)
        {
            Scalar skewness = 0.0;
            const std::array<std::array<Point3, 4>, 6> quads{{
                {points[0], points[3], points[2], points[1]},
                {points[4], points[5], points[6], points[7]},
                {points[0], points[1], points[5], points[4]},
                {points[1], points[2], points[6], points[5]},
                {points[2], points[3], points[7], points[6]},
                {points[3], points[0], points[4], points[7]}}};

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
            std::optional<JacobianSampleLocation> location = std::nullopt)
        {
            return VolumeCellEvaluationError{
                VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult,
                VolumeCellKind::Hexa,
                0.0,
                std::nullopt,
                location};
        }
    }

    Result<VolumeCellEvaluation, VolumeCellEvaluationError>
    evaluateHexa(
        const HexaPoints &points,
        const VolumeCellQualityOptions &options)
    {
        using EvaluationResult =
            Result<VolumeCellEvaluation, VolumeCellEvaluationError>;

        if (const auto error = quality_internal::validateQualityInput(
                points.data(),
                points.size(),
                VolumeCellKind::Hexa,
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
                JacobianSampleLocation{JacobianSampleKind::Center, 0}));
        }

        quality_internal::JacobianAccumulator accumulator{
            options.relative_jacobian_tolerance};
        Scalar signed_volume = 0.0;

        for (const ReferenceSample &sample : samples)
        {
            const Eigen::Matrix<Scalar, 3, 3> jacobian =
                hexaJacobian(points, sample.r, sample.s, sample.t);
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

        const Scalar length_tolerance =
            *characteristic_length * options.relative_length_tolerance;
        if (!std::isfinite(length_tolerance))
        {
            return EvaluationResult::failure(intermediateError());
        }

        bool face_degenerate = *characteristic_length == 0.0;
        const auto skewness = hexaSkewness(
            points,
            length_tolerance,
            face_degenerate);
        if (!skewness || !std::isfinite(*skewness))
        {
            return EvaluationResult::failure(intermediateError(
                JacobianSampleLocation{JacobianSampleKind::Center, 0}));
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
