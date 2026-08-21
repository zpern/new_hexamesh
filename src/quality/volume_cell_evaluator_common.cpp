#include <algorithm>
#include <cmath>

#include "volume_cell_evaluator_internal.hpp"

namespace boundary_mesh::quality_internal
{
    JacobianAccumulator::JacobianAccumulator(
        Scalar sample_tolerance) noexcept
        : tolerance(sample_tolerance)
    {
    }

    bool JacobianAccumulator::add(
        Scalar determinant,
        Scalar local_scale,
        JacobianSampleLocation location) noexcept
    {
        if (!std::isfinite(determinant) ||
            !std::isfinite(local_scale))
        {
            return false;
        }

        Scalar normalized = 0.0;
        if (local_scale > 0.0)
        {
            normalized = determinant / local_scale;
            if (!std::isfinite(normalized))
            {
                return false;
            }
        }

        if (!initialized)
        {
            minimum_jacobian = determinant;
            maximum_jacobian = determinant;
            minimum_normalized_jacobian = normalized;
            maximum_normalized_jacobian = normalized;
            worst_location = location;
            initialized = true;
        }
        else
        {
            minimum_jacobian = std::min(minimum_jacobian, determinant);
            maximum_jacobian = std::max(maximum_jacobian, determinant);
            maximum_normalized_jacobian =
                std::max(maximum_normalized_jacobian, normalized);

            if (normalized < minimum_normalized_jacobian)
            {
                minimum_normalized_jacobian = normalized;
                worst_location = location;
            }
        }

        if (local_scale <= 0.0)
        {
            has_degenerate = true;
        }
        else if (normalized > tolerance)
        {
            has_positive = true;
        }
        else if (normalized < -tolerance)
        {
            has_negative = true;
        }
        else
        {
            has_degenerate = true;
        }

        return true;
    }

    VolumeCellValidity JacobianAccumulator::validity(
        bool force_degenerate) const noexcept
    {
        if (has_positive && has_negative)
        {
            return VolumeCellValidity::LocallyInverted;
        }

        if (has_degenerate || force_degenerate)
        {
            return VolumeCellValidity::Degenerate;
        }

        if (has_negative)
        {
            return VolumeCellValidity::Reversed;
        }

        return VolumeCellValidity::Valid;
    }

    std::optional<VolumeCellEvaluationError> validateQualityInput(
        const Point3 *points,
        std::size_t point_count,
        VolumeCellKind cell_kind,
        const VolumeCellQualityOptions &options) noexcept
    {
        if (!std::isfinite(options.relative_jacobian_tolerance) ||
            options.relative_jacobian_tolerance <= 0.0)
        {
            return VolumeCellEvaluationError{
                VolumeCellEvaluationErrorCategory::InvalidRelativeJacobianTolerance,
                cell_kind,
                options.relative_jacobian_tolerance,
                std::nullopt,
                std::nullopt};
        }

        if (!std::isfinite(options.relative_length_tolerance) ||
            options.relative_length_tolerance <= 0.0)
        {
            return VolumeCellEvaluationError{
                VolumeCellEvaluationErrorCategory::InvalidRelativeLengthTolerance,
                cell_kind,
                options.relative_length_tolerance,
                std::nullopt,
                std::nullopt};
        }

        if (!std::isfinite(options.maximum_skewness) ||
            options.maximum_skewness < 0.0 ||
            options.maximum_skewness > 1.0)
        {
            return VolumeCellEvaluationError{
                VolumeCellEvaluationErrorCategory::InvalidMaximumSkewness,
                cell_kind,
                options.maximum_skewness,
                std::nullopt,
                std::nullopt};
        }

        for (std::size_t index = 0; index < point_count; ++index)
        {
            if (!points[index].allFinite())
            {
                return VolumeCellEvaluationError{
                    VolumeCellEvaluationErrorCategory::NonFiniteVertexCoordinate,
                    cell_kind,
                    0.0,
                    index,
                    std::nullopt};
            }
        }

        return std::nullopt;
    }

    std::optional<Scalar> characteristicLength(
        const Point3 *points,
        std::size_t point_count) noexcept
    {
        Scalar maximum_squared_distance = 0.0;
        for (std::size_t first = 0; first < point_count; ++first)
        {
            for (std::size_t second = first + 1;
                 second < point_count;
                 ++second)
            {
                const Scalar squared_distance =
                    (points[second] - points[first]).squaredNorm();
                if (!std::isfinite(squared_distance))
                {
                    return std::nullopt;
                }

                maximum_squared_distance = std::max(
                    maximum_squared_distance,
                    squared_distance);
            }
        }

        const Scalar length = std::sqrt(maximum_squared_distance);
        if (!std::isfinite(length))
        {
            return std::nullopt;
        }

        return length;
    }
}
