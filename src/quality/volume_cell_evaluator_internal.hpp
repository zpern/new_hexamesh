#pragma once

#include <cstddef>
#include <optional>

#include <boundary_mesh/quality/volume_cell_evaluation.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation_error.hpp>

namespace boundary_mesh::quality_internal
{
    struct JacobianAccumulator
    {
        explicit JacobianAccumulator(Scalar tolerance) noexcept;

        bool add(
            Scalar determinant,
            Scalar local_scale,
            JacobianSampleLocation location) noexcept;

        VolumeCellValidity validity(bool force_degenerate) const noexcept;

        Scalar tolerance{};
        Scalar minimum_jacobian{};
        Scalar maximum_jacobian{};
        Scalar minimum_normalized_jacobian{};
        Scalar maximum_normalized_jacobian{};
        JacobianSampleLocation worst_location{};
        bool initialized{};
        bool has_positive{};
        bool has_negative{};
        bool has_degenerate{};
    };

    std::optional<VolumeCellEvaluationError> validateQualityInput(
        const Point3 *points,
        std::size_t point_count,
        VolumeCellKind cell_kind,
        const VolumeCellQualityOptions &options) noexcept;

    std::optional<Scalar> characteristicLength(
        const Point3 *points,
        std::size_t point_count) noexcept;
}
