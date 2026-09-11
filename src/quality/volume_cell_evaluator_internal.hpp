#pragma once

#include <cstddef>
#include <optional>

#include <boundary_mesh/quality/volume_cell_evaluation.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation_error.hpp>

namespace boundary_mesh::quality_internal
{
    struct SubtetVolumeAccumulator
    {
        bool add(
            const Point3 &a,
            const Point3 &b,
            const Point3 &c,
            const Point3 &d,
            std::size_t subtet_index) noexcept;

        VolumeCellValidity validity() const noexcept;

        Scalar signed_volume{};
        Scalar minimum_signed_volume{};
        Scalar maximum_signed_volume{};
        std::size_t worst_subtet_index{};
        bool initialized{};
        bool has_positive{};
        bool has_negative{};
        bool has_zero{};
    };

    std::optional<VolumeCellEvaluationError> validateQualityInput(
        const Point3 *points,
        std::size_t point_count,
        VolumeCellKind cell_kind,
        const VolumeCellQualityOptions &options) noexcept;

    std::optional<Scalar> integratedJacobianVolume(
        const PyramidPoints &points) noexcept;

    std::optional<Scalar> integratedJacobianVolume(
        const PrismPoints &points) noexcept;

    std::optional<Scalar> integratedJacobianVolume(
        const HexaPoints &points) noexcept;

    VolumeCellValidity combinedValidity(
        VolumeCellValidity subtet_validity,
        Scalar integrated_volume) noexcept;

    std::optional<Scalar> minimumLocalJacobian(
        const TetraPoints &points) noexcept;
    std::optional<Scalar> minimumLocalJacobian(
        const PyramidPoints &points) noexcept;
    std::optional<Scalar> minimumLocalJacobian(
        const PrismPoints &points) noexcept;
    std::optional<Scalar> minimumLocalJacobian(
        const HexaPoints &points) noexcept;
}
