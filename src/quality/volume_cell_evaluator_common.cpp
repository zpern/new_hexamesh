#include <cmath>

#include <Eigen/Geometry>

#include "volume_cell_evaluator_internal.hpp"

namespace boundary_mesh::quality_internal
{
    bool SubtetVolumeAccumulator::add(
        const Point3 &a,
        const Point3 &b,
        const Point3 &c,
        const Point3 &d,
        std::size_t subtet_index) noexcept
    {
        const Vector3 b_minus_a = b - a;
        const Vector3 c_minus_a = c - a;
        const Vector3 d_minus_a = d - a;
        if (!b_minus_a.allFinite() ||
            !c_minus_a.allFinite() ||
            !d_minus_a.allFinite())
        {
            return false;
        }

        const Vector3 cross_product =
            c_minus_a.cross(d_minus_a);
        if (!cross_product.allFinite())
        {
            return false;
        }

        const Scalar signed_volume_6 =
            b_minus_a.dot(cross_product);
        if (!std::isfinite(signed_volume_6))
        {
            return false;
        }

        const Scalar subtet_signed_volume =
            signed_volume_6 / Scalar{6};
        const Scalar accumulated_volume =
            signed_volume + subtet_signed_volume;
        if (!std::isfinite(subtet_signed_volume) ||
            !std::isfinite(accumulated_volume))
        {
            return false;
        }

        if (!initialized)
        {
            minimum_signed_volume = subtet_signed_volume;
            maximum_signed_volume = subtet_signed_volume;
            worst_subtet_index = subtet_index;
            initialized = true;
        }
        else
        {
            if (subtet_signed_volume < minimum_signed_volume)
            {
                minimum_signed_volume = subtet_signed_volume;
                worst_subtet_index = subtet_index;
            }
            if (subtet_signed_volume > maximum_signed_volume)
            {
                maximum_signed_volume = subtet_signed_volume;
            }
        }

        signed_volume = accumulated_volume;
        if (signed_volume_6 > Scalar{0})
        {
            has_positive = true;
        }
        else if (signed_volume_6 < Scalar{0})
        {
            has_negative = true;
        }
        else
        {
            has_zero = true;
        }

        return true;
    }

    VolumeCellValidity SubtetVolumeAccumulator::validity() const noexcept
    {
        if (has_positive && has_negative)
        {
            return VolumeCellValidity::LocallyInverted;
        }

        if (has_zero || !initialized)
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
        if (!std::isfinite(options.maximum_skewness) ||
            options.maximum_skewness < Scalar{0} ||
            options.maximum_skewness > Scalar{1})
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
                    Scalar{0},
                    index,
                    std::nullopt};
            }
        }

        return std::nullopt;
    }
}
