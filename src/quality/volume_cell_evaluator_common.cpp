#include <cmath>

#include <Eigen/Geometry>

#include "volume_cell_evaluator_internal.hpp"

namespace boundary_mesh::quality_internal
{
    namespace
    {
        Scalar determinant(
            const Vector3 &dr,
            const Vector3 &ds,
            const Vector3 &dt) noexcept
        {
            return dr.dot(ds.cross(dt));
        }

        constexpr Scalar gauss_offset =
            Scalar{0.5} / Scalar{1.7320508075688772935274463415059};
        constexpr std::array<Scalar,2> gauss_points{{
            Scalar{0.5} - gauss_offset,
            Scalar{0.5} + gauss_offset}};
    }

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

    VolumeCellValidity combinedValidity(
        VolumeCellValidity subtet_validity,
        Scalar integrated_volume) noexcept
    {
        if (subtet_validity != VolumeCellValidity::Valid)
            return subtet_validity;
        if (integrated_volume > Scalar{0})
            return VolumeCellValidity::Valid;
        if (integrated_volume < Scalar{0})
            return VolumeCellValidity::LocallyInverted;
        return VolumeCellValidity::Degenerate;
    }

    std::optional<Scalar> integratedJacobianVolume(
        const PyramidPoints &p) noexcept
    {
        Scalar volume{};
        for (const Scalar r : gauss_points)
            for (const Scalar s : gauss_points)
                for (const Scalar t : gauss_points)
                {
                    const Vector3 dr = (Scalar{1}-t) * (
                        (Scalar{1}-s)*(p[1]-p[0]) +
                        s*(p[2]-p[3]));
                    const Vector3 ds = (Scalar{1}-t) * (
                        (Scalar{1}-r)*(p[3]-p[0]) +
                        r*(p[2]-p[1]));
                    const Point3 base =
                        (Scalar{1}-r)*(Scalar{1}-s)*p[0] +
                        r*(Scalar{1}-s)*p[1] + r*s*p[2] +
                        (Scalar{1}-r)*s*p[3];
                    volume += determinant(dr,ds,p[4]-base) /
                        Scalar{8};
                }
        return std::isfinite(volume)
            ? std::optional<Scalar>{volume} : std::nullopt;
    }

    std::optional<Scalar> integratedJacobianVolume(
        const PrismPoints &p) noexcept
    {
        constexpr std::array<std::array<Scalar,2>,3> triangle_points{{
            {{Scalar{1}/Scalar{6},Scalar{1}/Scalar{6}}},
            {{Scalar{2}/Scalar{3},Scalar{1}/Scalar{6}}},
            {{Scalar{1}/Scalar{6},Scalar{2}/Scalar{3}}}}};
        Scalar volume{};
        for (const auto &rs : triangle_points)
            for (const Scalar t : gauss_points)
            {
                const Scalar r = rs[0];
                const Scalar s = rs[1];
                const Vector3 dr = (Scalar{1}-t)*(p[1]-p[0]) +
                    t*(p[4]-p[3]);
                const Vector3 ds = (Scalar{1}-t)*(p[2]-p[0]) +
                    t*(p[5]-p[3]);
                const Point3 bottom = (Scalar{1}-r-s)*p[0] +
                    r*p[1] + s*p[2];
                const Point3 top = (Scalar{1}-r-s)*p[3] +
                    r*p[4] + s*p[5];
                volume += determinant(dr,ds,top-bottom) /
                    Scalar{12};
            }
        return std::isfinite(volume)
            ? std::optional<Scalar>{volume} : std::nullopt;
    }

    std::optional<Scalar> integratedJacobianVolume(
        const HexaPoints &p) noexcept
    {
        Scalar volume{};
        for (const Scalar r : gauss_points)
            for (const Scalar s : gauss_points)
                for (const Scalar t : gauss_points)
                {
                    const Vector3 dr =
                        (Scalar{1}-s)*(Scalar{1}-t)*(p[1]-p[0]) +
                        s*(Scalar{1}-t)*(p[2]-p[3]) +
                        (Scalar{1}-s)*t*(p[5]-p[4]) +
                        s*t*(p[6]-p[7]);
                    const Vector3 ds =
                        (Scalar{1}-r)*(Scalar{1}-t)*(p[3]-p[0]) +
                        r*(Scalar{1}-t)*(p[2]-p[1]) +
                        (Scalar{1}-r)*t*(p[7]-p[4]) +
                        r*t*(p[6]-p[5]);
                    const Vector3 dt =
                        (Scalar{1}-r)*(Scalar{1}-s)*(p[4]-p[0]) +
                        r*(Scalar{1}-s)*(p[5]-p[1]) +
                        r*s*(p[6]-p[2]) +
                        (Scalar{1}-r)*s*(p[7]-p[3]);
                    volume += determinant(dr,ds,dt) / Scalar{8};
                }
        return std::isfinite(volume)
            ? std::optional<Scalar>{volume} : std::nullopt;
    }

    namespace
    {
        std::optional<Scalar> determinantValue(
            const Vector3 &dr,
            const Vector3 &ds,
            const Vector3 &dt) noexcept
        {
            const Scalar value = determinant(dr, ds, dt);
            return std::isfinite(value) ? std::optional<Scalar>{value}
                                        : std::nullopt;
        }
    }

    std::optional<Scalar> minimumLocalJacobian(
        const TetraPoints &p) noexcept
    {
        return determinantValue(
            p[1] - p[0], p[2] - p[0], p[3] - p[0]);
    }

    std::optional<Scalar> minimumLocalJacobian(
        const PyramidPoints &p) noexcept
    {
        Scalar minimum = std::numeric_limits<Scalar>::infinity();
        for (const Scalar r : {Scalar{0}, Scalar{1}, Scalar{0.5}})
            for (const Scalar s : {Scalar{0}, Scalar{1}, Scalar{0.5}})
                for (const Scalar t : {Scalar{0}, Scalar{0.5}})
                {
                    const Vector3 dr = (Scalar{1} - t) *
                        ((Scalar{1} - s) * (p[1] - p[0]) +
                         s * (p[2] - p[3]));
                    const Vector3 ds = (Scalar{1} - t) *
                        ((Scalar{1} - r) * (p[3] - p[0]) +
                         r * (p[2] - p[1]));
                    const Point3 base =
                        (Scalar{1} - r) * (Scalar{1} - s) * p[0] +
                        r * (Scalar{1} - s) * p[1] + r * s * p[2] +
                        (Scalar{1} - r) * s * p[3];
                    const auto value = determinantValue(
                        dr, ds, p[4] - base);
                    if (!value) return std::nullopt;
                    minimum = std::min(minimum, *value);
                }
        return minimum;
    }

    std::optional<Scalar> minimumLocalJacobian(
        const PrismPoints &p) noexcept
    {
        constexpr std::array<std::array<Scalar, 3>, 9> samples{{
            {{0, 0, 0}}, {{1, 0, 0}}, {{0, 1, 0}},
            {{0, 0, 1}}, {{1, 0, 1}}, {{0, 1, 1}},
            {{Scalar{1}/3, Scalar{1}/3, 0}},
            {{Scalar{1}/3, Scalar{1}/3, 1}},
            {{Scalar{1}/3, Scalar{1}/3, Scalar{1}/2}}}};
        Scalar minimum = std::numeric_limits<Scalar>::infinity();
        for (const auto &sample : samples)
        {
            const Scalar r = sample[0];
            const Scalar s = sample[1];
            const Scalar t = sample[2];
            const Scalar a = Scalar{1} - t;
            const Scalar b = t;
            const Vector3 dr = a * (p[1] - p[0]) + b * (p[4] - p[3]);
            const Vector3 ds = a * (p[2] - p[0]) + b * (p[5] - p[3]);
            const Vector3 dt =
                (Scalar{1} - r - s) * (p[3] - p[0]) +
                r * (p[4] - p[1]) + s * (p[5] - p[2]);
            const auto value = determinantValue(dr, ds, dt);
            if (!value) return std::nullopt;
            minimum = std::min(minimum, *value);
        }
        return minimum;
    }

    std::optional<Scalar> minimumLocalJacobian(
        const HexaPoints &p) noexcept
    {
        Scalar minimum = std::numeric_limits<Scalar>::infinity();
        for (const Scalar r : {Scalar{0}, Scalar{1}, Scalar{0.5}})
            for (const Scalar s : {Scalar{0}, Scalar{1}, Scalar{0.5}})
                for (const Scalar t : {Scalar{0}, Scalar{1}, Scalar{0.5}})
                {
                    const Vector3 dr =
                        (Scalar{1}-s)*(Scalar{1}-t)*(p[1]-p[0]) +
                        s*(Scalar{1}-t)*(p[2]-p[3]) +
                        (Scalar{1}-s)*t*(p[5]-p[4]) +
                        s*t*(p[6]-p[7]);
                    const Vector3 ds =
                        (Scalar{1}-r)*(Scalar{1}-t)*(p[3]-p[0]) +
                        r*(Scalar{1}-t)*(p[2]-p[1]) +
                        (Scalar{1}-r)*t*(p[7]-p[4]) +
                        r*t*(p[6]-p[5]);
                    const Vector3 dt =
                        (Scalar{1}-r)*(Scalar{1}-s)*(p[4]-p[0]) +
                        r*(Scalar{1}-s)*(p[5]-p[1]) +
                        r*s*(p[6]-p[2]) +
                        (Scalar{1}-r)*s*(p[7]-p[3]);
                    const auto value = determinantValue(dr, ds, dt);
                    if (!value) return std::nullopt;
                    minimum = std::min(minimum, *value);
                }
        return minimum;
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
