#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>

#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    using EvaluationResult =
        Result<VolumeCellEvaluation, VolumeCellEvaluationError>;

    template <typename Points>
    using Evaluator = EvaluationResult (*)(
        const Points &,
        const VolumeCellQualityOptions &);

    PrismPoints idealPrism()
    {
        const Scalar root_three = std::sqrt(Scalar{3});
        const Scalar height = Scalar{2} / root_three;
        return PrismPoints{
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.5, root_three / 2.0, 0.0},
            Point3{0.0, 0.0, height},
            Point3{1.0, 0.0, height},
            Point3{0.5, root_three / 2.0, height}};
    }

    PrismPoints distortedPrism()
    {
        return PrismPoints{
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0},
            Point3{0.0, 1.0, 1.0}};
    }

    HexaPoints unitHexa()
    {
        return HexaPoints{
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{1.0, 1.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0},
            Point3{1.0, 1.0, 1.0},
            Point3{0.0, 1.0, 1.0}};
    }

    HexaPoints distortedHexa()
    {
        return HexaPoints{
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{2.0, 1.0, 0.0},
            Point3{1.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0},
            Point3{2.0, 1.0, 1.0},
            Point3{1.0, 1.0, 1.0}};
    }

    template <typename Points>
    Points scaled(
        const Points &points,
        Scalar factor)
    {
        Points result = points;
        for (Point3 &point : result)
        {
            point *= factor;
        }
        return result;
    }

    bool sameConfigurationValue(
        Scalar actual,
        Scalar expected)
    {
        if (std::isnan(expected))
        {
            return std::isnan(actual);
        }
        return actual == expected;
    }

    template <typename Points>
    bool hasConfigurationError(
        const Points &points,
        Evaluator<Points> evaluator,
        const VolumeCellQualityOptions &options,
        VolumeCellEvaluationErrorCategory expected_category,
        VolumeCellKind expected_kind,
        Scalar expected_value)
    {
        const EvaluationResult result = evaluator(points, options);
        if (result.hasValue())
        {
            return false;
        }

        const VolumeCellEvaluationError &error = result.error();
        return error.category == expected_category &&
               error.cell_kind == expected_kind &&
               sameConfigurationValue(
                   error.configuration_value,
                   expected_value) &&
               !error.local_vertex_index.has_value() &&
               !error.jacobian_sample_location.has_value();
    }

    template <typename Points>
    bool checkConfigurationErrors(
        const Points &points,
        Evaluator<Points> evaluator,
        VolumeCellKind expected_kind)
    {
        const Scalar nan =
            std::numeric_limits<Scalar>::quiet_NaN();
        const Scalar infinity =
            std::numeric_limits<Scalar>::infinity();
        const std::array<Scalar, 4> invalid_tolerances{
            Scalar{0}, Scalar{-1}, nan, infinity};

        for (const Scalar value : invalid_tolerances)
        {
            VolumeCellQualityOptions options;
            options.relative_jacobian_tolerance = value;
            if (!hasConfigurationError(
                    points,
                    evaluator,
                    options,
                    VolumeCellEvaluationErrorCategory::
                        InvalidRelativeJacobianTolerance,
                    expected_kind,
                    value))
            {
                return false;
            }
        }

        for (const Scalar value : invalid_tolerances)
        {
            VolumeCellQualityOptions options;
            options.relative_length_tolerance = value;
            if (!hasConfigurationError(
                    points,
                    evaluator,
                    options,
                    VolumeCellEvaluationErrorCategory::
                        InvalidRelativeLengthTolerance,
                    expected_kind,
                    value))
            {
                return false;
            }
        }

        const std::array<Scalar, 4> invalid_skewness{
            Scalar{-0.01}, Scalar{1.01}, nan, infinity};
        for (const Scalar value : invalid_skewness)
        {
            VolumeCellQualityOptions options;
            options.maximum_skewness = value;
            if (!hasConfigurationError(
                    points,
                    evaluator,
                    options,
                    VolumeCellEvaluationErrorCategory::
                        InvalidMaximumSkewness,
                    expected_kind,
                    value))
            {
                return false;
            }
        }

        return true;
    }

    template <typename Points>
    bool hasVertexError(
        const Points &points,
        Evaluator<Points> evaluator,
        VolumeCellKind expected_kind,
        std::size_t expected_index)
    {
        const EvaluationResult result =
            evaluator(points, VolumeCellQualityOptions{});
        if (result.hasValue())
        {
            return false;
        }

        const VolumeCellEvaluationError &error = result.error();
        return error.category ==
                   VolumeCellEvaluationErrorCategory::
                       NonFiniteVertexCoordinate &&
               error.cell_kind == expected_kind &&
               error.configuration_value == 0.0 &&
               error.local_vertex_index == expected_index &&
               !error.jacobian_sample_location.has_value();
    }

    template <typename Points>
    bool checkNonFiniteVertices(
        const Points &finite_points,
        Evaluator<Points> evaluator,
        VolumeCellKind expected_kind,
        std::size_t nan_index,
        std::size_t infinity_index)
    {
        Points nan_points = finite_points;
        nan_points[nan_index].y() =
            std::numeric_limits<Scalar>::quiet_NaN();
        if (!hasVertexError(
                nan_points,
                evaluator,
                expected_kind,
                nan_index))
        {
            return false;
        }

        Points infinity_points = finite_points;
        infinity_points[infinity_index].z() =
            std::numeric_limits<Scalar>::infinity();
        return hasVertexError(
            infinity_points,
            evaluator,
            expected_kind,
            infinity_index);
    }

    template <typename Points>
    bool hasIntermediateError(
        const Points &points,
        Evaluator<Points> evaluator,
        const VolumeCellQualityOptions &options,
        VolumeCellKind expected_kind,
        std::optional<JacobianSampleLocation> expected_location)
    {
        const EvaluationResult result = evaluator(points, options);
        if (result.hasValue())
        {
            return false;
        }

        const VolumeCellEvaluationError &error = result.error();
        if (error.category !=
                VolumeCellEvaluationErrorCategory::
                    NonFiniteIntermediateResult ||
            error.cell_kind != expected_kind ||
            error.configuration_value != 0.0 ||
            error.local_vertex_index.has_value() ||
            error.jacobian_sample_location.has_value() !=
                expected_location.has_value())
        {
            return false;
        }

        if (!expected_location)
        {
            return true;
        }

        return error.jacobian_sample_location->kind ==
                   expected_location->kind &&
               error.jacobian_sample_location->index ==
                   expected_location->index;
    }

    template <typename Points>
    bool checkIntermediateErrors(
        const Points &points,
        Evaluator<Points> evaluator,
        VolumeCellKind expected_kind)
    {
        VolumeCellQualityOptions overflowing_length_options;
        overflowing_length_options.relative_length_tolerance =
            std::numeric_limits<Scalar>::max();
        if (!hasIntermediateError(
                points,
                evaluator,
                overflowing_length_options,
                expected_kind,
                std::nullopt))
        {
            return false;
        }

        Points overflowing_difference_points = points;
        overflowing_difference_points[0].x() =
            std::numeric_limits<Scalar>::max();
        overflowing_difference_points[1].x() =
            -std::numeric_limits<Scalar>::max();
        if (!hasIntermediateError(
                overflowing_difference_points,
                evaluator,
                VolumeCellQualityOptions{},
                expected_kind,
                std::nullopt))
        {
            return false;
        }

        const Points overflowing_jacobian_points =
            scaled(points, Scalar{1e200});
        return hasIntermediateError(
            overflowing_jacobian_points,
            evaluator,
            VolumeCellQualityOptions{},
            expected_kind,
            JacobianSampleLocation{
                JacobianSampleKind::Vertex,
                0});
    }

    bool relativelyNear(
        Scalar first,
        Scalar second,
        Scalar relative_tolerance = 1e-12)
    {
        const Scalar scale = std::max(
            std::abs(first),
            std::abs(second));
        if (scale == 0.0)
        {
            return true;
        }
        return std::abs(first - second) <=
               relative_tolerance * scale;
    }

    template <typename Points>
    bool scaleInvariant(
        const Points &points,
        Evaluator<Points> evaluator)
    {
        const EvaluationResult baseline =
            evaluator(points, VolumeCellQualityOptions{});
        if (!baseline.hasValue())
        {
            return false;
        }

        for (const Scalar factor :
             std::array<Scalar, 2>{Scalar{1e-9}, Scalar{1e9}})
        {
            const EvaluationResult transformed = evaluator(
                scaled(points, factor),
                VolumeCellQualityOptions{});
            if (!transformed.hasValue())
            {
                return false;
            }

            const Scalar volume_factor =
                factor * factor * factor;
            if (transformed.value().validity !=
                    baseline.value().validity ||
                transformed.value().acceptable !=
                    baseline.value().acceptable ||
                !relativelyNear(
                    transformed.value().skewness,
                    baseline.value().skewness) ||
                !relativelyNear(
                    transformed.value().signed_volume,
                    baseline.value().signed_volume * volume_factor,
                    5e-12))
            {
                return false;
            }
        }

        return true;
    }

    bool checkThinOrthogonalCells()
    {
        const Scalar thickness = 1e-100;
        PrismPoints prism = distortedPrism();
        prism[3].z() = thickness;
        prism[4].z() = thickness;
        prism[5].z() = thickness;

        HexaPoints hexa = unitHexa();
        hexa[4].z() = thickness;
        hexa[5].z() = thickness;
        hexa[6].z() = thickness;
        hexa[7].z() = thickness;

        VolumeCellQualityOptions options;
        options.relative_length_tolerance = 1e-120;
        const EvaluationResult prism_result =
            evaluatePrism(prism, options);
        const EvaluationResult hexa_result =
            evaluateHexa(hexa, options);

        return prism_result.hasValue() &&
               prism_result.value().validity ==
                   VolumeCellValidity::Valid &&
               prism_result.value().acceptable &&
               prism_result.value().minimum_normalized_jacobian > 0.7 &&
               hexa_result.hasValue() &&
               hexa_result.value().validity ==
                   VolumeCellValidity::Valid &&
               hexa_result.value().acceptable &&
               hexa_result.value().minimum_normalized_jacobian > 0.9;
    }

    bool checkLocalScaleUnderflow()
    {
        const EvaluationResult prism_result = evaluatePrism(
            scaled(distortedPrism(), Scalar{1e-108}),
            VolumeCellQualityOptions{});
        const EvaluationResult hexa_result = evaluateHexa(
            scaled(unitHexa(), Scalar{1e-108}),
            VolumeCellQualityOptions{});

        return prism_result.hasValue() &&
               prism_result.value().validity ==
                   VolumeCellValidity::Degenerate &&
               !prism_result.value().acceptable &&
               hexa_result.hasValue() &&
               hexa_result.value().validity ==
                   VolumeCellValidity::Degenerate &&
               !hexa_result.value().acceptable;
    }

    template <typename Points>
    bool hasEarliestEqualWorstLocation(
        const Points &points,
        Evaluator<Points> evaluator)
    {
        const EvaluationResult result =
            evaluator(points, VolumeCellQualityOptions{});
        return result.hasValue() &&
               result.value().worst_jacobian_location.kind ==
                   JacobianSampleKind::Vertex &&
               result.value().worst_jacobian_location.index == 0;
    }

    bool mixedPrismWithDegenerateSampleIsLocallyInverted()
    {
        PrismPoints points = distortedPrism();
        points[3].z() = 0.0;
        points[4].z() = -1.0;
        points[5].z() = 1.0;
        const EvaluationResult result =
            evaluatePrism(points, VolumeCellQualityOptions{});
        return result.hasValue() &&
               result.value().validity ==
                   VolumeCellValidity::LocallyInverted &&
               result.value().minimum_normalized_jacobian < 0.0 &&
               result.value().maximum_normalized_jacobian > 0.0;
    }

    bool mixedHexaWithDegenerateSampleIsLocallyInverted()
    {
        HexaPoints points = unitHexa();
        points[4].z() = -3.0;
        const EvaluationResult result =
            evaluateHexa(points, VolumeCellQualityOptions{});
        return result.hasValue() &&
               result.value().validity ==
                   VolumeCellValidity::LocallyInverted &&
               result.value().minimum_normalized_jacobian < 0.0 &&
               result.value().maximum_normalized_jacobian > 0.0;
    }
}

int main()
{
    if (!checkConfigurationErrors(
            idealPrism(),
            &evaluatePrism,
            VolumeCellKind::Prism))
    {
        return 1;
    }
    if (!checkConfigurationErrors(
            unitHexa(),
            &evaluateHexa,
            VolumeCellKind::Hexa))
    {
        return 2;
    }
    if (!checkNonFiniteVertices(
            idealPrism(),
            &evaluatePrism,
            VolumeCellKind::Prism,
            2,
            5))
    {
        return 3;
    }
    if (!checkNonFiniteVertices(
            unitHexa(),
            &evaluateHexa,
            VolumeCellKind::Hexa,
            3,
            7))
    {
        return 4;
    }
    if (!checkIntermediateErrors(
            idealPrism(),
            &evaluatePrism,
            VolumeCellKind::Prism))
    {
        return 5;
    }
    if (!checkIntermediateErrors(
            unitHexa(),
            &evaluateHexa,
            VolumeCellKind::Hexa))
    {
        return 6;
    }
    if (!scaleInvariant(idealPrism(), &evaluatePrism) ||
        !scaleInvariant(distortedPrism(), &evaluatePrism))
    {
        return 7;
    }
    if (!scaleInvariant(unitHexa(), &evaluateHexa) ||
        !scaleInvariant(distortedHexa(), &evaluateHexa))
    {
        return 8;
    }
    if (!checkThinOrthogonalCells())
    {
        return 9;
    }
    if (!checkLocalScaleUnderflow())
    {
        return 10;
    }
    if (!hasEarliestEqualWorstLocation(
            distortedPrism(),
            &evaluatePrism))
    {
        return 11;
    }
    if (!hasEarliestEqualWorstLocation(
            unitHexa(),
            &evaluateHexa))
    {
        return 12;
    }
    if (!mixedPrismWithDegenerateSampleIsLocallyInverted())
    {
        return 13;
    }
    if (!mixedHexaWithDegenerateSampleIsLocallyInverted())
    {
        return 14;
    }

    return 0;
}
