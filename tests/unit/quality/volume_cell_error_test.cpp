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
    bool checkConfigurationErrors(
        const Points &points,
        Evaluator<Points> evaluator,
        VolumeCellKind expected_kind)
    {
        const Scalar nan =
            std::numeric_limits<Scalar>::quiet_NaN();
        const Scalar infinity =
            std::numeric_limits<Scalar>::infinity();
        const std::array<Scalar, 4> invalid_skewness{
            Scalar{-0.01}, Scalar{1.01}, nan, infinity};

        for (const Scalar value : invalid_skewness)
        {
            VolumeCellQualityOptions options;
            options.maximum_skewness = value;
            const EvaluationResult result = evaluator(points, options);
            if (result.hasValue())
            {
                return false;
            }

            const VolumeCellEvaluationError &error = result.error();
            if (error.category !=
                    VolumeCellEvaluationErrorCategory::
                        InvalidMaximumSkewness ||
                error.cell_kind != expected_kind ||
                !sameConfigurationValue(
                    error.configuration_value,
                    value) ||
                error.local_vertex_index.has_value() ||
                error.subtet_index.has_value())
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
               error.configuration_value == Scalar{0} &&
               error.local_vertex_index == expected_index &&
               !error.subtet_index.has_value();
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
        VolumeCellKind expected_kind,
        std::optional<std::size_t> expected_subtet_index)
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
                       NonFiniteIntermediateResult &&
               error.cell_kind == expected_kind &&
               error.configuration_value == Scalar{0} &&
               !error.local_vertex_index.has_value() &&
               error.subtet_index == expected_subtet_index;
    }

    template <typename Points>
    bool checkIntermediateErrors(
        const Points &points,
        Evaluator<Points> evaluator,
        VolumeCellKind expected_kind)
    {
        Points overflowing_difference_points = points;
        overflowing_difference_points[0].x() =
            std::numeric_limits<Scalar>::max();
        overflowing_difference_points[1].x() =
            -std::numeric_limits<Scalar>::max();
        if (!hasIntermediateError(
                overflowing_difference_points,
                evaluator,
                expected_kind,
                std::size_t{0}))
        {
            return false;
        }

        return hasIntermediateError(
            scaled(points, Scalar{1e200}),
            evaluator,
            expected_kind,
            std::size_t{0});
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

    return 0;
}
