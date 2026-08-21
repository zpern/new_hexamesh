#include <cmath>
#include <limits>

#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    bool near(
        Scalar first,
        Scalar second,
        Scalar tolerance = 1e-12)
    {
        return std::abs(first - second) <= tolerance;
    }
}

int main()
{
    using namespace boundary_mesh;

    const HexaPoints unit_hexa{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        Point3{0.0, 0.0, 1.0},
        Point3{1.0, 0.0, 1.0},
        Point3{1.0, 1.0, 1.0},
        Point3{0.0, 1.0, 1.0}};

    const auto result = evaluateHexa(unit_hexa);

    if (!result.hasValue())
    {
        return 1;
    }

    const VolumeCellEvaluation &evaluation = result.value();
    if (evaluation.validity != VolumeCellValidity::Valid ||
        !evaluation.acceptable ||
        !near(evaluation.signed_volume, 1.0) ||
        evaluation.minimum_jacobian <= 0.0 ||
        evaluation.maximum_jacobian <= 0.0 ||
        evaluation.minimum_normalized_jacobian <= 0.0 ||
        evaluation.maximum_normalized_jacobian <= 0.0 ||
        !near(evaluation.skewness, 0.0))
    {
        return 2;
    }

    const Scalar thin_height = 1e-15;
    const HexaPoints thin_hexa{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        Point3{0.0, 0.0, thin_height},
        Point3{1.0, 0.0, thin_height},
        Point3{1.0, 1.0, thin_height},
        Point3{0.0, 1.0, thin_height}};
    VolumeCellQualityOptions thin_options;
    thin_options.relative_length_tolerance = 1e-18;

    const auto thin_result = evaluateHexa(thin_hexa, thin_options);

    if (!thin_result.hasValue() ||
        thin_result.value().validity != VolumeCellValidity::Valid ||
        !thin_result.value().acceptable ||
        thin_result.value().minimum_normalized_jacobian <= 0.8 ||
        !near(thin_result.value().signed_volume, thin_height, 1e-27))
    {
        return 3;
    }

    const HexaPoints reversed_hexa{
        unit_hexa[0],
        unit_hexa[3],
        unit_hexa[2],
        unit_hexa[1],
        unit_hexa[4],
        unit_hexa[7],
        unit_hexa[6],
        unit_hexa[5]};
    const auto reversed_result = evaluateHexa(reversed_hexa);

    if (!reversed_result.hasValue() ||
        reversed_result.value().validity != VolumeCellValidity::Reversed ||
        reversed_result.value().acceptable ||
        !near(reversed_result.value().signed_volume, -1.0) ||
        reversed_result.value().minimum_jacobian >= 0.0 ||
        reversed_result.value().maximum_jacobian >= 0.0)
    {
        return 4;
    }

    HexaPoints folded_hexa = unit_hexa;
    folded_hexa[4].z() = -0.5;
    const auto folded_result = evaluateHexa(folded_hexa);

    if (!folded_result.hasValue() ||
        folded_result.value().validity !=
            VolumeCellValidity::LocallyInverted ||
        folded_result.value().acceptable ||
        folded_result.value().minimum_normalized_jacobian >= 0.0 ||
        folded_result.value().maximum_normalized_jacobian <= 0.0 ||
        folded_result.value().worst_jacobian_location.kind !=
            JacobianSampleKind::Vertex ||
        folded_result.value().worst_jacobian_location.index != 0)
    {
        return 5;
    }

    HexaPoints collapsed_hexa = unit_hexa;
    collapsed_hexa[4] = collapsed_hexa[0];
    collapsed_hexa[5] = collapsed_hexa[1];
    collapsed_hexa[6] = collapsed_hexa[2];
    collapsed_hexa[7] = collapsed_hexa[3];
    const auto collapsed_result = evaluateHexa(collapsed_hexa);

    if (!collapsed_result.hasValue() ||
        collapsed_result.value().validity != VolumeCellValidity::Degenerate ||
        collapsed_result.value().acceptable ||
        !near(collapsed_result.value().signed_volume, 0.0) ||
        !near(collapsed_result.value().minimum_normalized_jacobian, 0.0) ||
        !near(collapsed_result.value().maximum_normalized_jacobian, 0.0) ||
        !near(collapsed_result.value().skewness, 1.0))
    {
        return 6;
    }

    const HexaPoints skewed_hexa{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{2.0, 1.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 0.0, 1.0},
        Point3{1.0, 0.0, 1.0},
        Point3{2.0, 1.0, 1.0},
        Point3{1.0, 1.0, 1.0}};
    VolumeCellQualityOptions skewness_options;
    skewness_options.maximum_skewness = 0.4;
    const auto skewed_result = evaluateHexa(
        skewed_hexa,
        skewness_options);

    if (!skewed_result.hasValue() ||
        skewed_result.value().validity != VolumeCellValidity::Valid ||
        skewed_result.value().acceptable ||
        !near(skewed_result.value().signed_volume, 1.0) ||
        !near(skewed_result.value().skewness, 0.5))
    {
        return 7;
    }

    VolumeCellQualityOptions overflowing_tolerance_options;
    overflowing_tolerance_options.relative_length_tolerance =
        std::numeric_limits<Scalar>::max();
    const auto overflowing_tolerance_result = evaluateHexa(
        unit_hexa,
        overflowing_tolerance_options);

    if (overflowing_tolerance_result.hasValue() ||
        overflowing_tolerance_result.error().category !=
            VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult ||
        overflowing_tolerance_result.error().cell_kind !=
            VolumeCellKind::Hexa ||
        overflowing_tolerance_result.error().local_vertex_index.has_value() ||
        overflowing_tolerance_result.error()
            .jacobian_sample_location.has_value())
    {
        return 8;
    }

    return 0;
}
