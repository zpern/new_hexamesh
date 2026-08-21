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

    const Scalar root_three = std::sqrt(Scalar{3});
    const Scalar height = Scalar{2} / root_three;
    const PrismPoints standard_prism{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{0.5, root_three / 2.0, 0.0},
        Point3{0.0, 0.0, height},
        Point3{1.0, 0.0, height},
        Point3{0.5, root_three / 2.0, height}};

    const auto result = evaluatePrism(standard_prism);

    if (!result.hasValue())
    {
        return 1;
    }

    const VolumeCellEvaluation &evaluation = result.value();
    if (evaluation.validity != VolumeCellValidity::Valid ||
        !evaluation.acceptable ||
        !near(evaluation.signed_volume, 0.5) ||
        evaluation.minimum_jacobian <= 0.0 ||
        evaluation.maximum_jacobian <= 0.0 ||
        evaluation.minimum_normalized_jacobian <= 0.0 ||
        evaluation.maximum_normalized_jacobian <= 0.0 ||
        !near(evaluation.skewness, 0.0))
    {
        return 2;
    }

    const Scalar thin_height = 1e-15;
    const PrismPoints thin_prism{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{0.5, root_three / 2.0, 0.0},
        Point3{0.0, 0.0, thin_height},
        Point3{1.0, 0.0, thin_height},
        Point3{0.5, root_three / 2.0, thin_height}};
    VolumeCellQualityOptions thin_options;
    thin_options.relative_length_tolerance = 1e-18;

    const auto thin_result = evaluatePrism(thin_prism, thin_options);

    if (!thin_result.hasValue() ||
        thin_result.value().validity != VolumeCellValidity::Valid ||
        !thin_result.value().acceptable ||
        thin_result.value().minimum_normalized_jacobian <= 0.8 ||
        !near(
            thin_result.value().signed_volume,
            root_three * thin_height / 4.0,
            1e-27))
    {
        return 3;
    }

    const PrismPoints reversed_prism{
        standard_prism[0],
        standard_prism[2],
        standard_prism[1],
        standard_prism[3],
        standard_prism[5],
        standard_prism[4]};
    const auto reversed_result = evaluatePrism(reversed_prism);

    if (!reversed_result.hasValue() ||
        reversed_result.value().validity != VolumeCellValidity::Reversed ||
        reversed_result.value().acceptable ||
        !near(reversed_result.value().signed_volume, -0.5) ||
        reversed_result.value().minimum_jacobian >= 0.0 ||
        reversed_result.value().maximum_jacobian >= 0.0)
    {
        return 4;
    }

    PrismPoints folded_prism = standard_prism;
    folded_prism[3].z() = -2.0 * height;
    const auto folded_result = evaluatePrism(folded_prism);

    if (!folded_result.hasValue() ||
        folded_result.value().validity !=
            VolumeCellValidity::LocallyInverted ||
        folded_result.value().acceptable ||
        !near(folded_result.value().signed_volume, 0.0) ||
        folded_result.value().minimum_normalized_jacobian >= 0.0 ||
        folded_result.value().maximum_normalized_jacobian <= 0.0)
    {
        return 5;
    }

    PrismPoints collapsed_prism = standard_prism;
    collapsed_prism[3] = collapsed_prism[0];
    collapsed_prism[4] = collapsed_prism[1];
    collapsed_prism[5] = collapsed_prism[2];
    const auto collapsed_result = evaluatePrism(collapsed_prism);

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

    const PrismPoints skewed_prism{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        Point3{0.0, 0.0, 1.0},
        Point3{1.0, 0.0, 1.0},
        Point3{0.0, 1.0, 1.0}};
    VolumeCellQualityOptions skewness_options;
    skewness_options.maximum_skewness = 0.2;
    const auto skewed_result = evaluatePrism(
        skewed_prism,
        skewness_options);

    if (!skewed_result.hasValue() ||
        skewed_result.value().validity != VolumeCellValidity::Valid ||
        skewed_result.value().acceptable ||
        !near(skewed_result.value().signed_volume, 0.5) ||
        !near(skewed_result.value().skewness, 0.25))
    {
        return 7;
    }

    VolumeCellQualityOptions overflowing_tolerance_options;
    overflowing_tolerance_options.relative_length_tolerance =
        std::numeric_limits<Scalar>::max();
    const auto overflowing_tolerance_result = evaluatePrism(
        standard_prism,
        overflowing_tolerance_options);

    if (overflowing_tolerance_result.hasValue() ||
        overflowing_tolerance_result.error().category !=
            VolumeCellEvaluationErrorCategory::NonFiniteIntermediateResult ||
        overflowing_tolerance_result.error().cell_kind !=
            VolumeCellKind::Prism ||
        overflowing_tolerance_result.error().local_vertex_index.has_value() ||
        overflowing_tolerance_result.error()
            .jacobian_sample_location.has_value())
    {
        return 8;
    }

    return 0;
}
