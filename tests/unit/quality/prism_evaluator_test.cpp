#include <cmath>

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
        !near(evaluation.minimum_subtet_signed_volume, Scalar{1} / Scalar{6}) ||
        !near(evaluation.maximum_subtet_signed_volume, Scalar{1} / Scalar{6}) ||
        evaluation.worst_subtet_index != 0 ||
        !near(evaluation.skewness, 0.0))
    {
        return 2;
    }

    const PrismPoints reflex_prism{
        Point3{0.0, 0.0, 0.0},
        Point3{2.0, 0.0, 0.0},
        Point3{0.0, 2.0, 0.0},
        Point3{0.0, 0.0, 1.0},
        Point3{-1.0, 0.0, 0.8},
        Point3{0.0, 2.0, 1.0}};
    VolumeCellQualityOptions maximum_threshold;
    maximum_threshold.maximum_skewness = Scalar{1};
    const auto reflex_result = evaluatePrism(
        reflex_prism,
        maximum_threshold);
    if (!reflex_result.hasValue() ||
        !(reflex_result.value().skewness > Scalar{1}) ||
        reflex_result.value().acceptable)
    {
        return 3;
    }

    return 0;
}
