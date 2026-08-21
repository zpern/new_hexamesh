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

    return 0;
}
