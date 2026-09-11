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
        !near(evaluation.minimum_subtet_signed_volume, Scalar{1} / Scalar{6}) ||
        !near(evaluation.maximum_subtet_signed_volume, Scalar{1} / Scalar{6}) ||
        evaluation.worst_subtet_index != 0 ||
        !near(evaluation.skewness, 0.0))
    {
        return 2;
    }

    const Scalar minimum_positive_height =
        std::numeric_limits<Scalar>::denorm_min();
    HexaPoints minimum_positive_hexa = unit_hexa;
    minimum_positive_hexa[4].z() = minimum_positive_height;
    minimum_positive_hexa[5].z() = minimum_positive_height;
    minimum_positive_hexa[6].z() = minimum_positive_height;
    minimum_positive_hexa[7].z() = minimum_positive_height;

    const auto minimum_positive_result =
        evaluateHexa(minimum_positive_hexa);
    if (!minimum_positive_result.hasValue())
    {
        return 3;
    }

    const VolumeCellEvaluation &minimum_positive_evaluation =
        minimum_positive_result.value();
    if (minimum_positive_evaluation.validity !=
            VolumeCellValidity::Degenerate ||
        minimum_positive_evaluation.acceptable ||
        minimum_positive_evaluation.signed_volume != Scalar{0} ||
        minimum_positive_evaluation.minimum_subtet_signed_volume !=
            Scalar{0} ||
        minimum_positive_evaluation.maximum_subtet_signed_volume !=
            Scalar{0} ||
        minimum_positive_evaluation.worst_subtet_index != 0 ||
        minimum_positive_evaluation.skewness != Scalar{1})
    {
        return 4;
    }

    const HexaPoints reflex_hexa{
        Point3{0.0, 0.0, 0.0},
        Point3{2.0, 0.0, 0.0},
        Point3{1.0, 0.5, 0.0},
        Point3{2.0, 1.0, 0.0},
        Point3{0.0, 0.0, 1.0},
        Point3{2.0, 0.0, 1.0},
        Point3{1.0, 0.5, 1.0},
        Point3{2.0, 1.0, 1.0}};
    VolumeCellQualityOptions maximum_threshold;
    maximum_threshold.maximum_skewness = Scalar{1};

    const auto reflex_result = evaluateHexa(
        reflex_hexa,
        maximum_threshold);
    if (!reflex_result.hasValue() ||
        !(reflex_result.value().skewness > Scalar{1}) ||
        reflex_result.value().acceptable)
    {
        return 5;
    }

    return 0;
}
