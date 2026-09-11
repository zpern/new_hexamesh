#include <algorithm>
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
        const Scalar scale = std::max(
            Scalar{1},
            std::max(std::abs(first), std::abs(second)));
        return std::abs(first - second) <= tolerance * scale;
    }

    PrismPoints prismWithTopHeights(
        Scalar h3,
        Scalar h4,
        Scalar h5)
    {
        return PrismPoints{
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, h3},
            Point3{1.0, 0.0, h4},
            Point3{0.0, 1.0, h5}};
    }

    HexaPoints hexaWithTopHeights(
        Scalar h4,
        Scalar h5,
        Scalar h6,
        Scalar h7)
    {
        return HexaPoints{
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{1.0, 1.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, h4},
            Point3{1.0, 0.0, h5},
            Point3{1.0, 1.0, h6},
            Point3{0.0, 1.0, h7}};
    }

    template <typename Points>
    Points translated(
        Points points,
        const Vector3 &offset)
    {
        for (Point3 &point : points)
        {
            point += offset;
        }
        return points;
    }

    template <typename Points>
    Points scaled(
        Points points,
        Scalar factor)
    {
        for (Point3 &point : points)
        {
            point *= factor;
        }
        return points;
    }
}

int main()
{
    using namespace boundary_mesh;

    // Prism 的三个固定子体积六倍值依次等于 h3、h4、h5。
    const auto reversed_prism =
        evaluatePrism(prismWithTopHeights(-1.0, -1.0, -1.0));
    if (!reversed_prism.hasValue() ||
        reversed_prism.value().validity != VolumeCellValidity::Reversed ||
        reversed_prism.value().acceptable ||
        !near(reversed_prism.value().signed_volume, -0.5) ||
        !near(reversed_prism.value().minimum_subtet_signed_volume, -1.0 / 6.0) ||
        !near(reversed_prism.value().maximum_subtet_signed_volume, -1.0 / 6.0) ||
        reversed_prism.value().worst_subtet_index != 0)
    {
        return 1;
    }

    const auto inverted_prism =
        evaluatePrism(prismWithTopHeights(1.0, -1.0, 1.0));
    if (!inverted_prism.hasValue() ||
        inverted_prism.value().validity != VolumeCellValidity::LocallyInverted ||
        inverted_prism.value().acceptable ||
        !near(inverted_prism.value().signed_volume, 1.0 / 6.0) ||
        !near(inverted_prism.value().minimum_subtet_signed_volume, -1.0 / 6.0) ||
        !near(inverted_prism.value().maximum_subtet_signed_volume, 1.0 / 6.0) ||
        inverted_prism.value().worst_subtet_index != 1)
    {
        return 2;
    }

    const auto degenerate_prism =
        evaluatePrism(prismWithTopHeights(1.0, 0.0, 1.0));
    if (!degenerate_prism.hasValue() ||
        degenerate_prism.value().validity != VolumeCellValidity::Degenerate ||
        degenerate_prism.value().acceptable ||
        !near(degenerate_prism.value().signed_volume, 1.0 / 3.0) ||
        degenerate_prism.value().minimum_subtet_signed_volume != 0.0 ||
        !near(degenerate_prism.value().maximum_subtet_signed_volume, 1.0 / 6.0) ||
        degenerate_prism.value().worst_subtet_index != 1)
    {
        return 3;
    }

    const auto mixed_zero_prism =
        evaluatePrism(prismWithTopHeights(1.0, -1.0, 0.0));
    if (!mixed_zero_prism.hasValue() ||
        mixed_zero_prism.value().validity != VolumeCellValidity::LocallyInverted ||
        mixed_zero_prism.value().acceptable ||
        !near(mixed_zero_prism.value().signed_volume, 0.0) ||
        mixed_zero_prism.value().worst_subtet_index != 1)
    {
        return 4;
    }

    // Hexa 的六个固定子体积六倍值依次为 h6,h6,h7,h4,h4,h5。
    const auto reversed_hexa =
        evaluateHexa(hexaWithTopHeights(-1.0, -1.0, -1.0, -1.0));
    if (!reversed_hexa.hasValue() ||
        reversed_hexa.value().validity != VolumeCellValidity::Reversed ||
        reversed_hexa.value().acceptable ||
        !near(reversed_hexa.value().signed_volume, -1.0) ||
        !near(reversed_hexa.value().minimum_subtet_signed_volume, -1.0 / 6.0) ||
        !near(reversed_hexa.value().maximum_subtet_signed_volume, -1.0 / 6.0) ||
        reversed_hexa.value().worst_subtet_index != 0)
    {
        return 5;
    }

    const auto inverted_hexa =
        evaluateHexa(hexaWithTopHeights(-1.0, 1.0, 1.0, 1.0));
    if (!inverted_hexa.hasValue() ||
        inverted_hexa.value().validity != VolumeCellValidity::LocallyInverted ||
        inverted_hexa.value().acceptable ||
        !near(inverted_hexa.value().signed_volume, 0.5) ||
        !near(inverted_hexa.value().minimum_subtet_signed_volume, -1.0 / 6.0) ||
        !near(inverted_hexa.value().maximum_subtet_signed_volume, 1.0 / 6.0) ||
        inverted_hexa.value().worst_subtet_index != 3)
    {
        return 6;
    }

    const auto degenerate_hexa =
        evaluateHexa(hexaWithTopHeights(0.0, 1.0, 1.0, 1.0));
    if (!degenerate_hexa.hasValue() ||
        degenerate_hexa.value().validity != VolumeCellValidity::Degenerate ||
        degenerate_hexa.value().acceptable ||
        !near(degenerate_hexa.value().signed_volume, 0.75) ||
        degenerate_hexa.value().minimum_subtet_signed_volume != 0.0 ||
        !near(degenerate_hexa.value().maximum_subtet_signed_volume, 1.0 / 6.0) ||
        degenerate_hexa.value().worst_subtet_index != 3)
    {
        return 7;
    }

    const auto mixed_zero_hexa =
        evaluateHexa(hexaWithTopHeights(-1.0, 0.0, 1.0, 1.0));
    if (!mixed_zero_hexa.hasValue() ||
        mixed_zero_hexa.value().validity != VolumeCellValidity::LocallyInverted ||
        mixed_zero_hexa.value().acceptable ||
        !near(mixed_zero_hexa.value().signed_volume, 0.25) ||
        mixed_zero_hexa.value().worst_subtet_index != 3)
    {
        return 8;
    }

    const PrismPoints unit_prism = prismWithTopHeights(1.0, 1.0, 1.0);
    const auto base_prism = evaluatePrism(unit_prism);
    const auto moved_prism = evaluatePrism(
        translated(unit_prism, Vector3{8.0, -4.0, 2.0}));
    const auto scaled_prism = evaluatePrism(scaled(unit_prism, 2.0));
    if (!base_prism.hasValue() || !moved_prism.hasValue() ||
        !scaled_prism.hasValue() ||
        moved_prism.value().validity != base_prism.value().validity ||
        !near(moved_prism.value().signed_volume, base_prism.value().signed_volume) ||
        !near(moved_prism.value().skewness, base_prism.value().skewness) ||
        scaled_prism.value().validity != base_prism.value().validity ||
        !near(scaled_prism.value().signed_volume, 8.0 * base_prism.value().signed_volume) ||
        !near(scaled_prism.value().skewness, base_prism.value().skewness))
    {
        return 9;
    }

    const HexaPoints unit_hexa = hexaWithTopHeights(1.0, 1.0, 1.0, 1.0);
    const auto base_hexa = evaluateHexa(unit_hexa);
    const auto moved_hexa = evaluateHexa(
        translated(unit_hexa, Vector3{8.0, -4.0, 2.0}));
    const auto scaled_hexa = evaluateHexa(scaled(unit_hexa, 2.0));
    if (!base_hexa.hasValue() || !moved_hexa.hasValue() ||
        !scaled_hexa.hasValue() ||
        moved_hexa.value().validity != base_hexa.value().validity ||
        !near(moved_hexa.value().signed_volume, base_hexa.value().signed_volume) ||
        !near(moved_hexa.value().skewness, base_hexa.value().skewness) ||
        scaled_hexa.value().validity != base_hexa.value().validity ||
        !near(scaled_hexa.value().signed_volume, 8.0 * base_hexa.value().signed_volume) ||
        !near(scaled_hexa.value().skewness, base_hexa.value().skewness))
    {
        return 10;
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
    VolumeCellQualityOptions strict_options;
    strict_options.maximum_skewness = 0.49;
    const auto skewed_result = evaluateHexa(skewed_hexa, strict_options);
    if (!skewed_result.hasValue() ||
        skewed_result.value().validity != VolumeCellValidity::Valid ||
        skewed_result.value().acceptable ||
        !near(skewed_result.value().signed_volume, 1.0) ||
        !near(skewed_result.value().skewness, 0.5))
    {
        return 11;
    }

    return 0;
}
