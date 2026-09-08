#include <cmath>

#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

int main()
{
    using namespace boundary_mesh;

    const TetraPoints tetra{{
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
    const auto tetra_result = evaluateTetra(tetra);
    if (!tetra_result.hasValue() ||
        tetra_result.value().validity != VolumeCellValidity::Valid ||
        std::abs(tetra_result.value().signed_volume - 1.0 / 6.0) > 1e-12)
        return 1;

    TetraPoints reversed_tetra = tetra;
    std::swap(reversed_tetra[1], reversed_tetra[2]);
    const auto reversed_tetra_result = evaluateTetra(reversed_tetra);
    if (!reversed_tetra_result.hasValue() ||
        reversed_tetra_result.value().validity != VolumeCellValidity::Reversed)
        return 2;

    TetraPoints degenerate_tetra = tetra;
    degenerate_tetra[3].z() = 0.0;
    const auto degenerate_tetra_result = evaluateTetra(degenerate_tetra);
    if (!degenerate_tetra_result.hasValue() ||
        degenerate_tetra_result.value().validity != VolumeCellValidity::Degenerate)
        return 3;

    const PyramidPoints pyramid{{
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
        {0.5, 0.5, 1.0}}};
    const auto pyramid_result = evaluatePyramid(pyramid);
    if (!pyramid_result.hasValue() ||
        pyramid_result.value().validity != VolumeCellValidity::Valid ||
        std::abs(pyramid_result.value().signed_volume - 1.0 / 3.0) > 1e-12)
        return 4;

    PyramidPoints reversed_pyramid = pyramid;
    std::swap(reversed_pyramid[1], reversed_pyramid[3]);
    const auto reversed_pyramid_result = evaluatePyramid(reversed_pyramid);
    if (!reversed_pyramid_result.hasValue() ||
        reversed_pyramid_result.value().validity != VolumeCellValidity::Reversed)
        return 5;

    PyramidPoints inverted_pyramid = pyramid;
    inverted_pyramid[3] = Point3{2.0, 0.0, 0.0};
    const auto inverted_pyramid_result = evaluatePyramid(inverted_pyramid);
    if (!inverted_pyramid_result.hasValue() ||
        inverted_pyramid_result.value().validity !=
            VolumeCellValidity::LocallyInverted)
        return 6;

    PyramidPoints degenerate_pyramid = pyramid;
    degenerate_pyramid[4].z() = 0.0;
    const auto degenerate_pyramid_result = evaluatePyramid(degenerate_pyramid);
    if (!degenerate_pyramid_result.hasValue() ||
        degenerate_pyramid_result.value().validity !=
            VolumeCellValidity::Degenerate)
        return 7;
}
