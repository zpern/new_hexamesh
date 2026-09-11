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

    // Regression for 50BMG cell 226682: the fixed 0-2 subtets are both
    // positive, but the signed Jacobian integral of the warped Pyramid is
    // negative.  Both conditions are required for admission.
    const PyramidPoints warped_pyramid{{
        {-2.927689685516501, -9.48732403347229, 3.8684608195958603},
        {-2.964854039309542, -9.757107468289282, 3.5991724882226444},
        {-2.948590659174931, -9.14568692353377, 4.965043200339041},
        {-3.0352775022018452, -8.728943694054616, 5.264768867665156},
        {-2.9748311763354915, -9.12684847972812, 4.929127118606514}}};
    const auto warped_result = evaluatePyramid(warped_pyramid);
    if (!warped_result.hasValue() ||
        warped_result.value().validity == VolumeCellValidity::Valid ||
        warped_result.value().acceptable ||
        !(warped_result.value().minimum_subtet_signed_volume > 0.0) ||
        !(warped_result.value().signed_volume < 0.0))
        return 8;
}
