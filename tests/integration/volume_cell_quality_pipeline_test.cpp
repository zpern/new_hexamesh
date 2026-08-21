#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    enum class CandidateDecision
    {
        Commit, // 候选质量合格，可以正式写入体网格
        Stop    // 候选无效或质量超限，停止对应面片生长
    };

    CandidateDecision decide(
        const VolumeCellEvaluation &evaluation)
    {
        return evaluation.acceptable
                   ? CandidateDecision::Commit
                   : CandidateDecision::Stop;
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
}

int main()
{
    using namespace boundary_mesh;

    const auto valid_prism =
        evaluatePrism(prismWithTopHeights(1.0, 1.0, 1.0));
    if (!valid_prism.hasValue() ||
        decide(valid_prism.value()) != CandidateDecision::Commit)
    {
        return 1;
    }

    const auto valid_hexa = evaluateHexa(unitHexa());
    if (!valid_hexa.hasValue() ||
        decide(valid_hexa.value()) != CandidateDecision::Commit)
    {
        return 2;
    }

    const auto reversed_prism =
        evaluatePrism(prismWithTopHeights(-1.0, -1.0, -1.0));
    if (!reversed_prism.hasValue() ||
        reversed_prism.value().validity != VolumeCellValidity::Reversed ||
        decide(reversed_prism.value()) != CandidateDecision::Stop)
    {
        return 3;
    }

    const auto inverted_prism =
        evaluatePrism(prismWithTopHeights(1.0, -1.0, 1.0));
    if (!inverted_prism.hasValue() ||
        inverted_prism.value().validity != VolumeCellValidity::LocallyInverted ||
        decide(inverted_prism.value()) != CandidateDecision::Stop)
    {
        return 4;
    }

    const auto degenerate_prism =
        evaluatePrism(prismWithTopHeights(1.0, 0.0, 1.0));
    if (!degenerate_prism.hasValue() ||
        degenerate_prism.value().validity != VolumeCellValidity::Degenerate ||
        decide(degenerate_prism.value()) != CandidateDecision::Stop)
    {
        return 5;
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
    const auto poor_quality_hexa =
        evaluateHexa(skewed_hexa, strict_options);
    if (!poor_quality_hexa.hasValue() ||
        poor_quality_hexa.value().validity != VolumeCellValidity::Valid ||
        decide(poor_quality_hexa.value()) != CandidateDecision::Stop)
    {
        return 6;
    }

    return 0;
}
