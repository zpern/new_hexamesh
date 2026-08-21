#include <array>
#include <cstddef>
#include <type_traits>

#include <boundary_mesh/mesh/mesh_volume.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation_error.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    static_assert(std::tuple_size_v<PrismPoints> == 6);
    static_assert(std::tuple_size_v<HexaPoints> == 8);
    static_assert(std::is_same_v<
                  decltype(VolumeCellQualityOptions{}.relative_jacobian_tolerance),
                  Scalar>);
    static_assert(std::is_same_v<
                  decltype(VolumeCellQualityOptions{}.relative_length_tolerance),
                  Scalar>);
    static_assert(std::is_same_v<
                  decltype(VolumeCellQualityOptions{}.maximum_skewness),
                  Scalar>);
    static_assert(std::is_same_v<
                  decltype(VolumeCellEvaluation{}.worst_jacobian_location),
                  JacobianSampleLocation>);
}

int main()
{
    using namespace boundary_mesh;

    const VolumeCellQualityOptions default_options{};
    if (default_options.relative_jacobian_tolerance != 1e-12 ||
        default_options.relative_length_tolerance != 1e-12 ||
        default_options.maximum_skewness != 0.95)
    {
        return 1;
    }

    if (PrismVertexOrder != std::array<std::size_t, 6>{0, 1, 2, 3, 4, 5} ||
        HexaVertexOrder != std::array<std::size_t, 8>{0, 1, 2, 3, 4, 5, 6, 7})
    {
        return 2;
    }

    const JacobianSampleLocation default_location{};
    if (default_location.kind != JacobianSampleKind::Center ||
        default_location.index != 0)
    {
        return 3;
    }

    const VolumeCellEvaluation default_evaluation{};
    if (default_evaluation.validity != VolumeCellValidity::Degenerate ||
        default_evaluation.signed_volume != 0.0 ||
        default_evaluation.minimum_jacobian != 0.0 ||
        default_evaluation.maximum_jacobian != 0.0 ||
        default_evaluation.minimum_normalized_jacobian != 0.0 ||
        default_evaluation.maximum_normalized_jacobian != 0.0 ||
        default_evaluation.skewness != 0.0 ||
        default_evaluation.acceptable)
    {
        return 4;
    }

    const VolumeCellEvaluationError default_error{};
    if (default_error.category != VolumeCellEvaluationErrorCategory::InvalidRelativeJacobianTolerance ||
        default_error.cell_kind != VolumeCellKind::Prism ||
        default_error.configuration_value != 0.0 ||
        default_error.local_vertex_index.has_value() ||
        default_error.jacobian_sample_location.has_value())
    {
        return 5;
    }

    return 0;
}
