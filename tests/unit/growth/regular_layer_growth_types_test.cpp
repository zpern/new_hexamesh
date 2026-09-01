#include <type_traits>
#include <variant>

#include <boundary_mesh/growth/regular_layer_growth.hpp>
#include <boundary_mesh/growth/regular_layer_growth_error.hpp>

int main()
{
    using namespace boundary_mesh;

    static_assert(std::is_same_v<
                  decltype(RegularLayerGrowthResult{}.mesh),
                  VolumeMesh>);
    static_assert(std::is_same_v<
                  decltype(LayerStepResult{}.next_front),
                  GrowthFront>);

    const FaceGrowthRecord face;
    if (face.status != FaceGrowthStatus::Active ||
        face.stop_reason != FaceStopReason::None ||
        face.accepted_layer_count != 0 ||
        face.stop_layer != 0)
    {
        return 1;
    }

    const VertexGrowthRecord vertex;
    if (vertex.accepted_layer_count != 0 ||
        vertex.profile.growth_ratio != Scalar{1})
    {
        return 2;
    }

    RegularLayerGrowthError error = VolumeVertexIdOverflow{42};
    const auto *overflow = std::get_if<VolumeVertexIdOverflow>(&error);
    if (overflow == nullptr || overflow->attempted_index != 42)
    {
        return 3;
    }

    const LayerStepResult step;
    if (step.layer != 0 || step.next_front.layer != 0 ||
        !step.previous_front_vertex_indices.empty() ||
        !step.previous_front_face_indices.empty() ||
        !step.stopped_faces.empty() ||
        !step.accepted_stopped_faces.empty() ||
        !step.completed_faces.empty() ||
        step.smoothing_diagnostics.activated_vertices != 0 ||
        step.smoothing_diagnostics.updated_vertices != 0)
    {
        return 4;
    }

    const RegularLayerGrowthOptions options;
    if (options.isotropic_height != Scalar{1})
    {
        return 5;
    }
    if (!options.field_smoothing.skewness.enabled ||
        options.field_smoothing.skewness.activation_skewness != Scalar{0.8} ||
        options.field_smoothing.skewness.first_angle_degrees != Scalar{5} ||
        options.field_smoothing.skewness.second_angle_degrees != Scalar{2} ||
        options.field_smoothing.skewness.azimuth_samples != 6 ||
        options.field_smoothing.skewness.maximum_levels != 2)
    {
        return 6;
    }

    const RegularLayerGrowthResult empty_result;
    if (empty_result.smoothing_diagnostics.activated_vertices != 0 ||
        empty_result.smoothing_diagnostics.updated_vertices != 0 ||
        empty_result.smoothing_diagnostics.maximum_skewness_before !=
            Scalar{0} ||
        empty_result.smoothing_diagnostics.maximum_skewness_after !=
            Scalar{0})
    {
        return 7;
    }

    static_assert(
        static_cast<std::size_t>(
            FaceStopReason::IsotropicHeightReached) == 9);

    return 0;
}
