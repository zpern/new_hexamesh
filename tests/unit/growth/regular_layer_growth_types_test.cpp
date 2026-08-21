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
        !step.completed_faces.empty())
    {
        return 4;
    }

    return 0;
}
