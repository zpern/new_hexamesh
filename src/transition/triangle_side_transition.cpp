#include <utility>

#include <boundary_mesh/transition/triangle_side_transition.hpp>

namespace boundary_mesh
{
    TransitionTemplateResult buildTriangleSideTransition(
        const TriangleSideTransitionInput &input)
    {
        if (input.high_edge_local_index >= 3)
            return TransitionTemplateResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{input.source_face_id}});

        const std::size_t first = input.high_edge_local_index;
        const std::size_t second = (first + 1) % 3;
        const std::size_t apex = (first + 2) % 3;
        TransitionTemplateOutput result;
        result.source_face_id = input.source_face_id;
        result.volume_cells.push_back(Pyramid{{
            input.low[first], input.high[first],
            input.high[second], input.low[second], input.low[apex]}});
        result.metadata.push_back({
            CellRole::LayerTransition,
            input.source_face_id,
            input.low_layer + 1});
        result.top_faces = {
            Triangle{{input.high[first], input.high[second], input.low[apex]}},
            Triangle{{input.low[first], input.high[first], input.low[apex]}},
            Triangle{{input.high[second], input.low[second], input.low[apex]}}};
        return TransitionTemplateResult::success(std::move(result));
    }
}
