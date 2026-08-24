#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include <boundary_mesh/transition/transition_templates.hpp>

namespace boundary_mesh
{
    TransitionTemplateResult buildTriangleTransition(
        const TriangleTransitionInput &input)
    {
        if (input.layer_vertex_ids.size() !=
                static_cast<std::size_t>(input.trial_layers) + 1 ||
            (input.high_edge_local_index.has_value() &&
             *input.high_edge_local_index >= 3))
        {
            return TransitionTemplateResult::failure(
                InvalidTransitionTemplateInput{input.source_face_id});
        }

        SourceTransitionResult result;
        result.source_face_id = input.source_face_id;
        const std::uint32_t occupied =
            input.trial_layers > 0 ? input.trial_layers - 1 : 0;

        for (std::uint32_t layer = 1; layer <= occupied; ++layer)
        {
            const auto &bottom = input.layer_vertex_ids[layer - 1];
            const auto &top = input.layer_vertex_ids[layer];
            result.volume_cells.push_back(Prism{{
                bottom[0], bottom[1], bottom[2],
                top[0], top[1], top[2]}});
            result.metadata.push_back(CellMetadata{
                CellRole::RegularLayer,
                input.source_face_id,
                layer});
        }

        if (!input.high_edge_local_index.has_value())
        {
            result.top_faces.push_back(
                Triangle{input.layer_vertex_ids[occupied]});
            return TransitionTemplateResult::success(std::move(result));
        }

        const std::size_t first = *input.high_edge_local_index;
        const std::size_t second = (first + 1) % 3;
        const std::size_t apex = (first + 2) % 3;
        const auto &low = input.layer_vertex_ids[occupied];
        const auto &high = input.layer_vertex_ids[occupied + 1];

        result.volume_cells.push_back(Pyramid{{
            low[first], low[second],
            high[second], high[first], low[apex]}});
        result.metadata.push_back(CellMetadata{
            CellRole::Transition,
            input.source_face_id,
            occupied + 1});

        result.top_faces = {
            Triangle{{high[first], high[second], low[apex]}},
            Triangle{{low[first], high[first], low[apex]}},
            Triangle{{high[second], low[second], low[apex]}}};
        return TransitionTemplateResult::success(std::move(result));
    }
}
