#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <boundary_mesh/transition/transition_template_types.hpp>

namespace boundary_mesh
{
    struct TriangleSideTransitionInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t low_layer{};
        std::array<VertexId, 3> low{};
        std::array<VertexId, 3> high{};
        std::size_t high_edge_local_index{};
    };

    TransitionTemplateResult buildTriangleSideTransition(
        const TriangleSideTransitionInput &input);
}
