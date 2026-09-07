#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <boundary_mesh/transition/transition_template_types.hpp>

namespace boundary_mesh
{
    struct QuadTopCapInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        std::array<VertexId, 4> bottom{};
        std::array<VertexId, 4> top{};
        const std::vector<Point3> *mesh_vertices{};
        VertexId center_vertex_id{};
        QuadDiagonal diagonal{};
    };

    struct QuadSideTransitionInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t low_layer{};
        std::array<VertexId, 4> low{};
        std::array<VertexId, 4> high{};
        std::size_t high_edge_local_index{};
        QuadDiagonal low_diagonal{};
    };

    struct QuadAdjacentSideTransitionInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t low_layer{};
        std::array<VertexId, 4> low{};
        std::array<VertexId, 4> high{};
        std::size_t first_high_edge_local_index{};
        std::size_t second_high_edge_local_index{};
        QuadDiagonal low_diagonal{};
        const std::vector<Point3> *mesh_vertices{};
        Scalar length_tolerance{1e-12};
    };

    TransitionTemplateResult
    buildQuadTopCap(const QuadTopCapInput &input);

    TransitionTemplateResult
    buildQuadSideTransition(const QuadSideTransitionInput &input);

    TransitionTemplateResult
    buildQuadAdjacentSideTransition(
        const QuadAdjacentSideTransitionInput &input);
}
