#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

#include <boundary_mesh/transition/transition_template_types.hpp>

namespace boundary_mesh
{
    struct PositiveQuadTopCapCenterInput
    {
        std::array<Point3, 4> bottom{};
        std::array<Point3, 4> top{};
        QuadDiagonal diagonal{};
        Scalar volume_tolerance{1e-12};
    };

    struct QuadTopCapAspectRatioInput
    {
        std::array<Point3, 4> bottom{};
        std::array<Point3, 4> top{};
    };

    Scalar quadTopCapAspectRatio(
        const QuadTopCapAspectRatioInput &input);

    std::optional<Point3> findPositiveQuadTopCapCenter(
        const PositiveQuadTopCapCenterInput &input);

    struct QuadTopCapInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        std::array<VertexId, 4> bottom{};
        std::array<VertexId, 4> top{};
        const std::vector<Point3> *mesh_vertices{};
        VertexId center_vertex_id{};
        QuadDiagonal diagonal{};
        std::optional<Point3> center_point;
    };

    struct ExternalQuadPatchInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        std::array<VertexId, 4> low{};
        std::array<VertexId, 4> high{};
        std::vector<std::size_t> high_edges;
        const std::vector<Point3> *mesh_vertices{};
        VertexId apex_vertex_id{};
        Scalar distance_scale{0.25};
        Scalar length_tolerance{1e-12};
        std::optional<Point3> apex_point;
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
    buildExternalQuadPatch(const ExternalQuadPatchInput &input);

    TransitionTemplateResult
    buildQuadSideTransition(const QuadSideTransitionInput &input);

    TransitionTemplateResult
    buildQuadAdjacentSideTransition(
        const QuadAdjacentSideTransitionInput &input);
}
