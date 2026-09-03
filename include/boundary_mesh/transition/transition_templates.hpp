#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_volume.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>
#include <boundary_mesh/transition/quad_diagonal.hpp>

namespace boundary_mesh
{
    struct SourceTransitionResult
    {
        SurfaceFaceId source_face_id{};
        std::vector<Point3> created_vertices;
        std::vector<VolumeCell> volume_cells;
        std::vector<CellMetadata> metadata;
        std::vector<Triangle> top_faces;
        std::vector<VolumeCell> side_cells;
    };

    struct TriangleTransitionInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t trial_layers{};
        std::optional<std::size_t> high_edge_local_index;
        std::optional<std::size_t> continuing_edge_local_index;
        std::vector<std::array<VertexId, 3>> layer_vertex_ids;
    };

    struct QuadTransitionInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t trial_layers{};
        std::optional<std::size_t> high_edge_local_index;
        std::optional<std::size_t> second_high_edge_local_index;
        std::optional<std::size_t> third_continuing_vertex_local_index;
        std::optional<std::size_t> continuing_edge_local_index;
        std::vector<std::array<VertexId, 4>> layer_vertex_ids;
        const std::vector<Point3> *mesh_vertices{};
        VertexId center_vertex_id{};
        Scalar length_tolerance{1e-12};
    };

    struct InvalidTransitionTemplateInput
    {
        SurfaceFaceId source_face_id{};
    };

    using TransitionTemplateError = std::variant<
        InvalidTransitionTemplateInput,
        FaceEvaluationError>;

    using TransitionTemplateResult = Result<
        SourceTransitionResult,
        TransitionTemplateError>;

    TransitionTemplateResult buildTriangleTransition(
        const TriangleTransitionInput &input);

    TransitionTemplateResult buildQuadTransition(
        const QuadTransitionInput &input);
}
