#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    struct DirectionInputMismatch
    {
        std::uint32_t front_layer{};
        std::uint32_t evaluation_layer{};
    };
    struct DirectionCornerFailure
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::size_t front_face_index{};
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        FaceEvaluationError cause{FaceEvaluationError::DegenerateEdge};
    };
    struct UndefinedGrowthDirection
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::uint32_t layer{};
    };

    using GrowthDirectionError = std::variant<
        DirectionInputMismatch,
        DirectionCornerFailure,
        UndefinedGrowthDirection>;
}
