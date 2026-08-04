#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    struct InvalidSurfaceEvaluationOptions
    {
        Scalar relative_length_tolerance{}; // 非有限或非正的相对容差
    };
    struct EmptyGrowthFront
    {
        std::uint32_t layer{}; // 空前沿的层号
    };
    struct FrontMappingMismatch
    {
        std::size_t vertex_count{};
        std::size_t source_vertex_count{};
        std::size_t vertex_boundary_count{};
        std::size_t face_count{};
        std::size_t source_face_count{};
        std::uint32_t layer{};
    };
    struct NonFiniteFrontVertex
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::uint32_t layer{};
    };
    struct InvalidFrontVertexReference
    {
        std::size_t front_face_index{};
        SurfaceFaceId source_face_id{};
        VertexId front_vertex_id{};
        std::uint32_t layer{};
    };
    struct DegenerateFrontScale
    {
        std::uint32_t layer{};
    };
    struct DegenerateFrontFace
    {
        std::size_t front_face_index{};
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        FaceEvaluationError cause{FaceEvaluationError::DegenerateAreaVector};
    };

    using FrontEvaluationError = std::variant<
        InvalidSurfaceEvaluationOptions,
        EmptyGrowthFront,
        FrontMappingMismatch,
        NonFiniteFrontVertex,
        InvalidFrontVertexReference,
        DegenerateFrontScale,
        DegenerateFrontFace>;
}
