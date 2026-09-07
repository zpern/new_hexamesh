#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct SlidingInputMismatch { std::uint32_t region_id{}; };
    struct InvalidSlidingSurface
    {
        std::uint32_t region_id{};
        SurfaceFaceId source_face_id{};
    };
    struct OverConstrainedGrowthVertex
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::uint32_t layer{};
    };
    struct UndefinedConstrainedDirection
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::uint32_t layer{};
    };
    struct SlidingProjectionNotConverged
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::uint32_t layer{};
        std::vector<std::uint32_t> region_ids;
        std::uint32_t iterations{};
        Scalar position_change{};
        Scalar max_surface_residual{};
    };
    using SlidingError = std::variant<
        SlidingInputMismatch,
        InvalidSlidingSurface,
        OverConstrainedGrowthVertex,
        UndefinedConstrainedDirection,
        SlidingProjectionNotConverged>;
}
