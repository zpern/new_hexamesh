#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct SlidingVertexInput
    {
        std::size_t vertex_index{};
        VertexId source_vertex_id{};
        std::vector<std::uint32_t> region_ids;
    };
}
