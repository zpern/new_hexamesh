#pragma once

#include <cstddef>
#include <variant>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/multi_normal_types.hpp>
#include <boundary_mesh/mesh/mesh_volume.hpp>

namespace boundary_mesh
{
    struct MultiNormalMergeInputMismatch {};
    struct MultiNormalMergeVertexMismatch { std::size_t vertex_index{}; };
    struct MultiNormalMergeInvalidVertexReference
    {
        std::size_t cell_index{};
        VertexId vertex_id{};
    };
    struct MultiNormalMergeVertexIdOverflow { std::size_t vertex_count{}; };

    using MultiNormalMergeError = std::variant<
        MultiNormalMergeInputMismatch,
        MultiNormalMergeVertexMismatch,
        MultiNormalMergeInvalidVertexReference,
        MultiNormalMergeVertexIdOverflow>;

    Result<VolumeMesh, MultiNormalMergeError>
    mergeMultiNormalAndRegularMeshes(
        const MultiNormalTransitionResult &transition,
        const VolumeMesh &regular);
}
