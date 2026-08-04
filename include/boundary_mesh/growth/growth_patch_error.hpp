#pragma once

#include <cstddef>
#include <variant>

namespace boundary_mesh
{
    struct EmptyGrowthPatch
    {
    };

    struct MeshTopologyMismatch
    {
        std::size_t mesh_vertex_count{}; // 当前网格顶点数
        std::size_t topology_vertex_count{}; // 拓扑快照顶点数
        std::size_t mesh_face_count{}; // 当前网格面数
        std::size_t topology_face_count{}; // 拓扑快照面数
    };

    using GrowthPatchError = std::variant<
        EmptyGrowthPatch,
        MeshTopologyMismatch>; // GrowthPatch 构建的全部可预期错误
}
