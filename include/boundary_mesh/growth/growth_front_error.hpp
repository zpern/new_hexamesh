#pragma once

#include <variant>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct InvalidPatchVertex
    {
        VertexId source_vertex_id{}; // 超出当前 SurfaceMesh 的源顶点
    };

    struct InvalidPatchFace
    {
        SurfaceFaceId source_face_id{}; // 超出当前 SurfaceMesh 的源面
    };

    struct PatchFaceUsesUnknownVertex
    {
        SurfaceFaceId source_face_id{}; // 使用未知顶点的源面
        VertexId source_vertex_id{}; // 未包含在 PatchVertex 中的源顶点
    };

    using GrowthFrontError = std::variant<
        InvalidPatchVertex,
        InvalidPatchFace,
        PatchFaceUsesUnknownVertex>; // 初始前沿构建的全部可预期错误
}
