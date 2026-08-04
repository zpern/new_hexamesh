#pragma once

#include <array>
#include <cstddef>
#include <variant>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    /// 输入表面不包含任何面片。
    struct EmptySurface
    {
    };

    /// 面片数量与逐面边界标签数量不一致。
    struct FaceTagCountMismatch
    {
        std::size_t face_count{};
        std::size_t face_tag_count{};
    };

    /// 输入顶点至少包含一个 NaN 或无穷坐标分量。
    struct NonFiniteVertex
    {
        VertexId vertex_id{};
    };

    /// 某个面片引用了 vertices 范围之外的顶点编号。
    struct InvalidVertexReference
    {
        SurfaceFaceId face_id{};
        VertexId vertex_id{};
    };

    /// 某个面片内部重复使用顶点，形成离散拓扑退化。
    /// 几何面积退化不属于本错误的检查范围。
    struct DegenerateFace
    {
        SurfaceFaceId face_id{};
    };

    /// 两个面片使用同一组顶点，构成重复面。
    struct DuplicateFace
    {
        SurfaceFaceId first_face_id{};
        SurfaceFaceId duplicate_face_id{};
    };

    /// 一条边只关联一个面片，说明完整输入表面没有封闭。
    struct BoundaryEdge
    {
        std::array<VertexId, 2> edge_vertices{};
        SurfaceFaceId face_id{};
    };

    /// 一条边关联三个或更多面片；记录最先遇到的三个面。
    struct NonManifoldEdge
    {
        std::array<VertexId, 2> edge_vertices{};
        std::array<SurfaceFaceId, 3> face_ids{};
    };

    /// 两个相邻面沿共享边使用了相同方向。
    struct InconsistentOrientation
    {
        std::array<VertexId, 2> edge_vertices{};
        SurfaceFaceId first_face_id{};
        SurfaceFaceId second_face_id{};
    };

    /// SurfaceTopologyBuilder 可能返回的全部离散拓扑错误。
    using SurfaceTopologyError = std::variant<
        EmptySurface,
        FaceTagCountMismatch,
        NonFiniteVertex,
        InvalidVertexReference,
        DegenerateFace,
        DuplicateFace,
        BoundaryEdge,
        NonManifoldEdge,
        InconsistentOrientation>;
}
