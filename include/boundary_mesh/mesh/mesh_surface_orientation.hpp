#pragma once

#include <array>
#include <cstddef>
#include <variant>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_error.hpp>

namespace boundary_mesh
{
    struct NonOrientableSurface
    {
        std::array<VertexId, 2> edge_vertices{};
        SurfaceFaceId first_face_id{};
        SurfaceFaceId second_face_id{};
    };

    using SurfaceOrientationError = std::variant<
        DegenerateFace,
        NonManifoldEdge,
        NonOrientableSurface>;

    Result<std::size_t, SurfaceOrientationError>
    unifySurfaceOrientation(SurfaceMesh &mesh);

    /// 统一反转 SurfaceMesh 中全部 Triangle 和 Quad 的顶点绕序。
    void reverseSurfaceOrientation(
        SurfaceMesh &mesh) noexcept;
}
