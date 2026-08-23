#pragma once

#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    /// 统一反转 SurfaceMesh 中全部 Triangle 和 Quad 的顶点绕序。
    void reverseSurfaceOrientation(
        SurfaceMesh &mesh) noexcept;
}
