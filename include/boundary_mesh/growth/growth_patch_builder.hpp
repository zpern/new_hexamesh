#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_patch.hpp>
#include <boundary_mesh/growth/growth_patch_error.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology.hpp>

namespace boundary_mesh
{
    /// 无状态地从完整表面提取 Wall 面及逐顶点对称区域归属。
    class GrowthPatchBuilder
    {
    public:
        Result<GrowthPatch, GrowthPatchError>
        build(
            const SurfaceMesh &mesh,
            const SurfaceTopology &topology) const;
    };
}
