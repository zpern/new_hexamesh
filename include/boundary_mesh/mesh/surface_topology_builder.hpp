#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/mesh/surface_mesh.hpp>
#include <boundary_mesh/mesh/surface_topology.hpp>
#include <boundary_mesh/mesh/surface_topology_error.hpp>

namespace boundary_mesh
{
    /// 验证 SurfaceMesh 并构造只读表面拓扑快照。
    ///
    /// 构建器本身不保存状态，也不会修改输入网格。
    class SurfaceTopologyBuilder
    {
    public:
        /// 构建表面拓扑。
        ///
        /// 成功时返回完整 SurfaceTopology；失败时返回按照
        /// 确定性扫描顺序遇到的第一个 SurfaceTopologyError。
        Result<SurfaceTopology, SurfaceTopologyError>
        build(const SurfaceMesh &mesh) const;
    };
}