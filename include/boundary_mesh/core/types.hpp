#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace boundary_mesh
{
    using Scalar = double; // 网格算法统一使用的浮点标量类型
    using Point3 = Eigen::Vector3d; // 三维空间中的点坐标类型
    using Vector3 = Eigen::Vector3d; // 三维空间中的方向或位移向量类型

    using VertexId = std::uint32_t; // 顶点容器的稳定索引类型
    using EdgeId = std::uint32_t; // 表面边容器的稳定索引类型
    using SurfaceFaceId = std::uint32_t; // 表面面片容器的稳定索引类型
    using VolumeCellId = std::uint32_t; // 体单元容器的稳定索引类型
}
