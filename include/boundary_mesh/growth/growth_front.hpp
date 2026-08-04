#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    struct FrontVertexBoundary
    {
        std::vector<std::uint32_t> symmetry_region_ids; // 当前顶点继承的对称区域
    };

    /// 当前生长层的紧凑活动前沿及其源实体映射。
    struct GrowthFront
    {
        std::uint32_t layer{}; // 当前层号，第 0 层对应输入 Wall
        std::vector<Point3> vertices; // 使用局部紧凑编号的当前坐标
        std::vector<SurfaceFace> faces; // 顶点编号索引当前 vertices
        std::vector<VertexId> source_vertex_ids; // 局部顶点到源顶点的映射
        std::vector<SurfaceFaceId> source_face_ids; // 局部面到源面的映射
        std::vector<FrontVertexBoundary> vertex_boundaries; // 局部顶点的对称归属
    };
}
