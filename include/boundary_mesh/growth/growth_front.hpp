#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    struct FrontVertexBoundary
    {
        std::vector<std::uint32_t> sliding_region_ids; // 当前顶点继承的滑移区域
    };

    /// 当前活动前沿上的一个点及其逐层生成状态。
    struct GrowthFrontVertex
    {
        GrowthFrontVertex() = default;

        GrowthFrontVertex(
            Point3 initial_position,
            VertexId source_id,
            FrontVertexBoundary initial_boundary = {})
            : position(std::move(initial_position)),
              root_position(position),
              source_vertex_id(source_id),
              boundary(std::move(initial_boundary))
        {
        }

        Point3 position{Point3::Zero()}; // 当前活动层坐标
        Point3 root_position{Point3::Zero()}; // 第 0 层 Wall 源点坐标
        VertexId source_vertex_id{}; // 输入 Wall 顶点编号
        FrontVertexBoundary boundary; // 当前点继承的边界约束
        Vector3 direction{Vector3::Zero()}; // 最近一次平滑后的单位生长方向
        Scalar actual_height{}; // 最近一层真正采用的推出步长
        Scalar visibility_cosine{1}; // 原始法向对最不利关联面的点积
        bool complex_corner{}; // 是否属于低可见性复杂角点
        std::uint32_t branch_id{}; // 同一源点的多法向分支编号，普通点为 0
        bool multi_normal_branch{}; // 当前点是否由多法向拆点产生
    };

    /// 当前生长层的紧凑活动前沿及其源实体映射。
    struct GrowthFront
    {
        std::uint32_t layer{}; // 当前层号，第 0 层对应输入 Wall
        std::vector<GrowthFrontVertex> vertices; // 当前紧凑活动点及生成状态
        std::vector<SurfaceFace> faces; // 顶点编号索引当前 vertices
        std::vector<SurfaceFaceId> source_face_ids; // 局部面到源面的映射
    };
}
