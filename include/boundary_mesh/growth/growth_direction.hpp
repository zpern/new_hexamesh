#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_direction_error.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace boundary_mesh
{
    struct GrowthDirections
    {
        std::uint32_t layer{}; // 方向对应的当前层号
        std::vector<VertexId> source_vertex_ids; // 与 values 一一对应的源顶点
        std::vector<Vector3> values; // 当前活动面的角度加权单位方向
    };

    Result<GrowthDirections, GrowthDirectionError>
    computeGrowthDirections(
        const GrowthFront &front,
        const FrontEvaluation &evaluation);
}
