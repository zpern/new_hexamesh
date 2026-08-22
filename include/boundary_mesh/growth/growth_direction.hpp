#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_direction_error.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace boundary_mesh
{
    struct GrowthDirectionSelection
    {
        Vector3 value{Vector3::Zero()}; // 选中的原始单位生长方向
        Scalar visibility_cosine{}; // 对全部活动关联面的最小点积
        bool complex_corner{}; // 可见性是否低于 cos(30°)
    };

    struct GrowthDirections
    {
        std::uint32_t layer{}; // 方向对应的当前层号
        std::vector<GrowthDirectionSelection> vertices; // 与活动点一一对应的候选选择结果
    };

    Result<GrowthDirections, GrowthDirectionError>
    computeGrowthDirections(
        const GrowthFront &front,
        const FrontEvaluation &evaluation,
        const FrontAdjacency &adjacency);
}
