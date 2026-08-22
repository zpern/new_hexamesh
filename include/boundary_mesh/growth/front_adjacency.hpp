#pragma once

#include <cstddef>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/front_adjacency_error.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace boundary_mesh
{
    struct FrontAdjacency
    {
        std::vector<std::vector<std::size_t>> vertex_neighbors; // 当前活动点的一环活动邻点
        std::vector<std::vector<std::size_t>> vertex_incident_faces; // 当前活动点的关联活动面
    };

    /// 只根据当前紧凑活动面建立确定性一环邻接。
    Result<FrontAdjacency, FrontAdjacencyError>
    buildFrontAdjacency(const GrowthFront &front);
}
