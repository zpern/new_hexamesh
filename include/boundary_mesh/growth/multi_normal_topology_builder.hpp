#pragma once

#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/multi_normal_error.hpp>
#include <boundary_mesh/growth/multi_normal_split_planner.hpp>
#include <boundary_mesh/growth/multi_normal_types.hpp>

namespace boundary_mesh
{
    struct MultiNormalTopology
    {
        GrowthFront front;
        std::vector<SplitVertexMapping> vertex_mapping;
        std::vector<TransitionFaceOrigin> transition_face_origins;
    };

    Result<MultiNormalTopology, MultiNormalError>
    buildMultiNormalTopology(
        const GrowthFront &front,
        const std::vector<VertexSplitPlan> &plans);
}
