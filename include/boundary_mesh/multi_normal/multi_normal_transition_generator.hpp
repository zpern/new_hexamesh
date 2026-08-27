#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/multi_normal/multi_normal_error.hpp>
#include <boundary_mesh/multi_normal/multi_normal_types.hpp>

namespace boundary_mesh
{
    Result<MultiNormalTransitionResult, MultiNormalError>
    generateMultiNormalTransition(
        const GrowthFront &front,
        const MultiNormalOptions &options);
}
