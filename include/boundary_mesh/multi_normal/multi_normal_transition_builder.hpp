#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/multi_normal/multi_normal_error.hpp>
#include <boundary_mesh/multi_normal/multi_normal_topology_builder.hpp>
#include <boundary_mesh/multi_normal/multi_normal_types.hpp>

namespace boundary_mesh
{
    Result<MultiNormalTransitionResult, MultiNormalError>
    buildMultiNormalTransition(
        const MultiNormalTopology &topology,
        const MultiNormalOptions &options);

}
