#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/multi_normal/multi_normal_error.hpp>
#include <boundary_mesh/multi_normal/multi_normal_topology_builder.hpp>

namespace boundary_mesh
{
    Result<MultiNormalTopology, MultiNormalError>
    triangulateMultiNormalQuads(
        const MultiNormalTopology &topology);
}
