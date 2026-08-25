#pragma once

#include <cstddef>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/multi_normal_topology_builder.hpp>

namespace boundary_mesh
{
    struct ResolvedMultiNormalLengths
    {
        std::vector<Scalar> lengths;
        std::size_t shrink_iterations{};
        bool used_zero_retry{};
    };

    Result<ResolvedMultiNormalLengths, MultiNormalError>
    resolveMultiNormalLengths(
        const MultiNormalTopology &topology,
        std::vector<Scalar> initial_lengths,
        const MultiNormalOptions &options);
}
