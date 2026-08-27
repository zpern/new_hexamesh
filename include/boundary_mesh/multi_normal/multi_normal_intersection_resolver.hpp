#pragma once

#include <cstddef>
#include <set>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/multi_normal/multi_normal_topology_builder.hpp>

namespace boundary_mesh
{
    struct ResolvedMultiNormalLengths
    {
        std::vector<Scalar> lengths;
        std::size_t shrink_iterations{};
        bool used_zero_retry{};
        bool fallback_to_single_normal{};
    };

    std::set<std::size_t> findMultiNormalIntersectionBadPoints(
        const MultiNormalTopology &topology,
        const MultiNormalTransitionResult &candidate);

    void smoothMultiNormalLengths(
        const MultiNormalTopology &topology,
        std::vector<Scalar> &lengths);

    Result<ResolvedMultiNormalLengths, MultiNormalError>
    resolveMultiNormalLengths(
        const MultiNormalTopology &topology,
        std::vector<Scalar> initial_lengths,
        const MultiNormalOptions &options);
}
