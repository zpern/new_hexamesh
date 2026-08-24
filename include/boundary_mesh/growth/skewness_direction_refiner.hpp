#pragma once

#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_field_smoothing_options.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace boundary_mesh
{
    struct SkewnessDirectionRefinement
    {
        std::vector<Vector3> directions;
        GrowthFieldSmoothingDiagnostics diagnostics;
    };

    SkewnessDirectionRefinement refineDirectionsForSkewness(
        const GrowthFront &front,
        const FrontEvaluation &evaluation,
        const FrontAdjacency &adjacency,
        const std::vector<Vector3> &baseline_directions,
        const std::vector<Scalar> &fixed_actual_heights,
        const SkewnessNormalOptimizationOptions &options);
}
