#pragma once

#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/regular_layer_growth_error.hpp>

namespace boundary_mesh
{
    struct IsotropicStopEvaluation
    {
        std::vector<bool> vertex_candidates;
        std::vector<bool> vertex_stops;
        std::vector<bool> face_stops;
    };

    class IsotropicStopEvaluator
    {
    public:
        Result<IsotropicStopEvaluation, InvalidLayerFrontMapping>
        evaluate(
            const GrowthFront &current_front,
            const GrowthFront &candidate_front,
            const FrontAdjacency &adjacency,
            Scalar isotropic_height) const;
    };
}
