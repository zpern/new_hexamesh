#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_direction_error.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/sliding/sliding_constraints.hpp>

namespace boundary_mesh
{
    GrowthDirectionError toGrowthDirectionError(
        const SlidingError &error);

    Result<SlidingConstraints, GrowthDirectionError>
    buildGrowthSlidingConstraints(
        const SlidingSurfaceSet &surfaces,
        const GrowthFront &front,
        const FrontEvaluation &evaluation);
}
