#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/transition/incremental_transition_types.hpp>
#include <boundary_mesh/transition/transition_coordination.hpp>

namespace boundary_mesh
{
    struct CornerSuppressionInput
    {
        GrowthFront current_front;
        GrowthFront candidate_front;
        LayerFaceSets face_sets;
        std::uint32_t completed_layer{};
        Scalar length_tolerance{1e-12};
    };

    struct CornerSuppressionResult
    {
        LayerFaceSets face_sets;
        std::vector<SurfaceFaceId> retained_high_faces;
        std::vector<SurfaceFaceId> removed_high_faces;
    };

    Result<CornerSuppressionResult, TransitionCoordinationError>
    applyCornerSuppression(const CornerSuppressionInput &input);
}
