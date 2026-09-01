#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/face_layer_constraint.hpp>
#include <boundary_mesh/growth/growth_profile.hpp>
#include <boundary_mesh/growth/regular_layer_growth.hpp>
#include <boundary_mesh/growth/regular_layer_growth_error.hpp>
#include <boundary_mesh/growth/sliding_surface.hpp>

namespace boundary_mesh
{
    class RegularLayerStepper
    {
    public:
        /// 在不修改输入对象的前提下预推出并评价一个目标层。
        Result<LayerStepResult, RegularLayerGrowthError>
        step(
            const GrowthFront &current_front,
            const GrowthProfileTable &profiles,
            const FaceLayerConstraintTable &constraints,
            const RegularLayerGrowthOptions &options = {}) const;

        Result<LayerStepResult, RegularLayerGrowthError>
        step(
            const GrowthFront &current_front,
            const GrowthProfileTable &profiles,
            const FaceLayerConstraintTable &constraints,
            const SlidingSurfaceSet &sliding_surfaces,
            const RegularLayerGrowthOptions &options = {}) const;
    };
}
