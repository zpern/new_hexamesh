#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/growth_front_error.hpp>
#include <boundary_mesh/growth/growth_patch.hpp>

namespace boundary_mesh
{
    class GrowthFrontBuilder
    {
    public:
        /// 从 Wall Patch 构建具有紧凑局部编号的第 0 层前沿。
        Result<GrowthFront, GrowthFrontError>
        buildInitial(
            const SurfaceMesh &mesh,
            const GrowthPatch &patch) const;
    };
}
