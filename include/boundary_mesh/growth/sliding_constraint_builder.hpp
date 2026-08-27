#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/sliding_constraints.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    /// 从前沿顶点的 region 归属建立确定性的对称平面约束。
    class SlidingConstraintBuilder
    {
    public:
        Result<SlidingConstraints, GrowthDirectionError>
        build(
            const SurfaceMesh &mesh,
            const GrowthFront &front,
            const FrontEvaluation &evaluation) const;
    };
}
