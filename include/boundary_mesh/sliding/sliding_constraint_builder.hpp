#pragma once

#include <boundary_mesh/core/result.hpp>
#include <cstdint>
#include <vector>

#include <boundary_mesh/sliding/sliding_constraints.hpp>
#include <boundary_mesh/sliding/sliding_error.hpp>
#include <boundary_mesh/sliding/sliding_vertex_input.hpp>

namespace boundary_mesh
{
    /// 从前沿顶点的 region 归属建立确定性的对称平面约束。
    class SlidingConstraintBuilder
    {
    public:
        Result<SlidingConstraints, SlidingError>
        build(
            const SlidingSurfaceSet &surfaces,
            const std::vector<SlidingVertexInput> &vertices,
            Scalar characteristic_length,
            Scalar effective_length_tolerance,
            std::uint32_t layer) const;
    };
}
