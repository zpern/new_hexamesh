#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_direction.hpp>
#include <boundary_mesh/growth/growth_field_smoothing_error.hpp>
#include <boundary_mesh/growth/growth_field_smoothing_options.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace boundary_mesh
{
    class SlidingConstraints;

    struct SmoothedGrowthFields
    {
        std::uint32_t layer{}; // 平滑结果所属的当前活动层
        std::vector<Vector3> directions; // 每个活动点的平滑单位方向
        std::vector<Scalar> actual_heights; // 每个活动点的本层实际步长
        GrowthFieldSmoothingDiagnostics diagnostics; // skewness 法向优化统计
    };

    class GrowthFieldSmoother
    {
    public:
        /// 对当前紧凑活动前沿同步平滑法向和步长字段。
        Result<SmoothedGrowthFields, GrowthFieldSmoothingError>
        smooth(
            const GrowthFront &front,
            const FrontEvaluation &evaluation,
            const FrontAdjacency &adjacency,
            const GrowthDirections &raw_directions,
            const std::vector<Scalar> &reference_heights,
            const std::vector<Scalar> &provisional_heights,
            const GrowthFieldSmoothingOptions &options = {},
            const SlidingConstraints *sliding_constraints = nullptr) const;
    };
}
