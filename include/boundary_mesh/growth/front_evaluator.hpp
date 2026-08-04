#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/front_evaluation_error.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace boundary_mesh
{
    class FrontEvaluator
    {
    public:
        /// 对整个当前前沿进行一次无缓存、全有或全无的动态评价。
        Result<FrontEvaluation, FrontEvaluationError>
        evaluate(
            const GrowthFront &front,
            const SurfaceEvaluationOptions &options = {}) const;
    };
}
