#pragma once

#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_patch.hpp>
#include <boundary_mesh/growth/growth_profile.hpp>
#include <boundary_mesh/growth/growth_profile_error.hpp>

namespace boundary_mesh
{
    class GrowthProfileBuilder
    {
    public:
        /// 验证每个 Patch 顶点恰好有一条合法参数并建立确定性查询表。
        Result<GrowthProfileTable, GrowthProfileError>
        build(
            const GrowthPatch &patch,
            const std::vector<SourceVertexGrowthProfile> &profiles) const;
    };
}
