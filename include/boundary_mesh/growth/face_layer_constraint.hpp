#pragma once

#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/growth_patch.hpp>
#include <boundary_mesh/growth/growth_profile.hpp>
#include <boundary_mesh/growth/regular_layer_growth.hpp>
#include <boundary_mesh/growth/regular_layer_growth_error.hpp>

namespace boundary_mesh
{
    enum class FaceLayerLimitKind
    {
        Requested,          // 当前上限仍等于源面原始请求
        NeighborConstraint, // 当前上限由共享边邻域传播降低
        DirectStop          // 当前上限由质量或碰撞直接降低
    };

    struct FaceLayerConstraint
    {
        SurfaceFaceId source_face_id{}; // 输入 Wall 面编号
        std::uint32_t requested_layer_count{}; // 源面顶点请求的最小层数
        std::uint32_t allowed_layer_count{}; // 当前最大允许层数
        FaceLayerLimitKind limit_kind{FaceLayerLimitKind::Requested}; // 上限来源
        FaceStopReason direct_reason{FaceStopReason::None}; // 直接停止原因
    };

    class FaceLayerConstraintTable
    {
    public:
        const FaceLayerConstraint *find(
            SurfaceFaceId source_face_id) const noexcept;

        FaceLayerConstraint *find(
            SurfaceFaceId source_face_id) noexcept;

        const std::vector<FaceLayerConstraint> &entries() const noexcept;

    private:
        friend Result<FaceLayerConstraintTable, InvalidFaceConstraintState>
        buildFaceLayerConstraints(
            const GrowthPatch &patch,
            const GrowthFront &initial_front,
            const GrowthProfileTable &profiles);

        std::vector<FaceLayerConstraint> entries_; // 按源面编号升序保存的约束
    };

    Result<FaceLayerConstraintTable, InvalidFaceConstraintState>
    buildFaceLayerConstraints(
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const GrowthProfileTable &profiles);
}
