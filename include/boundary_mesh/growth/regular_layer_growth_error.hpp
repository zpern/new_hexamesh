#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include <boundary_mesh/growth/front_evaluation_error.hpp>
#include <boundary_mesh/growth/growth_direction_error.hpp>
#include <boundary_mesh/growth/growth_profile.hpp>
#include <boundary_mesh/growth/growth_profile_error.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation_error.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    struct GrowthProfileFailure
    {
        GrowthProfileError cause; // 参数整理阶段的具体失败
    };

    struct FrontEvaluationFailure
    {
        std::uint32_t target_layer{}; // 本次尝试生成的目标层号
        FrontEvaluationError cause;   // 阶段 03 的动态前沿错误
    };

    struct GrowthDirectionFailure
    {
        std::uint32_t target_layer{}; // 本次尝试生成的目标层号
        GrowthDirectionError cause;   // 阶段 03 的方向错误
    };

    struct CellEvaluationFailure
    {
        SurfaceFaceId source_face_id{}; // 候选单元对应的源 Wall 面
        std::uint32_t layer{};           // 候选单元目标层号
        VolumeCellEvaluationError cause; // 阶段 04 的具体失败
    };

    struct InvalidLayerFrontMapping
    {
        std::uint32_t layer{}; // 映射不一致的当前层号
    };

    struct VolumeVertexIdOverflow
    {
        std::size_t attempted_index{}; // 无法转换为 VertexId 的体网格下标
    };

    struct CollisionInitializationFailure
    {
        SpatialError cause; // 原始表面碰撞索引建立失败的具体原因
    };

    struct CollisionStateFailure
    {
        std::uint32_t layer{}; // 外露边界或本层碰撞状态无效的目标层号
        SpatialError cause; // 空间模块返回的具体程序错误
    };

    using RegularLayerGrowthError = std::variant<
        GrowthProfileFailure,
        FrontEvaluationFailure,
        GrowthDirectionFailure,
        CellEvaluationFailure,
        NonFiniteLayerHeight,
        InvalidLayerFrontMapping,
        VolumeVertexIdOverflow,
        CollisionInitializationFailure,
        CollisionStateFailure>; // 规则层生成过程中可诊断的程序级错误
}
