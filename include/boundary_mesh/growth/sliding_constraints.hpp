#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/growth_direction_error.hpp>
#include <boundary_mesh/growth/sliding_surface.hpp>

namespace boundary_mesh
{
    struct SlidingPlane
    {
        std::uint32_t region_id{};                 // 输入表面中的对称区域编号
        Point3 point{Point3::Zero()};              // 稳定选取的参考面中心点
        Vector3 unit_normal{Vector3::Zero()};      // 参考面的单位法向
    };

    struct VertexSlidingConstraint
    {
        std::size_t front_vertex_index{};          // 前沿紧凑顶点下标
        VertexId source_vertex_id{};               // 对应的输入表面顶点编号
        std::vector<std::size_t> plane_indices;    // 线性独立平面在 planes 中的下标
    };

    struct SlidingPositionProjection
    {
        Point3 position{Point3::Zero()};
        std::uint32_t iterations{};
        Scalar position_change{};
        Scalar max_surface_residual{};
    };

    class SlidingConstraintBuilder;

    /// 已验证的逐顶点对称平面约束，可重复应用到本层的生长方向。
    class SlidingConstraints
    {
    public:
        const std::vector<SlidingPlane> &planes() const noexcept;
        const std::vector<VertexSlidingConstraint> &vertices() const noexcept;

        /// 将原始方向投影到指定前沿顶点允许的切向子空间并归一化。
        Result<Vector3, GrowthDirectionError>
        apply(
            std::size_t front_vertex_index,
            const Vector3 &raw_direction) const;

        Result<Vector3, GrowthDirectionError> constrainDirection(
            std::size_t front_vertex_index,
            const Point3 &current_position,
            const Vector3 &raw_direction) const;

        Result<SlidingPositionProjection, GrowthDirectionError>
        projectPosition(
            std::size_t front_vertex_index,
            const Point3 &candidate) const;

    private:
        friend class SlidingConstraintBuilder;

        Scalar angular_tolerance_{};       // 判断法向线性相关的无量纲容差
        Scalar direction_tolerance_{};     // 判断约束后方向是否退化的模长容差
        std::uint32_t layer_{};            // 约束所属的前沿层号
        std::vector<SlidingPlane> planes_; // 按 region_id 升序排列的有效平面
        std::vector<VertexSlidingConstraint> vertices_; // 与前沿顶点一一对应
        SlidingSurfaceSet surfaces_;
    };
}
