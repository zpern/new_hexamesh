#pragma once

#include <array>
#include <cstddef>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    using PrismPoints = std::array<Point3, 6>; // Prism 候选单元按公共顶点顺序排列的六个坐标
    using HexaPoints = std::array<Point3, 8>;  // Hexa 候选单元按公共顶点顺序排列的八个坐标

    struct VolumeCellQualityOptions
    {
        Scalar relative_jacobian_tolerance{1e-12}; // 归一化 Jacobian 符号判定的相对容差
        Scalar relative_length_tolerance{1e-12};   // 面角计算中零长度边判定的相对容差
        Scalar maximum_skewness{0.95};             // 可接受候选单元允许的最大等角偏斜度
    };

    enum class VolumeCellValidity
    {
        Valid,           // 所有规定采样点的 Jacobian 均为正
        Degenerate,      // 不存在正负混合但至少一个规定采样点退化
        Reversed,        // 所有规定采样点的 Jacobian 均为负
        LocallyInverted  // 规定采样点同时出现正负 Jacobian
    };

    enum class JacobianSampleKind
    {
        Vertex,           // 参考单元顶点处的 Jacobian 采样
        Center,           // 参考单元中心处的 Jacobian 采样
        IntegrationPoint  // 用于体积积分的 Gauss 点 Jacobian 采样
    };

    struct JacobianSampleLocation
    {
        JacobianSampleKind kind{JacobianSampleKind::Center}; // Jacobian 采样位置的类别
        std::size_t index{};                                 // 同类别采样位置的固定顺序下标
    };

    struct VolumeCellEvaluation
    {
        VolumeCellValidity validity{VolumeCellValidity::Degenerate}; // 候选单元的 Jacobian 有效性分类
        Scalar signed_volume{};                                      // 积分得到的有向体积
        Scalar minimum_jacobian{};                                   // 全部规定采样点中的最小原始 Jacobian
        Scalar maximum_jacobian{};                                   // 全部规定采样点中的最大原始 Jacobian
        Scalar minimum_normalized_jacobian{};                        // 全部规定采样点中的最小归一化 Jacobian
        Scalar maximum_normalized_jacobian{};                        // 全部规定采样点中的最大归一化 Jacobian
        Scalar skewness{};                                           // 所有组成面中的最大等角偏斜度
        JacobianSampleLocation worst_jacobian_location{};            // 最小归一化 Jacobian 对应的最早采样位置
        bool acceptable{};                                           // 有效且偏斜度不大于阈值时为真
    };
}
