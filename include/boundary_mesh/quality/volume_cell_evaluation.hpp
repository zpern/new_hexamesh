#pragma once

#include <array>
#include <cstddef>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    using TetraPoints = std::array<Point3, 4>; // 按 TetraVertexOrder 排列
    using PyramidPoints = std::array<Point3, 5>; // 按 PyramidVertexOrder 排列
    using PrismPoints = std::array<Point3, 6>; // Prism 候选单元按公共顶点顺序排列的六个坐标
    using HexaPoints = std::array<Point3, 8>;  // Hexa 候选单元按公共顶点顺序排列的八个坐标

    struct VolumeCellQualityOptions
    {
        Scalar maximum_skewness{0.95}; // 可接受候选单元允许的最大等角偏斜度
    };

    enum class VolumeCellValidity
    {
        Valid,          // 所有固定子四面体的有向体积均为正
        Degenerate,     // 无正负混合，但至少一个固定子四面体的有向体积为零
        Reversed,       // 所有固定子四面体的有向体积均为负
        LocallyInverted // 固定子四面体同时出现正负有向体积
    };

    struct VolumeCellEvaluation
    {
        VolumeCellValidity validity{VolumeCellValidity::Degenerate}; // 固定子四面体的几何有效性分类
        Scalar signed_volume{};                                      // 所有固定子四面体有向体积之和
        Scalar minimum_subtet_signed_volume{};                       // 最小固定子四面体有向体积
        Scalar maximum_subtet_signed_volume{};                       // 最大固定子四面体有向体积
        std::size_t worst_subtet_index{};                            // 最小子体积对应的最早固定下标
        Scalar skewness{};                                           // 所有组成面中的最大等角偏斜度
        Scalar minimum_local_jacobian{};                             // 采样点中的最小有向 Jacobian
        bool acceptable{};                                           // 有效且偏斜度不大于阈值时为真
    };
}
