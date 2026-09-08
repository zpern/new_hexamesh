#pragma once

#include <cstddef>
#include <optional>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation.hpp>

namespace boundary_mesh
{
    enum class VolumeCellKind
    {
        Prism, // 六顶点三棱柱候选单元
        Hexa,  // 八顶点六面体候选单元
        Tetra,
        Pyramid
    };

    enum class VolumeCellEvaluationErrorCategory
    {
        InvalidMaximumSkewness,    // 最大偏斜度不在闭区间 [0, 1] 内
        NonFiniteVertexCoordinate, // 输入顶点坐标包含非有限分量
        NonFiniteIntermediateResult // 中间几何计算产生非有限数值
    };

    struct VolumeCellEvaluationError
    {
        VolumeCellEvaluationErrorCategory category{
            VolumeCellEvaluationErrorCategory::InvalidMaximumSkewness}; // 失败的输入、配置或数值错误类别
        VolumeCellKind cell_kind{VolumeCellKind::Prism};                 // 发生错误的体单元类别
        Scalar configuration_value{};                                   // 触发配置错误的原始数值，没有配置时为零
        std::optional<std::size_t> local_vertex_index{};                 // 非有限输入对应的可选局部顶点下标
        std::optional<std::size_t> subtet_index{};                       // 数值错误对应的可选固定子四面体下标
    };
}
