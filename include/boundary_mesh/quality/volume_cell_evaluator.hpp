#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation_error.hpp>

namespace boundary_mesh
{
    /// 评估按 TetraVertexOrder 排列的 Tetra；不修改节点顺序。
    Result<VolumeCellEvaluation, VolumeCellEvaluationError>
    evaluateTetra(
        const TetraPoints &points,
        const VolumeCellQualityOptions &options = {});

    /// 评估按 PyramidVertexOrder 排列的 Pyramid；不修改节点顺序。
    Result<VolumeCellEvaluation, VolumeCellEvaluationError>
    evaluatePyramid(
        const PyramidPoints &points,
        const VolumeCellQualityOptions &options = {});

    /// 评估按 PrismVertexOrder 排列的 Prism 候选单元；该声明不修改网格或全局状态。
    Result<VolumeCellEvaluation, VolumeCellEvaluationError>
    evaluatePrism(
        const PrismPoints &points,
        const VolumeCellQualityOptions &options = {});

    /// 评估按 HexaVertexOrder 排列的 Hexa 候选单元；该声明不修改网格或全局状态。
    Result<VolumeCellEvaluation, VolumeCellEvaluationError>
    evaluateHexa(
        const HexaPoints &points,
        const VolumeCellQualityOptions &options = {});
}
