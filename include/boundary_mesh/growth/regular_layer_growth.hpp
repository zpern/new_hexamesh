#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/growth_field_smoothing_options.hpp>
#include <boundary_mesh/growth/growth_profile.hpp>
#include <boundary_mesh/mesh/mesh_volume.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation.hpp>

namespace boundary_mesh
{
    enum class FaceGrowthStatus
    {
        Active,    // 仍可尝试生成下一层
        Completed, // 因请求层数上限正常完成
        Stopped    // 因候选单元质量不合格提前停止
    };

    enum class FaceStopReason
    {
        None,                     // 未完成或停止
        VertexLayerLimit,         // 至少一个面顶点达到请求层数上限
        DegenerateCandidate,      // 候选单元退化
        ReversedCandidate,        // 候选单元整体反转
        LocallyInvertedCandidate, // 候选单元局部翻转
        SkewnessExceeded,         // 候选单元偏斜度超过阈值
        Collision,                // 候选单元发生非法几何接触
        SlidingProjectionFailure, // 滑移方向或最终位置投影失败
        NeighborLayerConstraint,  // 因共享边邻域层数上限传播而提前停止
        IsotropicHeightReached    // 当前单元已接受，达到各向同性阈值后停止
    };

    struct FaceStopEvent
    {
        std::size_t previous_front_face_index{};     // 对应上一层 Front 面下标
        SurfaceFaceId source_face_id{};              // 对应输入 Wall 面编号
        std::uint32_t layer{};                       // 首个未接受的目标层号
        FaceStopReason reason{FaceStopReason::None}; // 完成或停止原因
    };

    struct LayerStepResult
    {
        std::uint32_t layer{};                                  // 本次尝试生成的目标层号
        GrowthFront next_front;                                 // 只包含质量合格面的下一层 Front
        std::vector<std::size_t> previous_front_vertex_indices; // 下一层局部点到上一层局部点
        std::vector<std::size_t> previous_front_face_indices;   // 下一层局部面到上一层局部面
        std::vector<FaceStopEvent> stopped_faces;               // 质量不合格的源面
        std::vector<FaceStopEvent> accepted_stopped_faces;      // 当前单元已接受、但不再进入后续层的源面
        std::vector<FaceStopEvent> completed_faces;             // 达到层数上限的源面
        GrowthFieldSmoothingDiagnostics smoothing_diagnostics;  // 本层法向优化统计
    };

    struct LayerVertexRecord
    {
        VertexId source_vertex_id{};            // 输入 Wall 顶点编号
        std::vector<VertexId> layer_vertex_ids; // 从第 0 层开始的实际体网格顶点编号
        std::uint32_t branch_id{};              // 同一源点的多法向分支编号
    };

    using LayerVertexTable = std::vector<LayerVertexRecord>; // 全部源顶点的显式层编号映射

    struct VertexGrowthRecord
    {
        VertexId source_vertex_id{};          // 对应输入 Wall 顶点编号
        VertexGrowthProfile profile;          // 外部请求的完整生长参数
        std::uint32_t accepted_layer_count{}; // 实际进入最终网格的最大层数
    };

    struct FaceGrowthRecord
    {
        SurfaceFaceId source_face_id{};                    // 对应输入 Wall 面编号
        std::uint32_t accepted_layer_count{};              // 实际提交的规则层数量
        FaceGrowthStatus status{FaceGrowthStatus::Active}; // 当前最终状态
        FaceStopReason stop_reason{FaceStopReason::None};  // 完成或停止原因
        std::uint32_t stop_layer{};                        // 首个未被接受的目标层号
    };

    struct RegularLayerGrowthOptions
    {
        VolumeCellQualityOptions cell_quality;       // 阶段 04 的候选单元质量参数
        GrowthFieldSmoothingOptions field_smoothing; // 法向与步长字段平滑参数
        std::uint32_t max_layer_diff{1};             // 共享边两侧最大允许层数差
        Scalar isotropic_height{1};                  // BLMesh 风格实际层高与前沿多尺度的停止阈值
        bool enforce_single_high_edge{false};        // 过渡试生长按 HexaMesh 规则限制唯一高邻边
    };

    struct RegularLayerGrowthResult
    {
        VolumeMesh mesh;                                       // 独立的规则边界层体网格
        LayerVertexTable layer_vertices;                       // 源顶点到实际层顶点的显式映射
        std::vector<VertexGrowthRecord> vertices;              // 逐源顶点请求值和实际接受层数
        std::vector<FaceGrowthRecord> faces;                   // 逐源面状态、层数和停止原因
        SurfaceMesh farfield_boundary;                         // 原始 Farfield 与边界层最终外露接口组成的远场边界
        SurfaceMesh top_surface;                               // 边界层最终真实外露顶面
        GrowthFieldSmoothingDiagnostics smoothing_diagnostics; // 全部规则层的法向优化统计
    };
}
