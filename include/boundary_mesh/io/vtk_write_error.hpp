#pragma once

#include <cstddef>
#include <filesystem>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    enum class VtkWriteErrorCode
    {
        FileOpenFailure,        // 无法创建临时输出文件
        InvalidVertexReference, // 单元引用了不存在的顶点
        InvalidCellMetadataCount, // 体单元与元数据数量不一致
        CountOverflow,          // Legacy VTK 连接计数不可表示
        WriteFailure,           // 流写入或关闭失败
        ReplaceFailure          // 无法用完整临时文件替换目标
    };

    struct VtkWriteError
    {
        VtkWriteErrorCode code{
            VtkWriteErrorCode::FileOpenFailure}; // 错误分类
        std::filesystem::path path; // 目标 VTK 路径
        std::size_t cell_index{}; // 相关单元下标，不适用时为 0
        VertexId vertex_id{}; // 相关顶点编号，不适用时为 0
    };
}
