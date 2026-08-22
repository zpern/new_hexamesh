#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include <boundary_mesh/io/boundary_condition_map_error.hpp>

namespace boundary_mesh
{
    enum class CgnsSurfaceErrorCode
    {
        FileOpenFailure,               // 无法打开 CGNS 文件
        BoundaryMapFailure,            // 同名 .bc.txt 解析失败
        InvalidBaseCount,              // CGNS Base 数量不是 1
        InvalidDimensions,             // Base 不是二维单元、三维物理空间
        InvalidZoneType,               // Zone 不是非结构类型
        InvalidZoneId,                 // Zone 名不是唯一的正 uint32_t
        MissingCoordinate,             // Zone 缺少 X、Y 或 Z 坐标
        NonFiniteCoordinate,           // 顶点坐标包含 NaN 或无穷值
        UnsupportedElementType,        // 元素不是 TRI_3 或 QUAD_4
        InvalidElementReference,       // 面元素引用了 Zone 外的顶点
        InvalidConnectivity,           // Zone 连接信息不完整或越界
        ConnectivityCoordinateMismatch,// 显式连接两端坐标不完全相同
        VertexIdOverflow,              // 输出顶点数量超出 VertexId
        FaceIdOverflow,                // 输出面数量超出 SurfaceFaceId
        CgnsLibraryFailure             // cgnslib 调用失败
    };

    struct CgnsSurfaceError
    {
        CgnsSurfaceErrorCode code{
            CgnsSurfaceErrorCode::FileOpenFailure}; // 错误分类
        std::filesystem::path path; // 相关 CGNS 或边界映射文件
        std::uint32_t zone_id{}; // 相关数字 Zone，不适用时为 0
        std::uint64_t element_id{}; // 相关 CGNS 元素，不适用时为 0
        BoundaryMapError boundary_map_error{}; // 边界映射失败的具体原因
        std::string detail; // cgnslib 或格式诊断文本
    };
}
