#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>

namespace boundary_mesh
{
    enum class BoundaryMapErrorCode
    {
        FileOpenFailure,    // 无法打开同名边界映射文件
        InvalidSection,     // 区段不是 Far 或 Wall
        ZoneOutsideSection, // Zone 出现在任何合法区段以前
        InvalidZoneId,      // Zone 不是可表示的正 uint32_t
        DuplicateZone,      // 同一 Zone 被配置多次
        MissingZone,        // CGNS Zone 未出现在映射中
        UnknownZone         // 映射引用了 CGNS 中不存在的 Zone
    };

    struct BoundaryMapError
    {
        BoundaryMapErrorCode code{
            BoundaryMapErrorCode::FileOpenFailure}; // 错误分类
        std::filesystem::path path; // 发生错误的映射文件
        std::size_t line{}; // 发生语法错误的 1-based 行号
        std::uint32_t zone_id{}; // 与错误相关的 Zone，未知时为 0
    };
}
