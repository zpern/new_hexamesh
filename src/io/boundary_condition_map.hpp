#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/io/boundary_condition_map_error.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    struct BoundaryZoneEntry
    {
        std::uint32_t zone_id{}; // CGNS 中的数字 Zone 名
        SurfaceBoundaryKind kind{
            SurfaceBoundaryKind::Farfield}; // Zone 对应的边界类别
        std::uint32_t region_id{}; // 写入 SurfaceBoundaryTag 的区域编号
    };

    struct BoundaryZoneMap
    {
        std::vector<BoundaryZoneEntry> entries; // 按 zone_id 升序保存的映射

        const BoundaryZoneEntry *find(
            std::uint32_t zone_id) const noexcept;
    };

    using BoundaryMapResult =
        Result<BoundaryZoneMap, BoundaryMapError>; // 边界映射解析结果

    BoundaryMapResult readBoundaryConditionMap(
        const std::filesystem::path &path,
        const std::vector<std::uint32_t> &available_zone_ids);
}
