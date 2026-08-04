#pragma once

#include <array>
#include <cstdint>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct Tetra
    {
        std::array<VertexId, 4> vertex_ids{}; // 四面体的四个顶点编号
    };

    struct Pyramid
    {
        std::array<VertexId, 5> vertex_ids{}; // 金字塔的五个顶点编号
    };

    struct Prism
    {
        std::array<VertexId, 6> vertex_ids{}; // 三棱柱的六个顶点编号
    };

    struct Hexa
    {
        std::array<VertexId, 8> vertex_ids{}; // 六面体的八个顶点编号
    };

    using VolumeCell = std::variant<Tetra, Pyramid, Prism, Hexa>; // 支持的混合体单元

    enum class CellType
    {
        Tetra,   // 四面体
        Pyramid, // 金字塔
        Prism,   // 三棱柱
        Hexa     // 六面体
    };

    /// 返回混合体单元当前保存的具体单元类型。
    inline CellType cellType(const VolumeCell &cell)
    {
        return std::visit(
            [](const auto &value)
            {
                using Cell = std::decay_t<decltype(value)>;

                if constexpr (
                    std::is_same_v<Cell, Tetra>)
                {
                    return CellType::Tetra;
                }
                else if constexpr (
                    std::is_same_v<Cell, Pyramid>)
                {
                    return CellType::Pyramid;
                }
                else if constexpr (
                    std::is_same_v<Cell, Prism>)
                {
                    return CellType::Prism;
                }
                else
                {
                    return CellType::Hexa;
                }
            },
            cell);
    }

    enum class CellRole
    {
        RegularLayer, // 规则边界层单元
        Transition    // 局部停止或拓扑变化产生的过渡单元
    };

    struct CellMetadata
    {
        CellRole role{CellRole::RegularLayer}; // 体单元在边界层中的职责
        SurfaceFaceId source_face_id{}; // 生成该体单元的源表面面片编号
        std::uint32_t layer{0}; // 体单元所属的边界层层号
    };

    struct VolumeMesh
    {
        std::vector<Point3> vertices; // 全部体网格顶点坐标，数组下标即 VertexId
        std::vector<VolumeCell> cells; // 全部混合体单元，数组下标即 VolumeCellId
        std::vector<CellMetadata> metadata; // 与 cells 一一对应的生成信息
    };
}
