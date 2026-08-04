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
        std::array<VertexId, 4> vertex_ids{};
    };
    struct Pyramid
    {
        std::array<VertexId, 5> vertex_ids{};
    };
    struct Prism
    {
        std::array<VertexId, 6> vertex_ids{};
    };
    struct Hexa
    {
        std::array<VertexId, 8> vertex_ids{};
    };

    using VolumeCell = std::variant<Tetra, Pyramid, Prism, Hexa>;

    enum class CellType
    {
        Tetra,
        Pyramid,
        Prism,
        Hexa
    };
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
        RegularLayer,
        Transition
    };

    struct CellMetadata
    {
        CellRole role{CellRole::RegularLayer};
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{0};
    };

    struct VolumeMesh
    {
        std::vector<Point3> vertices;
        std::vector<VolumeCell> cells;
        std::vector<CellMetadata> metadata;
    };
}