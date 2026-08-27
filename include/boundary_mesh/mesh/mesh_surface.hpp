#pragma once

#include <array>
#include <cstdint>
#include <variant>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct Triangle
    {
        std::array<VertexId, 3> vertex_ids{}; // 按面绕序保存的三个顶点编号
    };

    struct Quad
    {
        std::array<VertexId, 4> vertex_ids{}; // 按面绕序保存的四个顶点编号
    };

    using SurfaceFace = std::variant<Triangle, Quad>; // 三角形或四边形表面单元

    enum class SurfaceBoundaryKind
    {
        Farfield,              // 远场边界
        Wall,                  // 需要生成边界层的壁面
        Symmetry,              // 对称面
        Internal,              // 内部面
        MatchNoPush,           // 仅用于匹配的面，禁止在边界层生成中推动
        BoundaryLayerInterface // 边界层与后续远场体网格之间的界面
    };

    struct SurfaceBoundaryTag
    {
        SurfaceBoundaryKind kind{SurfaceBoundaryKind::Farfield}; // 面片的边界类别
        std::uint32_t region_id{};                               // 同类边界中的区域编号
    };

    struct SurfaceMesh
    {
        std::vector<Point3> vertices;              // 全部表面顶点坐标，数组下标即 VertexId
        std::vector<SurfaceFace> faces;            // 全部混合表面面片，数组下标即 SurfaceFaceId
        std::vector<SurfaceBoundaryTag> face_tags; // 与 faces 一一对应的边界标签
    };
}
