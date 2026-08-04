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
        std::array<VertexId, 3> vertex_ids{};
    };

    struct Quad
    {
        std::array<VertexId, 4> vertex_ids{};
    };

    using SurfaceFace = std::variant<Triangle, Quad>;

    enum class SurfaceBoundaryKind
    {
        Farfield,
        Wall,
        Symmetry
    };

    struct SurfaceBoundaryTag
    {
        SurfaceBoundaryKind kind{SurfaceBoundaryKind::Farfield};
        std::uint32_t region_id{};
    };

    struct SurfaceMesh
    {
        std::vector<Point3> vertices;
        std::vector<SurfaceFace> faces;
        std::vector<SurfaceBoundaryTag> face_tags;
    };
}