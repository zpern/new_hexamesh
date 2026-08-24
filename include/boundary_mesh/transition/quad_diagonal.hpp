#pragma once

#include <array>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    enum class QuadDiagonal
    {
        ZeroTwo,
        OneThree
    };

    struct OrientedQuad
    {
        std::array<VertexId, 4> vertex_ids{};
        std::array<Point3, 4> points{};
    };

    struct QuadDiagonalSelection
    {
        QuadDiagonal diagonal{};
        std::array<Triangle, 2> triangles{};
        Scalar worst_skewness{};
    };

    Result<QuadDiagonalSelection, FaceEvaluationError>
    chooseQuadDiagonal(
        const OrientedQuad &quad,
        Scalar length_tolerance);
}
