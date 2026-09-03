#include <array>
#include <cassert>
#include <cstddef>
#include <variant>
#include <vector>

#include <boundary_mesh/transition/incremental_transition_templates.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>

using namespace boundary_mesh;

namespace
{
    template <typename Cell>
    std::size_t countCells(const std::vector<VolumeCell> &cells)
    {
        std::size_t count = 0;
        for (const VolumeCell &cell : cells)
            if (std::holds_alternative<Cell>(cell)) ++count;
        return count;
    }
}

int main()
{
    const std::vector<Point3> points{
        {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
        {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1}};
    const OrientedQuad bottom{
        {0, 1, 2, 3},
        {points[0], points[1], points[2], points[3]}};

    LayerQuadDiagonalTable diagonals;
    const auto selected = diagonals.resolve(
        {7, 0}, bottom, QuadDiagonal::OneThree, 1e-12);
    assert(selected.hasValue());
    assert(selected.value() == QuadDiagonal::OneThree);
    const auto reused = diagonals.resolve(
        {7, 0}, bottom, std::nullopt, 1e-12);
    assert(reused.hasValue());
    assert(reused.value() == QuadDiagonal::OneThree);
    const auto conflict = diagonals.resolve(
        {7, 0}, bottom, QuadDiagonal::ZeroTwo, 1e-12);
    assert(!conflict.hasValue());

    const auto cap = buildQuadTopCap({
        7, 1, {0, 1, 2, 3}, {4, 5, 6, 7},
        &points, 8, QuadDiagonal::OneThree});
    assert(cap.hasValue());
    assert(cap.value().created_vertices.size() == 1);
    assert(cap.value().volume_cells.size() == 7);
    assert(countCells<Pyramid>(cap.value().volume_cells) == 5);
    assert(countCells<Tetra>(cap.value().volume_cells) == 2);
    assert(cap.value().top_faces.size() == 2);

    const auto side = buildQuadSideTransition({
        7, 0, {0, 1, 2, 3}, {4, 5, 6, 7},
        0, QuadDiagonal::OneThree});
    assert(side.hasValue());
    assert(side.value().volume_cells.size() == 2);
    assert(side.value().top_faces.size() == 4);
    assert(side.value().low_diagonal == QuadDiagonal::OneThree);
    return 0;
}
