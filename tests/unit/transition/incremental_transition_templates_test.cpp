#include <array>
#include <cassert>
#include <cstddef>
#include <cmath>
#include <variant>
#include <vector>

#include <boundary_mesh/transition/incremental_transition_templates.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

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

    bool positive(const VolumeCell &cell, const std::vector<Point3> &points)
    {
        return std::visit([&](const auto &value)
        {
            using Cell = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<Cell, Tetra>)
            {
                TetraPoints cell_points{};
                for (std::size_t i = 0; i < 4; ++i)
                    cell_points[i] = points[value.vertex_ids[i]];
                const auto result = evaluateTetra(cell_points);
                return result.hasValue() &&
                    result.value().validity == VolumeCellValidity::Valid;
            }
            else if constexpr (std::is_same_v<Cell, Pyramid>)
            {
                PyramidPoints cell_points{};
                for (std::size_t i = 0; i < 5; ++i)
                    cell_points[i] = points[value.vertex_ids[i]];
                const auto result = evaluatePyramid(cell_points);
                return result.hasValue() &&
                    result.value().validity == VolumeCellValidity::Valid;
            }
            return true;
        }, cell);
    }

    bool allPositive(
        const std::vector<VolumeCell> &cells,
        const std::vector<Point3> &points)
    {
        return std::all_of(cells.begin(), cells.end(),
            [&](const VolumeCell &cell) { return positive(cell, points); });
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

    const auto regular_center = findPositiveQuadTopCapCenter({
        {points[0], points[1], points[2], points[3]},
        {points[4], points[5], points[6], points[7]},
        QuadDiagonal::ZeroTwo, Scalar{1e-12}});
    if (!regular_center.has_value() ||
        ((*regular_center - Point3{0.5, 0.5, 0.5}).norm() > 1e-10))
        return 1;
    const Scalar regular_ratio = quadTopCapAspectRatio({
        {points[0], points[1], points[2], points[3]},
        {points[4], points[5], points[6], points[7]}});
    if (std::abs(regular_ratio - Scalar{1}) > Scalar{1e-12})
        return 2;

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
    std::vector<Point3> cap_points = points;
    cap_points.insert(cap_points.end(), cap.value().created_vertices.begin(),
        cap.value().created_vertices.end());
    const bool cap_positive = allPositive(cap.value().volume_cells, cap_points);

    const auto side = buildQuadSideTransition({
        7, 0, {0, 1, 2, 3}, {4, 5, 6, 7},
        0, QuadDiagonal::OneThree});
    assert(side.hasValue());
    assert(side.value().volume_cells.size() == 2);
    assert(side.value().top_faces.size() == 4);
    assert(side.value().low_diagonal == QuadDiagonal::OneThree);
    const bool side_positive = allPositive(side.value().volume_cells, points);

    const auto adjacent = buildQuadAdjacentSideTransition({
        7, 0, {0,1,2,3}, {4,5,6,3}, 0, 1,
        QuadDiagonal::OneThree, &points, 1e-12});
    assert(adjacent.hasValue());
    assert(adjacent.value().volume_cells.size() == 2);
    assert(countCells<Pyramid>(adjacent.value().volume_cells) == 2);
    assert(countCells<Tetra>(adjacent.value().volume_cells) == 0);
    assert(adjacent.value().top_faces.size() == 4);
    assert(adjacent.value().low_diagonal == QuadDiagonal::OneThree);
    const bool adjacent_positive = allPositive(adjacent.value().volume_cells, points);
    if (!cap_positive) return 10;
    if (!side_positive) return 11;
    if (!adjacent_positive) return 12;

    const auto no_high_external = buildExternalQuadPatch({
        9, 1, {0,1,2,3}, {0,1,2,3}, {}, &points, 8,
        Scalar{0.25}, Scalar{1e-12}});
    if (!no_high_external.hasValue() ||
        no_high_external.value().volume_cells.size() != 1 ||
        no_high_external.value().created_vertices.size() != 1)
        return 13;
    std::vector<Point3> no_high_points = points;
    no_high_points.push_back(no_high_external.value().created_vertices[0]);
    if (!allPositive(no_high_external.value().volume_cells, no_high_points))
        return 14;

    const auto one_high_external = buildExternalQuadPatch({
        9, 1, {0,1,2,3}, {4,5,2,3}, {0}, &points, 8,
        Scalar{0.25}, Scalar{1e-12}});
    if (!one_high_external.hasValue() ||
        one_high_external.value().volume_cells.size() != 2)
        return 15;
    std::vector<Point3> one_high_points = points;
    one_high_points.push_back(one_high_external.value().created_vertices[0]);
    if (!allPositive(one_high_external.value().volume_cells, one_high_points))
        return 16;

    const auto two_high_external = buildExternalQuadPatch({
        9, 1, {0,1,2,3}, {4,5,6,3}, {0,1}, &points, 8,
        Scalar{0.25}, Scalar{1e-12}});
    if (!two_high_external.hasValue() ||
        two_high_external.value().volume_cells.size() != 3)
        return 17;
    std::vector<Point3> two_high_points = points;
    two_high_points.push_back(two_high_external.value().created_vertices[0]);
    if (!allPositive(two_high_external.value().volume_cells, two_high_points))
        return 18;

    for (const QuadDiagonal diagonal : {
             QuadDiagonal::ZeroTwo, QuadDiagonal::OneThree})
    {
        const auto rotated_cap = buildQuadTopCap({
            8, 1, {0, 1, 2, 3}, {4, 5, 6, 7},
            &points, 8, diagonal});
        if (!rotated_cap.hasValue()) return 20;
        std::vector<Point3> rotated_cap_points = points;
        rotated_cap_points.insert(rotated_cap_points.end(),
            rotated_cap.value().created_vertices.begin(),
            rotated_cap.value().created_vertices.end());
        if (!allPositive(rotated_cap.value().volume_cells,
                         rotated_cap_points))
            return 21;

        for (std::size_t edge = 0; edge < 4; ++edge)
        {
            const auto rotated_side = buildQuadSideTransition({
                8, 0, {0, 1, 2, 3}, {4, 5, 6, 7}, edge, diagonal});
            if (!rotated_side.hasValue() ||
                !allPositive(rotated_side.value().volume_cells, points))
                return 30 + static_cast<int>(diagonal) * 4 +
                    static_cast<int>(edge);
        }
    }

    for (std::size_t common = 0; common < 4; ++common)
    {
        const QuadDiagonal required = common % 2 == 0
            ? QuadDiagonal::ZeroTwo
            : QuadDiagonal::OneThree;
        const auto rotated_adjacent = buildQuadAdjacentSideTransition({
            8, 0, {0, 1, 2, 3}, {4, 5, 6, 7},
            (common + 3) % 4, common, required, &points, 1e-12});
        if (!rotated_adjacent.hasValue() ||
            !allPositive(rotated_adjacent.value().volume_cells, points))
            return 23;
    }
    return 0;
}
