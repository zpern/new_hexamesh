#include <cassert>
#include <variant>

#include <boundary_mesh/transition/triangle_side_transition.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

using namespace boundary_mesh;

int main()
{
    const std::array<Point3, 6> points{{
        Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{0, 1, 0},
        Point3{0, 0, 1}, Point3{1, 0, 1}, Point3{0, 1, 1}}};
    for (std::size_t edge = 0; edge < 3; ++edge)
    {
        const auto result = buildTriangleSideTransition({
            7, 4, {0, 1, 2}, {3, 4, 5}, edge});
        assert(result.hasValue());
        assert(result.value().volume_cells.size() == 1);
        assert(std::holds_alternative<Pyramid>(
            result.value().volume_cells.front()));
        assert(result.value().metadata.size() == 1);
        assert(result.value().metadata.front().source_face_id == 7);
        assert(result.value().metadata.front().layer == 5);
        assert(result.value().top_faces.size() == 3);
        const auto &cell = std::get<Pyramid>(
            result.value().volume_cells.front());
        PyramidPoints cell_points{};
        for (std::size_t i = 0; i < 5; ++i)
            cell_points[i] = points[cell.vertex_ids[i]];
        const auto evaluation = evaluatePyramid(cell_points);
        if (!evaluation.hasValue() ||
            evaluation.value().validity != VolumeCellValidity::Valid)
            return static_cast<int>(edge + 1);
    }

    const auto invalid = buildTriangleSideTransition({
        9, 0, {0, 1, 2}, {3, 4, 5}, 3});
    assert(!invalid.hasValue());
}
