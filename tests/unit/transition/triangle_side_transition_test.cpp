#include <cassert>
#include <variant>

#include <boundary_mesh/transition/triangle_side_transition.hpp>

using namespace boundary_mesh;

int main()
{
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
    }

    const auto invalid = buildTriangleSideTransition({
        9, 0, {0, 1, 2}, {3, 4, 5}, 3});
    assert(!invalid.hasValue());
}
