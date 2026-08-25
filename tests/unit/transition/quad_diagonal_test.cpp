#include <array>
#include <cassert>

#include <boundary_mesh/transition/quad_diagonal.hpp>

using namespace boundary_mesh;

int main()
{
    const OrientedQuad skewed{
        {VertexId{10}, VertexId{11}, VertexId{12}, VertexId{13}},
        {Point3{0, 0, 0}, Point3{2, 0, 0},
         Point3{1.8, 1, 0}, Point3{0, 1, 0}}};
    const auto chosen = chooseQuadDiagonal(skewed, 1e-12);
    assert(chosen.hasValue());
    assert(chosen.value().worst_skewness >= 0.0);
    assert(chosen.value().worst_skewness <= 1.0);

    const OrientedQuad square{
        {VertexId{4}, VertexId{1}, VertexId{3}, VertexId{2}},
        {Point3{0, 0, 0}, Point3{1, 0, 0},
         Point3{1, 1, 0}, Point3{0, 1, 0}}};
    const auto tie = chooseQuadDiagonal(square, 1e-12);
    assert(tie.hasValue());
    assert(tie.value().diagonal == QuadDiagonal::OneThree);
    assert((tie.value().triangles[0].vertex_ids ==
            std::array<VertexId, 3>{1, 3, 2}));
    assert((tie.value().triangles[1].vertex_ids ==
            std::array<VertexId, 3>{1, 2, 4}));

    OrientedQuad degenerate = square;
    degenerate.points[2] = degenerate.points[1];
    assert(!chooseQuadDiagonal(degenerate, 1e-12).hasValue());
}
