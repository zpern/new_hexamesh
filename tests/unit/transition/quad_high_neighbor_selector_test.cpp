#include <array>
#include <cassert>
#include <optional>
#include <vector>

#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>

using namespace boundary_mesh;

namespace
{
    QuadHighNeighbor edge(std::size_t local, SurfaceFaceId neighbor)
    {
        return {local, neighbor};
    }

    QuadHighNeighborSelection select(
        std::vector<QuadHighNeighbor> highs)
    {
        const std::vector<Point3> points{
            {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
            {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}};
        const auto result = selectQuadHighNeighbors({
            7, 0, std::move(highs),
            {0,1,2,3}, {4,5,6,7}, &points, 1e-12});
        assert(result.hasValue());
        return result.value();
    }
}

int main()
{
    assert(select({}).retained_local_edges.empty());
    assert((select({edge(0, 20)}).retained_local_edges ==
            std::vector<std::size_t>{0}));
    const auto adjacent = select({edge(0, 20), edge(1, 21)});
    assert((adjacent.retained_local_edges ==
            std::vector<std::size_t>{0, 1}));
    assert(adjacent.required_low_diagonal ==
           QuadDiagonal::OneThree);
    assert((select({edge(0, 20), edge(2, 22)}).retained_local_edges ==
            std::vector<std::size_t>{0}));
    assert((select({edge(0, 20), edge(1, 21), edge(2, 22)})
                .retained_local_edges ==
            std::vector<std::size_t>{0, 1}));
    const auto all = select({
        edge(3, 23), edge(2, 22), edge(1, 21), edge(0, 20)});
    assert((all.retained_local_edges ==
            std::vector<std::size_t>{0, 1}));
    assert((all.suppressed_neighbor_faces ==
            std::vector<SurfaceFaceId>{22, 23}));
    return 0;
}
