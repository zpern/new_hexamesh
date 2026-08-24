#include <array>
#include <cassert>
#include <variant>
#include <vector>

#include <boundary_mesh/transition/transition_templates.hpp>

using namespace boundary_mesh;

int main()
{
    const std::vector<Point3> points{
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1},
        {0,0,2}, {1,0,2}, {1,1,2}, {0,1,2}};
    QuadTransitionInput input;
    input.source_face_id = 7;
    input.trial_layers = 2;
    input.high_edge_local_index = 0;
    input.layer_vertex_ids = {
        std::array<VertexId,4>{0,1,2,3},
        std::array<VertexId,4>{4,5,6,7},
        std::array<VertexId,4>{8,9,10,11}};
    input.mesh_vertices = &points;
    input.center_vertex_id = 12;

    const auto result = buildQuadTransition(input);
    assert(result.hasValue());
    assert(result.value().side_cells.size() == 2);
    assert((std::get<Pyramid>(result.value().side_cells[0]).vertex_ids ==
            std::array<VertexId,5>{8,9,4,5,6}));
    assert((std::get<Tetra>(result.value().side_cells[1]).vertex_ids ==
            std::array<VertexId,4>{4,7,6,8}));
    assert(result.value().top_faces.size() == 4);
    assert((result.value().top_faces[0].vertex_ids ==
            std::array<VertexId,3>{4,8,7}));
    assert((result.value().top_faces[1].vertex_ids ==
            std::array<VertexId,3>{8,6,7}));
    assert((result.value().top_faces[2].vertex_ids ==
            std::array<VertexId,3>{8,9,6}));
    assert((result.value().top_faces[3].vertex_ids ==
            std::array<VertexId,3>{9,5,6}));
    assert(result.value().volume_cells.size() == 9);

    for (std::size_t edge = 0; edge < 4; ++edge)
    {
        auto rotated = input;
        rotated.high_edge_local_index = edge;
        const auto value = buildQuadTransition(rotated);
        assert(value.hasValue());
        assert(value.value().side_cells.size() == 2);
        assert(value.value().top_faces.size() == 4);
    }
}
