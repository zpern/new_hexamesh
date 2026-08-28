#include <array>
#include <cassert>
#include <optional>
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
    input.layer_vertex_ids = {
        std::array<VertexId,4>{0,1,2,3},
        std::array<VertexId,4>{4,5,6,7},
        std::array<VertexId,4>{8,9,10,11}};
    input.mesh_vertices = &points;
    input.center_vertex_id = 12;

    const auto result = buildQuadTransition(input);
    assert(result.hasValue());
    assert(result.value().created_vertices.size() == 1);
    assert(result.value().volume_cells.size() == 7);
    std::size_t pyramids = 0;
    std::size_t tetras = 0;
    for (const VolumeCell &cell : result.value().volume_cells)
    {
        pyramids += std::holds_alternative<Pyramid>(cell) ? 1 : 0;
        tetras += std::holds_alternative<Tetra>(cell) ? 1 : 0;
    }
    assert(pyramids == 5);
    assert(tetras == 2);
    assert((std::get<Pyramid>(result.value().volume_cells[0]).vertex_ids ==
            std::array<VertexId,5>{0,1,2,3,12}));
    assert(result.value().top_faces.size() == 2);
    assert(result.value().metadata.size() == 7);
    for (const CellMetadata &metadata : result.value().metadata)
        assert(metadata.role == CellRole::ReservedLayerTransition);

    auto one = input;
    one.trial_layers = 1;
    one.layer_vertex_ids.resize(2);
    const auto no_volume = buildQuadTransition(one);
    assert(no_volume.hasValue());
    assert(no_volume.value().volume_cells.empty());
    assert(no_volume.value().top_faces.size() == 2);
}
