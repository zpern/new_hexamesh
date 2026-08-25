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
            std::array<VertexId,5>{8,9,5,4,6}));
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

    auto alternate = input;
    alternate.high_edge_local_index = 1;
    const auto alternate_result = buildQuadTransition(alternate);
    assert(alternate_result.hasValue());
    assert((std::get<Pyramid>(
                alternate_result.value().side_cells[0]).vertex_ids ==
            std::array<VertexId,5>{10,9,5,6,4}));

    for (std::size_t edge = 0; edge < 4; ++edge)
    {
        auto rotated = input;
        rotated.high_edge_local_index = edge;
        const auto value = buildQuadTransition(rotated);
        assert(value.hasValue());
        assert(value.value().side_cells.size() == 2);
        assert(value.value().top_faces.size() == 4);
    }

    auto double_high = input;
    double_high.second_high_edge_local_index = 3;
    const auto double_result = buildQuadTransition(double_high);
    assert(double_result.hasValue());
    assert(double_result.value().side_cells.size() == 4);
    assert((std::get<Pyramid>(double_result.value().side_cells[0]).vertex_ids ==
            std::array<VertexId,5>{4,5,9,8,6}));
    assert((std::get<Pyramid>(double_result.value().side_cells[1]).vertex_ids ==
            std::array<VertexId,5>{4,8,11,7,6}));
    assert((std::get<Tetra>(double_result.value().side_cells[2]).vertex_ids ==
            std::array<VertexId,4>{8,11,10,6}));
    assert((std::get<Tetra>(double_result.value().side_cells[3]).vertex_ids ==
            std::array<VertexId,4>{8,9,10,6}));
    assert(double_result.value().top_faces.size() == 6);
    assert(double_result.value().volume_cells.size() == 11);

    auto opposite_high = input;
    opposite_high.second_high_edge_local_index = 2;
    assert(!buildQuadTransition(opposite_high).hasValue());

    auto one_trial_double = input;
    one_trial_double.trial_layers = 1;
    one_trial_double.layer_vertex_ids.resize(2);
    one_trial_double.high_edge_local_index = 0;
    one_trial_double.second_high_edge_local_index = 3;
    const auto one_trial_result = buildQuadTransition(one_trial_double);
    assert(one_trial_result.hasValue());
    assert(one_trial_result.value().created_vertices.empty());
    assert(one_trial_result.value().volume_cells.size() == 4);
    assert(one_trial_result.value().side_cells.size() == 4);
    assert((std::get<Pyramid>(one_trial_result.value().volume_cells[0]).vertex_ids ==
            std::array<VertexId,5>{0,1,5,4,2}));
    assert((std::get<Pyramid>(one_trial_result.value().volume_cells[1]).vertex_ids ==
            std::array<VertexId,5>{0,4,7,3,2}));
    assert((std::get<Tetra>(one_trial_result.value().volume_cells[2]).vertex_ids ==
            std::array<VertexId,4>{4,7,6,2}));
    assert((std::get<Tetra>(one_trial_result.value().volume_cells[3]).vertex_ids ==
            std::array<VertexId,4>{4,5,6,2}));
    assert(one_trial_result.value().top_faces.size() == 6);
    if (!one_trial_result.hasValue()) return 91;
    if ((std::get<Tetra>(
             one_trial_result.value().side_cells[2]).vertex_ids !=
         std::array<VertexId,4>{4,7,6,2})) return 92;
    if ((std::get<Tetra>(
             one_trial_result.value().side_cells[3]).vertex_ids !=
         std::array<VertexId,4>{4,5,6,2})) return 93;
    if (one_trial_result.value().top_faces[2].vertex_ids !=
        std::array<VertexId,3>{4,5,6}) return 94;
    if (one_trial_result.value().top_faces[3].vertex_ids !=
        std::array<VertexId,3>{4,7,6}) return 95;

    const std::vector<Point3> method_one_points{
        {-279.64478348526256, -272.9420187520721, 5.470843498903428},
        {-281.82791018652745, -271.6794076379535, -1.2864021832957364},
        {-278.67243494384246, -270.1705992816772, -13.349875285139477},
        {-278.5964526640703, -273.5173263001512, -6.450193092521454},
        {-281.73067648048544, -275.8063524607499, 6.218587915360811},
        {-283.9412849828099, -274.58168289749466, -1.346893708782532},
        {-280.45855813503584, -272.6279857499271, -15.407812200633824},
        {-280.6254496190468, -276.3048686620009, -7.614122328053261}};
    auto method_one = one_trial_double;
    method_one.mesh_vertices = &method_one_points;
    method_one.layer_vertex_ids = {
        std::array<VertexId,4>{0,1,2,3},
        std::array<VertexId,4>{4,5,6,7}};
    const auto method_one_result = buildQuadTransition(method_one);
    if (!method_one_result.hasValue()) return 101;
    if (method_one_result.value().side_cells.size() != 4) return 102;
    if ((std::get<Pyramid>(
             method_one_result.value().side_cells[0]).vertex_ids !=
         std::array<VertexId,5>{0,1,5,4,2})) return 103;
    if ((std::get<Pyramid>(
             method_one_result.value().side_cells[1]).vertex_ids !=
         std::array<VertexId,5>{0,4,7,3,2})) return 104;
    if ((std::get<Tetra>(
             method_one_result.value().side_cells[2]).vertex_ids !=
         std::array<VertexId,4>{4,7,5,2})) return 105;
    if ((std::get<Tetra>(
             method_one_result.value().side_cells[3]).vertex_ids !=
         std::array<VertexId,4>{7,5,6,2})) return 106;
    if (method_one_result.value().top_faces.size() != 6) return 107;
    if (method_one_result.value().top_faces[2].vertex_ids !=
        std::array<VertexId,3>{4,7,5}) return 108;
    if (method_one_result.value().top_faces[3].vertex_ids !=
        std::array<VertexId,3>{7,5,6}) return 109;

    auto zero_trial_high = input;
    zero_trial_high.trial_layers = 0;
    zero_trial_high.layer_vertex_ids.resize(1);
    assert(!buildQuadTransition(zero_trial_high).hasValue());
}
