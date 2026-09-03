#include <array>
#include <cassert>
#include <optional>
#include <variant>
#include <vector>

#include <boundary_mesh/transition/transition_templates.hpp>

using namespace boundary_mesh;

namespace
{
    TriangleTransitionInput input(
        std::uint32_t layers,
        std::optional<std::size_t> high_edge)
    {
        TriangleTransitionInput value;
        value.source_face_id = 12;
        value.trial_layers = layers;
        value.high_edge_local_index = high_edge;
        value.layer_vertex_ids = {
            std::array<VertexId, 3>{0, 1, 2},
            std::array<VertexId, 3>{3, 4, 5},
            std::array<VertexId, 3>{6, 7, 8}};
        value.layer_vertex_ids.resize(layers + 1);
        return value;
    }
}

int main()
{
    const auto zero = buildTriangleTransition(input(0, std::nullopt));
    assert(zero.hasValue());
    assert(zero.value().volume_cells.empty());
    assert((zero.value().top_faces[0].vertex_ids ==
            std::array<VertexId, 3>{0, 1, 2}));
    assert(!buildTriangleTransition(input(0, 0)).hasValue());

    const auto one = buildTriangleTransition(input(1, std::nullopt));
    assert(one.hasValue());
    assert(one.value().volume_cells.empty());
    assert(one.value().top_faces.size() == 1);

    const auto step = buildTriangleTransition(input(2, 0));
    assert(step.hasValue());
    assert(step.value().volume_cells.size() == 2);
    assert(std::holds_alternative<Prism>(step.value().volume_cells[0]));
    assert((std::get<Pyramid>(step.value().volume_cells[1]).vertex_ids ==
            std::array<VertexId, 5>{3, 4, 7, 6, 5}));
    assert(step.value().top_faces.size() == 3);
    assert(step.value().metadata.size() == 2);
    assert(step.value().metadata[1].role ==
           CellRole::ReservedLayerTransition);

    for (std::size_t edge = 0; edge < 3; ++edge)
    {
        const auto rotated = buildTriangleTransition(input(2, edge));
        assert(rotated.hasValue());
        assert(rotated.value().volume_cells.size() == 2);
        assert(rotated.value().top_faces.size() == 3);
    }

    auto continuing = input(1, std::nullopt);
    continuing.continuing_edge_local_index = 0;
    const auto continuing_result = buildTriangleTransition(continuing);
    assert(continuing_result.hasValue());
    assert(continuing_result.value().volume_cells.size() == 1);
    assert((std::get<Pyramid>(
                continuing_result.value().volume_cells[0]).vertex_ids ==
            std::array<VertexId, 5>{0, 3, 4, 1, 2}));
    assert(continuing_result.value().top_faces.size() == 3);
    assert(continuing_result.value().metadata.size() == 1);
    assert(continuing_result.value().metadata[0].role ==
           CellRole::ReservedLayerTransition);
    auto continuing_two = input(2, std::nullopt);
    continuing_two.continuing_edge_local_index = 0;
    const auto continuing_two_result = buildTriangleTransition(
        continuing_two);
    assert(continuing_two_result.hasValue());
    assert(continuing_two_result.value().volume_cells.size() == 2);
    assert(std::holds_alternative<Prism>(
        continuing_two_result.value().volume_cells[0]));
    assert((std::get<Pyramid>(
                continuing_two_result.value().volume_cells[1]).vertex_ids ==
            std::array<VertexId, 5>{3, 6, 7, 4, 5}));
    for (std::size_t edge = 0; edge < 3; ++edge)
    {
        auto rotated = input(1, std::nullopt);
        rotated.continuing_edge_local_index = edge;
        const auto rotated_result = buildTriangleTransition(rotated);
        assert(rotated_result.hasValue());
        assert(rotated_result.value().volume_cells.size() == 1);
        assert(rotated_result.value().top_faces.size() == 3);
    }

    auto conflicting = continuing;
    conflicting.high_edge_local_index = 0;
    assert(!buildTriangleTransition(conflicting).hasValue());
    auto invalid_continuing = continuing;
    invalid_continuing.continuing_edge_local_index = 3;
    assert(!buildTriangleTransition(invalid_continuing).hasValue());
    auto zero_continuing = input(0, std::nullopt);
    zero_continuing.continuing_edge_local_index = 0;
    assert(!buildTriangleTransition(zero_continuing).hasValue());

    auto malformed = input(2, 0);
    malformed.layer_vertex_ids.pop_back();
    assert(!buildTriangleTransition(malformed).hasValue());
}
