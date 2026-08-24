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
    assert(step.value().metadata[1].role == CellRole::Transition);

    for (std::size_t edge = 0; edge < 3; ++edge)
    {
        const auto rotated = buildTriangleTransition(input(2, edge));
        assert(rotated.hasValue());
        assert(rotated.value().volume_cells.size() == 2);
        assert(rotated.value().top_faces.size() == 3);
    }

    auto malformed = input(2, 0);
    malformed.layer_vertex_ids.pop_back();
    assert(!buildTriangleTransition(malformed).hasValue());
}
