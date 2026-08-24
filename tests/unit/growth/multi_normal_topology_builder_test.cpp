#include <algorithm>
#include <variant>

#include <boundary_mesh/growth/multi_normal_topology_builder.hpp>

int main()
{
    using namespace boundary_mesh;

    GrowthFront front;
    front.vertices = {
        {Point3{0, 0, 0}, VertexId{100}},
        {Point3{1, 0, 0}, VertexId{101}},
        {Point3{0, 1, 0}, VertexId{102}},
        {Point3{-1, 0, 0}, VertexId{103}},
        {Point3{-1, -1, 0}, VertexId{104}}};
    front.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
        Quad{{VertexId{0}, VertexId{3}, VertexId{4}, VertexId{1}}}};
    front.source_face_ids = {10, 11};

    VertexSplitPlan split;
    split.front_vertex_index = 0;
    split.source_vertex_id = 100;
    split.splitter_neighbors = {1};
    split.branches = {
        SplitBranch{{0}, Vector3::UnitZ(), Scalar{1}},
        SplitBranch{{1}, Vector3::UnitY(), Scalar{1}}};

    const auto result = buildMultiNormalTopology(front, {split});
    if (!result.hasValue()) return 1;
    const MultiNormalTopology &topology = result.value();
    if (topology.front.vertices.size() != 6 ||
        topology.front.faces.size() != 3 ||
        topology.front.source_face_ids.size() != 3)
    {
        return 2;
    }

    const auto *triangle = std::get_if<Triangle>(&topology.front.faces[0]);
    const auto *quad = std::get_if<Quad>(&topology.front.faces[1]);
    const auto *stitch = std::get_if<Triangle>(&topology.front.faces[2]);
    if (triangle == nullptr || quad == nullptr || stitch == nullptr ||
        triangle->vertex_ids[0] == quad->vertex_ids[0])
    {
        return 3;
    }

    const VertexId first_copy = triangle->vertex_ids[0];
    const VertexId second_copy = quad->vertex_ids[0];
    const std::array<VertexId, 3> expected{
        first_copy, second_copy, VertexId{1}};
    std::array<VertexId, 3> actual = stitch->vertex_ids;
    std::sort(actual.begin(), actual.end());
    auto sorted_expected = expected;
    std::sort(sorted_expected.begin(), sorted_expected.end());
    if (actual != sorted_expected) return 4;

    if (topology.vertex_mapping.size() != 6 ||
        topology.front.vertices[first_copy].source_vertex_id != 100 ||
        topology.front.vertices[second_copy].source_vertex_id != 100 ||
        topology.front.vertices[first_copy].branch_id ==
            topology.front.vertices[second_copy].branch_id)
    {
        return 5;
    }
    if (topology.transition_face_origins.size() != 1 ||
        topology.transition_face_origins[0].source_face_ids !=
            std::vector<SurfaceFaceId>({10, 11}))
    {
        return 6;
    }

    return 0;
}
