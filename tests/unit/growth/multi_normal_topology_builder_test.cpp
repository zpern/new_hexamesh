#include <algorithm>
#include <variant>

#include <boundary_mesh/growth/multi_normal_topology_builder.hpp>

int main()
{
    using namespace boundary_mesh;
    const auto local = [](std::size_t branch) {
        SplitVirtualPoint point;
        point.kind = SplitVirtualPoint::Kind::LocalBranch;
        point.branch_index = branch;
        return point;
    };
    const auto far = [](VertexId vertex) {
        SplitVirtualPoint point;
        point.kind = SplitVirtualPoint::Kind::FarVertex;
        point.far_vertex_id = vertex;
        return point;
    };

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
    split.virtual_triangles = {
        {{{local(0), local(1), far(1)}}}};

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

    // BLMesh does not triangulate the four-copy strip with a fixed diagonal.
    // It interleaves both virtual-sphere triangle chains.  A 2+1 chain must
    // therefore create three stitching triangles.
    GrowthFront paired;
    paired.vertices = {
        {Point3{0, 0, 0}, VertexId{200}},
        {Point3{1, 0, 0}, VertexId{201}},
        {Point3{0, 1, 0}, VertexId{202}},
        {Point3{1, 1, 0}, VertexId{203}}};
    paired.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
        Triangle{{VertexId{1}, VertexId{0}, VertexId{3}}}};
    paired.source_face_ids = {20, 21};

    VertexSplitPlan left;
    left.front_vertex_index = 0;
    left.source_vertex_id = 200;
    left.splitter_neighbors = {1};
    left.branches = {
        SplitBranch{{0}, Vector3::UnitZ(), Scalar{1}},
        SplitBranch{{1}, Vector3::UnitY(), Scalar{1}},
        SplitBranch{{}, Vector3::UnitX(), Scalar{1}}};
    left.virtual_triangles = {
        {{{local(0), local(1), local(2)}}}};
    left.neighbor_triangle_chains = {{1, {
        {1, 0, {{local(0), local(1)}}, Vector3::UnitZ()},
        {2, 0, {{local(1), local(2)}}, Vector3::UnitY()}}}};

    VertexSplitPlan right;
    right.front_vertex_index = 1;
    right.source_vertex_id = 201;
    right.splitter_neighbors = {0};
    right.branches = {
        SplitBranch{{1}, Vector3::UnitZ(), Scalar{1}},
        SplitBranch{{0}, Vector3::UnitY(), Scalar{1}}};
    right.neighbor_triangle_chains = {{0, {
        {0, 0, {{local(0), local(1)}}, Vector3::UnitX()}}}};

    const auto paired_result = buildMultiNormalTopology(paired, {left, right});
    if (!paired_result.hasValue()) return 7;
    if (paired_result.value().front.faces.size() != 6 ||
        paired_result.value().transition_face_origins.size() != 4)
        return 8;

    return 0;
}
