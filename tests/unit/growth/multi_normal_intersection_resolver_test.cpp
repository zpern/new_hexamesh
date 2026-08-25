#include <boundary_mesh/growth/multi_normal_intersection_resolver.hpp>
#include <boundary_mesh/growth/multi_normal_transition_builder.hpp>

int main()
{
    using namespace boundary_mesh;
    MultiNormalTopology topology;
    topology.front.vertices = {
        {Point3{0, 0, 0}, 0},
        {Point3{1, 0, 0}, 1},
        {Point3{0, 1, 0}, 2}};
    topology.front.vertices[0].multi_normal_branch = true;
    topology.front.vertices[0].direction = Vector3::UnitZ();
    topology.front.faces = {Triangle{{0, 1, 2}}};
    topology.front.source_face_ids = {0};

    MultiNormalOptions options;
    options.enabled = true;
    options.transition_height = Scalar{0.1};
    const std::vector<Scalar> input{Scalar{0.1}, Scalar{0}, Scalar{0}};
    const auto result = resolveMultiNormalLengths(topology, input, options);
    if (!result.hasValue() || result.value().lengths != input ||
        result.value().shrink_iterations != 0 ||
        result.value().used_zero_retry)
    {
        return 1;
    }

    // BLMesh treats geometric shared edges as legal even when one triangle is
    // from the grown surface and the other is from the bottom surface.
    topology.front.vertices.push_back({Point3{1, 1, 0}, 3});
    topology.front.faces.push_back(Triangle{{1, 3, 2}});
    topology.front.source_face_ids.push_back(1);
    const std::vector<Scalar> shared_edge_lengths{
        Scalar{0.1}, Scalar{0}, Scalar{0}, Scalar{0}};
    MultiNormalOptions candidate_options = options;
    candidate_options.resolved_transition_lengths = shared_edge_lengths;
    const auto candidate = buildMultiNormalTransition(topology, candidate_options);
    if (!candidate.hasValue()) return 2;
    if (!findMultiNormalIntersectionBadPoints(topology, candidate.value()).empty())
        return 3;

    std::vector<Scalar> uneven{Scalar{1}, Scalar{2}, Scalar{0}, Scalar{0}};
    smoothMultiNormalLengths(topology, uneven);
    if (std::abs(uneven[1] - Scalar{1.1}) > Scalar{1e-12} ||
        uneven[2] != Scalar{0} || uneven[3] != Scalar{0})
        return 4;
    return 0;
}
