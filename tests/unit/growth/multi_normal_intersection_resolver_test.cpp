#include <boundary_mesh/growth/multi_normal_intersection_resolver.hpp>

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
    return 0;
}
