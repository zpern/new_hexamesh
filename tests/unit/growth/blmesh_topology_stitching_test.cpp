#include <boundary_mesh/growth/blmesh_topology_stitching.hpp>

int main()
{
    using namespace boundary_mesh;
    const auto chain = orderBlmeshDirectedTriangleChain({
        {2, 30, 40}, {0, 10, 20}, {1, 20, 30}});
    if (!chain.hasValue() || chain.value().size() != 3 ||
        chain.value()[0].triangle_index != 0 ||
        chain.value()[1].triangle_index != 1 ||
        chain.value()[2].triangle_index != 2) return 1;

    const std::vector<Vector3> left{
        Vector3::UnitX(), Vector3::UnitY()};
    const std::vector<Vector3> right{
        Vector3::UnitZ(), (Vector3::UnitX() + Vector3::UnitZ()).normalized()};
    const auto interleaving = findBlmeshSmoothestInterleaving(left, right);
    if (!interleaving.hasValue() || interleaving.value().size() != 4)
        return 2;
    std::size_t left_count = 0, right_count = 0;
    for (const int entry : interleaving.value())
    {
        left_count += entry > 0;
        right_count += entry < 0;
    }
    return left_count == 2 && right_count == 2 ? 0 : 3;
}
