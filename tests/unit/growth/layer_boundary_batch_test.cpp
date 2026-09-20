#include <cassert>

#include <boundary_mesh/growth/layer_boundary_batch.hpp>

using namespace boundary_mesh;

namespace
{
    BoundaryFace face(
        std::initializer_list<Point3> points,
        std::initializer_list<CollisionVertexKey> keys,
        std::uint32_t layer)
    {
        std::vector<CollisionVertexKey> top_keys(keys);
        for (CollisionVertexKey &key : top_keys) key.layer = layer;
        return BoundaryFace{
            std::vector<Point3>(points), std::move(top_keys), layer, 1};
    }

    LayerBoundaryCandidate quad(
        Scalar x0,
        Scalar x1,
        std::array<std::uint32_t, 4> ids)
    {
        const std::initializer_list<Point3> bottom_points{
            {x0, 0, 0}, {x1, 0, 0}, {x1, 1, 0}, {x0, 1, 0}};
        const std::initializer_list<Point3> top_points{
            {x0, 0, 1}, {x1, 0, 1}, {x1, 1, 1}, {x0, 1, 1}};
        const std::initializer_list<CollisionVertexKey> keys{
            {ids[0], 0}, {ids[1], 0}, {ids[2], 0}, {ids[3], 0}};
        return {face(bottom_points, keys, 0), face(top_points, keys, 1)};
    }
}

int main()
{
    const auto isolated = LayerBoundaryBatch::build({quad(0, 1, {0, 1, 2, 3})});
    assert(isolated.hasValue());
    assert(isolated.value().owners().size() == 1);
    assert(isolated.value().owners()[0].triangles.size() == 10);
    assert(isolated.value().diagnostics().omitted_shared_sides == 0);

    const auto adjacent = LayerBoundaryBatch::build({
        quad(0, 1, {0, 1, 2, 3}),
        quad(1, 2, {1, 4, 5, 2})});
    assert(adjacent.hasValue());
    assert(adjacent.value().owners().size() == 2);
    assert(adjacent.value().diagnostics().triangle_count == 16);
    assert(adjacent.value().diagnostics().omitted_shared_sides == 2);

    const auto retained = LayerBoundaryBatch::build({
        quad(0, 1, {0, 1, 2, 3})});
    assert(retained.hasValue());
    assert(retained.value().diagnostics().triangle_count == 10);

    LayerBoundaryCandidate triangle{
        face({{1, 0, 0}, {2, 0, 0}, {1, 1, 0}},
             {{1, 0}, {4, 0}, {2, 0}}, 0),
        face({{1, 0, 1}, {2, 0, 1}, {1, 1, 1}},
             {{1, 0}, {4, 0}, {2, 0}}, 1)};
    const auto mixed = LayerBoundaryBatch::build({
        quad(0, 1, {0, 1, 2, 3}), triangle});
    assert(mixed.hasValue());
    assert(mixed.value().diagnostics().triangle_count == 13);
    assert(mixed.value().diagnostics().omitted_shared_sides == 2);

    LayerBoundaryCandidate third = triangle;
    third.bottom.points[1] = {1.5, -1, 0};
    third.top.points[1] = {1.5, -1, 1};
    third.bottom.vertex_keys[1] = {9, 0};
    third.top.vertex_keys[1] = {9, 1};
    const auto non_manifold = LayerBoundaryBatch::build({
        quad(0, 1, {0, 1, 2, 3}), triangle, third});
    assert(!non_manifold.hasValue());
    assert(non_manifold.error() == SpatialError::InvalidTopologyReference);
}
