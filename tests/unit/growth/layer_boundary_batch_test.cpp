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
        LayerBoundaryCandidate candidate{
            face(bottom_points, keys, 0), face(top_points, keys, 1)};
        candidate.bottom.source_face_id = 100 + ids[0];
        candidate.top.source_face_id = 100 + ids[0];
        return candidate;
    }
}

int main()
{
    const auto isolated = LayerBoundaryBatch::build({quad(0, 1, {0, 1, 2, 3})});
    assert(isolated.hasValue());
    assert(isolated.value().owners().size() == 1);
    assert(isolated.value().owners()[0].triangles.size() == 10);
    assert(isolated.value().diagnostics().omitted_shared_sides == 0);
    assert(isolated.value().adjacentSourceFaceIds()[0].empty());

    const auto adjacent = LayerBoundaryBatch::build({
        quad(0, 1, {0, 1, 2, 3}),
        quad(1, 2, {1, 4, 5, 2})});
    assert(adjacent.hasValue());
    assert(adjacent.value().owners().size() == 2);
    assert(adjacent.value().diagnostics().triangle_count == 16);
    assert(adjacent.value().diagnostics().omitted_shared_sides == 2);
    assert((adjacent.value().adjacentSourceFaceIds()[0] ==
            std::vector<SurfaceFaceId>{101}));
    assert((adjacent.value().adjacentSourceFaceIds()[1] ==
            std::vector<SurfaceFaceId>{100}));

    auto incremental = adjacent.value();
    const auto changed_after_stop = incremental.deactivateOwners({1});
    assert(changed_after_stop.hasValue());
    if (changed_after_stop.value() != std::vector<std::size_t>{0} ||
        !incremental.activeOwners()[0] || incremental.activeOwners()[1] ||
        !incremental.owners()[1].triangles.empty() ||
        incremental.owners()[0].triangles.size() != 10)
        return 1;
    const auto full_after_stop = LayerBoundaryBatch::build({
        quad(0, 1, {0, 1, 2, 3})});
    assert(full_after_stop.hasValue());
    if (incremental.owners()[0].triangles.size() !=
            full_after_stop.value().owners()[0].triangles.size())
        return 2;
    for (std::size_t triangle_index = 0;
         triangle_index < incremental.owners()[0].triangles.size();
         ++triangle_index)
        if (incremental.owners()[0].triangles[triangle_index].points !=
            full_after_stop.value().owners()[0].triangles[triangle_index].points)
            return 3;
    auto separated_neighbor = quad(5, 6, {10, 11, 12, 13});
    auto stable_slots = LayerBoundaryBatch::build({
        quad(0, 1, {0, 1, 2, 3}),
        quad(1, 2, {1, 4, 5, 2}),
        separated_neighbor});
    assert(stable_slots.hasValue());
    const auto stable_bounds = stable_slots.value().owners();
    std::vector<Aabb> conservative_bounds;
    conservative_bounds.reserve(stable_slots.value().candidates().size());
    for (const auto &candidate : stable_slots.value().candidates())
    {
        Aabb envelope{candidate.bottom.points.front(),
                      candidate.bottom.points.front()};
        for (const Point3 &point : candidate.bottom.points)
        {
            envelope.minimum = envelope.minimum.cwiseMin(point);
            envelope.maximum = envelope.maximum.cwiseMax(point);
        }
        for (const Point3 &point : candidate.top.points)
        {
            envelope.minimum = envelope.minimum.cwiseMin(point);
            envelope.maximum = envelope.maximum.cwiseMax(point);
        }
        conservative_bounds.push_back(envelope);
    }
    auto persistent = BatchSelfCollisionIndex::build(
        stable_bounds, conservative_bounds);
    assert(persistent.hasValue());
    const auto stable_changed = stable_slots.value().deactivateOwners({1});
    assert(stable_changed.hasValue());
    const auto local_collisions = persistent.value().detectChanged(
        stable_slots.value().owners(), stable_changed.value());
    assert(local_collisions.hasValue());
    std::vector<LayerBoundaryOwner> compact_active;
    for (std::size_t owner_index = 0;
         owner_index < stable_slots.value().owners().size(); ++owner_index)
        if (stable_slots.value().activeOwners()[owner_index])
            compact_active.push_back(stable_slots.value().owners()[owner_index]);
    const auto full_collisions =
        BatchSelfCollisionDetector::detectOwners(compact_active);
    assert(full_collisions.hasValue());
    if (local_collisions.value().illegal_owner_ids !=
        full_collisions.value().illegal_owner_ids)
        return 4;

    const auto retained = LayerBoundaryBatch::build({
        quad(0, 1, {0, 1, 2, 3})});
    assert(retained.hasValue());
    assert(retained.value().diagnostics().triangle_count == 10);

    LayerBoundaryCandidate triangle{
        face({{1, 0, 0}, {2, 0, 0}, {1, 1, 0}},
             {{1, 0}, {4, 0}, {2, 0}}, 0),
        face({{1, 0, 1}, {2, 0, 1}, {1, 1, 1}},
             {{1, 0}, {4, 0}, {2, 0}}, 1)};
    triangle.bottom.source_face_id = 200;
    triangle.top.source_face_id = 200;
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
