#include <algorithm>
#include <cassert>

#include <boundary_mesh/spatial/incremental_collision_index.hpp>

using namespace boundary_mesh;

namespace
{
    CollisionTriangle horizontalTriangle()
    {
        CollisionTriangle triangle;
        triangle.points = {{{-1.0, -1.0, 0.0},
                            {1.0, -1.0, 0.0},
                            {0.0, 1.0, 0.0}}};
        triangle.vertex_keys = {{{0, 0}, {1, 0}, {2, 0}}};
        triangle.owner_kind = CollisionOwnerKind::ExposedBoundary;
        triangle.owner_id = 7;
        for (std::size_t index = 0; index < 3; ++index)
        {
            triangle.boundary_points[index] = triangle.points[index];
            triangle.boundary_vertex_keys[index] = triangle.vertex_keys[index];
        }
        triangle.boundary_vertex_count = 3;
        return triangle;
    }

    CollisionTriangle shiftedTriangle(Scalar offset, std::uint32_t owner)
    {
        CollisionTriangle triangle = horizontalTriangle();
        for (Point3 &point : triangle.points) point.x() += offset;
        for (std::size_t index = 0; index < 3; ++index)
            triangle.boundary_points[index] = triangle.points[index];
        triangle.owner_id = owner;
        return triangle;
    }

    CollisionTriangle tinyTriangle(Point3 center, std::uint32_t owner)
    {
        CollisionTriangle triangle = horizontalTriangle();
        triangle.points = {{{center.x() - 0.05, center.y() - 0.05, center.z()},
                            {center.x() + 0.05, center.y() - 0.05, center.z()},
                            {center.x(), center.y() + 0.05, center.z()}}};
        for (std::size_t index = 0; index < 3; ++index)
            triangle.boundary_points[index] = triangle.points[index];
        triangle.owner_id = owner;
        return triangle;
    }

    CollisionTriangle verticalTriangle()
    {
        CollisionTriangle triangle;
        triangle.points = {{{0.0, 0.0, -1.0},
                            {0.0, 0.0, 1.0},
                            {0.0, 0.5, 0.0}}};
        triangle.vertex_keys = {{{10, 0}, {11, 0}, {12, 0}}};
        triangle.owner_kind = CollisionOwnerKind::LayerCandidate;
        triangle.owner_id = 8;
        for (std::size_t index = 0; index < 3; ++index)
        {
            triangle.boundary_points[index] = triangle.points[index];
            triangle.boundary_vertex_keys[index] = triangle.vertex_keys[index];
        }
        triangle.boundary_vertex_count = 3;
        return triangle;
    }
}

int main()
{
    const auto empty = IncrementalCollisionIndex::build({});
    assert(empty.hasValue());
    auto index = std::move(empty.value());

    const CollisionGroupId group{7};
    assert(index.insertGroup({group, {horizontalTriangle()}}).hasValue());
    assert(index.queryIllegalContacts(verticalTriangle()).size() == 1);

    assert(index.eraseGroup(group).hasValue());
    assert(index.queryIllegalContacts(verticalTriangle()).empty());

    const auto missing = index.eraseGroup(group);
    assert(!missing.hasValue());
    assert(missing.error() == SpatialError::MissingPrimitiveGroup);

    IncrementalCollisionIndexOptions options;
    options.target_leaf_capacity = 2;
    options.rebuild_leaf_capacity = 3;
    options.rebuild_inactive_ratio = Scalar{0.25};
    const auto seeded = IncrementalCollisionIndex::build(
        {{1, {shiftedTriangle(0.0, 1)}},
         {2, {shiftedTriangle(0.1, 2)}},
         {3, {shiftedTriangle(0.2, 3)}},
         {4, {shiftedTriangle(0.3, 4)}}},
        options);
    assert(seeded.hasValue());
    auto tree = std::move(seeded.value());
    assert(tree.diagnostics().maximum_leaf_load >= 4);
    const auto rebuilt = tree.rebuildIfDegraded();
    assert(rebuilt.hasValue());
    assert(rebuilt.value());
    assert(tree.diagnostics().rebuilds == 1);

    const auto stable_id = tree.queryCandidates(
        makeAabb(Point3{-2, -2, -1}, Point3{2, -2, -1},
                 Point3{0, 2, 1}).value()).front();
    const auto stable_owner = tree.primitive(stable_id).owner_id;
    assert(tree.primitive(stable_id).owner_id == stable_owner);

    assert(tree.insertGroup({99, {shiftedTriangle(100.0, 99)}}).hasValue());
    assert(tree.diagnostics().root_expansions == 1);

    CollisionTriangle degenerate = shiftedTriangle(3.0, 101);
    degenerate.points[2] = degenerate.points[1];
    const auto active_before = tree.diagnostics().active_primitives;
    const auto rejected = tree.insertGroup(
        {100, {shiftedTriangle(2.0, 100), degenerate}});
    assert(!rejected.hasValue());
    assert(rejected.error() == SpatialError::DegenerateTriangle);
    assert(tree.diagnostics().active_primitives == active_before);

    IncrementalCollisionIndexOptions sparse_options;
    sparse_options.target_leaf_capacity = 1;
    const auto sparse_result = IncrementalCollisionIndex::build(
        {{1, {tinyTriangle({-1, -1, -1}, 1)}},
         {2, {tinyTriangle({1, 1, 1}, 2)}}}, sparse_options);
    assert(sparse_result.hasValue());
    auto sparse = std::move(sparse_result.value());
    const CollisionTriangle new_octant = tinyTriangle({1, -1, -1}, 3);
    assert(sparse.insertGroup({3, {new_octant}}).hasValue());
    const auto new_octant_bounds = makeAabb(
        new_octant.points[0], new_octant.points[1], new_octant.points[2]);
    assert(new_octant_bounds.hasValue());
    const auto candidates = sparse.queryCandidates(new_octant_bounds.value());
    assert(std::any_of(
        candidates.begin(), candidates.end(),
        [&](CollisionPrimitiveId id) { return sparse.primitive(id).owner_id == 3; }));
}
