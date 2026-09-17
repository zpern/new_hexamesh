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
}
