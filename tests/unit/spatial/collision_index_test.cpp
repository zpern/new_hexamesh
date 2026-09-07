#include <array>
#include <cassert>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/spatial/collision_boundary_policy.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>

using namespace boundary_mesh;

int main()
{
    const CollisionBoundaryPolicy policy;
    for (const auto origin : {
             CollisionSurfaceOrigin::InputSurface,
             CollisionSurfaceOrigin::GeneratedBoundary})
    {
        assert(!policy.isObstacle(SurfaceBoundaryKind::Symmetry, origin));
        assert(!policy.isObstacle(SurfaceBoundaryKind::Internal, origin));
        assert(policy.isObstacle(SurfaceBoundaryKind::Wall, origin));
        assert(policy.isObstacle(SurfaceBoundaryKind::Farfield, origin));
        assert(policy.isObstacle(
            SurfaceBoundaryKind::BoundaryLayerInterface, origin));
    }

    SurfaceMesh mesh;
    mesh.vertices = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0},
        {1.0, 1.0, 1.0}, {0.0, 1.0, 1.0}};
    mesh.faces = {
        Quad{{0, 3, 2, 1}}, Quad{{4, 5, 6, 7}},
        Quad{{0, 1, 5, 4}}, Quad{{1, 2, 6, 5}},
        Quad{{2, 3, 7, 6}}, Quad{{3, 0, 4, 7}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Wall, 1},
        {SurfaceBoundaryKind::Farfield, 2},
        {SurfaceBoundaryKind::Symmetry, 3},
        {SurfaceBoundaryKind::Symmetry, 3},
        {SurfaceBoundaryKind::Symmetry, 3},
        {SurfaceBoundaryKind::Symmetry, 3}};

    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    assert(topology.hasValue());
    mesh.face_tags[2] = {SurfaceBoundaryKind::Internal, 4};
    const auto index = buildOriginalSurfaceCollisionIndex(
        mesh,
        topology.value());
    assert(index.hasValue());
    assert(index.value().primitiveCount() == 4);

    const CollisionTriangle &wall_primitive =
        index.value().primitive(0);
    assert(wall_primitive.boundary_vertex_count == 4);
    assert(
        wall_primitive.boundary_vertex_keys[3].source_vertex_id == 1);
    assert(wall_primitive.boundary_vertex_keys[3].layer == 0);

    const CollisionTriangle wall_hit{
        {{{0.0, 0.0, 0.0},
          {0.0, 1.0, 0.0},
          {1.0, 1.0, 0.0}}},
        {{{100, 0}, {101, 0}, {102, 0}}},
        CollisionOwnerKind::LayerCandidate,
        7};
    const auto wall_contacts = index.value().queryIllegalContacts(wall_hit);
    assert(wall_contacts.size() == 2);

    const CollisionTriangle symmetry_only_hit{
        {{{0.2, 0.0, 0.2},
          {0.8, 0.0, 0.2},
          {0.8, 0.0, 0.8}}},
        {{{100, 0}, {101, 0}, {102, 0}}},
        CollisionOwnerKind::LayerCandidate,
        8};
    assert(index.value().queryIllegalContacts(symmetry_only_hit).empty());

    const std::array<Point3, 4> quad_points{{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0},
        {0.0, 1.0, 0.0}}};
    const std::array<CollisionVertexKey, 4> quad_keys{{
        {10, 0}, {11, 0}, {12, 0}, {13, 0}}};
    CollisionTriangle stored;
    stored.points = {{quad_points[1], quad_points[2], quad_points[3]}};
    stored.vertex_keys = {{quad_keys[1], quad_keys[2], quad_keys[3]}};
    stored.boundary_points =
        {{quad_points[1], quad_points[0], quad_points[3], quad_points[2]}};
    stored.boundary_vertex_keys =
        {{quad_keys[1], quad_keys[0], quad_keys[3], quad_keys[2]}};
    stored.boundary_vertex_count = 4;
    const auto quad_index = CollisionIndex::build({stored});
    assert(quad_index.hasValue());

    CollisionTriangle query;
    query.points = {{quad_points[0], quad_points[1], quad_points[2]}};
    query.vertex_keys = {{quad_keys[0], quad_keys[1], quad_keys[2]}};
    query.boundary_points = quad_points;
    query.boundary_vertex_keys = quad_keys;
    query.boundary_vertex_count = 4;
    assert(quad_index.value().queryIllegalContacts(query).empty());
}
