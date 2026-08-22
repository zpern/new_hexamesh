#include <array>
#include <cassert>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>

using namespace boundary_mesh;

int main()
{
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
    const auto index = buildOriginalSurfaceCollisionIndex(
        mesh,
        topology.value());
    assert(index.hasValue());
    assert(index.value().primitiveCount() == 4);

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
}
