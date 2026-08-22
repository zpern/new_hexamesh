#include <array>
#include <cassert>
#include <utility>

#include <boundary_mesh/spatial/triangle_contact.hpp>

using namespace boundary_mesh;

int main()
{
    const TrianglePoints first{{
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {0.0, 2.0, 0.0}}};
    const TrianglePoints disjoint{{
        {0.0, 0.0, 1.0},
        {2.0, 0.0, 1.0},
        {0.0, 2.0, 1.0}}};
    const TrianglePoints crossing{{
        {0.5, 0.5, -1.0},
        {0.5, 0.5, 1.0},
        {1.5, 0.5, 0.0}}};
    const TrianglePoints vertex_touch{{
        {0.0, 0.0, 0.0},
        {-1.0, 0.0, 1.0},
        {0.0, -1.0, 1.0}}};
    const TrianglePoints shared_edge{{
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {0.0, -1.0, 1.0}}};
    const TrianglePoints overlap{{
        {0.25, 0.25, 0.0},
        {1.0, 0.25, 0.0},
        {0.25, 1.0, 0.0}}};
    const TrianglePoints coplanar_vertex_touch{{
        {0.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0}}};

    assert(classifyTriangleContact(first, disjoint).value() ==
           TriangleContactKind::Disjoint);
    assert(classifyTriangleContact(first, crossing).value() ==
           TriangleContactKind::ProperIntersect);
    assert(classifyTriangleContact(first, vertex_touch).value() ==
           TriangleContactKind::VertexTouch);
    assert(classifyTriangleContact(first, shared_edge).value() ==
           TriangleContactKind::EdgeTouch);
    assert(classifyTriangleContact(first, overlap).value() ==
           TriangleContactKind::CoplanarOverlap);
    assert(classifyTriangleContact(first, coplanar_vertex_touch).value() ==
           TriangleContactKind::VertexTouch);

    const std::array<CollisionVertexKey, 3> first_keys{{
        {0, 0}, {1, 0}, {2, 0}}};
    const std::array<CollisionVertexKey, 3> edge_keys{{
        {0, 0}, {1, 0}, {3, 0}}};
    const std::array<CollisionVertexKey, 3> unrelated_keys{{
        {4, 0}, {5, 0}, {6, 0}}};

    assert(!hasIllegalTriangleContact(
                first,
                first_keys,
                shared_edge,
                edge_keys)
                .value());
    assert(hasIllegalTriangleContact(
               first,
               first_keys,
               overlap,
               unrelated_keys)
               .value());
    assert(!hasIllegalTriangleContact(
                first,
                first_keys,
                first,
                first_keys)
                .value());

    TrianglePoints reversed_first{{first[0], first[2], first[1]}};
    std::array<CollisionVertexKey, 3> reversed_keys{{
        first_keys[0], first_keys[2], first_keys[1]}};
    assert(classifyTriangleContact(reversed_first, crossing).value() ==
           TriangleContactKind::ProperIntersect);
    assert(hasIllegalTriangleContact(
               crossing,
               unrelated_keys,
               reversed_first,
               reversed_keys)
               .value());
}
