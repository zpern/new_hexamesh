#include <array>
#include <cassert>
#include <utility>

#include <boundary_mesh/spatial/collision_index.hpp>

using namespace boundary_mesh;

namespace
{
    CollisionTriangle makeCollisionTriangle(
        const TrianglePoints &points,
        const std::array<CollisionVertexKey, 3> &keys,
        const std::array<Point3, 4> &boundary_points,
        const std::array<CollisionVertexKey, 4> &boundary_keys,
        std::uint8_t boundary_count)
    {
        CollisionTriangle triangle;
        triangle.points = points;
        triangle.vertex_keys = keys;
        triangle.boundary_points = boundary_points;
        triangle.boundary_vertex_keys = boundary_keys;
        triangle.boundary_vertex_count = boundary_count;
        return triangle;
    }
}

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

    const CollisionTriangle shared_vertex_first = makeCollisionTriangle(
        first,
        first_keys,
        {{first[0], first[1], first[2], Point3{}}},
        {{{0, 0}, {1, 0}, {2, 0}, {}}},
        3);
    const CollisionTriangle shared_vertex_touch = makeCollisionTriangle(
        vertex_touch,
        {{{0, 0}, {3, 0}, {4, 0}}},
        {{vertex_touch[0], vertex_touch[1], vertex_touch[2], Point3{}}},
        {{{0, 0}, {3, 0}, {4, 0}, {}}},
        3);
    assert(!hasIllegalTriangleContact(
                shared_vertex_first,
                shared_vertex_touch)
                .value());

    CollisionTriangle inconsistent_shared_vertex = shared_vertex_touch;
    inconsistent_shared_vertex.boundary_points[0] = {0.0, 0.0, 0.5};
    assert(hasIllegalTriangleContact(
               shared_vertex_first,
               inconsistent_shared_vertex)
               .value());

    const TrianglePoints vertex_cross_points{{
        first[0],
        {0.5, 0.5, -1.0},
        {0.5, 0.5, 1.0}}};
    const CollisionTriangle shared_vertex_cross = makeCollisionTriangle(
        vertex_cross_points,
        {{{0, 0}, {3, 0}, {4, 0}}},
        {{vertex_cross_points[0],
          vertex_cross_points[1],
          vertex_cross_points[2],
          Point3{}}},
        {{{0, 0}, {3, 0}, {4, 0}, {}}},
        3);
    assert(hasIllegalTriangleContact(
               shared_vertex_first,
               shared_vertex_cross)
               .value());

    const CollisionTriangle shared_edge_first = makeCollisionTriangle(
        first,
        first_keys,
        {{first[0], first[1], first[2], Point3{}}},
        {{{0, 0}, {1, 0}, {2, 0}, {}}},
        3);
    const CollisionTriangle shared_edge_touch = makeCollisionTriangle(
        shared_edge,
        edge_keys,
        {{shared_edge[0], shared_edge[1], shared_edge[2], Point3{}}},
        {{{0, 0}, {1, 0}, {3, 0}, {}}},
        3);
    assert(!hasIllegalTriangleContact(
                shared_edge_first,
                shared_edge_touch)
                .value());

    const TrianglePoints edge_cross_points{{
        first[0], first[1], {0.5, 0.5, 0.0}}};
    const CollisionTriangle shared_edge_cross = makeCollisionTriangle(
        edge_cross_points,
        edge_keys,
        {{edge_cross_points[0],
          edge_cross_points[1],
          edge_cross_points[2],
          Point3{}}},
        {{{0, 0}, {1, 0}, {3, 0}, {}}},
        3);
    assert(hasIllegalTriangleContact(
               shared_edge_first,
               shared_edge_cross)
               .value());

    const std::array<Point3, 4> quad_points{{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 1.0, 0.0},
        {0.0, 1.0, 0.0}}};
    const std::array<CollisionVertexKey, 4> quad_keys{{
        {10, 0}, {11, 0}, {12, 0}, {13, 0}}};
    const CollisionTriangle quad_split_first = makeCollisionTriangle(
        {{quad_points[0], quad_points[1], quad_points[2]}},
        {{quad_keys[0], quad_keys[1], quad_keys[2]}},
        quad_points,
        quad_keys,
        4);
    const CollisionTriangle quad_split_second = makeCollisionTriangle(
        {{quad_points[1], quad_points[2], quad_points[3]}},
        {{quad_keys[1], quad_keys[2], quad_keys[3]}},
        {{quad_points[1], quad_points[0], quad_points[3], quad_points[2]}},
        {{quad_keys[1], quad_keys[0], quad_keys[3], quad_keys[2]}},
        4);
    assert(!hasIllegalTriangleContact(
                quad_split_first,
                quad_split_second)
                .value());

    const TrianglePoints diagonal_touch_points{{
        quad_points[0],
        quad_points[2],
        {0.5, 0.5, 1.0}}};
    const CollisionTriangle diagonal_touch = makeCollisionTriangle(
        diagonal_touch_points,
        {{{10, 0}, {12, 0}, {20, 0}}},
        {{diagonal_touch_points[0],
          diagonal_touch_points[1],
          diagonal_touch_points[2],
          Point3{}}},
        {{{10, 0}, {12, 0}, {20, 0}, {}}},
        3);
    assert(hasIllegalTriangleContact(
               quad_split_first,
               diagonal_touch)
               .value());

    CollisionTriangle coincident_without_shared_keys = quad_split_first;
    coincident_without_shared_keys.vertex_keys =
        {{{30, 0}, {31, 0}, {32, 0}}};
    coincident_without_shared_keys.boundary_vertex_keys =
        {{{30, 0}, {31, 0}, {32, 0}, {33, 0}}};
    assert(hasIllegalTriangleContact(
               quad_split_first,
               coincident_without_shared_keys)
               .value());

    const std::array<Point3, 4> oblique_quad_points{{
        {0.1, 0.2, 0.3},
        {1.1, 0.2, 0.5},
        {1.1, 1.2, 0.8},
        {0.1, 1.2, 0.6}}};
    const CollisionTriangle oblique_first = makeCollisionTriangle(
        {{oblique_quad_points[0],
          oblique_quad_points[1],
          oblique_quad_points[2]}},
        {{quad_keys[0], quad_keys[1], quad_keys[2]}},
        oblique_quad_points,
        quad_keys,
        4);
    const CollisionTriangle oblique_second = makeCollisionTriangle(
        {{oblique_quad_points[1],
          oblique_quad_points[2],
          oblique_quad_points[3]}},
        {{quad_keys[1], quad_keys[2], quad_keys[3]}},
        {{oblique_quad_points[1],
          oblique_quad_points[0],
          oblique_quad_points[3],
          oblique_quad_points[2]}},
        {{quad_keys[1], quad_keys[0], quad_keys[3], quad_keys[2]}},
        4);
    assert(classifyTriangleContact(
               oblique_first.points,
               oblique_second.points)
               .value() == TriangleContactKind::CoplanarOverlap);
    assert(!hasIllegalTriangleContact(oblique_first, oblique_second).value());

    const Point3 oblique_edge_first{0.1, 0.2, 0.3};
    const Point3 oblique_edge_second{1.1, 0.4, 0.7};
    const TrianglePoints oblique_edge_triangle{{
        oblique_edge_first,
        oblique_edge_second,
        {0.2, 1.3, 0.9}}};
    const TrianglePoints oblique_edge_neighbor{{
        oblique_edge_first,
        oblique_edge_second,
        {0.4, -0.8, 1.2}}};
    const CollisionTriangle oblique_edge_a = makeCollisionTriangle(
        oblique_edge_triangle,
        {{{40, 1}, {41, 1}, {42, 1}}},
        {{oblique_edge_triangle[0],
          oblique_edge_triangle[1],
          oblique_edge_triangle[2],
          Point3{}}},
        {{{40, 1}, {41, 1}, {42, 1}, {}}},
        3);
    const CollisionTriangle oblique_edge_b = makeCollisionTriangle(
        oblique_edge_neighbor,
        {{{40, 1}, {41, 1}, {43, 0}}},
        {{oblique_edge_neighbor[0],
          oblique_edge_neighbor[1],
          oblique_edge_neighbor[2],
          Point3{}}},
        {{{40, 1}, {41, 1}, {43, 0}, {}}},
        3);
    assert(!hasIllegalTriangleContact(oblique_edge_a, oblique_edge_b).value());
}
