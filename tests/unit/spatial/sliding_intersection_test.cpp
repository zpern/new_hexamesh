#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include <boundary_mesh/spatial/sliding_intersection.hpp>

using namespace boundary_mesh;

int main()
{
    const TrianglePoints surface{{
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {0.0, 2.0, 0.0}}};
    const SlidingContactPermission none;

    const TrianglePoints crossing{{
        {0.5, 0.5, -1.0},
        {0.5, 0.5, 1.0},
        {1.5, 1.5, 1.0}}};
    const auto crossing_result =
        hasInvalidSlidingIntersection(crossing, surface, none);
    assert(crossing_result.hasValue() && crossing_result.value());

    const TrianglePoints vertex_touch{{
        {0.0, 0.0, 0.0},
        {-1.0, 0.0, 1.0},
        {0.0, -1.0, 1.0}}};
    SlidingContactPermission vertex_permission;
    vertex_permission.vertex_mask = 0b001;
    const auto vertex_result = hasInvalidSlidingIntersection(
        vertex_touch, surface, vertex_permission);
    assert(vertex_result.hasValue() && !vertex_result.value());

    const TrianglePoints beyond_vertex{{
        {0.0, 0.0, 0.0},
        {0.5, 0.5, -1.0},
        {-1.0, 0.0, 1.0}}};
    const auto beyond_result = hasInvalidSlidingIntersection(
        beyond_vertex, surface, vertex_permission);
    assert(beyond_result.hasValue() && beyond_result.value());

    const TrianglePoints edge_touch{{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.5, -1.0, 1.0}}};
    SlidingContactPermission edge_permission;
    edge_permission.vertex_mask = 0b011;
    edge_permission.edge_mask = 0b001;
    const auto edge_result = hasInvalidSlidingIntersection(
        edge_touch, surface, edge_permission);
    assert(edge_result.hasValue() && !edge_result.value());

    SlidingContactPermission diagonal_permission;
    diagonal_permission.vertex_mask = 0b011;
    const auto diagonal_result = hasInvalidSlidingIntersection(
        edge_touch, surface, diagonal_permission);
    assert(diagonal_result.hasValue() && diagonal_result.value());

    const TrianglePoints coplanar_area{{
        {0.1, 0.1, 0.0},
        {0.8, 0.1, 0.0},
        {0.1, 0.8, 0.0}}};
    const auto coplanar_result = hasInvalidSlidingIntersection(
        coplanar_area, surface, none);
    assert(coplanar_result.hasValue() && coplanar_result.value());
    SlidingContactPermission complete;
    complete.complete_face_exemption = true;
    const auto exempt_result = hasInvalidSlidingIntersection(
        coplanar_area, surface, complete);
    assert(exempt_result.hasValue() && !exempt_result.value());

    TrianglePoints non_finite = crossing;
    non_finite[0].x() = std::numeric_limits<Scalar>::quiet_NaN();
    assert(!hasInvalidSlidingIntersection(non_finite, surface, none).hasValue());

    const TrianglePoints degenerate{{
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {2.0, 0.0, 0.0}}};
    assert(!hasInvalidSlidingIntersection(degenerate, surface, none).hasValue());

    assert(slidingSideValuesStayOnOneSide(
        std::vector<Scalar>{0.0, 0.1, 0.2}, Scalar{1e-10}));
    assert(!slidingSideValuesStayOnOneSide(
        std::vector<Scalar>{-0.1, 0.0, 0.2}, Scalar{1e-10}));
    assert(!slidingSideValuesStayOnOneSide(
        std::vector<Scalar>{
            std::numeric_limits<Scalar>::infinity(), 0.2},
        Scalar{1e-10}));

    const std::array<std::vector<std::uint32_t>, 3> associations{{
        {10}, {10}, {11}}};
    const auto permissions = buildSlidingContactPermissions(
        associations, 0b111, std::vector<std::uint32_t>{11});
    assert(permissions.at(10).vertex_mask == 0b011);
    assert(permissions.at(10).edge_mask == 0b001);
    assert(!permissions.at(10).complete_face_exemption);
    assert(permissions.at(11).vertex_mask == 0b100);
    assert(permissions.at(11).edge_mask == 0);
    assert(permissions.at(11).complete_face_exemption);
}
