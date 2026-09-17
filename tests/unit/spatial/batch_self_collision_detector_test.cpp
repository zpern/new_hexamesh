#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

#include <boundary_mesh/spatial/batch_self_collision_detector.hpp>

using namespace boundary_mesh;

namespace
{
    CollisionTriangle triangle(
        std::uint32_t owner,
        const Point3 &a,
        const Point3 &b,
        const Point3 &c)
    {
        CollisionTriangle value;
        value.points = {{a, b, c}};
        value.vertex_keys = {{{owner * 10 + 0, 1},
                              {owner * 10 + 1, 1},
                              {owner * 10 + 2, 1}}};
        value.owner_kind = CollisionOwnerKind::LayerCandidate;
        value.owner_id = owner;
        value.boundary_points = {{a, b, c, c}};
        value.boundary_vertex_keys = {{{owner * 10 + 0, 1},
                                       {owner * 10 + 1, 1},
                                       {owner * 10 + 2, 1},
                                       {owner * 10 + 2, 1}}};
        value.boundary_vertex_count = 3;
        return value;
    }
}

int main()
{
    const auto empty = BatchSelfCollisionDetector::detect({});
    assert(empty.hasValue());
    assert(empty.value().illegal_owner_ids.empty());
    assert(empty.value().diagnostics.triangle_count == 0);

    const CollisionTriangle horizontal = triangle(
        4,
        {-1.0, -1.0, 0.0},
        {1.0, -1.0, 0.0},
        {0.0, 1.0, 0.0});
    const CollisionTriangle vertical = triangle(
        2,
        {0.0, -0.5, -1.0},
        {0.0, 0.5, 1.0},
        {0.0, 0.5, -1.0});
    const CollisionTriangle distant = triangle(
        9,
        {5.0, 5.0, 5.0},
        {6.0, 5.0, 5.0},
        {5.0, 6.0, 5.0});

    const auto separated = BatchSelfCollisionDetector::detect(
        {horizontal, distant});
    assert(separated.hasValue());
    assert(separated.value().illegal_owner_ids.empty());
    assert(separated.value().diagnostics.exact_tests == 0);

    const auto intersecting = BatchSelfCollisionDetector::detect(
        {horizontal, vertical, distant});
    assert(intersecting.hasValue());
    assert((intersecting.value().illegal_owner_ids ==
            std::vector<std::uint32_t>{2, 4}));
    assert(intersecting.value().diagnostics.unique_pairs >= 1);
    assert(intersecting.value().diagnostics.exact_tests <=
           intersecting.value().diagnostics.unique_pairs);

    CollisionTriangle same_owner = vertical;
    same_owner.owner_id = horizontal.owner_id;
    const auto internal = BatchSelfCollisionDetector::detect(
        {horizontal, same_owner});
    assert(internal.hasValue());
    assert(internal.value().illegal_owner_ids.empty());
    assert(internal.value().diagnostics.same_owner_skips == 1);
    assert(internal.value().diagnostics.exact_tests == 0);

    const auto three_way = BatchSelfCollisionDetector::detect(
        {horizontal,
         vertical,
         triangle(
             7,
             {-0.5, 0.0, -1.0},
             {0.5, 0.0, 1.0},
             {0.5, 0.0, -1.0})});
    assert(three_way.hasValue());
    assert(std::is_sorted(
        three_way.value().illegal_owner_ids.begin(),
        three_way.value().illegal_owner_ids.end()));
    assert(std::adjacent_find(
               three_way.value().illegal_owner_ids.begin(),
               three_way.value().illegal_owner_ids.end()) ==
           three_way.value().illegal_owner_ids.end());

    CollisionTriangle invalid = horizontal;
    invalid.points[0].x() = std::numeric_limits<Scalar>::quiet_NaN();
    const auto invalid_result = BatchSelfCollisionDetector::detect({invalid});
    assert(!invalid_result.hasValue());
    assert(invalid_result.error() == SpatialError::NonFiniteCoordinate);
}
