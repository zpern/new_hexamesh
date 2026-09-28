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

    CollisionOwnerTriangles owner(
        std::uint32_t id,
        std::initializer_list<CollisionTriangle> triangles)
    {
        CollisionOwnerTriangles result;
        result.owner_id = id;
        result.triangles.assign(triangles);
        const CollisionTriangle &first = result.triangles.front();
        result.bounds = makeAabb(
            first.points[0], first.points[1], first.points[2]).value();
        for (const CollisionTriangle &value : result.triangles)
        {
            const Aabb box = makeAabb(
                value.points[0], value.points[1], value.points[2]).value();
            result.bounds.minimum = result.bounds.minimum.cwiseMin(box.minimum);
            result.bounds.maximum = result.bounds.maximum.cwiseMax(box.maximum);
        }
        return result;
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

    const auto owner_separated = BatchSelfCollisionDetector::detectOwners({
        owner(4, {horizontal}), owner(9, {distant})});
    assert(owner_separated.hasValue());
    assert(owner_separated.value().diagnostics.exact_tests == 0);

    const auto owner_intersecting = BatchSelfCollisionDetector::detectOwners({
        owner(4, {horizontal, horizontal}), owner(2, {vertical, vertical})});
    assert(owner_intersecting.hasValue());
    assert((owner_intersecting.value().illegal_owner_ids ==
            std::vector<std::uint32_t>{2, 4}));
    assert(owner_intersecting.value().diagnostics.owner_pairs == 1);

    const auto reverse_order = BatchSelfCollisionDetector::detectOwners({
        owner(2, {vertical}), owner(4, {horizontal})});
    assert(reverse_order.hasValue());
    assert(reverse_order.value().illegal_owner_ids ==
           owner_intersecting.value().illegal_owner_ids);
    assert(reverse_order.value().diagnostics.owner_pairs == 1);

    const std::vector<CollisionOwnerTriangles> changed_owner_set{
        owner(4, {horizontal}), owner(2, {vertical}), owner(9, {distant})};
    const auto persistent_index = BatchSelfCollisionIndex::build(
        changed_owner_set,
        {changed_owner_set[0].bounds,
         changed_owner_set[1].bounds,
         changed_owner_set[2].bounds});
    assert(persistent_index.hasValue());
    const auto persistent_changed = persistent_index.value().detectChanged(
        changed_owner_set, {0});
    assert(persistent_changed.hasValue());
    assert((persistent_changed.value().illegal_owner_ids ==
            std::vector<std::uint32_t>{2, 4}));
    assert(persistent_changed.value().diagnostics.exact_tests == 1);
    Aabb too_small = changed_owner_set[0].bounds;
    too_small.maximum.x() = too_small.minimum.x();
    const auto invalid_persistent_index = BatchSelfCollisionIndex::build(
        changed_owner_set,
        {too_small,
         changed_owner_set[1].bounds,
         changed_owner_set[2].bounds});
    assert(!invalid_persistent_index.hasValue());
    assert(invalid_persistent_index.error() == SpatialError::InvalidAabb);

    CollisionOwnerTriangles invalid_owner = owner(4, {horizontal});
    invalid_owner.bounds.minimum.x() =
        std::numeric_limits<Scalar>::quiet_NaN();
    const auto invalid_owner_result =
        BatchSelfCollisionDetector::detectOwners({invalid_owner});
    assert(!invalid_owner_result.hasValue());
    assert(invalid_owner_result.error() == SpatialError::InvalidAabb);
}
