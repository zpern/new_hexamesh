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
    IncrementalCollisionIndexOptions overlap_options;
    overlap_options.target_leaf_capacity = 1;
    overlap_options.maximum_depth = 6;
    std::vector<CollisionPrimitiveGroup> overlapping_leaves;
    overlapping_leaves.push_back({100, {horizontalTriangle()}});
    for (std::uint32_t id = 0; id < 12; ++id)
    {
        const Scalar x = (id & 1) ? Scalar{0.8} : Scalar{-0.8};
        const Scalar y = (id & 2) ? Scalar{0.8} : Scalar{-0.8};
        overlapping_leaves.push_back({101 + id,
            {tinyTriangle({x,y,Scalar(id % 3) * Scalar{0.01}}, id)}});
    }
    auto overlapping_index = IncrementalCollisionIndex::build(
        std::move(overlapping_leaves), overlap_options);
    if (!overlapping_index.hasValue()) return 42;
    const Aabb all_overlaps{
        Point3{-2,-2,-1}, Point3{2,2,1}};
    const auto overlap_candidates =
        overlapping_index.value().queryCandidates(all_overlaps);
    const auto repeated_overlap_candidates =
        overlapping_index.value().queryCandidates(all_overlaps);
    auto sorted_unique_candidates = overlap_candidates;
    std::sort(sorted_unique_candidates.begin(), sorted_unique_candidates.end());
    sorted_unique_candidates.erase(std::unique(
        sorted_unique_candidates.begin(), sorted_unique_candidates.end()),
        sorted_unique_candidates.end());
    if (overlap_candidates != repeated_overlap_candidates ||
        overlap_candidates.size() != sorted_unique_candidates.size() ||
        overlapping_index.value().diagnostics().duplicate_candidate_visits == 0)
        return 43;

    const auto one_pass_pairs = IncrementalCollisionIndex::build({
        {100, {horizontalTriangle()}},
        {200, {verticalTriangle()}}});
    if (!one_pass_pairs.hasValue()) return 40;
    const auto forward_pair = one_pass_pairs.value().queryIllegalContactsAfter(
        horizontalTriangle(), 0);
    const auto reverse_pair = one_pass_pairs.value().queryIllegalContactsAfter(
        verticalTriangle(), 1);
    std::vector<CollisionPrimitiveId> reusable_candidates{99};
    std::vector<CollisionPrimitiveId> reusable_contacts{99};
    one_pass_pairs.value().queryIllegalContactsAfter(
        horizontalTriangle(), 0, reusable_candidates, reusable_contacts);
    const auto full_forward_pair =
        one_pass_pairs.value().queryIllegalContacts(
            horizontalTriangle(), CollisionGroupId{100});
    const auto full_reverse_pair =
        one_pass_pairs.value().queryIllegalContacts(
            verticalTriangle(), CollisionGroupId{200});
    if (forward_pair != full_forward_pair ||
        reusable_contacts != forward_pair ||
        reusable_candidates.empty() ||
        forward_pair.size() != 1 ||
        forward_pair.front() != 1 || !reverse_pair.empty() ||
        full_reverse_pair.size() != 1 || full_reverse_pair.front() != 0)
        return 41;
    one_pass_pairs.value().queryIllegalContactsAfter(
        shiftedTriangle(100.0, 9), 0, reusable_candidates,
        reusable_contacts);
    if (!reusable_contacts.empty() || !reusable_candidates.empty()) return 44;

    std::vector<CollisionPrimitiveGroup> spread;
    for (std::uint32_t id = 0; id < 100; ++id)
        spread.push_back({id, {shiftedTriangle(id * 10.0, id)}});
    const auto bulk = IncrementalCollisionIndex::build(std::move(spread));
    if (!bulk.hasValue() || bulk.value().diagnostics().tree_builds != 1 ||
        bulk.value().diagnostics().active_primitives != 100)
        return 1;
    const auto duplicate = IncrementalCollisionIndex::build(
        {{1, {horizontalTriangle()}}, {1, {horizontalTriangle()}}});
    if (duplicate.hasValue()) return 2;
    const auto empty = IncrementalCollisionIndex::build({});
    assert(empty.hasValue());
    auto index = std::move(empty.value());

    const CollisionGroupId group{7};
    const auto horizontal_inserted =
        index.insertGroup({group, {horizontalTriangle()}});
    if (!horizontal_inserted.hasValue() ||
        index.queryIllegalContacts(verticalTriangle()).size() != 1)
        return 21;
    if (!index.queryIllegalContacts(
            verticalTriangle(), std::set<CollisionGroupId>{group}).empty())
        return 20;
    const auto distant_inserted = index.insertGroup(
        {8, {shiftedTriangle(100.0, 8)}});
    if (!distant_inserted.hasValue()) return 22;
    const Aabb broad_bounds{
        Point3{-200, -200, -200}, Point3{200, 200, 200}};
    const auto broad_candidates = index.queryCandidates(broad_bounds);
    const auto exact_tests_before = index.diagnostics().exact_tests;
    const auto cached_contacts = index.queryIllegalContacts(
        verticalTriangle(), broad_candidates, std::set<CollisionGroupId>{});
    if (cached_contacts.size() != 1 ||
        index.diagnostics().exact_tests != exact_tests_before + 1)
        return 23;

    const auto horizontal_erased = index.eraseGroup(group);
    if (!horizontal_erased.hasValue() ||
        !index.queryIllegalContacts(verticalTriangle()).empty())
        return 24;

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
