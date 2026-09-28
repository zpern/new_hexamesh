#include <algorithm>
#include <cassert>

#include <boundary_mesh/growth/exposed_boundary.hpp>

using namespace boundary_mesh;

namespace
{
    BoundaryFace triangleFace(
        std::initializer_list<Point3> points,
        std::initializer_list<CollisionVertexKey> keys,
        SurfaceFaceId source_face_id)
    {
        return BoundaryFace{
            std::vector<Point3>(points),
            std::vector<CollisionVertexKey>(keys),
            source_face_id,
            1};
    }

    BoundaryFace offsetTriangleFace(std::uint32_t id)
    {
        const Scalar x = static_cast<Scalar>(id) * Scalar{2};
        return triangleFace(
            {{x, 0, 0}, {x + 1, 0, 0}, {x, 1, 0}},
            {{id * 3, 0}, {id * 3 + 1, 0}, {id * 3 + 2, 0}},
            id);
    }
}

int main()
{
    SurfaceMesh initial_walls;
    initial_walls.vertices = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
        {10, 0, 0}, {11, 0, 0}, {10, 1, 0}};
    initial_walls.faces = {Triangle{{0, 1, 2}}, Triangle{{3, 4, 5}}};
    initial_walls.face_tags = {
        {SurfaceBoundaryKind::Wall, 1},
        {SurfaceBoundaryKind::Wall, 1}};
    ExposedBoundaryTracker wall_boundary;
    assert(wall_boundary.initializeWallSurface(initial_walls).hasValue());
    assert(wall_boundary.collisionIndex().diagnostics().active_primitives == 2);
    assert(wall_boundary.faces().empty());

    CollisionTriangle ungrown_wall_probe;
    ungrown_wall_probe.points = {{{10.2, 0.2, 0},
                                  {10.6, 0.2, 0},
                                  {10.2, 0.6, 0}}};
    ungrown_wall_probe.vertex_keys = {{{100, 0}, {101, 0}, {102, 0}}};
    assert(!wall_boundary.collisionIndex()
                .queryIllegalContacts(ungrown_wall_probe).empty());

    const LayerBoundaryCandidate grown_wall{
        triangleFace(
            {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
            {{0, 0}, {1, 0}, {2, 0}}, 0),
        triangleFace(
            {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}},
            {{0, 1}, {1, 1}, {2, 1}}, 0)};
    const auto wall_update = wall_boundary.prepare({grown_wall});
    assert(wall_update.hasValue());
    assert(wall_boundary.apply(wall_update.value()).hasValue());
    CollisionTriangle covered_wall_probe;
    covered_wall_probe.points = {{{0.2, 0.2, 0},
                                  {0.6, 0.2, 0},
                                  {0.2, 0.6, 0}}};
    covered_wall_probe.vertex_keys = {{{103, 0}, {104, 0}, {105, 0}}};
    assert(wall_boundary.collisionIndex()
               .queryIllegalContacts(covered_wall_probe).empty());
    assert(wall_boundary.collisionIndex().diagnostics().active_primitives == 8);
    assert(wall_boundary.faces().size() == 4);

    const LayerBoundaryCandidate first{
        triangleFace(
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}},
            {{0, 0}, {1, 0}, {2, 0}},
            0),
        triangleFace(
            {{0, 0, 1}, {1, 0, 1}, {1, 1, 1}},
            {{0, 1}, {1, 1}, {2, 1}},
            0),
        {{SurfaceBoundaryKind::Symmetry, 7},
         {SurfaceBoundaryKind::BoundaryLayerInterface, 1},
         {SurfaceBoundaryKind::BoundaryLayerInterface, 1}}};
    const LayerBoundaryCandidate second{
        triangleFace(
            {{0, 0, 0}, {1, 1, 0}, {0, 1, 0}},
            {{0, 0}, {2, 0}, {3, 0}},
            1),
        triangleFace(
            {{0, 0, 1}, {1, 1, 1}, {0, 1, 1}},
            {{0, 1}, {2, 1}, {3, 1}},
            1)};

    ExposedBoundaryTracker tracker;
    const auto first_update = tracker.prepare({first});
    assert(first_update.hasValue());
    assert(tracker.apply(first_update.value()).hasValue());
    assert(tracker.collisionIndex().diagnostics().root_expansions == 0);
    assert(tracker.faceCount() == 4);
    assert(std::any_of(
        tracker.faces().begin(), tracker.faces().end(),
        [](const BoundaryFace &face)
        {
            return face.boundary_kind == SurfaceBoundaryKind::Symmetry &&
                   face.region_id == 7;
        }));

    const auto second_update = tracker.prepare({second});
    assert(second_update.hasValue());
    assert(tracker.apply(second_update.value()).hasValue());
    assert(tracker.faceCount() == 6);
    assert(tracker.collisionIndex().diagnostics().inactive_primitives == 0);

    const auto triangles = tracker.collisionTriangles();
    assert(triangles.hasValue());
    assert(triangles.value().size() == 8);

    CollisionTriangle probe;
    probe.points = {{{0.5, -0.5, 0.5},
                     {0.5, 1.5, 0.5},
                     {0.5, 0.5, 1.5}}};
    probe.vertex_keys = {{{100, 0}, {101, 0}, {102, 0}}};
    const auto fresh = CollisionIndex::build(triangles.value());
    assert(fresh.hasValue());
    assert(tracker.collisionIndex().queryIllegalContacts(probe).size() ==
           fresh.value().queryIllegalContacts(probe).size());

    const std::size_t faces_before = tracker.faceCount();
    const std::size_t primitives_before =
        tracker.collisionIndex().diagnostics().active_primitives;
    BoundaryFace degenerate = triangleFace(
        {{0, 0, 0}, {1, 0, 0}, {1, 0, 0}},
        {{20, 0}, {21, 0}, {22, 0}}, 20);
    ExposedBoundaryUpdate invalid;
    invalid.insert_faces.push_back(std::move(degenerate));
    const auto rejected = tracker.apply(invalid);
    assert(!rejected.hasValue());
    assert(rejected.error() == SpatialError::DegenerateTriangle);
    assert(tracker.faceCount() == faces_before);
    assert(tracker.collisionIndex().diagnostics().active_primitives ==
           primitives_before);

    ExposedBoundaryTracker mode_tracker;
    ExposedBoundaryUpdate seed;
    for (std::uint32_t id = 1; id <= 5; ++id)
        seed.insert_faces.push_back(offsetTriangleFace(id));
    const auto seeded_mode_tracker = mode_tracker.apply(seed);
    if (!seeded_mode_tracker.hasValue()) return 10;
    assert(mode_tracker.lastApplyDiagnostics().bulk_rebuild);
    assert(mode_tracker.lastApplyDiagnostics().inserted_groups == 5);

    const BoundaryFaceKey erased_key =
        makeBoundaryFaceKey(offsetTriangleFace(1)).value();
    ExposedBoundaryUpdate exact_twenty_percent;
    exact_twenty_percent.erase_faces.push_back(erased_key);
    exact_twenty_percent.insert_faces.push_back(offsetTriangleFace(10));
    const auto exact_twenty_percent_result =
        mode_tracker.apply(exact_twenty_percent);
    if (!exact_twenty_percent_result.hasValue()) return 11;
    assert(mode_tracker.lastApplyDiagnostics().bulk_rebuild);
    assert(mode_tracker.lastApplyDiagnostics().erased_groups == 1);
    assert(mode_tracker.lastApplyDiagnostics().inserted_groups == 1);

    ExposedBoundaryUpdate add_sixth;
    add_sixth.insert_faces.push_back(offsetTriangleFace(11));
    const auto added_sixth = mode_tracker.apply(add_sixth);
    if (!added_sixth.hasValue()) return 12;
    assert(mode_tracker.lastApplyDiagnostics().bulk_rebuild);
    assert(mode_tracker.faceCount() == 6);
    ExposedBoundaryUpdate below_twenty_percent;
    below_twenty_percent.erase_faces.push_back(
        makeBoundaryFaceKey(offsetTriangleFace(2)).value());
    below_twenty_percent.insert_faces.push_back(offsetTriangleFace(12));
    const auto below_threshold = mode_tracker.apply(below_twenty_percent);
    if (!below_threshold.hasValue()) return 13;
    assert(!mode_tracker.lastApplyDiagnostics().bulk_rebuild);

    const auto tree_builds_before_tombstone_threshold =
        mode_tracker.collisionIndex().diagnostics().tree_builds;
    const std::array<std::uint32_t, 3> replaced_ids{{3, 4, 5}};
    for (std::size_t index = 0; index < replaced_ids.size(); ++index)
    {
        ExposedBoundaryUpdate incremental_replace;
        incremental_replace.erase_faces.push_back(
            makeBoundaryFaceKey(offsetTriangleFace(replaced_ids[index]))
                .value());
        BoundaryFace replacement = offsetTriangleFace(replaced_ids[index]);
        replacement.source_face_id =
            20 + static_cast<std::uint32_t>(index);
        incremental_replace.insert_faces.push_back(std::move(replacement));
        const auto applied = mode_tracker.apply(incremental_replace);
        if (!applied.hasValue() ||
            mode_tracker.lastApplyDiagnostics().bulk_rebuild)
            return 14;
    }
    if (mode_tracker.collisionIndex().diagnostics().tree_builds <=
        tree_builds_before_tombstone_threshold)
        return 15;

    const std::vector<BoundaryFace> before_missing = mode_tracker.faces();
    ExposedBoundaryUpdate missing;
    missing.erase_faces.push_back(
        makeBoundaryFaceKey(offsetTriangleFace(99)).value());
    const auto missing_result = mode_tracker.apply(missing);
    assert(!missing_result.hasValue());
    assert(missing_result.error() == SpatialError::MissingPrimitiveGroup);
    assert(mode_tracker.faces().size() == before_missing.size());
    for (const BoundaryFace &face : before_missing)
        assert(mode_tracker.contains(makeBoundaryFaceKey(face).value()));

    const std::size_t count_before_duplicate = mode_tracker.faceCount();
    ExposedBoundaryUpdate duplicate;
    duplicate.insert_faces.push_back(offsetTriangleFace(3));
    const auto duplicate_result = mode_tracker.apply(duplicate);
    assert(!duplicate_result.hasValue());
    assert(duplicate_result.error() == SpatialError::InvalidTopologyReference);
    assert(mode_tracker.faceCount() == count_before_duplicate);

    ExposedBoundaryTracker unified_tracker;
    CollisionTriangle historical_transition;
    historical_transition.points = {{{40, 0, 0}, {41, 0, 0}, {40, 1, 0}}};
    historical_transition.vertex_keys = {{{400, 2}, {401, 2}, {402, 2}}};
    historical_transition.owner_kind = CollisionOwnerKind::LayerCandidate;
    historical_transition.owner_id = 0;
    historical_transition.boundary_points[0] = historical_transition.points[0];
    historical_transition.boundary_points[1] = historical_transition.points[1];
    historical_transition.boundary_points[2] = historical_transition.points[2];
    historical_transition.boundary_vertex_keys = {
        historical_transition.vertex_keys[0],
        historical_transition.vertex_keys[1],
        historical_transition.vertex_keys[2], {}};
    historical_transition.boundary_vertex_count = 3;
    assert(unified_tracker.appendTransitionTriangles(
        {historical_transition}).hasValue());
    CollisionTriangle transition_probe = historical_transition;
    transition_probe.vertex_keys = {{{500, 3}, {501, 3}, {502, 3}}};
    assert(!unified_tracker.collisionIndex()
                .queryIllegalContacts(transition_probe).empty());
    assert(unified_tracker.faces().empty());

    ExposedBoundaryUpdate regular_faces;
    for (std::uint32_t id = 20; id < 25; ++id)
        regular_faces.insert_faces.push_back(offsetTriangleFace(id));
    assert(unified_tracker.apply(regular_faces).hasValue());
    assert(unified_tracker.lastApplyDiagnostics().bulk_rebuild);
    assert(!unified_tracker.collisionIndex()
                .queryIllegalContacts(transition_probe).empty());
    assert(unified_tracker.faces().size() == 5);
}
