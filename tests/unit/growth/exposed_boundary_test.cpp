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
    assert(mode_tracker.apply(seed).hasValue());
    assert(mode_tracker.lastApplyDiagnostics().bulk_rebuild);
    assert(mode_tracker.lastApplyDiagnostics().inserted_groups == 5);

    const BoundaryFaceKey erased_key =
        makeBoundaryFaceKey(offsetTriangleFace(1)).value();
    ExposedBoundaryUpdate exact_twenty_percent;
    exact_twenty_percent.erase_faces.push_back(erased_key);
    exact_twenty_percent.insert_faces.push_back(offsetTriangleFace(10));
    assert(mode_tracker.apply(exact_twenty_percent).hasValue());
    assert(mode_tracker.lastApplyDiagnostics().bulk_rebuild);
    assert(mode_tracker.lastApplyDiagnostics().erased_groups == 1);
    assert(mode_tracker.lastApplyDiagnostics().inserted_groups == 1);

    ExposedBoundaryUpdate add_sixth;
    add_sixth.insert_faces.push_back(offsetTriangleFace(11));
    assert(mode_tracker.apply(add_sixth).hasValue());
    assert(mode_tracker.lastApplyDiagnostics().bulk_rebuild);
    assert(mode_tracker.faceCount() == 6);
    ExposedBoundaryUpdate below_twenty_percent;
    below_twenty_percent.erase_faces.push_back(
        makeBoundaryFaceKey(offsetTriangleFace(2)).value());
    below_twenty_percent.insert_faces.push_back(offsetTriangleFace(12));
    assert(mode_tracker.apply(below_twenty_percent).hasValue());
    assert(!mode_tracker.lastApplyDiagnostics().bulk_rebuild);

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
}
