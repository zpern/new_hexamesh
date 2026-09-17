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
}
