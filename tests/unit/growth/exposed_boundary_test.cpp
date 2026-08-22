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
            0)};
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
    tracker.apply(first_update.value());
    assert(tracker.faceCount() == 4);

    const auto second_update = tracker.prepare({second});
    assert(second_update.hasValue());
    tracker.apply(second_update.value());
    assert(tracker.faceCount() == 6);

    const auto triangles = tracker.collisionTriangles();
    assert(triangles.hasValue());
    assert(triangles.value().size() == 10);
}
