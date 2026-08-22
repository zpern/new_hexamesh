#include <cassert>

#include <boundary_mesh/growth/exposed_boundary.hpp>
#include <boundary_mesh/growth/farfield_boundary_builder.hpp>

using namespace boundary_mesh;

int main()
{
    SurfaceMesh original;
    original.vertices = {
        {0, 0, 0}, {1, 0, 0}, {0, 1, 0},
        {0, 0, 2}, {1, 0, 2}, {0, 1, 2}};
    original.faces = {
        Triangle{{0, 1, 2}},
        Triangle{{3, 4, 5}}};
    original.face_tags = {
        {SurfaceBoundaryKind::Wall, 9},
        {SurfaceBoundaryKind::Farfield, 4}};

    const BoundaryFace bottom{
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
        {{0, 0}, {1, 0}, {2, 0}},
        0,
        9};
    const BoundaryFace top{
        {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}},
        {{0, 1}, {1, 1}, {2, 1}},
        0,
        9};

    ExposedBoundaryTracker tracker;
    const auto update = tracker.prepare({{bottom, top}});
    assert(update.hasValue());
    tracker.apply(update.value());

    const auto result = buildFarfieldBoundary(original, tracker);
    assert(result.hasValue());
    assert(result.value().faces.size() == 5);
    assert(result.value().face_tags.front().kind ==
           SurfaceBoundaryKind::Farfield);
    for (std::size_t index = 1;
         index < result.value().face_tags.size();
         ++index)
    {
        assert(result.value().face_tags[index].kind ==
               SurfaceBoundaryKind::BoundaryLayerInterface);
        assert(result.value().face_tags[index].region_id == 9);
    }
}
