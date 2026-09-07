#include <cassert>
#include <map>
#include <set>

#include <boundary_mesh/spatial/sliding_intersection_index.hpp>

using namespace boundary_mesh;

int main()
{
    SurfaceMesh mesh;
    mesh.vertices = {
        {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {0.0, 1.0, 0.0},
        {0.0, -1.0, -1.0}, {0.0, 1.0, -1.0}, {0.0, 0.0, 1.0},
        {2.0, 2.0, 2.0}, {3.0, 2.0, 2.0}, {2.0, 3.0, 2.0}};
    mesh.faces = {
        Triangle{{0, 1, 2}}, Triangle{{3, 4, 5}}, Triangle{{6, 7, 8}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Symmetry, 10},
        {SurfaceBoundaryKind::Internal, 11},
        {SurfaceBoundaryKind::Wall, 12}};

    const auto built = SlidingIntersectionIndex::build(mesh);
    assert(built.hasValue());
    const SlidingIntersectionIndex &index = built.value();
    assert(index.primitiveCount() == 2);
    assert(index.hasRegion(10));
    assert(index.hasRegion(11));
    assert(!index.hasRegion(12));

    const TrianglePoints both{{
        {-0.5, 0.0, -0.5},
        {0.5, 0.0, 0.5},
        {0.5, 0.5, -0.5}}};
    const auto hit = index.query(both, {});
    assert(hit.hasValue() && hit.value().intersected);
    const std::uint32_t first_region = hit.value().region_id;
    const auto other = index.query(both, {}, {first_region});
    assert(other.hasValue() && other.value().intersected);
    assert(other.value().region_id != first_region);

    std::map<std::uint32_t, SlidingContactPermission> permissions;
    permissions[10].complete_face_exemption = true;
    const auto face_specific = index.query(both, permissions);
    assert(face_specific.hasValue() && face_specific.value().intersected);
    assert(face_specific.value().region_id == 11);

    const auto normal = index.faceNormalAtPoint(10, {0.0, 0.0, 0.0});
    assert(normal.hasValue());
    const auto positive = index.signedSideToRegion(
        10, {0.0, 0.0, 1.0}, normal.value());
    const auto negative = index.signedSideToRegion(
        10, {0.0, 0.0, -1.0}, normal.value());
    assert(positive.hasValue() && negative.hasValue());
    assert(positive.value() * negative.value() < 0.0);
    assert(!index.faceNormalAtPoint(99, {0.0, 0.0, 0.0}).hasValue());

    const TrianglePoints outside{{
        {10.0, 10.0, 10.0}, {11.0, 10.0, 10.0}, {10.0, 11.0, 10.0}}};
    const auto no_hit = index.query(outside, {});
    assert(no_hit.hasValue() && !no_hit.value().intersected);
}
