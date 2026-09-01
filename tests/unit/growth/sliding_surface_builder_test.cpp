#include <cmath>
#include <variant>

#include <boundary_mesh/growth/sliding_surface_builder.hpp>

int main()
{
    using namespace boundary_mesh;
    SurfaceMesh mesh;
    mesh.vertices = {
        {0,0,0}, {0,1,0}, {0,0,1},
        {2,0,0}, {3,0,0}, {2,0,1},
        {0,0,3}, {1,0,3}, {0,1,3},
        {4,0,0}, {5,0,1}, {4,1,2}};
    mesh.faces = {
        Triangle{{0,1,2}}, Triangle{{3,4,5}},
        Triangle{{6,7,8}}, Triangle{{9,10,11}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Symmetry, 10},
        {SurfaceBoundaryKind::Internal, 11},
        {SurfaceBoundaryKind::Symmetry, 12},
        {SurfaceBoundaryKind::Internal, 13}};

    const auto result = SlidingSurfaceBuilder{}.build(mesh);
    if (!result.hasValue() || result.value().surfaces().size() != 4) return 1;
    if (result.value().find(10)->kind != SlidingSurfaceKind::AxisX) return 2;
    if (result.value().find(11)->kind != SlidingSurfaceKind::AxisY) return 3;
    if (result.value().find(12)->kind != SlidingSurfaceKind::AxisZ) return 4;
    const SlidingSurface *curved = result.value().find(13);
    if (curved == nullptr || curved->kind != SlidingSurfaceKind::Curved ||
        curved->boundary_kind != SurfaceBoundaryKind::Internal ||
        !curved->curved_index.has_value()) return 5;
    if (std::abs(result.value().find(11)->axis_value) > 1e-12) return 6;

    SurfaceMesh conflict = mesh;
    conflict.faces.push_back(Triangle{{0,1,2}});
    conflict.face_tags.push_back({SurfaceBoundaryKind::Internal, 10});
    const auto invalid = SlidingSurfaceBuilder{}.build(conflict);
    if (invalid.hasValue() ||
        std::get_if<SlidingInputMismatch>(&invalid.error()) == nullptr)
        return 7;
}
