#include <cassert>

#include <boundary_mesh/spatial/aabb.hpp>

using namespace boundary_mesh;

int main()
{
    const Aabb unit{
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 1.0, 1.0}};

    assert(!overlaps(
        unit,
        Aabb{
            Point3{2.0, 0.0, 0.0},
            Point3{3.0, 1.0, 1.0}}));

    assert(overlaps(
        unit,
        Aabb{
            Point3{0.5, 0.5, 0.5},
            Point3{2.0, 2.0, 2.0}}));

    assert(overlaps(
        unit,
        Aabb{
            Point3{1.0, 0.0, 0.0},
            Point3{2.0, 1.0, 1.0}}));

    assert(overlaps(
        unit,
        Aabb{
            Point3{1.0, 1.0, 0.0},
            Point3{2.0, 2.0, 1.0}}));

    assert(overlaps(
        unit,
        Aabb{
            Point3{1.0, 1.0, 1.0},
            Point3{2.0, 2.0, 2.0}}));
}
