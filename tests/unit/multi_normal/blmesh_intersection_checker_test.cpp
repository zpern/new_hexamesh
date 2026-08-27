#include <boundary_mesh/multi_normal/blmesh_intersection_checker.hpp>

int main()
{
    using namespace boundary_mesh;
    const TrianglePoints base{
        Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{0, 1, 0}};
    if (blmeshTrianglesIntersect(base, base)) return 1;

    const TrianglePoints shared_edge{
        Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{0, -1, 0}};
    if (blmeshTrianglesIntersect(base, shared_edge)) return 2;

    const TrianglePoints shared_vertex_clear{
        Point3{0, 0, 0}, Point3{-1, 0, 1}, Point3{0, -1, 1}};
    if (blmeshTrianglesIntersect(base, shared_vertex_clear)) return 3;

    const TrianglePoints proper{
        Point3{0.25, 0.25, -1}, Point3{0.25, 0.25, 1},
        Point3{0.75, 0.25, 0}};
    if (!blmeshTrianglesIntersect(base, proper)) return 4;
    return 0;
}
