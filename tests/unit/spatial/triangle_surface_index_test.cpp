#include <cmath>
#include <vector>

#include <boundary_mesh/spatial/triangle_surface_index.hpp>

int main()
{
    using namespace boundary_mesh;

    const std::vector<SurfaceTriangle> triangles{{
        {Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{0, 1, 0}},
        SurfaceFaceId{7}, 0}};
    const auto index = TriangleSurfaceIndex::build(triangles);
    if (!index.hasValue()) return 1;

    const auto interior = index.value().closestPoint(Point3{0.2, 0.3, 2.0});
    if (!interior.hasValue() ||
        (interior.value().point - Point3{0.2, 0.3, 0.0}).norm() > 1e-12 ||
        interior.value().source_face_id != SurfaceFaceId{7} ||
        std::abs(interior.value().unit_normal.z() - 1.0) > 1e-12)
        return 2;

    const auto vertex = index.value().closestPoint(Point3{-1, -1, 0});
    if (!vertex.hasValue() || vertex.value().point.norm() > 1e-12)
        return 3;

    const std::vector<SurfaceTriangle> tied{
        {{Point3{0, 0, -1}, Point3{1, 0, -1}, Point3{0, 1, -1}}, 9, 0},
        {{Point3{0, 0, 1}, Point3{1, 0, 1}, Point3{0, 1, 1}}, 4, 2}};
    const auto tie_index = TriangleSurfaceIndex::build(tied);
    const auto tie = tie_index.value().closestPoint(Point3{0.2, 0.2, 0});
    if (!tie.hasValue() || tie.value().source_face_id != SurfaceFaceId{4} ||
        tie.value().local_triangle_id != 2)
        return 4;

    const auto degenerate = TriangleSurfaceIndex::build({{
        {Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{2, 0, 0}}, 1, 0}});
    if (degenerate.hasValue() ||
        degenerate.error() != SpatialError::DegenerateTriangle)
        return 5;

    const auto invalid_query = index.value().closestPoint(
        Point3{NAN, 0, 0});
    if (invalid_query.hasValue() ||
        invalid_query.error() != SpatialError::NonFiniteCoordinate)
        return 6;
}
