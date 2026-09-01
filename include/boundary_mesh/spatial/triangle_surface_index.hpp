#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/spatial/binary_aabb_tree.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>

namespace boundary_mesh
{
    struct SurfaceTriangle
    {
        std::array<Point3, 3> points{};
        SurfaceFaceId source_face_id{};
        std::uint32_t local_triangle_id{};
    };

    struct ClosestSurfacePoint
    {
        Point3 point{Point3::Zero()};
        Vector3 unit_normal{Vector3::Zero()};
        Scalar squared_distance{};
        SurfaceFaceId source_face_id{};
        std::uint32_t local_triangle_id{};
    };

    class TriangleSurfaceIndex
    {
    public:
        static Result<TriangleSurfaceIndex, SpatialError> build(
            std::vector<SurfaceTriangle> triangles);

        Result<ClosestSurfacePoint, SpatialError> closestPoint(
            const Point3 &query) const;

    private:
        std::vector<SurfaceTriangle> triangles_;
        BinaryAabbTree tree_;
    };
}
