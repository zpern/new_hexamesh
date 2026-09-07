#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/spatial/binary_aabb_tree.hpp>
#include <boundary_mesh/spatial/sliding_intersection.hpp>

namespace boundary_mesh
{
    struct IndexedSlidingTriangle
    {
        TrianglePoints points{};
        SurfaceBoundaryKind boundary_kind{SurfaceBoundaryKind::Symmetry};
        std::uint32_t region_id{};
        SurfaceFaceId source_face_id{};
        std::uint32_t local_triangle_id{};
    };

    struct SlidingIntersectionHit
    {
        bool intersected{};
        std::uint32_t region_id{};
        std::size_t primitive_id{};
    };

    class SlidingIntersectionIndex
    {
    public:
        static Result<SlidingIntersectionIndex, SpatialError> build(
            const SurfaceMesh &mesh);

        Result<SlidingIntersectionHit, SpatialError> query(
            const TrianglePoints &triangle,
            const std::map<std::uint32_t, SlidingContactPermission> &permissions,
            const std::set<std::uint32_t> &ignored_regions = {}) const;

        Result<Vector3, SpatialError> faceNormalAtPoint(
            std::uint32_t region_id,
            const Point3 &point) const;

        Result<Scalar, SpatialError> signedSideToRegion(
            std::uint32_t region_id,
            const Point3 &point,
            const Vector3 &reference_normal) const;

        std::size_t primitiveCount() const noexcept;
        bool hasRegion(std::uint32_t region_id) const noexcept;

    private:
        std::vector<IndexedSlidingTriangle> triangles_;
        BinaryAabbTree tree_;
        std::set<std::uint32_t> region_ids_;
    };
}
