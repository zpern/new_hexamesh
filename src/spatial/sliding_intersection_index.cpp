#include <boundary_mesh/spatial/sliding_intersection_index.hpp>

#include <algorithm>
#include <limits>
#include <type_traits>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
        Point3 closestPointOnTriangle(
            const Point3 &point,
            const TrianglePoints &triangle)
        {
            const Point3 &a = triangle[0];
            const Point3 &b = triangle[1];
            const Point3 &c = triangle[2];
            const Vector3 ab = b - a;
            const Vector3 ac = c - a;
            const Vector3 ap = point - a;
            const Scalar d1 = ab.dot(ap);
            const Scalar d2 = ac.dot(ap);
            if (d1 <= 0 && d2 <= 0) return a;
            const Vector3 bp = point - b;
            const Scalar d3 = ab.dot(bp);
            const Scalar d4 = ac.dot(bp);
            if (d3 >= 0 && d4 <= d3) return b;
            const Scalar vc = d1 * d4 - d3 * d2;
            if (vc <= 0 && d1 >= 0 && d3 <= 0)
                return a + d1 / (d1 - d3) * ab;
            const Vector3 cp = point - c;
            const Scalar d5 = ab.dot(cp);
            const Scalar d6 = ac.dot(cp);
            if (d6 >= 0 && d5 <= d6) return c;
            const Scalar vb = d5 * d2 - d1 * d6;
            if (vb <= 0 && d2 >= 0 && d6 <= 0)
                return a + d2 / (d2 - d6) * ac;
            const Scalar va = d3 * d6 - d5 * d4;
            if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0)
                return b + (d4 - d3) /
                    ((d4 - d3) + (d5 - d6)) * (c - b);
            const Scalar inverse = Scalar{1} / (va + vb + vc);
            return a + vb * inverse * ab + vc * inverse * ac;
        }

        template <typename Face>
        Result<std::monostate, SpatialError> appendFace(
            const SurfaceMesh &mesh,
            const Face &face,
            const SurfaceBoundaryTag &tag,
            SurfaceFaceId source_face_id,
            std::vector<IndexedSlidingTriangle> &output)
        {
            using AppendResult = Result<std::monostate, SpatialError>;
            for (const VertexId id : face.vertex_ids)
                if (static_cast<std::size_t>(id) >= mesh.vertices.size())
                    return AppendResult::failure(
                        SpatialError::InvalidTopologyReference);
            const auto append = [&](VertexId a, VertexId b, VertexId c,
                                    std::uint32_t local) {
                output.push_back({
                    {mesh.vertices[a], mesh.vertices[b], mesh.vertices[c]},
                    tag.kind, tag.region_id, source_face_id, local});
            };
            if constexpr (std::is_same_v<Face, Triangle>)
                append(face.vertex_ids[0], face.vertex_ids[1],
                       face.vertex_ids[2], 0);
            else
            {
                append(face.vertex_ids[0], face.vertex_ids[1],
                       face.vertex_ids[2], 0);
                append(face.vertex_ids[0], face.vertex_ids[2],
                       face.vertex_ids[3], 1);
            }
            return AppendResult::success({});
        }

        Result<std::pair<Point3, Vector3>, SpatialError> closestOnRegion(
            const std::vector<IndexedSlidingTriangle> &triangles,
            std::uint32_t region_id,
            const Point3 &point)
        {
            using ClosestResult =
                Result<std::pair<Point3, Vector3>, SpatialError>;
            if (!point.allFinite())
                return ClosestResult::failure(SpatialError::NonFiniteCoordinate);
            Scalar best = std::numeric_limits<Scalar>::infinity();
            Point3 closest{};
            Vector3 best_normal{};
            bool found{};
            for (const IndexedSlidingTriangle &triangle : triangles)
            {
                if (triangle.region_id != region_id) continue;
                const Point3 candidate = closestPointOnTriangle(
                    point, triangle.points);
                const Scalar distance = (candidate - point).squaredNorm();
                if (!found || distance < best)
                {
                    Vector3 value = (triangle.points[1] - triangle.points[0])
                        .cross(triangle.points[2] - triangle.points[0]);
                    if (!value.allFinite() || value.squaredNorm() == 0)
                        return ClosestResult::failure(
                            SpatialError::DegenerateTriangle);
                    value.normalize();
                    best = distance;
                    closest = candidate;
                    best_normal = value;
                    found = true;
                }
            }
            if (!found)
                return ClosestResult::failure(
                    SpatialError::InvalidTopologyReference);
            return ClosestResult::success({closest, best_normal});
        }
    }

    Result<SlidingIntersectionIndex, SpatialError>
    SlidingIntersectionIndex::build(const SurfaceMesh &mesh)
    {
        using BuildResult = Result<SlidingIntersectionIndex, SpatialError>;
        if (mesh.faces.size() != mesh.face_tags.size())
            return BuildResult::failure(SpatialError::InvalidTopologyReference);
        SlidingIntersectionIndex output;
        for (std::size_t face_index = 0; face_index < mesh.faces.size(); ++face_index)
        {
            const SurfaceBoundaryTag tag = mesh.face_tags[face_index];
            if (!isSlidingBoundary(tag.kind)) continue;
            const auto status = std::visit(
                [&](const auto &face) {
                    return appendFace(mesh, face, tag,
                        static_cast<SurfaceFaceId>(face_index),
                        output.triangles_);
                },
                mesh.faces[face_index]);
            if (!status.hasValue())
                return BuildResult::failure(status.error());
            output.region_ids_.insert(tag.region_id);
        }
        std::vector<Aabb> bounds;
        bounds.reserve(output.triangles_.size());
        for (const IndexedSlidingTriangle &triangle : output.triangles_)
        {
            for (const Point3 &point : triangle.points)
                if (!point.allFinite())
                    return BuildResult::failure(
                        SpatialError::NonFiniteCoordinate);
            const Vector3 area = (triangle.points[1] - triangle.points[0])
                .cross(triangle.points[2] - triangle.points[0]);
            if (!area.allFinite() || area.squaredNorm() == 0)
                return BuildResult::failure(SpatialError::DegenerateTriangle);
            const auto box = makeAabb(
                triangle.points[0], triangle.points[1], triangle.points[2]);
            if (!box.hasValue()) return BuildResult::failure(box.error());
            bounds.push_back(box.value());
        }
        const auto tree = BinaryAabbTree::build(bounds);
        if (!tree.hasValue()) return BuildResult::failure(tree.error());
        output.tree_ = tree.value();
        return BuildResult::success(std::move(output));
    }

    Result<SlidingIntersectionHit, SpatialError>
    SlidingIntersectionIndex::query(
        const TrianglePoints &triangle,
        const std::map<std::uint32_t, SlidingContactPermission> &permissions,
        const std::set<std::uint32_t> &ignored_regions) const
    {
        using QueryResult = Result<SlidingIntersectionHit, SpatialError>;
        for (const Point3 &point : triangle)
            if (!point.allFinite())
                return QueryResult::failure(SpatialError::NonFiniteCoordinate);
        const auto box = makeAabb(triangle[0], triangle[1], triangle[2]);
        if (!box.hasValue()) return QueryResult::failure(box.error());
        for (const std::size_t primitive : tree_.query(box.value()))
        {
            const auto &surface = triangles_[primitive];
            if (ignored_regions.count(surface.region_id) != 0) continue;
            const auto found = permissions.find(surface.region_id);
            const SlidingContactPermission empty;
            const auto invalid = hasInvalidSlidingIntersection(
                triangle, surface.points,
                found == permissions.end() ? empty : found->second);
            if (!invalid.hasValue()) return QueryResult::failure(invalid.error());
            if (invalid.value())
                return QueryResult::success(
                    {true, surface.region_id, primitive});
        }
        return QueryResult::success({});
    }

    Result<Vector3, SpatialError>
    SlidingIntersectionIndex::faceNormalAtPoint(
        std::uint32_t region_id,
        const Point3 &point) const
    {
        const auto closest = closestOnRegion(triangles_, region_id, point);
        if (!closest.hasValue())
            return Result<Vector3, SpatialError>::failure(closest.error());
        return Result<Vector3, SpatialError>::success(closest.value().second);
    }

    Result<Scalar, SpatialError>
    SlidingIntersectionIndex::signedSideToRegion(
        std::uint32_t region_id,
        const Point3 &point,
        const Vector3 &reference_normal) const
    {
        using SideResult = Result<Scalar, SpatialError>;
        if (!reference_normal.allFinite() || reference_normal.squaredNorm() == 0)
            return SideResult::failure(SpatialError::NonFiniteCoordinate);
        const auto closest = closestOnRegion(triangles_, region_id, point);
        if (!closest.hasValue()) return SideResult::failure(closest.error());
        Vector3 normal = closest.value().second;
        if (normal.dot(reference_normal) < 0) normal = -normal;
        const Scalar side = (point - closest.value().first).dot(normal);
        if (!std::isfinite(side))
            return SideResult::failure(SpatialError::NonFiniteCoordinate);
        return SideResult::success(side);
    }

    std::size_t SlidingIntersectionIndex::primitiveCount() const noexcept
    {
        return triangles_.size();
    }

    bool SlidingIntersectionIndex::hasRegion(
        std::uint32_t region_id) const noexcept
    {
        return region_ids_.count(region_id) != 0;
    }
}
