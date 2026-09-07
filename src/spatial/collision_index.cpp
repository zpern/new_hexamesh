#include <boundary_mesh/spatial/collision_index.hpp>

#include <limits>
#include <utility>

#include <Eigen/Geometry>

namespace boundary_mesh
{
    namespace
    {
        template <std::size_t VertexCount>
        Result<std::monostate, SpatialError> setBoundaryMetadata(
            const SurfaceMesh &mesh,
            const std::array<VertexId, VertexCount> &vertex_ids,
            CollisionTriangle &triangle)
        {
            static_assert(VertexCount == 3 || VertexCount == 4);
            triangle.boundary_vertex_count =
                static_cast<std::uint8_t>(VertexCount);
            for (std::size_t index = 0; index < VertexCount; ++index)
            {
                const auto vertex_index =
                    static_cast<std::size_t>(vertex_ids[index]);
                if (vertex_index >= mesh.vertices.size())
                {
                    return Result<std::monostate, SpatialError>::failure(
                        SpatialError::InvalidTopologyReference);
                }
                triangle.boundary_points[index] = mesh.vertices[vertex_index];
                triangle.boundary_vertex_keys[index] =
                    CollisionVertexKey{vertex_ids[index], 0};
            }
            return Result<std::monostate, SpatialError>::success({});
        }

        Result<CollisionTriangle, SpatialError> makeTriangle(
            const SurfaceMesh &mesh,
            const std::array<VertexId, 3> &vertex_ids,
            SurfaceFaceId face_id)
        {
            CollisionTriangle triangle;
            triangle.owner_kind = CollisionOwnerKind::OriginalSurface;
            triangle.owner_id = face_id;
            for (std::size_t index = 0; index < 3; ++index)
            {
                const auto vertex_index =
                    static_cast<std::size_t>(vertex_ids[index]);
                if (vertex_index >= mesh.vertices.size())
                {
                    return Result<CollisionTriangle, SpatialError>::failure(
                        SpatialError::InvalidTopologyReference);
                }
                triangle.points[index] = mesh.vertices[vertex_index];
                triangle.vertex_keys[index] =
                    CollisionVertexKey{vertex_ids[index], 0};
            }
            return Result<CollisionTriangle, SpatialError>::success(
                triangle);
        }

        Result<std::monostate, SpatialError> appendFaceTriangles(
            const SurfaceMesh &mesh,
            const SurfaceFace &face,
            SurfaceFaceId face_id,
            std::vector<CollisionTriangle> &triangles)
        {
            if (const auto *triangle = std::get_if<Triangle>(&face))
            {
                auto value = makeTriangle(
                    mesh,
                    triangle->vertex_ids,
                    face_id);
                if (!value.hasValue())
                {
                    return Result<std::monostate, SpatialError>::failure(
                        value.error());
                }
                const auto metadata = setBoundaryMetadata(
                    mesh,
                    triangle->vertex_ids,
                    value.value());
                if (!metadata.hasValue())
                {
                    return metadata;
                }
                triangles.push_back(value.value());
            }
            else
            {
                const Quad &quad = std::get<Quad>(face);
                for (const std::array<VertexId, 3> ids : {
                         std::array<VertexId, 3>{
                             quad.vertex_ids[0],
                             quad.vertex_ids[1],
                             quad.vertex_ids[2]},
                         std::array<VertexId, 3>{
                             quad.vertex_ids[0],
                             quad.vertex_ids[2],
                             quad.vertex_ids[3]}})
                {
                    auto value = makeTriangle(mesh, ids, face_id);
                    if (!value.hasValue())
                    {
                        return Result<std::monostate, SpatialError>::failure(
                            value.error());
                    }
                    const auto metadata = setBoundaryMetadata(
                        mesh,
                        quad.vertex_ids,
                        value.value());
                    if (!metadata.hasValue())
                    {
                        return metadata;
                    }
                    triangles.push_back(value.value());
                }
            }
            return Result<std::monostate, SpatialError>::success({});
        }
    }

    Result<CollisionIndex, SpatialError> CollisionIndex::build(
        std::vector<CollisionTriangle> triangles)
    {
        std::vector<Aabb> bounds;
        bounds.reserve(triangles.size());
        for (const CollisionTriangle &triangle : triangles)
        {
            const auto box = makeAabb(
                triangle.points[0],
                triangle.points[1],
                triangle.points[2]);
            if (!box.hasValue())
            {
                return Result<CollisionIndex, SpatialError>::failure(
                    box.error());
            }
            if ((triangle.points[1] - triangle.points[0])
                    .cross(triangle.points[2] - triangle.points[0])
                    .squaredNorm() == Scalar{0})
            {
                return Result<CollisionIndex, SpatialError>::failure(
                    SpatialError::DegenerateTriangle);
            }
            bounds.push_back(box.value());
        }

        auto tree = BinaryAabbTree::build(bounds);
        if (!tree.hasValue())
        {
            return Result<CollisionIndex, SpatialError>::failure(
                tree.error());
        }

        CollisionIndex index;
        index.triangles_ = std::move(triangles);
        index.tree_ = std::move(tree.value());
        return Result<CollisionIndex, SpatialError>::success(
            std::move(index));
    }

    std::vector<std::size_t> CollisionIndex::queryIllegalContacts(
        const CollisionTriangle &query) const
    {
        const auto bounds = makeAabb(
            query.points[0],
            query.points[1],
            query.points[2]);
        if (!bounds.hasValue())
        {
            return {};
        }

        std::vector<std::size_t> result;
        for (const std::size_t primitive : tree_.query(bounds.value()))
        {
            const auto illegal = hasIllegalTriangleContact(
                query,
                triangles_[primitive]);
            if (!illegal.hasValue() || illegal.value())
            {
                result.push_back(primitive);
            }
        }
        return result;
    }

    std::size_t CollisionIndex::primitiveCount() const noexcept
    {
        return triangles_.size();
    }

    const CollisionTriangle &CollisionIndex::primitive(
        std::size_t primitive_index) const
    {
        return triangles_.at(primitive_index);
    }

    Result<CollisionIndex, SpatialError>
    buildOriginalSurfaceCollisionIndex(
        const SurfaceMesh &mesh,
        const SurfaceTopology &topology,
        const CollisionBoundaryPolicy &policy)
    {
        if (mesh.faces.size() != mesh.face_tags.size() ||
            topology.faceEdges().size() != mesh.faces.size() ||
            topology.faceNeighbors().size() != mesh.faces.size() ||
            topology.vertexFaces().size() != mesh.vertices.size() ||
            mesh.faces.size() > static_cast<std::size_t>(
                std::numeric_limits<SurfaceFaceId>::max()))
        {
            return Result<CollisionIndex, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        }

        std::vector<CollisionTriangle> triangles;
        for (std::size_t face_index = 0;
             face_index < mesh.faces.size();
             ++face_index)
        {
            const SurfaceBoundaryKind kind =
                mesh.face_tags[face_index].kind;
            if (!policy.isObstacle(
                    kind,
                    CollisionSurfaceOrigin::InputSurface))
            {
                continue;
            }
            const auto status = appendFaceTriangles(
                mesh,
                mesh.faces[face_index],
                static_cast<SurfaceFaceId>(face_index),
                triangles);
            if (!status.hasValue())
            {
                return Result<CollisionIndex, SpatialError>::failure(
                    status.error());
            }
        }
        return CollisionIndex::build(std::move(triangles));
    }
}
