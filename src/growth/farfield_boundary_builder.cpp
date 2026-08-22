#include <boundary_mesh/growth/farfield_boundary_builder.hpp>

#include <algorithm>
#include <limits>
#include <type_traits>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
        bool sameKey(
            const CollisionVertexKey &left,
            const CollisionVertexKey &right)
        {
            return left.source_vertex_id == right.source_vertex_id &&
                   left.layer == right.layer;
        }

        Result<VertexId, SpatialError> appendVertex(
            SurfaceMesh &output,
            std::vector<CollisionVertexKey> &keys,
            const CollisionVertexKey &key,
            const Point3 &point)
        {
            for (std::size_t index = 0; index < keys.size(); ++index)
            {
                if (sameKey(keys[index], key))
                {
                    if (!(output.vertices[index].array() == point.array()).all())
                    {
                        return Result<VertexId, SpatialError>::failure(
                            SpatialError::InvalidTopologyReference);
                    }
                    return Result<VertexId, SpatialError>::success(
                        static_cast<VertexId>(index));
                }
            }
            if (output.vertices.size() >=
                static_cast<std::size_t>(
                    std::numeric_limits<VertexId>::max()))
            {
                return Result<VertexId, SpatialError>::failure(
                    SpatialError::PrimitiveIdOverflow);
            }
            const VertexId id = static_cast<VertexId>(output.vertices.size());
            output.vertices.push_back(point);
            keys.push_back(key);
            return Result<VertexId, SpatialError>::success(id);
        }
    }

    Result<SurfaceMesh, SpatialError> buildFarfieldBoundary(
        const SurfaceMesh &original_surface,
        const ExposedBoundaryTracker &exposed_boundary)
    {
        if (original_surface.faces.size() !=
            original_surface.face_tags.size())
        {
            return Result<SurfaceMesh, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        }

        SurfaceMesh output;
        std::vector<CollisionVertexKey> output_keys;
        for (std::size_t face_index = 0;
             face_index < original_surface.faces.size();
             ++face_index)
        {
            if (original_surface.face_tags[face_index].kind !=
                SurfaceBoundaryKind::Farfield)
            {
                continue;
            }
            const auto face_result = std::visit(
                [&](const auto &face)
                    -> Result<SurfaceFace, SpatialError>
                {
                    using Face = std::decay_t<decltype(face)>;
                    Face remapped;
                    for (std::size_t corner = 0;
                         corner < face.vertex_ids.size();
                         ++corner)
                    {
                        const VertexId source_id = face.vertex_ids[corner];
                        const std::size_t source_index =
                            static_cast<std::size_t>(source_id);
                        if (source_index >= original_surface.vertices.size())
                        {
                            return Result<SurfaceFace, SpatialError>::failure(
                                SpatialError::InvalidTopologyReference);
                        }
                        const auto id = appendVertex(
                            output,
                            output_keys,
                            {source_id, 0},
                            original_surface.vertices[source_index]);
                        if (!id.hasValue())
                        {
                            return Result<SurfaceFace, SpatialError>::failure(
                                id.error());
                        }
                        remapped.vertex_ids[corner] = id.value();
                    }
                    return Result<SurfaceFace, SpatialError>::success(remapped);
                },
                original_surface.faces[face_index]);
            if (!face_result.hasValue())
            {
                return Result<SurfaceMesh, SpatialError>::failure(
                    face_result.error());
            }
            output.faces.push_back(face_result.value());
            output.face_tags.push_back(original_surface.face_tags[face_index]);
        }

        for (const BoundaryFace &face : exposed_boundary.faces())
        {
            std::vector<VertexId> ids;
            ids.reserve(face.points.size());
            for (std::size_t reverse = face.points.size(); reverse > 0; --reverse)
            {
                const std::size_t index = reverse - 1;
                const auto id = appendVertex(
                    output,
                    output_keys,
                    face.vertex_keys[index],
                    face.points[index]);
                if (!id.hasValue())
                {
                    return Result<SurfaceMesh, SpatialError>::failure(
                        id.error());
                }
                ids.push_back(id.value());
            }
            if (ids.size() == 3)
            {
                output.faces.push_back(
                    Triangle{{ids[0], ids[1], ids[2]}});
            }
            else
            {
                output.faces.push_back(
                    Quad{{ids[0], ids[1], ids[2], ids[3]}});
            }
            output.face_tags.push_back(
                {SurfaceBoundaryKind::BoundaryLayerInterface,
                 face.region_id});
        }

        return Result<SurfaceMesh, SpatialError>::success(
            std::move(output));
    }
}
