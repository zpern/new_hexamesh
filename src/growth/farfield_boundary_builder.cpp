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
                   left.layer == right.layer &&
                   left.branch_id == right.branch_id;
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
        const GrowthFront &zero_layer_front,
        const ExposedBoundaryTracker &exposed_boundary,
        const std::vector<SurfaceFaceId> &zero_layer_source_face_ids)
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

        // 零层生长的 Wall 面未进入外露边界跟踪器，需要以相反绕向补回接口。
        std::vector<bool> selected(
            original_surface.faces.size(),
            false);
        for (const SurfaceFaceId face_id : zero_layer_source_face_ids)
        {
            const std::size_t face_index =
                static_cast<std::size_t>(face_id);
            if (face_index >= original_surface.faces.size() ||
                selected[face_index] ||
                original_surface.face_tags[face_index].kind !=
                    SurfaceBoundaryKind::Wall)
            {
                return Result<SurfaceMesh, SpatialError>::failure(
                    SpatialError::InvalidTopologyReference);
            }
            selected[face_index] = true;

            for (std::size_t front_face_index = 0;
                 front_face_index < zero_layer_front.faces.size();
                 ++front_face_index)
            {
                if (front_face_index >=
                        zero_layer_front.source_face_ids.size() ||
                    zero_layer_front.source_face_ids[front_face_index] !=
                        face_id)
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
                        const VertexId front_id = face.vertex_ids[
                            face.vertex_ids.size() - 1 - corner];
                        const std::size_t front_index =
                            static_cast<std::size_t>(front_id);
                        if (front_index >= zero_layer_front.vertices.size())
                        {
                            return Result<SurfaceFace, SpatialError>::failure(
                                SpatialError::InvalidTopologyReference);
                        }
                        const GrowthFrontVertex &vertex =
                            zero_layer_front.vertices[front_index];
                        const auto id = appendVertex(
                            output,
                            output_keys,
                            {vertex.source_vertex_id,
                             zero_layer_front.layer,
                             vertex.branch_id},
                            vertex.position);
                        if (!id.hasValue())
                        {
                            return Result<SurfaceFace, SpatialError>::failure(
                                id.error());
                        }
                        remapped.vertex_ids[corner] = id.value();
                    }
                    return Result<SurfaceFace, SpatialError>::success(
                        remapped);
                },
                zero_layer_front.faces[front_face_index]);
                if (!face_result.hasValue())
                {
                    return Result<SurfaceMesh, SpatialError>::failure(
                        face_result.error());
                }
                output.faces.push_back(face_result.value());
                output.face_tags.push_back(
                    {SurfaceBoundaryKind::BoundaryLayerInterface,
                     original_surface.face_tags[face_index].region_id});
            }
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

    Result<SurfaceMesh, SpatialError> extractBoundaryLayerTop(
        const SurfaceMesh &farfield_boundary)
    {
        if (farfield_boundary.faces.size() !=
            farfield_boundary.face_tags.size())
        {
            return Result<SurfaceMesh, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        }

        SurfaceMesh output;
        std::vector<VertexId> mapping(
            farfield_boundary.vertices.size(),
            std::numeric_limits<VertexId>::max());
        for (std::size_t face_index = 0;
             face_index < farfield_boundary.faces.size();
             ++face_index)
        {
            if (farfield_boundary.face_tags[face_index].kind !=
                SurfaceBoundaryKind::BoundaryLayerInterface)
            {
                continue;
            }
            const auto remapped = std::visit(
                [&](const auto &face)
                    -> Result<SurfaceFace, SpatialError>
                {
                    using Face = std::decay_t<decltype(face)>;
                    Face result;
                    for (std::size_t corner = 0;
                         corner < face.vertex_ids.size();
                         ++corner)
                    {
                        const std::size_t input_index =
                            static_cast<std::size_t>(face.vertex_ids[corner]);
                        if (input_index >= farfield_boundary.vertices.size())
                        {
                            return Result<SurfaceFace, SpatialError>::failure(
                                SpatialError::InvalidTopologyReference);
                        }
                        if (mapping[input_index] ==
                            std::numeric_limits<VertexId>::max())
                        {
                            if (output.vertices.size() >
                                static_cast<std::size_t>(
                                    std::numeric_limits<VertexId>::max()))
                            {
                                return Result<SurfaceFace, SpatialError>::failure(
                                    SpatialError::PrimitiveIdOverflow);
                            }
                            mapping[input_index] =
                                static_cast<VertexId>(output.vertices.size());
                            output.vertices.push_back(
                                farfield_boundary.vertices[input_index]);
                        }
                        result.vertex_ids[corner] = mapping[input_index];
                    }
                    return Result<SurfaceFace, SpatialError>::success(result);
                },
                farfield_boundary.faces[face_index]);
            if (!remapped.hasValue())
            {
                return Result<SurfaceMesh, SpatialError>::failure(
                    remapped.error());
            }
            output.faces.push_back(remapped.value());
            output.face_tags.push_back(
                farfield_boundary.face_tags[face_index]);
        }
        return Result<SurfaceMesh, SpatialError>::success(std::move(output));
    }
}
