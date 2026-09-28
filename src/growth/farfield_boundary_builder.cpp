#include <boundary_mesh/growth/farfield_boundary_builder.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <type_traits>
#include <unordered_map>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
        using OutputVertexKey = std::array<std::uint64_t, 2>;

        OutputVertexKey outputVertexKey(const CollisionVertexKey &key)
        {
            return {
                static_cast<std::uint64_t>(key.source_vertex_id),
                (static_cast<std::uint64_t>(key.layer) << 32) |
                    key.branch_id};
        }

        struct OutputVertexKeyHash
        {
            std::size_t operator()(const OutputVertexKey &key) const noexcept
            {
                const std::size_t first =
                    std::hash<std::uint64_t>{}(key[0]);
                const std::size_t second =
                    std::hash<std::uint64_t>{}(key[1]);
                return first ^ (second + 0x9e3779b9U +
                                (first << 6) + (first >> 2));
            }
        };

        Result<VertexId, SpatialError> appendVertex(
            SurfaceMesh &output,
            std::unordered_map<OutputVertexKey, VertexId,
                               OutputVertexKeyHash> &vertex_ids,
            const CollisionVertexKey &key,
            const Point3 &point)
        {
            const OutputVertexKey packed_key = outputVertexKey(key);
            const auto existing = vertex_ids.find(packed_key);
            if (existing != vertex_ids.end())
            {
                const VertexId id = existing->second;
                if (!(output.vertices[id].array() == point.array()).all())
                {
                    return Result<VertexId, SpatialError>::failure(
                        SpatialError::InvalidTopologyReference);
                }
                return Result<VertexId, SpatialError>::success(id);
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
            vertex_ids.emplace(packed_key, id);
            return Result<VertexId, SpatialError>::success(id);
        }
    }

    Result<SurfaceMesh, SpatialError> buildFarfieldBoundary(
        const SurfaceMesh &original_surface,
        const GrowthFront &zero_layer_front,
        const ExposedBoundaryTracker &exposed_boundary,
        const std::vector<SurfaceFaceId> &zero_layer_source_face_ids,
        bool include_boundary_layer_interface)
    {
        if (original_surface.faces.size() !=
            original_surface.face_tags.size())
        {
            return Result<SurfaceMesh, SpatialError>::failure(
                SpatialError::InvalidTopologyReference);
        }

        SurfaceMesh output;
        std::unordered_map<OutputVertexKey, VertexId, OutputVertexKeyHash>
            output_vertex_ids;
        output_vertex_ids.reserve(original_surface.vertices.size());
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
                            output_vertex_ids,
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
            if (!include_boundary_layer_interface)
                continue;

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
                            output_vertex_ids,
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
            if (!include_boundary_layer_interface &&
                face.boundary_kind ==
                    SurfaceBoundaryKind::BoundaryLayerInterface)
                continue;
            std::vector<VertexId> ids;
            ids.reserve(face.points.size());
            for (std::size_t reverse = face.points.size(); reverse > 0; --reverse)
            {
                const std::size_t index = reverse - 1;
                const auto id = appendVertex(
                    output,
                    output_vertex_ids,
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
                {face.boundary_kind, face.region_id});
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

    Result<SurfaceMesh, SpatialError> buildFarfieldBoundary(
        const SurfaceMesh &original_surface,
        const ExposedBoundaryTracker &exposed_boundary,
        const std::vector<SurfaceFaceId> &zero_layer_source_face_ids,
        bool include_boundary_layer_interface)
    {
        GrowthFront original_front;
        original_front.vertices.reserve(original_surface.vertices.size());
        for (std::size_t index = 0;
             index < original_surface.vertices.size();
             ++index)
        {
            if (index > static_cast<std::size_t>(
                            std::numeric_limits<VertexId>::max()))
            {
                return Result<SurfaceMesh, SpatialError>::failure(
                    SpatialError::PrimitiveIdOverflow);
            }
            original_front.vertices.emplace_back(
                original_surface.vertices[index],
                static_cast<VertexId>(index));
        }
        original_front.faces = original_surface.faces;
        original_front.source_face_ids.reserve(
            original_surface.faces.size());
        for (std::size_t index = 0;
             index < original_surface.faces.size();
             ++index)
        {
            if (index > static_cast<std::size_t>(
                            std::numeric_limits<SurfaceFaceId>::max()))
            {
                return Result<SurfaceMesh, SpatialError>::failure(
                    SpatialError::PrimitiveIdOverflow);
            }
            original_front.source_face_ids.push_back(
                static_cast<SurfaceFaceId>(index));
        }
        return buildFarfieldBoundary(
            original_surface,
            original_front,
            exposed_boundary,
            zero_layer_source_face_ids,
            include_boundary_layer_interface);
    }
}
