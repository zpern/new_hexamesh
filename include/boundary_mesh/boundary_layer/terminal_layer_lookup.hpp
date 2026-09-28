#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <utility>

#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/regular_layer_growth.hpp>
#include <boundary_mesh/quality/volume_cell_evaluation.hpp>

namespace boundary_mesh
{
    /// Per-resolve lookup tables for terminal hexahedron coordinates.
    class TerminalLayerLookup
    {
    public:
        TerminalLayerLookup(
            const GrowthFront &front,
            const LayerVertexTable &records)
        {
            face_indices_.reserve(front.source_face_ids.size());
            for (std::size_t index = 0;
                 index < front.source_face_ids.size(); ++index)
                face_indices_.emplace(front.source_face_ids[index], index);

            layer_record_indices_.reserve(records.size());
            for (std::size_t index = 0; index < records.size(); ++index)
                layer_record_indices_.emplace(
                    Key{records[index].source_vertex_id,
                        records[index].branch_id}, index);
        }

        std::optional<HexaPoints> hexaPoints(
            SurfaceFaceId face_id,
            std::uint32_t completed_layer,
            const GrowthFront &front,
            const LayerVertexTable &records,
            const VolumeMesh &mesh) const
        {
            if (completed_layer == 0) return std::nullopt;
            const auto face_position = face_indices_.find(face_id);
            if (face_position == face_indices_.end() ||
                face_position->second >= front.faces.size())
                return std::nullopt;
            const auto *quad = std::get_if<Quad>(
                &front.faces[face_position->second]);
            if (quad == nullptr) return std::nullopt;

            HexaPoints points{};
            for (std::size_t local = 0; local < 4; ++local)
            {
                if (quad->vertex_ids[local] >= front.vertices.size())
                    return std::nullopt;
                const auto &vertex = front.vertices[quad->vertex_ids[local]];
                const auto record_position = layer_record_indices_.find(
                    Key{vertex.source_vertex_id, vertex.branch_id});
                if (record_position == layer_record_indices_.end() ||
                    record_position->second >= records.size())
                    return std::nullopt;
                const auto &record = records[record_position->second];
                if (record.layer_vertex_ids.size() <= completed_layer)
                    return std::nullopt;
                const VertexId bottom =
                    record.layer_vertex_ids[completed_layer - 1];
                const VertexId top = record.layer_vertex_ids[completed_layer];
                if (static_cast<std::size_t>(bottom) >= mesh.vertices.size() ||
                    static_cast<std::size_t>(top) >= mesh.vertices.size())
                    return std::nullopt;
                points[local] = mesh.vertices[bottom];
                points[4 + local] = mesh.vertices[top];
            }
            return points;
        }

    private:
        using Key = std::pair<VertexId, std::uint32_t>;

        struct KeyHash
        {
            std::size_t operator()(const Key &key) const noexcept
            {
                const std::size_t first = std::hash<VertexId>{}(key.first);
                const std::size_t second =
                    std::hash<std::uint32_t>{}(key.second);
                return first ^ (second + 0x9e3779b9 + (first << 6) +
                                (first >> 2));
            }
        };

        std::unordered_map<SurfaceFaceId, std::size_t> face_indices_;
        std::unordered_map<Key, std::size_t, KeyHash> layer_record_indices_;
    };
}
