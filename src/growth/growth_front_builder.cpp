#include <cstddef>
#include <type_traits>
#include <unordered_map>
#include <variant>

#include <boundary_mesh/growth/growth_front_builder.hpp>

namespace boundary_mesh
{
    Result<GrowthFront, GrowthFrontError>
    GrowthFrontBuilder::buildInitial(
        const SurfaceMesh &mesh,
        const GrowthPatch &patch) const
    {
        using FrontResult = Result<GrowthFront, GrowthFrontError>;

        GrowthFront front;
        front.layer = 0;
        std::unordered_map<VertexId, VertexId> local_ids;

        for (const PatchVertex &patch_vertex : patch.vertices())
        {
            const std::size_t source_index =
                static_cast<std::size_t>(patch_vertex.source_vertex_id);
            if (source_index >= mesh.vertices.size())
            {
                return FrontResult::failure(
                    InvalidPatchVertex{patch_vertex.source_vertex_id});
            }

            const VertexId local_id = static_cast<VertexId>(front.vertices.size());
            local_ids.emplace(patch_vertex.source_vertex_id, local_id);
            const Point3 &position = mesh.vertices[source_index];
            front.vertices.push_back(
                GrowthFrontVertex{
                    position,
                    patch_vertex.source_vertex_id,
                    FrontVertexBoundary{
                        patch_vertex.sliding_region_ids}});
        }

        for (const SurfaceFaceId source_face_id : patch.sourceFaceIds())
        {
            const std::size_t source_index = static_cast<std::size_t>(source_face_id);
            if (source_index >= mesh.faces.size())
            {
                return FrontResult::failure(
                    InvalidPatchFace{source_face_id});
            }

            const SurfaceFace &source_face = mesh.faces[source_index];
            VertexId missing_id{};
            bool missing = false;
            std::visit(
                [&](const auto &face)
                {
                    for (const VertexId vertex_id : face.vertex_ids)
                    {
                        if (local_ids.find(vertex_id) == local_ids.end())
                        {
                            missing = true;
                            missing_id = vertex_id;
                            break;
                        }
                    }
                },
                source_face);

            if (missing)
            {
                return FrontResult::failure(PatchFaceUsesUnknownVertex{
                    source_face_id,
                    missing_id});
            }

            const SurfaceFace local_face = std::visit(
                [&](const auto &face) -> SurfaceFace
                {
                    using Face = std::decay_t<decltype(face)>;
                    Face value{};
                    for (std::size_t index = 0;
                         index < face.vertex_ids.size();
                         ++index)
                    {
                        value.vertex_ids[index] = local_ids.at(face.vertex_ids[index]);
                    }
                    return value;
                },
                source_face);

            front.faces.push_back(local_face);
            front.source_face_ids.push_back(source_face_id);
        }

        return FrontResult::success(std::move(front));
    }
}
