#include <algorithm>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>

#include <boundary_mesh/transition/incremental_boundary_layer_generator.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>

namespace boundary_mesh
{
    namespace
    {
        std::uint64_t cellKey(
            SurfaceFaceId id,
            std::uint32_t layer)
        {
            return (static_cast<std::uint64_t>(id) << 32) | layer;
        }

        std::uint64_t edgeKey(VertexId first, VertexId second)
        {
            if (second < first) std::swap(first, second);
            return (static_cast<std::uint64_t>(first) << 32) | second;
        }

        OrientedQuad oriented(
            const std::array<VertexId, 4> &ids,
            const std::vector<Point3> &points)
        {
            OrientedQuad quad;
            quad.vertex_ids = ids;
            for (std::size_t index = 0; index < 4; ++index)
                quad.points[index] = points[ids[index]];
            return quad;
        }

        void appendTriangles(
            SurfaceMesh &top,
            const std::vector<Triangle> &triangles,
            std::uint32_t region)
        {
            for (const Triangle &triangle : triangles)
            {
                top.faces.push_back(triangle);
                top.face_tags.push_back({
                    SurfaceBoundaryKind::BoundaryLayerInterface, region});
            }
        }

        const LayerVertexRecord *layerRecord(
            const LayerVertexTable &table,
            VertexId source_vertex_id,
            std::uint32_t branch_id)
        {
            const auto found = std::find_if(
                table.begin(), table.end(),
                [source_vertex_id, branch_id](const auto &record)
                {
                    return record.source_vertex_id == source_vertex_id &&
                           record.branch_id == branch_id;
                });
            return found == table.end() ? nullptr : &*found;
        }

        struct FinalQuad
        {
            SurfaceFaceId id{};
            std::uint32_t layer{};
            std::uint32_t region{};
            std::array<VertexId, 4> source_ids{};
            std::array<VertexId, 4> top_ids{};
            QuadDiagonal diagonal{};
            std::vector<Triangle> top_faces;
        };

        bool appendRemappedFace(
            SurfaceMesh &output,
            const SurfaceFace &input,
            const SurfaceBoundaryTag &tag,
            const std::vector<Point3> &points)
        {
            SurfaceFace remapped = input;
            bool valid = true;
            std::visit([&](auto &face)
            {
                for (VertexId &id : face.vertex_ids)
                {
                    if (static_cast<std::size_t>(id) >= points.size())
                    {
                        valid = false;
                        return;
                    }
                    const Point3 point = points[id];
                    id = static_cast<VertexId>(output.vertices.size());
                    output.vertices.push_back(point);
                }
            }, remapped);
            if (!valid) return false;
            output.faces.push_back(std::move(remapped));
            output.face_tags.push_back(tag);
            return true;
        }
    }

    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
    finalizeIncrementalLayerTopology(
        const SurfaceMesh &surface_mesh,
        const GrowthFront &initial_front,
        RegularLayerGrowthResult result)
    {
        using GrowthResult = Result<
            RegularLayerGrowthResult, IncrementalLayerGrowthError>;
        LayerQuadDiagonalTable diagonals;
        SurfaceMesh triangular_top;
        triangular_top.vertices = result.mesh.vertices;
        std::vector<FinalQuad> final_quads;
        std::unordered_map<SurfaceFaceId, const SurfaceFace *> source_faces;
        for (std::size_t index = 0;
             index < initial_front.source_face_ids.size(); ++index)
            source_faces[initial_front.source_face_ids[index]] =
                &initial_front.faces[index];
        std::unordered_map<SurfaceFaceId, const FaceGrowthRecord *> growth_faces;
        for (const FaceGrowthRecord &face : result.faces)
            growth_faces[face.source_face_id] = &face;
        std::unordered_map<std::uint64_t, std::vector<SurfaceFaceId>> edge_faces;
        for (std::size_t index = 0;
             index < initial_front.faces.size(); ++index)
        {
            const SurfaceFaceId id = initial_front.source_face_ids[index];
            std::visit([&](const auto &face)
            {
                for (std::size_t edge = 0;
                     edge < face.vertex_ids.size(); ++edge)
                    edge_faces[edgeKey(
                        face.vertex_ids[edge],
                        face.vertex_ids[(edge + 1) %
                                        face.vertex_ids.size()])]
                        .push_back(id);
            }, initial_front.faces[index]);
        }
        std::unordered_map<std::uint64_t, std::size_t> top_hexa;
        for (std::size_t index = 0; index < result.mesh.cells.size(); ++index)
            if (index < result.mesh.metadata.size() &&
                std::holds_alternative<Hexa>(result.mesh.cells[index]))
                top_hexa[cellKey(
                    result.mesh.metadata[index].source_face_id,
                    result.mesh.metadata[index].layer)] = index;

        for (const SurfaceFaceId id : initial_front.source_face_ids)
        {
            const auto source_found = source_faces.find(id);
            const auto growth_found = growth_faces.find(id);
            const SurfaceFace *face = source_found == source_faces.end()
                ? nullptr : source_found->second;
            const FaceGrowthRecord *growth = growth_found == growth_faces.end()
                ? nullptr : growth_found->second;
            if (face == nullptr || growth == nullptr ||
                static_cast<std::size_t>(id) >= surface_mesh.face_tags.size())
                return GrowthResult::failure(
                    IncrementalLayerGrowthError{
                        TransitionTemplateError{
                            InvalidTransitionTemplateInput{id}}});
            const std::uint32_t region =
                surface_mesh.face_tags[id].region_id;
            if (const auto *quad = std::get_if<Quad>(face))
            {
                std::array<VertexId, 4> top_ids = quad->vertex_ids;
                std::array<VertexId, 4> bottom_ids = quad->vertex_ids;
                if (growth->accepted_layer_count > 0)
                {
                    const auto cell = top_hexa.find(cellKey(
                        id, growth->accepted_layer_count));
                    if (cell == top_hexa.end())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{id}}});
                    const Hexa hexa = std::get<Hexa>(
                        result.mesh.cells[cell->second]);
                    std::copy_n(hexa.vertex_ids.begin(), 4,
                                bottom_ids.begin());
                    std::copy_n(hexa.vertex_ids.begin() + 4, 4,
                                top_ids.begin());
                    const auto diagonal = diagonals.resolve(
                        {id, growth->accepted_layer_count},
                        oriented(top_ids, result.mesh.vertices),
                        std::nullopt, 1e-12);
                    if (!diagonal.hasValue())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                diagonal.error()});
                    if (result.mesh.vertices.size() >
                        static_cast<std::size_t>(
                            std::numeric_limits<VertexId>::max()))
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                VolumeVertexIdOverflow{
                                    result.mesh.vertices.size()}});
                    const VertexId center = static_cast<VertexId>(
                        result.mesh.vertices.size());
                    const auto cap = buildQuadTopCap({
                        id, growth->accepted_layer_count,
                        bottom_ids, top_ids, &result.mesh.vertices,
                        center, diagonal.value()});
                    if (!cap.hasValue())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{cap.error()});
                    result.mesh.vertices.insert(
                        result.mesh.vertices.end(),
                        cap.value().created_vertices.begin(),
                        cap.value().created_vertices.end());
                    result.mesh.cells[cell->second] =
                        cap.value().volume_cells.front();
                    result.mesh.metadata[cell->second] =
                        cap.value().metadata.front();
                    result.mesh.cells.insert(
                        result.mesh.cells.end(),
                        cap.value().volume_cells.begin() + 1,
                        cap.value().volume_cells.end());
                    result.mesh.metadata.insert(
                        result.mesh.metadata.end(),
                        cap.value().metadata.begin() + 1,
                        cap.value().metadata.end());
                    final_quads.push_back({
                        id, growth->accepted_layer_count, region,
                        quad->vertex_ids, top_ids, diagonal.value(),
                        cap.value().top_faces});
                }
                else
                {
                    const auto diagonal = diagonals.resolve(
                        {id, 0}, oriented(top_ids, result.mesh.vertices),
                        std::nullopt, 1e-12);
                    if (!diagonal.hasValue())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{diagonal.error()});
                    const QuadDiagonalSelection split =
                        diagonal.value() == QuadDiagonal::ZeroTwo
                            ? QuadDiagonalSelection{
                                QuadDiagonal::ZeroTwo,
                                {Triangle{{top_ids[0],top_ids[1],top_ids[2]}},
                                 Triangle{{top_ids[0],top_ids[2],top_ids[3]}}},
                                0}
                            : QuadDiagonalSelection{
                                QuadDiagonal::OneThree,
                                {Triangle{{top_ids[1],top_ids[2],top_ids[3]}},
                                 Triangle{{top_ids[1],top_ids[3],top_ids[0]}}},
                                0};
                    final_quads.push_back({
                        id, 0, region, quad->vertex_ids, top_ids,
                        diagonal.value(),
                        {split.triangles.begin(), split.triangles.end()}});
                }
            }
            else
            {
                const Triangle &triangle = std::get<Triangle>(*face);
                triangular_top.faces.push_back(triangle);
                triangular_top.face_tags.push_back({
                    SurfaceBoundaryKind::BoundaryLayerInterface, region});
            }
        }

        for (FinalQuad &low : final_quads)
        {
            std::optional<std::size_t> high_edge;
            for (std::size_t edge = 0; edge < 4 && !high_edge; ++edge)
            {
                const auto uses = edge_faces.find(edgeKey(
                    low.source_ids[edge],
                    low.source_ids[(edge + 1) % 4]));
                if (uses == edge_faces.end()) continue;
                for (const SurfaceFaceId other_id : uses->second)
                {
                    if (other_id == low.id) continue;
                    const auto other = growth_faces.find(other_id);
                    if (other != growth_faces.end() &&
                        other->second->accepted_layer_count == low.layer + 1)
                    {
                        high_edge = edge;
                        break;
                    }
                }
            }
            if (high_edge.has_value())
            {
                std::array<VertexId, 4> high_ids = low.top_ids;
                for (const std::size_t local : {
                         *high_edge, (*high_edge + 1) % 4})
                {
                    const GrowthFrontVertex &vertex =
                        initial_front.vertices[low.source_ids[local]];
                    const LayerVertexRecord *record = layerRecord(
                        result.layer_vertices,
                        vertex.source_vertex_id,
                        vertex.branch_id);
                    if (record == nullptr ||
                        record->layer_vertex_ids.size() <= low.layer + 1)
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{
                                        low.id}}});
                    high_ids[local] =
                        record->layer_vertex_ids[low.layer + 1];
                }
                const auto side = buildQuadSideTransition({
                    low.id, low.layer, low.top_ids, high_ids,
                    *high_edge, low.diagonal});
                if (!side.hasValue())
                    return GrowthResult::failure(
                        IncrementalLayerGrowthError{side.error()});
                result.mesh.cells.insert(
                    result.mesh.cells.end(),
                    side.value().volume_cells.begin(),
                    side.value().volume_cells.end());
                result.mesh.metadata.insert(
                    result.mesh.metadata.end(),
                    side.value().metadata.begin(),
                    side.value().metadata.end());
                low.top_faces = side.value().top_faces;
            }
            appendTriangles(
                triangular_top, low.top_faces, low.region);
        }
        triangular_top.vertices = result.mesh.vertices;
        SurfaceMesh triangular_farfield;
        for (std::size_t index = 0;
             index < result.farfield_boundary.faces.size(); ++index)
        {
            if (index >= result.farfield_boundary.face_tags.size())
                return GrowthResult::failure(
                    IncrementalLayerGrowthError{
                        TransitionTemplateError{
                            InvalidTransitionTemplateInput{}}});
            if (result.farfield_boundary.face_tags[index].kind ==
                SurfaceBoundaryKind::BoundaryLayerInterface)
                continue;
            if (!appendRemappedFace(
                    triangular_farfield,
                    result.farfield_boundary.faces[index],
                    result.farfield_boundary.face_tags[index],
                    result.farfield_boundary.vertices))
                return GrowthResult::failure(
                    IncrementalLayerGrowthError{
                        TransitionTemplateError{
                            InvalidTransitionTemplateInput{}}});
        }
        for (std::size_t index = 0;
             index < triangular_top.faces.size(); ++index)
            if (!appendRemappedFace(
                    triangular_farfield,
                    triangular_top.faces[index],
                    triangular_top.face_tags[index],
                    result.mesh.vertices))
                return GrowthResult::failure(
                    IncrementalLayerGrowthError{
                        TransitionTemplateError{
                            InvalidTransitionTemplateInput{}}});
        result.farfield_boundary = std::move(triangular_farfield);
        result.top_surface = std::move(triangular_top);
        return GrowthResult::success(std::move(result));
    }

    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
    generateIncrementalBoundaryLayers(
        const SurfaceMesh &surface_mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options)
    {
        using GrowthResult = Result<
            RegularLayerGrowthResult, IncrementalLayerGrowthError>;
        auto regular = generateRegularLayers(
            surface_mesh, topology, patch, initial_front,
            profiles, options);
        if (!regular.hasValue())
            return GrowthResult::failure(
                IncrementalLayerGrowthError{regular.error()});
        return finalizeIncrementalLayerTopology(
            surface_mesh, initial_front, std::move(regular.value()));
    }
}
