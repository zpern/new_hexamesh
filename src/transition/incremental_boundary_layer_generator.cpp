#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

#include <boundary_mesh/transition/incremental_boundary_layer_generator.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>

namespace boundary_mesh
{
    namespace
    {
        const SurfaceFace *sourceFace(
            const GrowthFront &front,
            SurfaceFaceId id)
        {
            const auto found = std::find(
                front.source_face_ids.begin(),
                front.source_face_ids.end(), id);
            if (found == front.source_face_ids.end()) return nullptr;
            return &front.faces[static_cast<std::size_t>(
                found - front.source_face_ids.begin())];
        }

        const FaceGrowthRecord *growthFace(
            const RegularLayerGrowthResult &result,
            SurfaceFaceId id)
        {
            const auto found = std::find_if(
                result.faces.begin(), result.faces.end(),
                [id](const FaceGrowthRecord &face)
                { return face.source_face_id == id; });
            return found == result.faces.end() ? nullptr : &*found;
        }

        std::optional<std::size_t> topHexa(
            const VolumeMesh &mesh,
            SurfaceFaceId id,
            std::uint32_t layer)
        {
            for (std::size_t index = 0; index < mesh.cells.size(); ++index)
                if (index < mesh.metadata.size() &&
                    mesh.metadata[index].source_face_id == id &&
                    mesh.metadata[index].layer == layer &&
                    std::holds_alternative<Hexa>(mesh.cells[index]))
                    return index;
            return std::nullopt;
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

        RegularLayerGrowthResult result = std::move(regular.value());
        LayerQuadDiagonalTable diagonals;
        SurfaceMesh triangular_top;
        triangular_top.vertices = result.mesh.vertices;

        for (const SurfaceFaceId id : initial_front.source_face_ids)
        {
            const SurfaceFace *face = sourceFace(initial_front, id);
            const FaceGrowthRecord *growth = growthFace(result, id);
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
                    const auto cell_index = topHexa(
                        result.mesh, id, growth->accepted_layer_count);
                    if (!cell_index.has_value())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{id}}});
                    const Hexa hexa = std::get<Hexa>(
                        result.mesh.cells[*cell_index]);
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
                    result.mesh.cells.erase(
                        result.mesh.cells.begin() + *cell_index);
                    result.mesh.metadata.erase(
                        result.mesh.metadata.begin() + *cell_index);
                    result.mesh.vertices.insert(
                        result.mesh.vertices.end(),
                        cap.value().created_vertices.begin(),
                        cap.value().created_vertices.end());
                    result.mesh.cells.insert(
                        result.mesh.cells.end(),
                        cap.value().volume_cells.begin(),
                        cap.value().volume_cells.end());
                    result.mesh.metadata.insert(
                        result.mesh.metadata.end(),
                        cap.value().metadata.begin(),
                        cap.value().metadata.end());
                    appendTriangles(
                        triangular_top, cap.value().top_faces, region);
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
                    appendTriangles(triangular_top,
                        {split.triangles.begin(), split.triangles.end()},
                        region);
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
        triangular_top.vertices = result.mesh.vertices;
        result.top_surface = std::move(triangular_top);
        return GrowthResult::success(std::move(result));
    }
}
