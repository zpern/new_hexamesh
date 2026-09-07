#include <algorithm>
#include <limits>
#include <optional>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <boundary_mesh/boundary_layer/incremental_topology_finalizer.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>
#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>
#include <boundary_mesh/transition/triangle_side_transition.hpp>

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

        TransitionTemplateError atStage(
            TransitionTemplateError error, std::uint32_t stage)
        {
            if (auto *invalid =
                    std::get_if<InvalidTransitionTemplateInput>(&error))
                invalid->stage = stage;
            return error;
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
            std::vector<std::size_t> high_edges;
            std::vector<Triangle> top_faces;
        };

        struct FinalTriangle
        {
            SurfaceFaceId id{};
            std::uint32_t layer{};
            std::uint32_t region{};
            std::array<VertexId,3> source_ids{};
            std::array<VertexId,3> top_ids{};
            std::optional<std::size_t> high_edge;
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

        struct TriangleKey
        {
            std::array<VertexId,3> ids{};

            bool operator==(const TriangleKey &other) const noexcept
            {
                return ids == other.ids;
            }
        };

        struct TriangleKeyHash
        {
            std::size_t operator()(const TriangleKey &key) const noexcept
            {
                std::size_t hash = 1469598103934665603ull;
                for (const VertexId id : key.ids)
                {
                    hash ^= static_cast<std::size_t>(id);
                    hash *= 1099511628211ull;
                }
                return hash;
            }
        };

        TriangleKey triangleKey(std::array<VertexId,3> ids)
        {
            std::sort(ids.begin(), ids.end());
            return {ids};
        }

        void countCandidateTriangle(
            std::unordered_map<TriangleKey,std::uint32_t,TriangleKeyHash>
                &owners,
            std::array<VertexId,3> ids)
        {
            const auto found = owners.find(triangleKey(ids));
            if (found != owners.end()) ++found->second;
        }

        void retainVolumeBoundaryTopTriangles(
            SurfaceMesh &top,
            const VolumeMesh &mesh,
            const std::unordered_set<TriangleKey,TriangleKeyHash>
                &ownerless_zero_layer_caps)
        {
            // With a pure multi-normal transition there are no regular
            // cells yet; ownership can only be checked after the meshes are
            // merged, so the transformed-front candidates must survive.
            if (mesh.cells.empty()) return;

            std::unordered_map<TriangleKey,std::uint32_t,TriangleKeyHash>
                owners;
            owners.reserve(top.faces.size());
            for (const SurfaceFace &face : top.faces)
                if (const auto *triangle = std::get_if<Triangle>(&face))
                    owners.try_emplace(
                        triangleKey(triangle->vertex_ids),0);
            for (const VolumeCell &cell : mesh.cells)
                std::visit([&](const auto &value)
                {
                    using Cell = std::decay_t<decltype(value)>;
                    const auto &v = value.vertex_ids;
                    if constexpr (std::is_same_v<Cell,Tetra>)
                    {
                        countCandidateTriangle(owners,{v[0],v[1],v[2]});
                        countCandidateTriangle(owners,{v[0],v[3],v[1]});
                        countCandidateTriangle(owners,{v[1],v[3],v[2]});
                        countCandidateTriangle(owners,{v[2],v[3],v[0]});
                    }
                    else if constexpr (std::is_same_v<Cell,Prism>)
                    {
                        countCandidateTriangle(owners,{v[0],v[1],v[2]});
                        countCandidateTriangle(owners,{v[3],v[5],v[4]});
                    }
                    else if constexpr (std::is_same_v<Cell,Pyramid>)
                    {
                        countCandidateTriangle(owners,{v[0],v[4],v[1]});
                        countCandidateTriangle(owners,{v[1],v[4],v[2]});
                        countCandidateTriangle(owners,{v[2],v[4],v[3]});
                        countCandidateTriangle(owners,{v[3],v[4],v[0]});
                    }
                },cell);

            SurfaceMesh exposed;
            exposed.vertices = top.vertices;
            std::unordered_set<TriangleKey,TriangleKeyHash> emitted;
            emitted.reserve(owners.size());
            for (std::size_t index = 0; index < top.faces.size(); ++index)
                if (const auto *triangle =
                        std::get_if<Triangle>(&top.faces[index]))
                {
                    const TriangleKey key = triangleKey(
                        triangle->vertex_ids);
                    const auto found = owners.find(key);
                    if (found != owners.end() &&
                        (found->second == 1 ||
                         (found->second == 0 &&
                          ownerless_zero_layer_caps.find(key) !=
                              ownerless_zero_layer_caps.end())) &&
                        emitted.insert(key).second)
                    {
                        exposed.faces.push_back(*triangle);
                        exposed.face_tags.push_back(top.face_tags[index]);
                    }
                }
            top = std::move(exposed);
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
        std::unordered_set<TriangleKey,TriangleKeyHash>
            ownerless_zero_layer_caps;
        std::vector<FinalQuad> final_quads;
        std::vector<FinalTriangle> final_triangles;
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
        std::unordered_map<std::uint64_t, std::size_t> top_prism;
        for (std::size_t index = 0; index < result.mesh.cells.size(); ++index)
            if (index < result.mesh.metadata.size())
            {
                const std::uint64_t key = cellKey(
                    result.mesh.metadata[index].source_face_id,
                    result.mesh.metadata[index].layer);
                if (std::holds_alternative<Hexa>(result.mesh.cells[index]))
                    top_hexa[key] = index;
                else if (std::holds_alternative<Prism>(result.mesh.cells[index]))
                    top_prism[key] = index;
            }

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
                            InvalidTransitionTemplateInput{id,1}}});
            const std::uint32_t region =
                surface_mesh.face_tags[id].region_id;
            if (const auto *quad = std::get_if<Quad>(face))
            {
                std::array<VertexId, 4> top_ids = quad->vertex_ids;
                std::array<VertexId, 4> bottom_ids = quad->vertex_ids;
                std::vector<QuadHighNeighbor> high_neighbors;
                for (std::size_t edge = 0; edge < 4; ++edge)
                {
                    const auto uses = edge_faces.find(edgeKey(
                        quad->vertex_ids[edge],
                        quad->vertex_ids[(edge + 1) % 4]));
                    if (uses == edge_faces.end()) continue;
                    for (const SurfaceFaceId other_id : uses->second)
                    {
                        if (other_id == id) continue;
                        const auto other = growth_faces.find(other_id);
                        if (other != growth_faces.end() &&
                            other->second->accepted_layer_count ==
                                growth->accepted_layer_count + 1)
                            high_neighbors.push_back({edge, other_id});
                    }
                }
                if (growth->accepted_layer_count > 0)
                {
                    const auto cell = top_hexa.find(cellKey(
                        id, growth->accepted_layer_count));
                    if (cell == top_hexa.end())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{id,2}}});
                    const Hexa hexa = std::get<Hexa>(
                        result.mesh.cells[cell->second]);
                    std::copy_n(hexa.vertex_ids.begin(), 4,
                                bottom_ids.begin());
                    std::copy_n(hexa.vertex_ids.begin() + 4, 4,
                                top_ids.begin());
                    std::array<VertexId, 4> selector_high = top_ids;
                    for (const auto &neighbor : high_neighbors)
                        for (const std::size_t local : {
                                 neighbor.local_edge,
                                 (neighbor.local_edge + 1) % 4})
                        {
                            const auto &vertex = initial_front.vertices[
                                quad->vertex_ids[local]];
                            const auto *record = layerRecord(
                                result.layer_vertices,
                                vertex.source_vertex_id,
                                vertex.branch_id);
                            if (record != nullptr &&
                                record->layer_vertex_ids.size() >
                                    growth->accepted_layer_count + 1)
                                selector_high[local] = record->layer_vertex_ids[
                                    growth->accepted_layer_count + 1];
                        }
                    const auto selection = selectQuadHighNeighbors({
                        id, growth->accepted_layer_count, high_neighbors,
                        top_ids, selector_high, &result.mesh.vertices, 1e-12});
                    if (!selection.hasValue())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                atStage(selection.error(),6)});
                    const auto diagonal = diagonals.resolve(
                        {id, growth->accepted_layer_count},
                        oriented(top_ids, result.mesh.vertices),
                        selection.value().required_low_diagonal, 1e-12);
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
                            IncrementalLayerGrowthError{
                                atStage(cap.error(),7)});
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
                        selection.value().retained_local_edges,
                        cap.value().top_faces});
                }
                else
                {
                    std::array<VertexId, 4> selector_high = top_ids;
                    for (const auto &neighbor : high_neighbors)
                        for (const std::size_t local : {
                                 neighbor.local_edge,
                                 (neighbor.local_edge + 1) % 4})
                        {
                            const auto &vertex = initial_front.vertices[
                                quad->vertex_ids[local]];
                            const auto *record = layerRecord(
                                result.layer_vertices,
                                vertex.source_vertex_id,
                                vertex.branch_id);
                            if (record != nullptr &&
                                record->layer_vertex_ids.size() > 1)
                                selector_high[local] =
                                    record->layer_vertex_ids[1];
                        }
                    const auto selection = selectQuadHighNeighbors({
                        id, 0, high_neighbors, top_ids, selector_high,
                        &result.mesh.vertices, 1e-12});
                    if (!selection.hasValue())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                atStage(selection.error(),8)});
                    const auto diagonal = diagonals.resolve(
                        {id, 0}, oriented(top_ids, result.mesh.vertices),
                        selection.value().required_low_diagonal, 1e-12);
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
                        selection.value().retained_local_edges,
                        {split.triangles.begin(), split.triangles.end()}});
                }
            }
            else
            {
                Triangle triangle = std::get<Triangle>(*face);
                if (growth->accepted_layer_count > 0)
                {
                    const auto cell = top_prism.find(cellKey(
                        id, growth->accepted_layer_count));
                    if (cell == top_prism.end())
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{id,3}}});
                    const Prism &prism = std::get<Prism>(
                        result.mesh.cells[cell->second]);
                    std::copy_n(prism.vertex_ids.begin() + 3, 3,
                                triangle.vertex_ids.begin());
                }
                std::optional<std::size_t> high_edge;
                for (std::size_t edge = 0; edge < 3 && !high_edge; ++edge)
                {
                    const auto uses = edge_faces.find(edgeKey(
                        std::get<Triangle>(*face).vertex_ids[edge],
                        std::get<Triangle>(*face).vertex_ids[(edge+1)%3]));
                    if (uses == edge_faces.end()) continue;
                    for (const SurfaceFaceId other_id : uses->second)
                    {
                        if (other_id == id) continue;
                        const auto other = growth_faces.find(other_id);
                        if (other != growth_faces.end() &&
                            other->second->accepted_layer_count ==
                                growth->accepted_layer_count + 1)
                        {
                            high_edge = edge;
                            break;
                        }
                    }
                }
                final_triangles.push_back({
                    id, growth->accepted_layer_count, region,
                    std::get<Triangle>(*face).vertex_ids,
                    triangle.vertex_ids, high_edge, {triangle}});
            }
        }

        for (FinalQuad &low : final_quads)
        {
            if (!low.high_edges.empty())
            {
                std::array<VertexId, 4> high_ids = low.top_ids;
                for (const std::size_t edge : low.high_edges)
                    for (const std::size_t local : {
                             edge, (edge + 1) % 4})
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
                                        low.id,4}}});
                    high_ids[local] =
                        record->layer_vertex_ids[low.layer + 1];
                }
                const auto side = low.high_edges.size() == 1
                    ? buildQuadSideTransition({
                        low.id, low.layer, low.top_ids, high_ids,
                        low.high_edges[0], low.diagonal})
                    : buildQuadAdjacentSideTransition({
                        low.id, low.layer, low.top_ids, high_ids,
                        low.high_edges[0], low.high_edges[1],
                        low.diagonal, &result.mesh.vertices, 1e-12});
                if (!side.hasValue())
                    return GrowthResult::failure(
                        IncrementalLayerGrowthError{
                            atStage(side.error(),9)});
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
            if (low.layer == 0 && low.high_edges.empty())
                for (const Triangle &triangle : low.top_faces)
                    ownerless_zero_layer_caps.insert(
                        triangleKey(triangle.vertex_ids));
            appendTriangles(
                triangular_top, low.top_faces, low.region);
        }
        for (FinalTriangle &low : final_triangles)
        {
            if (low.high_edge.has_value())
            {
                std::array<VertexId,3> high = low.top_ids;
                for (const std::size_t local : {
                         *low.high_edge, (*low.high_edge+1)%3})
                {
                    const auto &vertex = initial_front.vertices[
                        low.source_ids[local]];
                    const LayerVertexRecord *record = layerRecord(
                        result.layer_vertices,
                        vertex.source_vertex_id, vertex.branch_id);
                    if (record == nullptr ||
                        record->layer_vertex_ids.size() <= low.layer + 1)
                        return GrowthResult::failure(
                            IncrementalLayerGrowthError{
                                TransitionTemplateError{
                                    InvalidTransitionTemplateInput{
                                        low.id,5}}});
                    high[local] = record->layer_vertex_ids[low.layer+1];
                }
                const auto side = buildTriangleSideTransition({
                    low.id, low.layer, low.top_ids, high, *low.high_edge});
                if (!side.hasValue())
                    return GrowthResult::failure(
                        IncrementalLayerGrowthError{
                            atStage(side.error(),10)});
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
            if (low.layer == 0 && !low.high_edge.has_value())
                for (const Triangle &triangle : low.top_faces)
                    ownerless_zero_layer_caps.insert(
                        triangleKey(triangle.vertex_ids));
            appendTriangles(
                triangular_top, low.top_faces, low.region);
        }
        triangular_top.vertices = result.mesh.vertices;
        retainVolumeBoundaryTopTriangles(
            triangular_top, result.mesh, ownerless_zero_layer_caps);
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
}
