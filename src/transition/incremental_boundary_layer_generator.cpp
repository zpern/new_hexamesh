#include <algorithm>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <boundary_mesh/transition/incremental_boundary_layer_generator.hpp>
#include <boundary_mesh/transition/corner_suppression.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>
#include <boundary_mesh/transition/layer_transition_resolver.hpp>
#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>

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

        std::vector<VertexId> faceIds(const SurfaceFace &face)
        {
            return std::visit([](const auto &value)
            {
                return std::vector<VertexId>(
                    value.vertex_ids.begin(), value.vertex_ids.end());
            }, face);
        }

        bool retainedFace(
            const std::vector<SurfaceFaceId> &retained, SurfaceFaceId id)
        {
            return std::binary_search(retained.begin(), retained.end(), id);
        }

        void appendOwnedTriangle(
            TransitionBoundaryInput &boundary,
            const Triangle &triangle,
            const std::vector<Point3> &points,
            const std::vector<CollisionVertexKey> &keys,
            const LayerBoundaryOwner &owner)
        {
            OwnedBoundaryTriangle output;
            output.owner = owner;
            for (std::size_t local = 0; local < 3; ++local)
            {
                const std::size_t id = triangle.vertex_ids[local];
                output.points[local] = points[id];
                output.vertex_keys[local] = keys[id];
            }
            boundary.candidate_triangles.push_back(std::move(output));
        }

        void appendFrontFace(
            TransitionBoundaryInput &boundary,
            const GrowthFront &front,
            std::size_t face_index,
            const LayerBoundaryOwner &owner)
        {
            const SurfaceFace &face = front.faces[face_index];
            const auto ids = faceIds(face);
            std::vector<Point3> points;
            std::vector<CollisionVertexKey> keys;
            points.reserve(ids.size());
            keys.reserve(ids.size());
            for (const VertexId local : ids)
            {
                const auto &vertex = front.vertices[local];
                points.push_back(vertex.position);
                keys.push_back({vertex.source_vertex_id,
                                front.layer, vertex.branch_id});
            }
            if (ids.size() == 3)
                appendOwnedTriangle(
                    boundary, Triangle{{0,1,2}}, points, keys, owner);
            else
            {
                const auto diagonal = chooseQuadDiagonal(
                    oriented({0,1,2,3}, points), 1e-12);
                if (!diagonal.hasValue()) return;
                for (const Triangle &triangle : diagonal.value().triangles)
                    appendOwnedTriangle(
                        boundary, triangle, points, keys, owner);
            }
        }

        ProvisionalLayerTransitionResult buildProvisionalTransition(
            const GrowthFront &current,
            const GrowthFront &candidate,
            const std::vector<SurfaceFaceId> &retained,
            const LayerFaceSets &face_sets,
            std::optional<TransitionTemplateError> &template_error)
        {
            ProvisionalLayerTransition provisional;
            provisional.all_top_faces_are_triangles = true;
            std::unordered_map<SurfaceFaceId, std::size_t> current_faces;
            std::unordered_map<std::uint64_t, std::vector<SurfaceFaceId>>
                current_edge_faces;
            for (std::size_t index = 0;
                 index < current.faces.size(); ++index)
            {
                const SurfaceFaceId id = current.source_face_ids[index];
                current_faces[id] = index;
                const auto ids = faceIds(current.faces[index]);
                for (std::size_t edge = 0; edge < ids.size(); ++edge)
                    current_edge_faces[edgeKey(
                        ids[edge], ids[(edge+1)%ids.size()])]
                        .push_back(id);
            }
            std::unordered_map<std::uint64_t, std::size_t>
                candidate_vertices;
            for (std::size_t index = 0;
                 index < candidate.vertices.size(); ++index)
            {
                const auto &vertex = candidate.vertices[index];
                candidate_vertices[cellKey(
                    vertex.source_vertex_id, vertex.branch_id)] = index;
            }
            for (std::size_t index = 0;
                 index < candidate.faces.size(); ++index)
            {
                const SurfaceFaceId id = candidate.source_face_ids[index];
                if (!retainedFace(retained, id)) continue;
                appendFrontFace(provisional.boundary, candidate, index,
                    {id, candidate.layer,
                     BoundaryOwnerRole::RegularCandidate, {id}});
            }

            for (const SurfaceFaceId low_id :
                 face_sets.transition_low_faces)
            {
                const auto low_position = current_faces.find(low_id);
                if (low_position == current_faces.end()) continue;
                const SurfaceFace *low_face = &current.faces[
                    low_position->second];
                const auto low_source_ids = faceIds(*low_face);
                if (low_source_ids.size() == 3)
                {
                    std::optional<std::size_t> high_edge;
                    std::vector<SurfaceFaceId> dependencies;
                    for (std::size_t edge = 0;
                         edge < 3 && !high_edge; ++edge)
                    {
                        const auto uses = current_edge_faces.find(edgeKey(
                            low_source_ids[edge],
                            low_source_ids[(edge+1)%3]));
                        if (uses == current_edge_faces.end()) continue;
                        for (const SurfaceFaceId other_id : uses->second)
                            if (other_id != low_id &&
                                retainedFace(retained, other_id))
                            {
                                high_edge = edge;
                                dependencies.push_back(other_id);
                                break;
                            }
                    }
                    if (!high_edge.has_value()) continue;
                    std::vector<Point3> points(6);
                    std::vector<CollisionVertexKey> keys(6);
                    std::array<VertexId,3> low{{0,1,2}};
                    std::array<VertexId,3> high{{0,1,2}};
                    for (std::size_t local = 0; local < 3; ++local)
                    {
                        const auto &vertex = current.vertices[
                            low_source_ids[local]];
                        points[local] = vertex.position;
                        keys[local] = {vertex.source_vertex_id,
                                       current.layer,vertex.branch_id};
                        points[3+local] = vertex.position;
                        keys[3+local] = keys[local];
                    }
                    for (const std::size_t local : {
                             *high_edge, (*high_edge+1)%3})
                    {
                        const auto &vertex = current.vertices[
                            low_source_ids[local]];
                        const auto found = candidate_vertices.find(cellKey(
                            vertex.source_vertex_id,vertex.branch_id));
                        if (found != candidate_vertices.end())
                        {
                            high[local] = static_cast<VertexId>(3+local);
                            points[3+local] = candidate.vertices[
                                found->second].position;
                            keys[3+local] = {vertex.source_vertex_id,
                                            candidate.layer,
                                            vertex.branch_id};
                        }
                    }
                    const auto side = buildTriangleTransition({
                        low_id,1,high_edge,std::nullopt,{low,high}});
                    if (!side.hasValue())
                    {
                        template_error = side.error();
                        continue;
                    }
                    const LayerBoundaryOwner owner{
                        low_id,current.layer,
                        BoundaryOwnerRole::SideTransition,dependencies};
                    for (const Triangle &triangle : side.value().top_faces)
                        appendOwnedTriangle(
                            provisional.boundary,triangle,
                            points,keys,owner);
                    continue;
                }
                if (low_source_ids.size() != 4) continue;

                std::vector<Point3> points(8);
                std::vector<CollisionVertexKey> keys(8);
                std::array<VertexId,4> low{{0,1,2,3}};
                std::array<VertexId,4> high{{0,1,2,3}};
                for (std::size_t local = 0; local < 4; ++local)
                {
                    const auto &vertex = current.vertices[
                        low_source_ids[local]];
                    points[local] = vertex.position;
                    keys[local] = {vertex.source_vertex_id,
                                   current.layer, vertex.branch_id};
                    points[4+local] = vertex.position;
                    keys[4+local] = keys[local];
                    const auto candidate_vertex = candidate_vertices.find(
                        cellKey(vertex.source_vertex_id,
                                vertex.branch_id));
                    if (candidate_vertex != candidate_vertices.end())
                    {
                        points[4+local] = candidate.vertices[
                            candidate_vertex->second].position;
                        keys[4+local] = {vertex.source_vertex_id,
                                        candidate.layer,
                                        vertex.branch_id};
                    }
                }

                std::vector<QuadHighNeighbor> neighbors;
                std::vector<SurfaceFaceId> dependencies;
                for (std::size_t edge = 0; edge < 4; ++edge)
                {
                    const auto uses = current_edge_faces.find(edgeKey(
                        low_source_ids[edge],
                        low_source_ids[(edge+1)%4]));
                    if (uses == current_edge_faces.end()) continue;
                    for (const SurfaceFaceId other_id : uses->second)
                        if (other_id != low_id &&
                            retainedFace(retained, other_id))
                        {
                            neighbors.push_back({edge,other_id});
                            dependencies.push_back(other_id);
                        }
                }
                if (neighbors.empty()) continue;
                for (const auto &neighbor : neighbors)
                    for (const std::size_t local : {
                             neighbor.local_edge,
                             (neighbor.local_edge+1)%4})
                        high[local] = static_cast<VertexId>(4+local);

                const auto selection = selectQuadHighNeighbors({
                    low_id, current.layer, neighbors, low, high,
                    &points, 1e-12});
                if (!selection.hasValue())
                {
                    template_error = selection.error();
                    continue;
                }
                dependencies.clear();
                for (const auto &neighbor : neighbors)
                    if (std::find(
                            selection.value().retained_local_edges.begin(),
                            selection.value().retained_local_edges.end(),
                            neighbor.local_edge) !=
                        selection.value().retained_local_edges.end())
                        dependencies.push_back(neighbor.neighbor_face_id);
                std::sort(dependencies.begin(), dependencies.end());
                dependencies.erase(std::unique(
                    dependencies.begin(), dependencies.end()),
                    dependencies.end());

                QuadDiagonal diagonal{};
                if (selection.value().required_low_diagonal.has_value())
                    diagonal = *selection.value().required_low_diagonal;
                else
                {
                    const auto chosen = chooseQuadDiagonal(
                        oriented(low, points), 1e-12);
                    if (!chosen.hasValue())
                    {
                        template_error = TransitionTemplateError{
                            chosen.error()};
                        continue;
                    }
                    diagonal = chosen.value().diagonal;
                }
                provisional.boundary.diagonal_requirements.push_back(
                    {{low_id,current.layer},diagonal});
                std::vector<Triangle> low_cap = diagonal ==
                        QuadDiagonal::ZeroTwo
                    ? std::vector<Triangle>{
                        Triangle{{0,1,2}}, Triangle{{0,2,3}}}
                    : std::vector<Triangle>{
                        Triangle{{1,2,3}}, Triangle{{1,3,0}}};
                const LayerBoundaryOwner owner{
                    low_id, current.layer, BoundaryOwnerRole::TopCap,
                    dependencies};
                for (const Triangle &triangle : low_cap)
                    appendOwnedTriangle(
                        provisional.boundary, triangle,
                        points, keys, owner);

                const auto side =
                    selection.value().retained_local_edges.size() == 1
                    ? buildQuadSideTransition({
                        low_id,current.layer,low,high,
                        selection.value().retained_local_edges[0],diagonal})
                    : buildQuadAdjacentSideTransition({
                        low_id,current.layer,low,high,
                        selection.value().retained_local_edges[0],
                        selection.value().retained_local_edges[1],
                        diagonal,&points,1e-12});
                if (!side.hasValue())
                {
                    template_error = side.error();
                    continue;
                }
                const LayerBoundaryOwner side_owner{
                    low_id, current.layer,
                    BoundaryOwnerRole::SideTransition, dependencies};
                for (const Triangle &triangle : side.value().top_faces)
                    appendOwnedTriangle(
                        provisional.boundary, triangle,
                        points, keys, side_owner);
            }
            return ProvisionalLayerTransitionResult::success(
                std::move(provisional));
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
                            InvalidTransitionTemplateInput{id}}});
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
                                    InvalidTransitionTemplateInput{id}}});
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
                                selection.error()});
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
                                selection.error()});
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
                                    InvalidTransitionTemplateInput{id}}});
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
                                        low.id}}});
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
                                        low.id}}});
                    high[local] = record->layer_vertex_ids[low.layer+1];
                }
                const auto side = buildTriangleTransition({
                    low.id, 1, low.high_edge, std::nullopt,
                    {low.top_ids,high}});
                if (!side.hasValue())
                    return GrowthResult::failure(
                        IncrementalLayerGrowthError{side.error()});
                result.mesh.cells.insert(
                    result.mesh.cells.end(),
                    side.value().volume_cells.begin(),
                    side.value().volume_cells.end());
                for (CellMetadata metadata : side.value().metadata)
                {
                    metadata.layer = low.layer + 1;
                    result.mesh.metadata.push_back(metadata);
                }
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
        RegularLayerGrowthOptions coordinated_options = options;
        std::optional<IncrementalLayerGrowthError> incremental_error;
        const auto upstream_rejections = options.candidate_rejections;
        coordinated_options.candidate_rejections =
            [upstream_rejections, &incremental_error](
                const GrowthFront &current,
                const LayerStepResult &candidate,
                const std::vector<VertexId> &current_global_ids,
                const CollisionIndex &original_surface,
                const ExposedBoundaryTracker &historical_boundary)
        {
            std::vector<SurfaceFaceId> rejected = upstream_rejections
                ? upstream_rejections(
                    current, candidate, current_global_ids,
                    original_surface, historical_boundary)
                : std::vector<SurfaceFaceId>{};
            LayerFaceSets face_sets;
            const std::unordered_set<SurfaceFaceId> continuing(
                candidate.next_front.source_face_ids.begin(),
                candidate.next_front.source_face_ids.end());
            const std::unordered_set<SurfaceFaceId> upstream_rejected_ids(
                rejected.begin(), rejected.end());
            GrowthFront filtered_candidate = candidate.next_front;
            filtered_candidate.faces.clear();
            filtered_candidate.source_face_ids.clear();
            for (std::size_t index = 0;
                 index < candidate.next_front.faces.size(); ++index)
                if (upstream_rejected_ids.find(
                        candidate.next_front.source_face_ids[index]) ==
                    upstream_rejected_ids.end())
                {
                    filtered_candidate.faces.push_back(
                        candidate.next_front.faces[index]);
                    filtered_candidate.source_face_ids.push_back(
                        candidate.next_front.source_face_ids[index]);
                }
            for (const SurfaceFaceId id : current.source_face_ids)
            {
                if (continuing.find(id) == continuing.end() ||
                    upstream_rejected_ids.find(id) !=
                        upstream_rejected_ids.end())
                    addInitialStop(face_sets, {
                        id, current.layer, StopOrigin::Quality});
            }
            if (face_sets.transition_low_faces.empty())
                return rejected;
            std::optional<TransitionTemplateError> template_error;
            LayerTransitionInput input;
            input.current_front = current;
            input.candidate_front = std::move(filtered_candidate);
            input.face_sets = std::move(face_sets);
            input.completed_layer = current.layer;
            input.original_surface = original_surface;
            input.historical_boundary = &historical_boundary;
            input.build_provisional =
                [&current, &candidate, &template_error](
                    const std::vector<SurfaceFaceId> &retained,
                    const LayerFaceSets &sets)
            {
                return buildProvisionalTransition(
                    current, candidate.next_front,
                    retained, sets, template_error);
            };
            const auto stable = LayerTransitionResolver{}.resolve(input);
            if (template_error.has_value())
            {
                incremental_error = IncrementalLayerGrowthError{
                    *template_error};
                return rejected;
            }
            if (!stable.hasValue())
            {
                std::visit([&](const auto &error)
                {
                    incremental_error = IncrementalLayerGrowthError{error};
                }, stable.error());
                return rejected;
            }
            for (const SurfaceFaceId id :
                 candidate.next_front.source_face_ids)
                if (!retainedFace(
                        stable.value().retained_high_faces, id))
                    rejected.push_back(id);
            std::sort(rejected.begin(), rejected.end());
            rejected.erase(
                std::unique(rejected.begin(), rejected.end()),
                rejected.end());
            return rejected;
        };
        auto regular = generateRegularLayers(
            surface_mesh, topology, patch, initial_front,
            profiles, coordinated_options);
        if (incremental_error.has_value())
            return GrowthResult::failure(
                *incremental_error);
        if (!regular.hasValue())
            return GrowthResult::failure(
                IncrementalLayerGrowthError{regular.error()});
        return finalizeIncrementalLayerTopology(
            surface_mesh, initial_front, std::move(regular.value()));
    }
}
