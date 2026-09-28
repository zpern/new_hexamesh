#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <optional>
#include <type_traits>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <iostream>

#include <boundary_mesh/boundary_layer/incremental_topology_finalizer.hpp>
#include <boundary_mesh/transition/layer_quad_diagonal_table.hpp>
#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>
#include <boundary_mesh/transition/triangle_side_transition.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool traceTransitionFace(SurfaceFaceId id)
        {
            const char *value = std::getenv("BOUNDARY_MESH_TRACE_FACE");
            if (value == nullptr) return false;
            while (*value != '\0')
            {
                char *end = nullptr;
                const auto parsed = std::strtoull(value, &end, 10);
                if (end != value && parsed == id) return true;
                if (end == value) break;
                value = end;
                while (*value == ',' || *value == ';' || *value == ' ')
                    ++value;
            }
            return false;
        }

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
            const std::unordered_map<std::uint64_t, const LayerVertexRecord *> &table,
            VertexId source_vertex_id,
            std::uint32_t branch_id)
        {
            const auto key = (static_cast<std::uint64_t>(source_vertex_id) << 32) | branch_id;
            const auto found = table.find(key);
            return found == table.end() ? nullptr : found->second;
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
        bool split_failed_column{};
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
            const std::vector<Point3> &points,
            std::vector<VertexId> &source_to_output,
            std::map<std::array<Scalar,3>,VertexId> &farfield_points,
            bool reuse_farfield_coordinate)
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
                    const VertexId source_id = id;
                    if (source_to_output[source_id] !=
                        std::numeric_limits<VertexId>::max())
                    {
                        id = source_to_output[source_id];
                        continue;
                    }
                    const Point3 &point = points[source_id];
                    const std::array<Scalar,3> point_key{
                        point.x(),point.y(),point.z()};
                    const auto existing = reuse_farfield_coordinate
                        ? farfield_points.find(point_key)
                        : farfield_points.end();
                    if (existing != farfield_points.end())
                        id = existing->second;
                    else
                    {
                        if (output.vertices.size() >
                            static_cast<std::size_t>(
                                std::numeric_limits<VertexId>::max()))
                        {
                            valid = false;
                            return;
                        }
                        id = static_cast<VertexId>(output.vertices.size());
                        output.vertices.push_back(point);
                        if (!reuse_farfield_coordinate)
                            farfield_points.emplace(point_key,id);
                    }
                    source_to_output[source_id] = id;
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

        struct HexaColumnSplitEvaluation
        {
            QuadDiagonal diagonal{};
            std::size_t bad_prism_count{};
            Scalar worst_skewness{};
            std::vector<std::array<Prism,2>> split_cells;
        };

        std::array<Prism,2> splitHexa(
            const Hexa &hexa, QuadDiagonal diagonal)
        {
            const auto &v = hexa.vertex_ids;
            if (diagonal == QuadDiagonal::ZeroTwo)
                return {Prism{{v[0],v[1],v[2],v[4],v[5],v[6]}},
                        Prism{{v[0],v[2],v[3],v[4],v[6],v[7]}}};
            return {Prism{{v[1],v[2],v[3],v[5],v[6],v[7]}},
                    Prism{{v[1],v[3],v[0],v[5],v[7],v[4]}}};
        }

        std::optional<HexaColumnSplitEvaluation> evaluateColumnSplit(
            const std::vector<Hexa> &hexas,
            const std::vector<Point3> &vertices,
            QuadDiagonal diagonal)
        {
            if (hexas.empty()) return std::nullopt;
            HexaColumnSplitEvaluation result;
            result.diagonal = diagonal;
            result.split_cells.reserve(hexas.size());
            for (const Hexa &hexa : hexas)
            {
                const auto prisms = splitHexa(hexa, diagonal);
                for (const Prism &prism : prisms)
                {
                    PrismPoints points{};
                    for (std::size_t corner = 0; corner < 6; ++corner)
                    {
                        const VertexId id = prism.vertex_ids[corner];
                        if (static_cast<std::size_t>(id) >= vertices.size())
                            return std::nullopt;
                        points[corner] = vertices[id];
                    }
                    const auto quality = evaluatePrism(
                        points, VolumeCellQualityOptions{Scalar{1}});
                    if (!quality.hasValue() ||
                        quality.value().validity != VolumeCellValidity::Valid ||
                        !std::isfinite(quality.value().skewness))
                        return std::nullopt;
                    if (quality.value().skewness > Scalar{0.9})
                        ++result.bad_prism_count;
                    result.worst_skewness = std::max(
                        result.worst_skewness, quality.value().skewness);
                }
                result.split_cells.push_back(prisms);
            }
            return result;
        }

    }


    Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
    finalizeIncrementalLayerTopology(
        const SurfaceMesh &surface_mesh,
        const GrowthFront &initial_front,
        RegularLayerGrowthResult result,
        const std::vector<ResolvedTransitionTopology> &resolved_topology,
        bool split_failed_hexa_columns)
    {
        using GrowthResult = Result<
            RegularLayerGrowthResult, IncrementalLayerGrowthError>;
        const auto finalization_started = std::chrono::steady_clock::now();
        LayerQuadDiagonalTable diagonals;
        std::unordered_map<std::uint64_t, const LayerVertexRecord *> layer_records;
        layer_records.reserve(result.layer_vertices.size());
        for (const auto &record : result.layer_vertices)
            layer_records.try_emplace(
                (static_cast<std::uint64_t>(record.source_vertex_id) << 32) | record.branch_id,
                &record);
        SurfaceMesh triangular_top;
        triangular_top.vertices = result.mesh.vertices;
        std::unordered_set<TriangleKey,TriangleKeyHash>
            ownerless_zero_layer_caps;
        std::vector<FinalQuad> final_quads;
        std::vector<FinalTriangle> final_triangles;
        std::size_t diagnostic_external_patches = 0;
        std::size_t diagnostic_internal_splits = 0;
        std::size_t diagnostic_external_bad = 0;
        std::size_t diagnostic_internal_bad = 0;
        std::size_t terminal_center_solves = 0;
        const auto count_bad = [&](const auto &cells, bool external)
        {
#ifndef NDEBUG
            for (const auto &cell : cells)
            {
                const auto quality = std::visit([&](const auto &value)
                {
                    using Cell = std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<Cell, Tetra>)
                        return evaluateTetra(TetraPoints{
                            result.mesh.vertices[value.vertex_ids[0]],
                            result.mesh.vertices[value.vertex_ids[1]],
                            result.mesh.vertices[value.vertex_ids[2]],
                            result.mesh.vertices[value.vertex_ids[3]]});
                    else if constexpr (std::is_same_v<Cell, Pyramid>)
                        return evaluatePyramid(PyramidPoints{
                            result.mesh.vertices[value.vertex_ids[0]],
                            result.mesh.vertices[value.vertex_ids[1]],
                            result.mesh.vertices[value.vertex_ids[2]],
                            result.mesh.vertices[value.vertex_ids[3]],
                            result.mesh.vertices[value.vertex_ids[4]]});
                    else if constexpr (std::is_same_v<Cell, Prism>)
                        return evaluatePrism(PrismPoints{
                            result.mesh.vertices[value.vertex_ids[0]],
                            result.mesh.vertices[value.vertex_ids[1]],
                            result.mesh.vertices[value.vertex_ids[2]],
                            result.mesh.vertices[value.vertex_ids[3]],
                            result.mesh.vertices[value.vertex_ids[4]],
                            result.mesh.vertices[value.vertex_ids[5]]});
                    else
                        return evaluateHexa(HexaPoints{
                            result.mesh.vertices[value.vertex_ids[0]],
                            result.mesh.vertices[value.vertex_ids[1]],
                            result.mesh.vertices[value.vertex_ids[2]],
                            result.mesh.vertices[value.vertex_ids[3]],
                            result.mesh.vertices[value.vertex_ids[4]],
                            result.mesh.vertices[value.vertex_ids[5]],
                            result.mesh.vertices[value.vertex_ids[6]],
                            result.mesh.vertices[value.vertex_ids[7]]});
                }, cell);
                if (quality.hasValue() && quality.value().skewness > Scalar{0.9})
                    external ? ++diagnostic_external_bad : ++diagnostic_internal_bad;
            }
#else
            (void)cells;
            (void)external;
#endif
        };
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
        std::unordered_map<std::uint64_t,
            const ResolvedTransitionTopology *> stable_topology;
        for (const auto &topology : resolved_topology)
            stable_topology[cellKey(
                topology.source_face_id, topology.layer)] = &topology;
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
                                layer_records,
                                vertex.source_vertex_id,
                                vertex.branch_id);
                            if (record != nullptr &&
                                record->layer_vertex_ids.size() >
                                    growth->accepted_layer_count + 1)
                                selector_high[local] = record->layer_vertex_ids[
                                    growth->accepted_layer_count + 1];
                        }
                    const auto stable_position = stable_topology.find(
                        cellKey(id, growth->accepted_layer_count));
                    const ResolvedTransitionTopology *stable =
                        stable_position == stable_topology.end()
                            ? nullptr : stable_position->second;
                    QuadDiagonal final_diagonal{};
                    std::vector<std::size_t> final_high_edges;
                    const bool has_stable_quad_topology =
                        stable != nullptr &&
                        stable->low_diagonal.has_value();
                    if (has_stable_quad_topology)
                    {
                        final_diagonal = *stable->low_diagonal;
                        final_high_edges = stable->retained_local_edges;
                    }
                    else
                    {
                        const auto selection = selectQuadHighNeighbors({
                            id, growth->accepted_layer_count, high_neighbors,
                            top_ids, selector_high, &result.mesh.vertices,
                            1e-12});
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
                        final_diagonal = diagonal.value();
                        final_high_edges =
                            selection.value().retained_local_edges;
                    }

                    const std::array<Point3, 4> bottom_points{{
                        result.mesh.vertices[bottom_ids[0]],
                        result.mesh.vertices[bottom_ids[1]],
                        result.mesh.vertices[bottom_ids[2]],
                        result.mesh.vertices[bottom_ids[3]]}};
                    const std::array<Point3, 4> top_points{{
                        result.mesh.vertices[top_ids[0]],
                        result.mesh.vertices[top_ids[1]],
                        result.mesh.vertices[top_ids[2]],
                        result.mesh.vertices[top_ids[3]]}};
                    std::optional<Point3> internal_center;
                    if (stable == nullptr ||
                        stable->terminal_quad_decision ==
                            TerminalQuadDecision::InternalSplit)
                    {
                        internal_center = stable != nullptr
                                ? stable->generated_point
                                : std::optional<Point3>{};
                        if (!internal_center.has_value())
                        {
                            ++terminal_center_solves;
                            internal_center =
                                findPositiveQuadTopCapCenter({
                                    bottom_points, top_points, final_diagonal,
                                    Scalar{1e-12}});
                        }
                    }

                    // The finalizer must never invent an unchecked external
                    // patch.  If no stable resolver decision exists and the
                    // internal split is invalid, preserve the Hexa and report
                    // the missing decision.
                    if (stable == nullptr && !internal_center.has_value())
                    {
                        const auto split = final_diagonal ==
                                QuadDiagonal::ZeroTwo
                            ? std::vector<Triangle>{
                                Triangle{{top_ids[0],top_ids[1],top_ids[2]}},
                                Triangle{{top_ids[0],top_ids[2],top_ids[3]}}}
                            : std::vector<Triangle>{
                                Triangle{{top_ids[1],top_ids[2],top_ids[3]}},
                                Triangle{{top_ids[1],top_ids[3],top_ids[0]}}};
                        result.terminal_transition_diagnostics.push_back({
                            id,growth->accepted_layer_count,cell->second,
                            quadTopCapAspectRatio({bottom_points,top_points}),
                            {},{},
                            "stable terminal decision unavailable; kept Hexa"});
                        final_quads.push_back({
                            id,growth->accepted_layer_count,region,
                            quad->vertex_ids,top_ids,final_diagonal,{},
                            split});
                        final_quads.back().split_failed_column = true;
                        continue;
                    }

                    if (stable != nullptr &&
                        stable->terminal_quad_decision ==
                            TerminalQuadDecision::ExternalPatch)
                    {
                        ++diagnostic_external_patches;
                        if (!stable->generated_point.has_value() ||
                            result.mesh.vertices.size() >
                                static_cast<std::size_t>(
                                    std::numeric_limits<VertexId>::max()))
                            return GrowthResult::failure(
                                IncrementalLayerGrowthError{
                                    TransitionTemplateError{
                                        InvalidTransitionTemplateInput{
                                            id,22}}});
                        const VertexId apex = static_cast<VertexId>(
                            result.mesh.vertices.size());
                        const auto patch = buildExternalQuadPatch({
                            id,growth->accepted_layer_count,
                            top_ids,selector_high,final_high_edges,
                            &result.mesh.vertices,apex,Scalar{0.25},
                            Scalar{1e-12},stable->generated_point});
                        if (!patch.hasValue())
                            return GrowthResult::failure(
                                IncrementalLayerGrowthError{
                                    atStage(patch.error(),23)});
                        result.mesh.vertices.insert(
                            result.mesh.vertices.end(),
                            patch.value().created_vertices.begin(),
                            patch.value().created_vertices.end());
                        result.mesh.cells.insert(
                            result.mesh.cells.end(),
                            patch.value().volume_cells.begin(),
                            patch.value().volume_cells.end());
                        result.mesh.metadata.insert(
                            result.mesh.metadata.end(),
                            patch.value().metadata.begin(),
                            patch.value().metadata.end());
                        count_bad(patch.value().volume_cells, true);
                        final_quads.push_back({
                            id,growth->accepted_layer_count,region,
                            quad->vertex_ids,top_ids,final_diagonal,{},
                            patch.value().top_faces});
                        continue;
                    }

                    if (stable != nullptr &&
                        stable->terminal_quad_decision ==
                            TerminalQuadDecision::KeepHexa)
                    {
                        const auto split = final_diagonal ==
                                QuadDiagonal::ZeroTwo
                            ? std::vector<Triangle>{
                                Triangle{{top_ids[0],top_ids[1],top_ids[2]}},
                                Triangle{{top_ids[0],top_ids[2],top_ids[3]}}}
                            : std::vector<Triangle>{
                                Triangle{{top_ids[1],top_ids[2],top_ids[3]}},
                                Triangle{{top_ids[1],top_ids[3],top_ids[0]}}};
                        result.terminal_transition_diagnostics.push_back({
                            id,growth->accepted_layer_count,cell->second,
                            stable->aspect_ratio,
                            stable->dependent_high_faces,{},
                            "no positive non-intersecting terminal quad patch"});
                        final_quads.push_back({
                            id,growth->accepted_layer_count,region,
                            quad->vertex_ids,top_ids,final_diagonal,{},split});
                        final_quads.back().split_failed_column = true;
                        continue;
                    }
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
                        center, final_diagonal,
                        stable != nullptr && stable->generated_point.has_value()
                            ? stable->generated_point
                            : internal_center});
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
                    count_bad(cap.value().volume_cells, false);
                    ++diagnostic_internal_splits;
                    final_quads.push_back({
                        id, growth->accepted_layer_count, region,
                        quad->vertex_ids, top_ids, final_diagonal,
                        final_high_edges,
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
                                layer_records,
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
                if (traceTransitionFace(id))
                {
                    std::cerr << "trace final triangle-side face=" << id
                              << " accepted=" << growth->accepted_layer_count
                              << " high_edge=";
                    if (high_edge) std::cerr << *high_edge;
                    else std::cerr << "none";
                    std::cerr << '\n';
                }
                final_triangles.push_back({
                    id, growth->accepted_layer_count, region,
                    std::get<Triangle>(*face).vertex_ids,
                    triangle.vertex_ids, high_edge, {triangle}});
            }
        }

        std::unordered_map<std::size_t, std::array<Prism,2>> prism_replacements;
        std::unordered_map<SurfaceFaceId, QuadDiagonal> split_diagonals;
        std::size_t split_column_count = 0;
        if (split_failed_hexa_columns)
            for (FinalQuad &terminal : final_quads)
            {
                if (!terminal.split_failed_column) continue;
                std::vector<std::pair<std::uint32_t,std::size_t>> column_indices;
                std::vector<Hexa> column_hexas;
                for (std::size_t index = 0;
                     index < result.mesh.cells.size() &&
                         index < result.mesh.metadata.size();
                     ++index)
                    if (result.mesh.metadata[index].source_face_id == terminal.id &&
                        std::holds_alternative<Hexa>(result.mesh.cells[index]))
                        column_indices.emplace_back(
                            result.mesh.metadata[index].layer,index);
                std::sort(column_indices.begin(), column_indices.end());
                for (const auto &[layer,index] : column_indices)
                {
                    (void)layer;
                    column_hexas.push_back(
                        std::get<Hexa>(result.mesh.cells[index]));
                }
                const auto zero_two = evaluateColumnSplit(
                    column_hexas,result.mesh.vertices,QuadDiagonal::ZeroTwo);
                const auto one_three = evaluateColumnSplit(
                    column_hexas,result.mesh.vertices,QuadDiagonal::OneThree);
                const HexaColumnSplitEvaluation *selected = nullptr;
                if (zero_two && one_three)
                    selected =
                        std::tie(zero_two->bad_prism_count,
                                 zero_two->worst_skewness) <=
                                std::tie(one_three->bad_prism_count,
                                         one_three->worst_skewness)
                            ? &*zero_two : &*one_three;
                else if (zero_two)
                    selected = &*zero_two;
                else if (one_three)
                    selected = &*one_three;
                if (selected == nullptr)
                {
                    const auto diagnostic = std::find_if(
                        result.terminal_transition_diagnostics.rbegin(),
                        result.terminal_transition_diagnostics.rend(),
                        [&](const auto &entry)
                        { return entry.source_face_id == terminal.id &&
                                 entry.layer == terminal.layer; });
                    if (diagnostic !=
                        result.terminal_transition_diagnostics.rend())
                        diagnostic->reason +=
                            "; both full-column prism splits were invalid; kept Hexa";
                    else
                        result.terminal_transition_diagnostics.push_back({
                            terminal.id,terminal.layer,0,Scalar{0},{},{},
                            "both full-column prism splits were invalid; kept Hexa"});
                    continue;
                }
                split_diagonals[terminal.id] = selected->diagonal;
                ++split_column_count;
                const auto diagnostic = std::find_if(
                    result.terminal_transition_diagnostics.rbegin(),
                    result.terminal_transition_diagnostics.rend(),
                    [&](const auto &entry)
                    { return entry.source_face_id == terminal.id &&
                             entry.layer == terminal.layer; });
                if (diagnostic !=
                    result.terminal_transition_diagnostics.rend())
                {
                    diagnostic->split_succeeded = true;
                    diagnostic->reason =
                        "no positive non-intersecting terminal quad patch; split column into prisms";
                }
                for (std::size_t index = 0;
                     index < column_indices.size(); ++index)
                    prism_replacements[column_indices[index].second] =
                        selected->split_cells[index];
            }

        if (!prism_replacements.empty())
        {
            std::vector<VolumeCell> split_cells;
            std::vector<CellMetadata> split_metadata;
            split_cells.reserve(result.mesh.cells.size() + prism_replacements.size());
            split_metadata.reserve(result.mesh.metadata.size() + prism_replacements.size());
            for (std::size_t index = 0; index < result.mesh.cells.size(); ++index)
            {
                const auto replacement = prism_replacements.find(index);
                if (replacement == prism_replacements.end())
                {
                    split_cells.push_back(std::move(result.mesh.cells[index]));
                    if (index < result.mesh.metadata.size())
                        split_metadata.push_back(result.mesh.metadata[index]);
                    continue;
                }
                const CellMetadata metadata = result.mesh.metadata[index];
                split_cells.emplace_back(replacement->second[0]);
                split_metadata.push_back(metadata);
                split_cells.emplace_back(replacement->second[1]);
                split_metadata.push_back(metadata);
            }
            result.mesh.cells = std::move(split_cells);
            result.mesh.metadata = std::move(split_metadata);
            for (FinalQuad &terminal : final_quads)
                if (const auto chosen = split_diagonals.find(terminal.id);
                    chosen != split_diagonals.end())
                {
                    terminal.diagonal = chosen->second;
                    const auto &v = terminal.top_ids;
                    terminal.top_faces = chosen->second == QuadDiagonal::ZeroTwo
                        ? std::vector<Triangle>{
                            Triangle{{v[0],v[1],v[2]}},
                            Triangle{{v[0],v[2],v[3]}}}
                        : std::vector<Triangle>{
                            Triangle{{v[1],v[2],v[3]}},
                            Triangle{{v[1],v[3],v[0]}}};
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
                        layer_records,
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
                        layer_records,
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
                if (traceTransitionFace(low.id))
                    for (const Triangle &triangle : side.value().top_faces)
                    {
                        std::cerr << "trace final triangle-side top-face=";
                        for (const VertexId vertex : triangle.vertex_ids)
                        {
                            const Point3 &point = result.mesh.vertices[vertex];
                            std::cerr << '(' << point.x() << ',' << point.y()
                                      << ',' << point.z() << ") ";
                        }
                        std::cerr << '\n';
                    }
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
        std::vector<VertexId> farfield_vertex_remap(
            result.farfield_boundary.vertices.size(),
            std::numeric_limits<VertexId>::max());
        std::vector<VertexId> top_vertex_remap(
            result.mesh.vertices.size(),
            std::numeric_limits<VertexId>::max());
        std::map<std::array<Scalar,3>,VertexId> farfield_points;
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
                    result.farfield_boundary.vertices,
                    farfield_vertex_remap,farfield_points,false))
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
                    result.mesh.vertices,
                    top_vertex_remap,farfield_points,true))
                return GrowthResult::failure(
                    IncrementalLayerGrowthError{
                        TransitionTemplateError{
                            InvalidTransitionTemplateInput{}}});
        result.farfield_boundary = std::move(triangular_farfield);
        result.top_surface = std::move(triangular_top);
        std::cerr << "temporary transition template counts: internal="
                  << diagnostic_internal_splits << " external="
                  << diagnostic_external_patches << " skewness>0.9 internal="
                  << diagnostic_internal_bad << " external="
                  << diagnostic_external_bad
                  << " center-solves=" << terminal_center_solves
                  << " split-columns=" << split_column_count << '\n';
        std::cerr << "temporary finalization ms="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - finalization_started).count() << '\n';
        return GrowthResult::success(std::move(result));
    }
}
