#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <type_traits>
#include <unordered_map>
#include <utility>

#include <boundary_mesh/transition/provisional_transition_builder.hpp>
#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>
#include <boundary_mesh/transition/triangle_side_transition.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>
#include <boundary_mesh/surface/face_skewness.hpp>

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

        void tracePoint(const Point3 &point)
        {
            std::cerr << '(' << point.x() << ',' << point.y() << ','
                      << point.z() << ')';
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

        bool hasStrictlyPositiveTemplateVolumes(
            const TransitionTemplateOutput &output,
            const std::vector<Point3> &points)
        {
            return std::all_of(output.volume_cells.begin(),
                output.volume_cells.end(), [&](const VolumeCell &cell)
            {
                return std::visit([&](const auto &value)
                {
                    using Cell = std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<Cell,Tetra>)
                    {
                        TetraPoints cell_points{};
                        for (std::size_t i = 0; i < 4; ++i)
                            cell_points[i] = points[value.vertex_ids[i]];
                        const auto quality = evaluateTetra(cell_points);
                        return quality.hasValue() &&
                            quality.value().validity ==
                                VolumeCellValidity::Valid;
                    }
                    else if constexpr (std::is_same_v<Cell,Pyramid>)
                    {
                        PyramidPoints cell_points{};
                        for (std::size_t i = 0; i < 5; ++i)
                            cell_points[i] = points[value.vertex_ids[i]];
                        const auto quality = evaluatePyramid(cell_points);
                        return quality.hasValue() &&
                            quality.value().validity ==
                                VolumeCellValidity::Valid;
                    }
                    return true;
                },cell);
            });
        }

        TransitionTemplateResult buildControlledExternalQuadPatch(
            ExternalQuadPatchInput input,
            const ExternalPatchControls &controls)
        {
            const auto explicit_apex = controls.explicit_apex_points.find(
                input.source_face_id);
            if (explicit_apex != controls.explicit_apex_points.end() &&
                input.high_edges.empty())
            {
                input.apex_point = explicit_apex->second;
                return buildExternalQuadPatch(input);
            }
            const auto robust_index = controls.robust_candidate_indices.find(
                input.source_face_id);
            if (robust_index != controls.robust_candidate_indices.end() &&
                input.high_edges.empty())
            {
                const auto candidates =
                    findRobustExternalQuadPatchApexCandidates(
                        input,robust_index->second);
                if (candidates.size() < robust_index->second)
                    return TransitionTemplateResult::failure(
                        TransitionTemplateError{
                            InvalidTransitionTemplateInput{
                                input.source_face_id,25}});
                input.apex_point = candidates[robust_index->second-1];
                return buildExternalQuadPatch(input);
            }
            const std::size_t candidate_index =
                controls.apexCandidateIndex(input.source_face_id);
            if (candidate_index > 0)
            {
                const auto candidates = findExternalQuadPatchApexCandidates(
                    input, candidate_index);
                if (candidates.size() < candidate_index)
                    return TransitionTemplateResult::failure(
                        TransitionTemplateError{
                            InvalidTransitionTemplateInput{
                                input.source_face_id, 24}});
                input.apex_point = candidates[candidate_index - 1];
            }
            auto patch = buildExternalQuadPatch(input);
            if (patch.hasValue() || !input.high_edges.empty()) return patch;
            const auto center_ray = findExternalQuadPatchApexCandidates(
                input,1);
            if (!center_ray.empty())
            {
                input.apex_point = center_ray.front();
                patch = buildExternalQuadPatch(input);
                if (patch.hasValue()) return patch;
            }
            const auto fallback = findRobustExternalQuadPatchApexCandidates(
                input,1);
            if (fallback.empty()) return patch;
            input.apex_point = fallback.front();
            return buildExternalQuadPatch(input);
        }

        bool containsRegion(
            const std::vector<std::uint32_t> &ids, std::uint32_t region)
        {
            return std::find(ids.begin(), ids.end(), region) != ids.end();
        }

        std::vector<std::uint32_t> commonRegions(
            const Triangle &triangle,
            const std::vector<std::vector<std::uint32_t>> &regions)
        {
            std::vector<std::uint32_t> common =
                regions[triangle.vertex_ids[0]];
            std::sort(common.begin(), common.end());
            common.erase(std::unique(common.begin(), common.end()),
                         common.end());
            for (std::size_t corner = 1; corner < 3; ++corner)
            {
                const auto &candidate = regions[triangle.vertex_ids[corner]];
                common.erase(std::remove_if(common.begin(), common.end(),
                    [&](std::uint32_t region)
                    { return !containsRegion(candidate, region); }),
                    common.end());
            }
            return common;
        }

        std::uint8_t physicalEdgeMask(
            const Triangle &triangle, std::size_t column_count)
        {
            std::uint8_t mask{};
            for (std::size_t edge = 0; edge < 3; ++edge)
            {
                const VertexId first = triangle.vertex_ids[edge];
                const VertexId second = triangle.vertex_ids[(edge + 1) % 3];
                const std::size_t first_column = first % column_count;
                const std::size_t second_column = second % column_count;
                const bool column_edge = first_column == second_column;
                const bool same_level =
                    (first < column_count) == (second < column_count);
                const bool source_edge = same_level &&
                    ((first_column + 1) % column_count == second_column ||
                     (second_column + 1) % column_count == first_column);
                if (column_edge || source_edge)
                    mask |= static_cast<std::uint8_t>(1u << edge);
            }
            return mask;
        }

        std::shared_ptr<SlidingColumnContext> columnContext(
            const GrowthFront &current,
            const GrowthFront &candidate,
            const std::vector<VertexId> &source_ids,
            const std::unordered_map<std::uint64_t, std::size_t>
                &candidate_vertices)
        {
            auto result = std::make_shared<SlidingColumnContext>();
            for (const VertexId source_id : source_ids)
            {
                const auto &low = current.vertices[source_id];
                result->low_points.push_back(low.position);
                result->low_region_ids.push_back(
                    low.boundary.sliding_region_ids);
                const auto high_position = candidate_vertices.find(
                    cellKey(low.source_vertex_id, low.branch_id));
                if (high_position == candidate_vertices.end())
                {
                    result->high_points.push_back(low.position);
                    result->high_region_ids.push_back(
                        low.boundary.sliding_region_ids);
                }
                else
                {
                    const auto &high = candidate.vertices[
                        high_position->second];
                    result->high_points.push_back(high.position);
                    result->high_region_ids.push_back(
                        high.boundary.sliding_region_ids);
                }
            }
            return result;
        }

        std::shared_ptr<SlidingColumnContext> candidateColumnContext(
            const GrowthFront &current,
            const GrowthFront &candidate,
            std::size_t face_index,
            const std::unordered_map<std::uint64_t, std::size_t>
                &current_vertices)
        {
            auto result = std::make_shared<SlidingColumnContext>();
            for (const VertexId local : faceIds(candidate.faces[face_index]))
            {
                const auto &high = candidate.vertices[local];
                const auto low_position = current_vertices.find(
                    cellKey(high.source_vertex_id, high.branch_id));
                if (low_position == current_vertices.end()) return {};
                const auto &low = current.vertices[low_position->second];
                result->low_points.push_back(low.position);
                result->high_points.push_back(high.position);
                result->low_region_ids.push_back(
                    low.boundary.sliding_region_ids);
                result->high_region_ids.push_back(
                    high.boundary.sliding_region_ids);
            }
            return result;
        }

        void appendOwnedTriangle(
            TransitionBoundaryInput &boundary,
            const Triangle &triangle,
            const std::vector<Point3> &points,
            const std::vector<CollisionVertexKey> &keys,
            const LayerBoundaryOwner &owner,
            const std::vector<std::vector<std::uint32_t>> &regions = {},
            std::uint8_t physical_edge_mask = 0,
            const std::vector<std::uint32_t> &complete_regions = {},
            std::shared_ptr<const SlidingColumnContext> columns = {})
        {
            OwnedBoundaryTriangle output;
            output.owner = owner;
            for (std::size_t local = 0; local < 3; ++local)
            {
                const std::size_t id = triangle.vertex_ids[local];
                output.points[local] = points[id];
                output.vertex_keys[local] = keys[id];
                if (id < regions.size())
                    output.vertex_sliding_region_ids[local] = regions[id];
            }
            output.physical_edge_mask = physical_edge_mask;
            output.complete_face_exemption_regions = complete_regions;
            output.sliding_columns = std::move(columns);
            boundary.candidate_triangles.push_back(std::move(output));
        }

        void appendFrontFace(
            TransitionBoundaryInput &boundary,
            const GrowthFront &front,
            std::size_t face_index,
            const LayerBoundaryOwner &owner,
            std::shared_ptr<const SlidingColumnContext> columns = {})
        {
            const SurfaceFace &face = front.faces[face_index];
            const auto ids = faceIds(face);
            std::vector<Point3> points;
            std::vector<CollisionVertexKey> keys;
            std::vector<std::vector<std::uint32_t>> regions;
            points.reserve(ids.size());
            keys.reserve(ids.size());
            regions.reserve(ids.size());
            for (const VertexId local : ids)
            {
                const auto &vertex = front.vertices[local];
                points.push_back(vertex.position);
                keys.push_back({vertex.source_vertex_id,
                                front.layer, vertex.branch_id});
                regions.push_back(vertex.boundary.sliding_region_ids);
            }
            if (ids.size() == 3)
                appendOwnedTriangle(
                    boundary, Triangle{{0,1,2}}, points, keys, owner,
                    regions, 0b111, {}, columns);
            else if (owner.role == BoundaryOwnerRole::RegularCandidate)
            {
                // A retained high face is only a collision proxy for an
                // unsplit regular cell. Match the regular-layer collision
                // checker; canonical/quality diagonals are resolved only
                // after the face becomes a transition low face or final cap.
                appendOwnedTriangle(
                    boundary, Triangle{{0,1,2}}, points, keys, owner,
                    regions, 0b011, {}, columns);
                appendOwnedTriangle(
                    boundary, Triangle{{0,2,3}}, points, keys, owner,
                    regions, 0b110, {}, columns);
            }
            else
            {
                const auto diagonal = chooseQuadDiagonal(
                    oriented({0,1,2,3}, points), 1e-12);
                if (!diagonal.hasValue()) return;
                for (const Triangle &triangle : diagonal.value().triangles)
                    appendOwnedTriangle(
                        boundary, triangle, points, keys, owner, regions,
                        0, {}, columns);
            }
        }
    }

    std::optional<Point3> ProvisionalTransitionBuildContext::positiveCenter(
        SurfaceFaceId id, const PositiveQuadTopCapCenterInput &input) const
    {
        const auto found = center_cache_.find(id);
        if (found != center_cache_.end() && found->second.input.bottom == input.bottom &&
            found->second.input.top == input.top && found->second.input.diagonal == input.diagonal &&
            found->second.input.volume_tolerance == input.volume_tolerance)
            return found->second.result;
        auto result = findPositiveQuadTopCapCenter(input);
        auto stored = input;
        stored.diagnostics = nullptr;
        center_cache_[id] = {stored, result};
        return result;
    }

    ProvisionalTransitionBuildContext::ProvisionalTransitionBuildContext(
        const GrowthFront &current,
        const GrowthFront &candidate)
        : current_(current), candidate_(candidate)
    {
        for (std::size_t index = 0; index < current.faces.size(); ++index)
        {
            const SurfaceFaceId id = current.source_face_ids[index];
            current_faces_[id] = index;
            const auto ids = faceIds(current.faces[index]);
            for (const VertexId vertex : ids)
                current_vertex_faces_[static_cast<std::uint64_t>(vertex)]
                    .push_back(id);
            for (std::size_t edge = 0; edge < ids.size(); ++edge)
                current_edge_faces_[edgeKey(
                    ids[edge], ids[(edge + 1) % ids.size()])]
                    .push_back(id);
        }
        for (std::size_t index = 0; index < candidate.faces.size(); ++index)
            candidate_faces_[candidate.source_face_ids[index]] = index;
        for (std::size_t index = 0;
             index < candidate.vertices.size(); ++index)
        {
            const auto &vertex = candidate.vertices[index];
            candidate_vertices_[cellKey(
                vertex.source_vertex_id, vertex.branch_id)] = index;
        }
        for (std::size_t index = 0;
             index < current.vertices.size(); ++index)
        {
            const auto &vertex = current.vertices[index];
            current_vertices_[cellKey(
                vertex.source_vertex_id, vertex.branch_id)] = index;
        }
    }

    std::vector<SurfaceFaceId> ProvisionalTransitionBuildContext::affectedFaces(
        const std::vector<SurfaceFaceId> &changed_faces) const
    {
        std::vector<SurfaceFaceId> affected = changed_faces;
        for (const auto id : changed_faces)
        {
            const auto found = current_faces_.find(id);
            if (found == current_faces_.end()) continue;
            const auto ids = faceIds(current_.faces[found->second]);
            for (const VertexId vertex : ids)
            {
                const auto uses = current_vertex_faces_.find(
                    static_cast<std::uint64_t>(vertex));
                if (uses != current_vertex_faces_.end())
                    affected.insert(affected.end(),
                        uses->second.begin(), uses->second.end());
            }
        }
        std::sort(affected.begin(), affected.end());
        affected.erase(std::unique(affected.begin(), affected.end()), affected.end());
        return affected;
    }

        static ProvisionalLayerTransitionResult buildProvisionalTransitionImpl(
            const ProvisionalTransitionBuildContext &context,
            const std::vector<SurfaceFaceId> &retained,
            const LayerFaceSets &face_sets,
            const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
                terminal_hexa_points,
            const ExternalPatchControls &external_controls,
            const std::vector<SurfaceFaceId> &terminal_candidate_faces,
            const std::vector<SurfaceFaceId> *selected_external_faces,
            bool include_regular = false)
        {
            const GrowthFront &current = context.current();
            const GrowthFront &candidate = context.candidate();
            const auto &current_faces = context.currentFaces();
            const auto &candidate_faces = context.candidateFaces();
            const auto &current_edge_faces = context.currentEdgeFaces();
            const auto &current_vertices = context.currentVertices();
            const auto &candidate_vertices = context.candidateVertices();
            ProvisionalLayerTransition provisional;
            provisional.all_top_faces_are_triangles = true;
            const bool external_only = selected_external_faces != nullptr;
            std::vector<SurfaceFaceId> selected;
            if (selected_external_faces != nullptr)
            {
                selected = *selected_external_faces;
                std::sort(selected.begin(), selected.end());
                selected.erase(
                    std::unique(selected.begin(), selected.end()),
                    selected.end());
            }
            const auto selectedExternal = [&](SurfaceFaceId id)
            {
                return !external_only ||
                    std::binary_search(selected.begin(), selected.end(), id);
            };
            std::vector<SurfaceFaceId> selected_terminal_faces;
            const std::vector<SurfaceFaceId> *terminal_faces_to_build =
                &terminal_candidate_faces;
            std::vector<SurfaceFaceId> selected_low_faces;
            const std::vector<SurfaceFaceId> *low_faces_to_build =
                &face_sets.transition_low_faces;
            if (external_only)
            {
                selected_terminal_faces.reserve(selected.size());
                selected_low_faces.reserve(selected.size());
                for (const SurfaceFaceId id : selected)
                {
                    if (std::binary_search(
                            terminal_candidate_faces.begin(),
                            terminal_candidate_faces.end(), id))
                        selected_terminal_faces.push_back(id);
                    // transition_low_faces and selected are both sorted and
                    // unique, so this intersection preserves build order.
                    if (std::binary_search(
                            face_sets.transition_low_faces.begin(),
                            face_sets.transition_low_faces.end(), id))
                        selected_low_faces.push_back(id);
                }
                terminal_faces_to_build = &selected_terminal_faces;
                low_faces_to_build = &selected_low_faces;
            }
            const std::size_t regular_count = external_only
                ? (include_regular ? selected.size() : 0) : candidate.faces.size();
            for (std::size_t slot = 0; slot < regular_count; ++slot)
            {
                std::size_t index = slot;
                if (external_only)
                {
                    const auto found = candidate_faces.find(selected[slot]);
                    if (found == candidate_faces.end()) continue;
                    index = found->second;
                }
                const SurfaceFaceId id = candidate.source_face_ids[index];
                if (!retainedFace(retained, id)) continue;
                if (std::binary_search(
                        terminal_candidate_faces.begin(),
                        terminal_candidate_faces.end(), id))
                    continue;
                appendFrontFace(provisional.boundary, candidate, index,
                    {id, candidate.layer,
                     BoundaryOwnerRole::RegularCandidate, {id}},
                    candidateColumnContext(
                        current, candidate, index, current_vertices));
            }

            for (const SurfaceFaceId id : *terminal_faces_to_build)
            {
                if (!selectedExternal(id)) continue;
                if (!retainedFace(retained, id)) continue;
                const auto position = candidate_faces.find(id);
                if (position == candidate_faces.end()) continue;
                const std::size_t face_index = position->second;
                const auto *quad = std::get_if<Quad>(
                    &candidate.faces[face_index]);
                if (quad == nullptr) continue;

                std::vector<Point3> points(8);
                std::vector<CollisionVertexKey> keys(8);
                std::vector<std::vector<std::uint32_t>> regions(8);
                std::array<VertexId,4> top{{4,5,6,7}};
                HexaPoints hexa{};
                bool complete = true;
                for (std::size_t local = 0; local < 4; ++local)
                {
                    const auto &high = candidate.vertices[
                        quad->vertex_ids[local]];
                    const auto low_position = current_vertices.find(
                        cellKey(high.source_vertex_id, high.branch_id));
                    if (low_position == current_vertices.end())
                    {
                        complete = false;
                        break;
                    }
                    const auto &low = current.vertices[low_position->second];
                    points[local] = low.position;
                    points[4+local] = high.position;
                    hexa[local] = low.position;
                    hexa[4+local] = high.position;
                    keys[local] = {low.source_vertex_id,
                                   current.layer,low.branch_id};
                    keys[4+local] = {high.source_vertex_id,
                                     candidate.layer,high.branch_id};
                    regions[local] = low.boundary.sliding_region_ids;
                    regions[4+local] = high.boundary.sliding_region_ids;
                }
                if (!complete) continue;
                const auto chosen = chooseQuadDiagonal(
                    oriented(top,points),1e-12);
                if (!chosen.hasValue())
                    return ProvisionalLayerTransitionResult::failure(
                        LayerTransitionError{
                            TransitionTemplateError{chosen.error()}});

                ResolvedTransitionTopology resolved{
                    id,candidate.layer,TransitionTemplateKind::QuadTopCap,
                    chosen.value().diagonal,{}};
                resolved.aspect_ratio = quadTopCapAspectRatio({
                    {hexa[0],hexa[1],hexa[2],hexa[3]},
                    {hexa[4],hexa[5],hexa[6],hexa[7]}});
                const std::array<Point3,4> bottom{{
                    hexa[0],hexa[1],hexa[2],hexa[3]}};
                const std::array<Point3,4> top_points{{
                    hexa[4],hexa[5],hexa[6],hexa[7]}};
                std::optional<Point3> internal_center;
                const bool external_available =
                    !external_controls.keepHexa(id) &&
                    !external_controls.forceKeepHexa(id);
                const bool prefer_external =
                    chooseTerminalQuadDecision(
                        resolved.aspect_ratio, Point3::Zero(),
                        external_available) ==
                    TerminalQuadDecision::ExternalPatch;
                if (!prefer_external)
                    internal_center = context.positiveCenter(id, {
                        bottom,top_points,chosen.value().diagonal,
                        Scalar{1e-12}});
                resolved.terminal_quad_decision =
                    chooseTerminalQuadDecision(
                        resolved.aspect_ratio,internal_center,
                        external_available);
                if (external_controls.forceKeepHexa(id))
                    resolved.terminal_quad_decision =
                        TerminalQuadDecision::KeepHexa;

                if (resolved.terminal_quad_decision ==
                    TerminalQuadDecision::ExternalPatch)
                {
                    const auto patch = buildControlledExternalQuadPatch({
                        id,candidate.layer,top,top,{},&points,8,
                        external_controls.distanceScale(id),Scalar{1e-12}},
                        external_controls);
                    if (!patch.hasValue())
                    {
                        if (!internal_center.has_value())
                            internal_center = context.positiveCenter(id, {
                                bottom,top_points,
                                chosen.value().diagonal,Scalar{1e-12}});
                        resolved.terminal_quad_decision =
                            chooseTerminalQuadDecision(
                                resolved.aspect_ratio,internal_center,false);
                    }
                    else
                    {
                        resolved.generated_point =
                            patch.value().created_vertices.front();
                        points.push_back(*resolved.generated_point);
                        keys.push_back({static_cast<VertexId>(id),
                            candidate.layer,
                            std::numeric_limits<std::uint32_t>::max()});
                        regions.push_back({});
                        const LayerBoundaryOwner owner{
                            id,candidate.layer,
                            BoundaryOwnerRole::ExternalPatch,{}};
                        for (const Triangle &triangle :
                             patch.value().top_faces)
                            appendOwnedTriangle(
                                provisional.boundary,triangle,
                                points,keys,owner,regions,
                                physicalEdgeMask(triangle,4),{},
                                candidateColumnContext(
                                    current,candidate,face_index,
                                    current_vertices));
                    }
                }

                if (resolved.terminal_quad_decision ==
                    TerminalQuadDecision::InternalSplit)
                {
                    resolved.generated_point = internal_center;
                    const LayerBoundaryOwner owner{
                        id,candidate.layer,BoundaryOwnerRole::TopCap,{}};
                    for (const Triangle &triangle : chosen.value().triangles)
                        appendOwnedTriangle(
                            provisional.boundary,triangle,
                            points,keys,owner,regions,
                            physicalEdgeMask(triangle,4),{},
                            candidateColumnContext(
                                current,candidate,face_index,
                                current_vertices));
                }
                else if (resolved.terminal_quad_decision ==
                         TerminalQuadDecision::KeepHexa)
                    appendFrontFace(
                        provisional.boundary,candidate,face_index,
                        {id,candidate.layer,
                         BoundaryOwnerRole::RegularCandidate,{id}},
                        candidateColumnContext(
                            current,candidate,face_index,current_vertices));
                provisional.resolved_topology.push_back(std::move(resolved));
            }

            for (const SurfaceFaceId low_id : *low_faces_to_build)
            {
                if (!selectedExternal(low_id)) continue;
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
                    if (traceTransitionFace(low_id))
                    {
                        std::cerr << "trace triangle-side face=" << low_id
                                  << " layer=" << current.layer
                                  << " high_edge=";
                        if (high_edge) std::cerr << *high_edge;
                        else std::cerr << "none";
                        std::cerr << " dependencies=";
                        for (const SurfaceFaceId dependency : dependencies)
                            std::cerr << dependency << ',';
                        std::cerr << '\n';
                    }
                    if (!high_edge.has_value()) continue;
                    std::vector<Point3> points(6);
                    std::vector<CollisionVertexKey> keys(6);
                    std::vector<std::vector<std::uint32_t>> regions(6);
                    std::array<VertexId,3> low{{0,1,2}};
                    std::array<VertexId,3> high{{0,1,2}};
                    for (std::size_t local = 0; local < 3; ++local)
                    {
                        const auto &vertex = current.vertices[
                            low_source_ids[local]];
                        points[local] = vertex.position;
                        keys[local] = {vertex.source_vertex_id,
                                       current.layer,vertex.branch_id};
                        regions[local] =
                            vertex.boundary.sliding_region_ids;
                        points[3+local] = vertex.position;
                        keys[3+local] = keys[local];
                        regions[3+local] = regions[local];
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
                            regions[3+local] = candidate.vertices[
                                found->second].boundary.sliding_region_ids;
                        }
                    }
                    const auto side = buildTriangleSideTransition({
                        low_id,current.layer,low,high,*high_edge});
                    if (!side.hasValue())
                        return ProvisionalLayerTransitionResult::failure(
                            LayerTransitionError{
                                atStage(side.error(),11)});
                    if (!hasStrictlyPositiveTemplateVolumes(
                            side.value(),points))
                    {
                        if (traceTransitionFace(low_id))
                            std::cerr << "trace triangle-side invalid-volume face="
                                      << low_id << '\n';
                        provisional.forced_rollback_by_source[low_id] = dependencies;
                        provisional.forced_rollback_high_faces.insert(
                            provisional.forced_rollback_high_faces.end(),
                            dependencies.begin(),dependencies.end());
                        continue;
                    }
                    const LayerBoundaryOwner owner{
                        low_id,current.layer,
                        BoundaryOwnerRole::SideTransition,dependencies};
                    for (const Triangle &triangle : side.value().top_faces)
                    {
                        if (traceTransitionFace(low_id))
                        {
                            std::cerr << "trace triangle-side top-face=";
                            for (const VertexId local : triangle.vertex_ids)
                            {
                                tracePoint(points[local]);
                                std::cerr << ' ';
                            }
                            std::cerr << '\n';
                        }
                        appendOwnedTriangle(
                            provisional.boundary,triangle,
                            points,keys,owner,regions,
                            physicalEdgeMask(triangle, 3),
                            commonRegions(triangle, regions),
                            columnContext(current, candidate,
                                low_source_ids, candidate_vertices));
                    }
                    continue;
                }
                if (low_source_ids.size() != 4) continue;

                std::vector<Point3> points(8);
                std::vector<CollisionVertexKey> keys(8);
                std::vector<std::vector<std::uint32_t>> regions(8);
                std::array<VertexId,4> low{{0,1,2,3}};
                std::array<VertexId,4> high{{0,1,2,3}};
                for (std::size_t local = 0; local < 4; ++local)
                {
                    const auto &vertex = current.vertices[
                        low_source_ids[local]];
                    points[local] = vertex.position;
                    keys[local] = {vertex.source_vertex_id,
                                   current.layer, vertex.branch_id};
                    regions[local] =
                        vertex.boundary.sliding_region_ids;
                    points[4+local] = vertex.position;
                    keys[4+local] = keys[local];
                    regions[4+local] = regions[local];
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
                        regions[4+local] = candidate.vertices[
                            candidate_vertex->second].boundary.sliding_region_ids;
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
                if (traceTransitionFace(low_id))
                {
                    std::cerr << "trace external face=" << low_id
                              << " layer=" << current.layer << " neighbors=";
                    for (const auto &neighbor : neighbors)
                        std::cerr << '[' << neighbor.local_edge << ':'
                                  << neighbor.neighbor_face_id << ']';
                    std::cerr << '\n';
                }
                if (neighbors.empty())
                {
                    const auto chosen = chooseQuadDiagonal(
                        oriented(low, points), 1e-12);
                    if (!chosen.hasValue())
                        return ProvisionalLayerTransitionResult::failure(
                            LayerTransitionError{
                                TransitionTemplateError{chosen.error()}});
                    const QuadDiagonal diagonal = chosen.value().diagonal;
                    ResolvedTransitionTopology resolved{
                        low_id,current.layer,TransitionTemplateKind::QuadTopCap,
                        diagonal,{}};
                    std::optional<Point3> internal_center;
                    if (terminal_hexa_points)
                        if (const auto hexa = terminal_hexa_points(low_id))
                        {
                            std::array<Point3,4> bottom{};
                            std::array<Point3,4> top{};
                            std::copy_n(hexa->begin(),4,bottom.begin());
                            std::copy_n(hexa->begin()+4,4,top.begin());
                            resolved.aspect_ratio = quadTopCapAspectRatio(
                                {bottom,top});
                            internal_center = context.positiveCenter(low_id,
                                {bottom,top,diagonal,Scalar{1e-12}});
                            resolved.terminal_quad_decision =
                                chooseTerminalQuadDecision(
                                    resolved.aspect_ratio,internal_center,
                                    !external_controls.keepHexa(low_id));
                            if (resolved.terminal_quad_decision ==
                                TerminalQuadDecision::InternalSplit)
                                resolved.generated_point = internal_center;
                        }
                    if (external_controls.forceKeepHexa(low_id))
                        resolved.terminal_quad_decision =
                            TerminalQuadDecision::KeepHexa;
                    if (traceTransitionFace(low_id))
                        std::cerr << "trace external face=" << low_id
                                  << " layer=" << current.layer
                                  << " path=top-cap aspect="
                                  << resolved.aspect_ratio
                                  << " internal_center="
                                  << internal_center.has_value()
                                  << " decision=" << static_cast<int>(
                                      resolved.terminal_quad_decision)
                                  << " distance_scale="
                                  << external_controls.distanceScale(low_id)
                                  << '\n';
                    if (resolved.terminal_quad_decision ==
                        TerminalQuadDecision::ExternalPatch)
                    {
                        const auto patch = buildControlledExternalQuadPatch({
                            low_id,current.layer,low,high,{},&points,8,
                            external_controls.distanceScale(low_id),
                            Scalar{1e-12}}, external_controls);
                        if (traceTransitionFace(low_id))
                        {
                            std::cerr << "trace external face=" << low_id
                                      << " layer=" << current.layer
                                      << " top-cap patch=" << patch.hasValue();
                            if (patch.hasValue() &&
                                !patch.value().created_vertices.empty())
                            {
                                std::cerr << " apex=";
                                tracePoint(patch.value().created_vertices.front());
                                std::cerr << " cells="
                                          << patch.value().volume_cells.size();
                            }
                            else if (!patch.hasValue())
                                std::cerr << " error_variant="
                                          << patch.error().index();
                            std::cerr << '\n';
                        }
                        if (resolved.terminal_quad_decision ==
                                TerminalQuadDecision::KeepHexa ||
                            !patch.hasValue())
                        {
                            resolved.terminal_quad_decision =
                                chooseTerminalQuadDecision(
                                    resolved.aspect_ratio,internal_center,
                                    false);
                            resolved.generated_point =
                                resolved.terminal_quad_decision ==
                                    TerminalQuadDecision::InternalSplit
                                ? internal_center : std::nullopt;
                        }
                        else
                        {
                            resolved.generated_point =
                                patch.value().created_vertices.front();
                            points.push_back(*resolved.generated_point);
                            keys.push_back({static_cast<VertexId>(low_id),
                                current.layer,
                                std::numeric_limits<std::uint32_t>::max()});
                            regions.push_back({});
                            const LayerBoundaryOwner owner{
                                low_id,current.layer,
                                BoundaryOwnerRole::ExternalPatch,{}};
                            for (const Triangle &triangle :
                                 patch.value().top_faces)
                                appendOwnedTriangle(
                                    provisional.boundary,triangle,
                                    points,keys,owner,regions,
                                    physicalEdgeMask(triangle,4),{},nullptr);
                        }
                    }
                    if (resolved.terminal_quad_decision ==
                        TerminalQuadDecision::InternalSplit)
                    {
                        const auto &triangles = chosen.value().triangles;
                        const LayerBoundaryOwner owner{
                            low_id,current.layer,
                            BoundaryOwnerRole::TopCap,{}};
                        for (const Triangle &triangle : triangles)
                            appendOwnedTriangle(
                                provisional.boundary,triangle,
                                points,keys,owner,regions,
                                physicalEdgeMask(triangle,4),{},
                                columnContext(current,candidate,
                                    low_source_ids,candidate_vertices));
                    }
                    else if (resolved.terminal_quad_decision ==
                             TerminalQuadDecision::KeepHexa)
                    {
                        const auto face_position = candidate_faces.find(low_id);
                        if (face_position != candidate_faces.end())
                            appendFrontFace(
                                provisional.boundary,candidate,
                                face_position->second,
                                {low_id,candidate.layer,
                                 BoundaryOwnerRole::RegularCandidate,{low_id}},
                                candidateColumnContext(
                                    current,candidate,face_position->second,
                                    current_vertices));
                    }
                    provisional.resolved_topology.push_back(
                        std::move(resolved));
                    continue;
                }
                for (const auto &neighbor : neighbors)
                    for (const std::size_t local : {
                             neighbor.local_edge,
                             (neighbor.local_edge+1)%4})
                        high[local] = static_cast<VertexId>(4+local);

                Scalar aspect_ratio{};
                std::optional<HexaPoints> terminal_hexa;
                if (terminal_hexa_points)
                    terminal_hexa = terminal_hexa_points(low_id);
                if (terminal_hexa)
                {
                    std::array<Point3,4> bottom{};
                    std::array<Point3,4> top{};
                    std::copy_n(terminal_hexa->begin(),4,bottom.begin());
                    std::copy_n(terminal_hexa->begin()+4,4,top.begin());
                    aspect_ratio = quadTopCapAspectRatio({bottom,top});
                }
                const bool external_available =
                    !external_controls.keepHexa(low_id) &&
                    !external_controls.forceKeepHexa(low_id);
                const bool prefer_external = terminal_hexa.has_value() &&
                    chooseTerminalQuadDecision(
                        aspect_ratio,std::nullopt,external_available) ==
                    TerminalQuadDecision::ExternalPatch;
                if (traceTransitionFace(low_id))
                    std::cerr << "trace external face=" << low_id
                              << " layer=" << current.layer
                              << " high-neighbor path aspect=" << aspect_ratio
                              << " terminal_hexa=" << terminal_hexa.has_value()
                              << " prefer_external=" << prefer_external
                              << " external_available=" << external_available
                              << " distance_scale="
                              << external_controls.distanceScale(low_id)
                              << '\n';
                const auto diagonal_for_external_edges =
                    [&](const std::vector<std::size_t> &edges)
                    -> std::optional<QuadDiagonal>
                {
                    if (edges.size() == 2)
                    {
                        const std::size_t common =
                            (edges[0]+1)%4 == edges[1]
                                ? edges[1] : edges[0];
                        return common % 2 == 0
                            ? QuadDiagonal::ZeroTwo
                            : QuadDiagonal::OneThree;
                    }
                    const auto chosen = chooseQuadDiagonal(
                        oriented(low,points),Scalar{1e-12});
                    if (!chosen.hasValue()) return std::nullopt;
                    return chosen.value().diagonal;
                };

                // For a thin terminal hexa, try external templates first
                // using the same high-edge combinations as the regular
                // selector.
                if (prefer_external)
                {
                    std::optional<TransitionTemplateOutput> best_patch;
                    std::vector<std::size_t> best_edges;
                    std::vector<SurfaceFaceId> best_dependencies;
                    Scalar best_skewness =
                        std::numeric_limits<Scalar>::infinity();
                    for (const auto &edges :
                         quadHighNeighborCandidates(neighbors))
                    {
                        auto patch = buildControlledExternalQuadPatch({
                            low_id,current.layer,low,high,edges,&points,8,
                            external_controls.distanceScale(low_id),
                            Scalar{1e-12}},external_controls);
                        if (!patch.hasValue() ||
                            patch.value().created_vertices.empty())
                            continue;
                        auto candidate_points = points;
                        candidate_points.push_back(
                            patch.value().created_vertices.front());
                        Scalar worst{};
                        bool valid = true;
                        for (const Triangle &triangle : patch.value().top_faces)
                        {
                            const std::array<Point3,3> triangle_points{{
                                candidate_points[triangle.vertex_ids[0]],
                                candidate_points[triangle.vertex_ids[1]],
                                candidate_points[triangle.vertex_ids[2]]}};
                            const auto skewness = triangleEquiangularSkewness(
                                triangle_points,Scalar{1e-12});
                            if (!skewness.hasValue())
                            {
                                valid = false;
                                break;
                            }
                            worst = std::max(worst,skewness.value());
                        }
                        if (!valid || worst >= best_skewness) continue;
                        best_skewness = worst;
                        best_edges = edges;
                        best_dependencies.clear();
                        for (const auto &neighbor : neighbors)
                            if (std::find(edges.begin(),edges.end(),
                                    neighbor.local_edge) != edges.end())
                                best_dependencies.push_back(
                                    neighbor.neighbor_face_id);
                        std::sort(best_dependencies.begin(),
                                  best_dependencies.end());
                        best_dependencies.erase(std::unique(
                            best_dependencies.begin(),best_dependencies.end()),
                            best_dependencies.end());
                        best_patch = std::move(patch.value());
                    }
                    if (best_patch)
                    {
                        const QuadDiagonal external_diagonal =
                            diagonal_for_external_edges(best_edges)
                                .value_or(QuadDiagonal::ZeroTwo);
                        ResolvedTransitionTopology external_topology{
                            low_id,current.layer,
                            best_edges.size() == 1
                                ? TransitionTemplateKind::QuadSingleHighSide
                                : TransitionTemplateKind::QuadAdjacentHighSide,
                            external_diagonal,best_dependencies};
                        external_topology.aspect_ratio = aspect_ratio;
                        external_topology.retained_local_edges = best_edges;
                        external_topology.terminal_quad_decision =
                            TerminalQuadDecision::ExternalPatch;
                        external_topology.generated_point =
                            best_patch->created_vertices.front();
                        provisional.boundary.diagonal_requirements.push_back(
                            {{low_id,current.layer},external_diagonal});
                        points.push_back(*external_topology.generated_point);
                        keys.push_back({static_cast<VertexId>(low_id),
                            current.layer,
                            std::numeric_limits<std::uint32_t>::max()});
                        regions.push_back({});
                        const LayerBoundaryOwner external_owner{
                            low_id,current.layer,
                            BoundaryOwnerRole::ExternalPatch,best_dependencies};
                        for (const Triangle &triangle : best_patch->top_faces)
                            appendOwnedTriangle(
                                provisional.boundary,triangle,points,keys,
                                external_owner,regions,
                                physicalEdgeMask(triangle,4),
                                commonRegions(triangle,regions),
                                columnContext(current,candidate,
                                    low_source_ids,candidate_vertices));
                        provisional.resolved_topology.push_back(
                            std::move(external_topology));
                        continue;
                    }
                    if (!neighbors.empty())
                    {
                        // The preferred external topology has exhausted its
                        // high-edge candidates. Ask the resolver to remove
                        // their actual high-face dependencies; the next
                        // provisional build will rediscover the now-empty
                        // adjacency and try the zero-high external template.
                        ResolvedTransitionTopology failed_external_topology{
                            low_id,current.layer,
                            TransitionTemplateKind::QuadTopCap,
                            std::nullopt,dependencies};
                        failed_external_topology.aspect_ratio = aspect_ratio;
                        failed_external_topology.terminal_quad_decision =
                            TerminalQuadDecision::KeepHexa;
                        provisional.forced_rollback_by_source[low_id] =
                            dependencies;
                        provisional.forced_rollback_high_faces.insert(
                            provisional.forced_rollback_high_faces.end(),
                            dependencies.begin(), dependencies.end());
                        provisional.resolved_topology.push_back(
                            std::move(failed_external_topology));
                        continue;
                    }
                }

                const auto selection = selectQuadHighNeighbors({
                    low_id, current.layer, neighbors, low, high,
                    &points, 1e-12});
                if (!selection.hasValue())
                {
                    if (!prefer_external && external_available)
                    {
                        std::optional<TransitionTemplateOutput> patch;
                        std::vector<std::size_t> external_edges;
                        Scalar best_skewness =
                            std::numeric_limits<Scalar>::infinity();
                        for (const auto &edges :
                             quadHighNeighborCandidates(neighbors))
                        {
                            auto candidate = buildControlledExternalQuadPatch({
                                low_id,current.layer,low,high,edges,&points,8,
                                external_controls.distanceScale(low_id),
                                Scalar{1e-12}},external_controls);
                            if (!candidate.hasValue() ||
                                candidate.value().created_vertices.empty())
                                continue;
                            auto candidate_points = points;
                            candidate_points.push_back(
                                candidate.value().created_vertices.front());
                            Scalar worst{};
                            bool valid = true;
                            for (const Triangle &triangle :
                                 candidate.value().top_faces)
                            {
                                const std::array<Point3,3> triangle_points{{
                                    candidate_points[triangle.vertex_ids[0]],
                                    candidate_points[triangle.vertex_ids[1]],
                                    candidate_points[triangle.vertex_ids[2]]}};
                                const auto skewness = triangleEquiangularSkewness(
                                    triangle_points,Scalar{1e-12});
                                if (!skewness.hasValue())
                                {
                                    valid = false;
                                    break;
                                }
                                worst = std::max(worst,skewness.value());
                            }
                            if (!valid || worst >= best_skewness) continue;
                            best_skewness = worst;
                            external_edges = edges;
                            patch = std::move(candidate.value());
                        }
                        if (patch)
                        {
                            std::vector<SurfaceFaceId> external_dependencies;
                            for (const auto &neighbor : neighbors)
                                if (std::find(external_edges.begin(),
                                        external_edges.end(),
                                        neighbor.local_edge) !=
                                    external_edges.end())
                                    external_dependencies.push_back(
                                        neighbor.neighbor_face_id);
                            std::sort(external_dependencies.begin(),
                                      external_dependencies.end());
                            external_dependencies.erase(std::unique(
                                external_dependencies.begin(),
                                external_dependencies.end()),
                                external_dependencies.end());
                            const QuadDiagonal diagonal =
                                diagonal_for_external_edges(external_edges)
                                    .value_or(QuadDiagonal::ZeroTwo);
                            ResolvedTransitionTopology topology{
                                low_id,current.layer,
                                external_edges.size() == 1
                                    ? TransitionTemplateKind::QuadSingleHighSide
                                    : TransitionTemplateKind::QuadAdjacentHighSide,
                                diagonal,external_dependencies};
                            topology.aspect_ratio = aspect_ratio;
                            topology.retained_local_edges = external_edges;
                            topology.terminal_quad_decision =
                                TerminalQuadDecision::ExternalPatch;
                            topology.generated_point =
                                patch->created_vertices.front();
                            provisional.boundary.diagonal_requirements.push_back(
                                {{low_id,current.layer},diagonal});
                            points.push_back(*topology.generated_point);
                            keys.push_back({static_cast<VertexId>(low_id),
                                current.layer,
                                std::numeric_limits<std::uint32_t>::max()});
                            regions.push_back({});
                            const LayerBoundaryOwner owner{
                                low_id,current.layer,
                                BoundaryOwnerRole::ExternalPatch,
                                external_dependencies};
                            for (const Triangle &triangle : patch->top_faces)
                                appendOwnedTriangle(
                                    provisional.boundary,triangle,points,keys,
                                    owner,regions,physicalEdgeMask(triangle,4),
                                    commonRegions(triangle,regions),
                                    columnContext(current,candidate,
                                        low_source_ids,candidate_vertices));
                            provisional.resolved_topology.push_back(
                                std::move(topology));
                            continue;
                        }
                    }
                    return ProvisionalLayerTransitionResult::failure(
                        LayerTransitionError{
                            atStage(selection.error(),12)});
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
                        return ProvisionalLayerTransitionResult::failure(
                            LayerTransitionError{
                                TransitionTemplateError{chosen.error()}});
                    diagonal = chosen.value().diagonal;
                }
                provisional.boundary.diagonal_requirements.push_back(
                    {{low_id,current.layer},diagonal});
                const auto &retained_edges =
                    selection.value().retained_local_edges;
                auto checked_side = retained_edges.size() == 1
                    ? buildQuadSideTransition({low_id,current.layer,low,high,
                        retained_edges[0],diagonal})
                    : buildQuadAdjacentSideTransition({low_id,current.layer,
                        low,high,retained_edges[0],retained_edges[1],diagonal,
                        &points,Scalar{1e-12}});
                const bool internal_side_valid = checked_side.hasValue() &&
                    hasStrictlyPositiveTemplateVolumes(
                        checked_side.value(),points);
                ResolvedTransitionTopology resolved{
                    low_id,current.layer,
                    selection.value().retained_local_edges.size() == 1
                        ? TransitionTemplateKind::QuadSingleHighSide
                        : TransitionTemplateKind::QuadAdjacentHighSide,
                    diagonal,dependencies};
                resolved.retained_local_edges =
                    selection.value().retained_local_edges;
                std::optional<Point3> internal_center;
                if (terminal_hexa)
                    {
                        std::array<Point3,4> bottom{};
                        std::array<Point3,4> top{};
                        std::copy_n(terminal_hexa->begin(),4,bottom.begin());
                        std::copy_n(terminal_hexa->begin()+4,4,top.begin());
                        resolved.aspect_ratio = aspect_ratio;
                        internal_center = context.positiveCenter(low_id,
                            {bottom,top,diagonal,Scalar{1e-12}});
                        resolved.terminal_quad_decision =
                            chooseTerminalQuadDecision(
                                resolved.aspect_ratio,
                                internal_side_valid
                                    ? internal_center : std::nullopt,
                                external_available && !prefer_external);
                        if (resolved.terminal_quad_decision ==
                            TerminalQuadDecision::InternalSplit)
                            resolved.generated_point = internal_center;
                    }
                if (external_controls.forceKeepHexa(low_id))
                    resolved.terminal_quad_decision =
                        TerminalQuadDecision::KeepHexa;
                if (resolved.terminal_quad_decision ==
                    TerminalQuadDecision::ExternalPatch)
                {
                    std::optional<TransitionTemplateOutput> patch;
                    std::vector<std::size_t> external_edges;
                    std::vector<SurfaceFaceId> external_dependencies;
                    Scalar best_skewness =
                        std::numeric_limits<Scalar>::infinity();
                    for (const auto &edges :
                         quadHighNeighborCandidates(neighbors))
                    {
                        auto candidate = buildControlledExternalQuadPatch({
                            low_id,current.layer,low,high,edges,&points,8,
                                external_controls.distanceScale(low_id),
                                Scalar{1e-12}},external_controls);
                        if (traceTransitionFace(low_id))
                        {
                            std::cerr << "trace external face=" << low_id
                                      << " layer=" << current.layer
                                      << " external_edges=";
                            for (const std::size_t edge : edges)
                                std::cerr << edge << ',';
                            std::cerr << " patch=" << candidate.hasValue();
                            if (candidate.hasValue() &&
                                !candidate.value().created_vertices.empty())
                            {
                                std::cerr << " apex=";
                                tracePoint(candidate.value().created_vertices.front());
                                std::cerr << " cells="
                                          << candidate.value().volume_cells.size();
                            }
                            else if (!candidate.hasValue())
                                std::cerr << " error_variant="
                                          << candidate.error().index();
                            std::cerr << '\n';
                        }
                        if (!candidate.hasValue() ||
                            candidate.value().created_vertices.empty())
                            continue;
                        auto candidate_points = points;
                        candidate_points.push_back(
                            candidate.value().created_vertices.front());
                        Scalar worst{};
                        bool valid = true;
                        for (const Triangle &triangle : candidate.value().top_faces)
                        {
                            const std::array<Point3,3> triangle_points{{
                                candidate_points[triangle.vertex_ids[0]],
                                candidate_points[triangle.vertex_ids[1]],
                                candidate_points[triangle.vertex_ids[2]]}};
                            const auto skewness = triangleEquiangularSkewness(
                                triangle_points,Scalar{1e-12});
                            if (!skewness.hasValue())
                            {
                                valid = false;
                                break;
                            }
                            worst = std::max(worst,skewness.value());
                        }
                        if (traceTransitionFace(low_id))
                            std::cerr << "trace external face=" << low_id
                                      << " layer=" << current.layer
                                      << " candidate_worst_top_skewness="
                                      << worst << " valid=" << valid
                                      << " best_before=" << best_skewness
                                      << '\n';
                        if (!valid || worst >= best_skewness) continue;
                        best_skewness = worst;
                        external_edges = edges;
                        external_dependencies.clear();
                        for (const auto &neighbor : neighbors)
                            if (std::find(edges.begin(),edges.end(),
                                    neighbor.local_edge) != edges.end())
                                external_dependencies.push_back(
                                    neighbor.neighbor_face_id);
                        std::sort(external_dependencies.begin(),
                                  external_dependencies.end());
                        external_dependencies.erase(std::unique(
                            external_dependencies.begin(),
                            external_dependencies.end()),
                            external_dependencies.end());
                        patch = std::move(candidate.value());
                    }
                    if (!patch)
                    {
                        if (traceTransitionFace(low_id))
                            std::cerr << "trace external face=" << low_id
                                      << " layer=" << current.layer
                                      << " no_usable_external_template"
                                      << " internal_side_valid="
                                      << internal_side_valid
                                      << " internal_center="
                                      << internal_center.has_value()
                                      << '\n';
                        resolved.terminal_quad_decision =
                            chooseTerminalQuadDecision(
                                resolved.aspect_ratio,
                                internal_side_valid
                                    ? internal_center : std::nullopt,
                                false);
                        resolved.generated_point =
                            resolved.terminal_quad_decision ==
                                TerminalQuadDecision::InternalSplit
                            ? internal_center : std::nullopt;
                        if (resolved.terminal_quad_decision ==
                                TerminalQuadDecision::KeepHexa &&
                            !dependencies.empty())
                        {
                            provisional.forced_rollback_by_source[low_id] = dependencies;
                            provisional.forced_rollback_high_faces.insert(
                                provisional.forced_rollback_high_faces.end(),
                                dependencies.begin(), dependencies.end());
                        }
                    }
                    else
                    {
                        if (traceTransitionFace(low_id))
                        {
                            std::cerr << "trace external face=" << low_id
                                      << " layer=" << current.layer
                                      << " selected_external_edges=";
                            for (const std::size_t edge : external_edges)
                                std::cerr << edge << ',';
                            std::cerr << " dependent_faces=";
                            for (const SurfaceFaceId face :
                                 external_dependencies)
                                std::cerr << face << ',';
                            std::cerr << " apex=";
                            tracePoint(patch->created_vertices.front());
                            std::cerr << '\n';
                        }
                        resolved.retained_local_edges = external_edges;
                        resolved.dependent_high_faces = external_dependencies;
                        resolved.template_kind = external_edges.size() == 1
                            ? TransitionTemplateKind::QuadSingleHighSide
                            : TransitionTemplateKind::QuadAdjacentHighSide;
                        dependencies = external_dependencies;
                        resolved.generated_point =
                            patch->created_vertices.front();
                        points.push_back(*resolved.generated_point);
                        keys.push_back({static_cast<VertexId>(low_id),
                            current.layer,
                            std::numeric_limits<std::uint32_t>::max()});
                        regions.push_back({});
                        const LayerBoundaryOwner owner{
                            low_id,current.layer,
                            BoundaryOwnerRole::ExternalPatch,dependencies};
                        for (const Triangle &triangle : patch->top_faces)
                            appendOwnedTriangle(
                                provisional.boundary,triangle,
                                points,keys,owner,regions,
                                physicalEdgeMask(triangle,4),
                                commonRegions(triangle,regions),
                                columnContext(current,candidate,
                                    low_source_ids,candidate_vertices));
                    }
                    if (resolved.terminal_quad_decision !=
                        TerminalQuadDecision::InternalSplit)
                    {
                        provisional.resolved_topology.push_back(
                            std::move(resolved));
                        continue;
                    }
                }
                if (resolved.terminal_quad_decision ==
                    TerminalQuadDecision::KeepHexa)
                {
                    if (!dependencies.empty())
                    {
                        provisional.forced_rollback_by_source[low_id] =
                            dependencies;
                        provisional.forced_rollback_high_faces.insert(
                            provisional.forced_rollback_high_faces.end(),
                            dependencies.begin(),dependencies.end());
                    }
                    provisional.resolved_topology.push_back(
                        std::move(resolved));
                    continue;
                }
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
                        points, keys, owner, regions,
                        physicalEdgeMask(triangle, 4), {},
                        columnContext(current, candidate,
                            low_source_ids, candidate_vertices));

                if (!internal_side_valid)
                {
                    provisional.forced_rollback_by_source[low_id] = dependencies;
                    provisional.forced_rollback_high_faces.insert(
                        provisional.forced_rollback_high_faces.end(),
                        dependencies.begin(),dependencies.end());
                    continue;
                }
                const LayerBoundaryOwner side_owner{
                    low_id, current.layer,
                    BoundaryOwnerRole::SideTransition, dependencies};
                for (const Triangle &triangle : checked_side.value().top_faces)
                    appendOwnedTriangle(
                        provisional.boundary, triangle,
                        points, keys, side_owner, regions,
                        physicalEdgeMask(triangle, 4),
                        commonRegions(triangle, regions),
                            columnContext(current, candidate,
                                low_source_ids, candidate_vertices));
                provisional.resolved_topology.push_back(
                    std::move(resolved));
            }
            return ProvisionalLayerTransitionResult::success(
                std::move(provisional));
        }

    ProvisionalLayerTransitionResult buildProvisionalTransitionPatches(
        const ProvisionalTransitionBuildContext &context,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::vector<SurfaceFaceId> &selected_faces,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &terminal_hexa_points,
        const ExternalPatchControls &external_controls,
        const std::vector<SurfaceFaceId> &terminal_candidate_faces)
    {
        return buildProvisionalTransitionImpl(context, retained, face_sets,
            terminal_hexa_points, external_controls, terminal_candidate_faces,
            &selected_faces, true);
    }

    ProvisionalLayerTransitionResult buildProvisionalTransition(
        const ProvisionalTransitionBuildContext &context,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points,
        const ExternalPatchControls &external_controls,
        const std::vector<SurfaceFaceId> &terminal_candidate_faces)
    {
        return buildProvisionalTransitionImpl(
            context, retained, face_sets, terminal_hexa_points,
            external_controls, terminal_candidate_faces, nullptr);
    }

    ProvisionalLayerTransitionResult buildProvisionalTransition(
        const GrowthFront &current,
        const GrowthFront &candidate,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points,
        const ExternalPatchControls &external_controls,
        const std::vector<SurfaceFaceId> &terminal_candidate_faces)
    {
        const ProvisionalTransitionBuildContext context{current, candidate};
        return buildProvisionalTransition(
            context, retained, face_sets, terminal_hexa_points,
            external_controls, terminal_candidate_faces);
    }

    ProvisionalLayerTransitionResult buildProvisionalExternalPatches(
        const ProvisionalTransitionBuildContext &context,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::vector<SurfaceFaceId> &selected_external_faces,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points,
        const ExternalPatchControls &external_controls,
        const std::vector<SurfaceFaceId> &terminal_candidate_faces)
    {
        return buildProvisionalTransitionImpl(
            context, retained, face_sets, terminal_hexa_points,
            external_controls, terminal_candidate_faces,
            &selected_external_faces);
    }

    ProvisionalLayerTransitionResult buildProvisionalExternalPatches(
        const GrowthFront &current,
        const GrowthFront &candidate,
        const std::vector<SurfaceFaceId> &retained,
        const LayerFaceSets &face_sets,
        const std::vector<SurfaceFaceId> &selected_external_faces,
        const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
            terminal_hexa_points,
        const ExternalPatchControls &external_controls,
        const std::vector<SurfaceFaceId> &terminal_candidate_faces)
    {
        const ProvisionalTransitionBuildContext context{current, candidate};
        return buildProvisionalExternalPatches(
            context, retained, face_sets, selected_external_faces,
            terminal_hexa_points, external_controls,
            terminal_candidate_faces);
    }
}
