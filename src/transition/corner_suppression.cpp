#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <boundary_mesh/transition/corner_suppression.hpp>
#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>

namespace boundary_mesh
{
    namespace
    {
        std::uint64_t edgeKey(VertexId first, VertexId second)
        {
            if (second < first) std::swap(first, second);
            return (static_cast<std::uint64_t>(first) << 32) | second;
        }

        std::vector<VertexId> vertices(const SurfaceFace &face)
        {
            return std::visit([](const auto &value)
            {
                return std::vector<VertexId>(
                    value.vertex_ids.begin(), value.vertex_ids.end());
            }, face);
        }

        bool retained(
            const std::vector<SurfaceFaceId> &ids,
            SurfaceFaceId id)
        {
            return std::binary_search(ids.begin(), ids.end(), id);
        }

        bool traceFace(SurfaceFaceId id)
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

        void removeHigh(
            CornerSuppressionResult &result,
            SurfaceFaceId id,
            std::uint32_t completed_layer)
        {
            const auto position = std::lower_bound(
                result.retained_high_faces.begin(),
                result.retained_high_faces.end(), id);
            if (position == result.retained_high_faces.end() ||
                *position != id)
                return;
            result.retained_high_faces.erase(position);
            result.removed_high_faces.push_back(id);
            addCornerSuppressedFace(
                result.face_sets,
                {id, completed_layer, StopOrigin::CornerSuppression});
        }
    }

    CornerSuppressionContext::CornerSuppressionContext(
        const GrowthFront &current_front,
        const GrowthFront &candidate_front)
        : current_front_(&current_front),
          candidate_front_(&candidate_front)
    {
        face_indices_.reserve(current_front.faces.size());
        edge_faces_.reserve(current_front.faces.size() * 2);
        vertex_faces_.reserve(current_front.vertices.size());
        const std::size_t face_count = std::min(
            current_front.faces.size(),
            current_front.source_face_ids.size());
        for (std::size_t index = 0; index < face_count; ++index)
        {
            const SurfaceFaceId id = current_front.source_face_ids[index];
            face_indices_[id] = index;
            const auto ids = vertices(current_front.faces[index]);
            for (std::size_t edge = 0; edge < ids.size(); ++edge)
                edge_faces_[edgeKey(ids[edge], ids[(edge + 1) % ids.size()])]
                    .push_back(id);
            for (const VertexId vertex : ids)
                vertex_faces_[vertex].push_back(id);
        }
        candidate_points_.reserve(candidate_front.vertices.size());
        for (const auto &vertex : candidate_front.vertices)
            candidate_points_.emplace(
                vertex.source_vertex_id, vertex.position);
    }

    Result<CornerSuppressionResult, TransitionCoordinationError>
    applyCornerSuppression(const CornerSuppressionView &input)
    {
        const CornerSuppressionContext context(
            input.current_front, input.candidate_front);
        return applyCornerSuppression(input, context);
    }

    Result<CornerSuppressionResult, TransitionCoordinationError>
    applyCornerSuppression(
        const CornerSuppressionView &input,
        const CornerSuppressionContext &context)
    {
        using SuppressionResult = Result<
            CornerSuppressionResult, TransitionCoordinationError>;
        if (context.current_front_ != &input.current_front ||
            context.candidate_front_ != &input.candidate_front ||
            input.current_front.faces.size() !=
                input.current_front.source_face_ids.size() ||
            input.candidate_front.faces.size() !=
                input.candidate_front.source_face_ids.size())
            return SuppressionResult::failure(
                TransitionCoordinationError{MissingTransitionFaceState{0}});

        CornerSuppressionResult result;
        result.face_sets = input.face_sets;
        result.retained_high_faces = input.retained_high_faces;
        std::sort(result.retained_high_faces.begin(),
                  result.retained_high_faces.end());
        result.retained_high_faces.erase(std::unique(
            result.retained_high_faces.begin(),
            result.retained_high_faces.end()),
            result.retained_high_faces.end());

        const std::vector<SurfaceFaceId> seeds =
            input.face_sets.corner_suppression_seeds;
        for (const SurfaceFaceId seed_id : seeds)
        {
            const auto seed_position = context.face_indices_.find(seed_id);
            if (seed_position == context.face_indices_.end())
                return SuppressionResult::failure(
                    TransitionCoordinationError{
                        MissingTransitionFaceState{seed_id}});
            const SurfaceFace *seed = &input.current_front.faces[
                seed_position->second];

            std::vector<QuadHighNeighbor> quad_highs;
            std::vector<std::pair<std::size_t, SurfaceFaceId>> highs;
            const auto seed_ids = vertices(*seed);
            for (std::size_t edge = 0; edge < seed_ids.size(); ++edge)
            {
                const auto uses = context.edge_faces_.find(edgeKey(
                    seed_ids[edge],
                    seed_ids[(edge + 1) % seed_ids.size()]));
                if (uses == context.edge_faces_.end()) continue;
                for (const SurfaceFaceId other_id : uses->second)
                    if (other_id != seed_id &&
                        retained(result.retained_high_faces, other_id))
                        highs.push_back({edge, other_id});
            }
            if (highs.empty()) continue;

            std::vector<std::size_t> selected_edges;
            std::vector<SurfaceFaceId> explicitly_suppressed;
            const auto seed_vertices = vertices(*seed);
            if (seed_vertices.size() == 4)
            {
                std::vector<Point3> points;
                points.reserve(8);
                std::array<VertexId, 4> low{};
                std::array<VertexId, 4> high{};
                for (std::size_t index = 0; index < 4; ++index)
                {
                    low[index] = static_cast<VertexId>(points.size());
                    const Point3 point = input.current_front.vertices[
                        seed_vertices[index]].position;
                    points.push_back(point);
                }
                for (std::size_t index = 0; index < 4; ++index)
                {
                    high[index] = static_cast<VertexId>(points.size());
                    const auto &low_vertex = input.current_front.vertices[
                        seed_vertices[index]];
                    const auto candidate = context.candidate_points_.find(
                        low_vertex.source_vertex_id);
                    points.push_back(candidate == context.candidate_points_.end()
                        ? low_vertex.position : candidate->second);
                }
                for (const auto &[edge, id] : highs)
                    quad_highs.push_back({edge, id});
                const auto selection = selectQuadHighNeighbors({
                    seed_id, input.completed_layer,
                    quad_highs, low, high, &points,
                    input.length_tolerance});
                if (!selection.hasValue())
                    return SuppressionResult::failure(
                        TransitionCoordinationError{
                            MissingTransitionFaceState{seed_id}});
                selected_edges = selection.value().retained_local_edges;
                explicitly_suppressed =
                    selection.value().suppressed_neighbor_faces;
            }
            else
            {
                std::sort(highs.begin(), highs.end());
                selected_edges.push_back(highs.front().first);
            }

            if (traceFace(seed_id))
            {
                std::cerr << "trace corner_suppression seed=" << seed_id
                          << " selected_edges=";
                for (const std::size_t edge : selected_edges)
                    std::cerr << edge << ',';
                std::cerr << " high_neighbors=";
                for (const auto &[edge, id] : highs)
                    std::cerr << edge << ':' << id << ',';
                std::cerr << " explicit_removals=";
                for (const SurfaceFaceId id : explicitly_suppressed)
                    std::cerr << id << ',';
                std::cerr << '\n';
            }

            for (const SurfaceFaceId id : explicitly_suppressed)
                removeHigh(result, id, input.completed_layer);

            std::vector<VertexId> covered;
            for (const std::size_t edge : selected_edges)
            {
                covered.push_back(seed_vertices[edge]);
                covered.push_back(
                    seed_vertices[(edge + 1) % seed_vertices.size()]);
            }
            std::vector<VertexId> non_contact;
            for (const VertexId vertex : seed_vertices)
                if (std::find(covered.begin(), covered.end(), vertex) ==
                    covered.end())
                    non_contact.push_back(vertex);

            std::vector<SurfaceFaceId> incident_highs;
            for (const VertexId vertex : non_contact)
            {
                const auto incident = context.vertex_faces_.find(vertex);
                if (incident != context.vertex_faces_.end())
                    incident_highs.insert(
                        incident_highs.end(), incident->second.begin(),
                        incident->second.end());
            }
            std::sort(incident_highs.begin(), incident_highs.end());
            incident_highs.erase(std::unique(
                incident_highs.begin(), incident_highs.end()),
                incident_highs.end());
            for (const SurfaceFaceId high_id : incident_highs)
            {
                if (high_id == seed_id ||
                    !retained(result.retained_high_faces, high_id) ||
                    std::find_if(highs.begin(), highs.end(),
                        [high_id, &selected_edges](const auto &high)
                        {
                            return high.second == high_id &&
                                std::find(selected_edges.begin(),
                                          selected_edges.end(),
                                          high.first) != selected_edges.end();
                        }) != highs.end())
                    continue;
                if (traceFace(seed_id))
                    std::cerr << "trace corner_suppression_removed seed="
                              << seed_id << " high=" << high_id << '\n';
                removeHigh(result, high_id, input.completed_layer);
            }
        }

        std::sort(result.removed_high_faces.begin(),
                  result.removed_high_faces.end());
        result.removed_high_faces.erase(std::unique(
            result.removed_high_faces.begin(),
            result.removed_high_faces.end()),
            result.removed_high_faces.end());
        return SuppressionResult::success(std::move(result));
    }

    Result<CornerSuppressionResult, TransitionCoordinationError>
    applyCornerSuppression(const CornerSuppressionInput &input)
    {
        return applyCornerSuppression(CornerSuppressionView{
            input.current_front,
            input.candidate_front,
            input.candidate_front.source_face_ids,
            input.face_sets,
            input.completed_layer,
            input.length_tolerance});
    }
}
