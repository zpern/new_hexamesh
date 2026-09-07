#include <algorithm>
#include <array>
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

    Result<CornerSuppressionResult, TransitionCoordinationError>
    applyCornerSuppression(const CornerSuppressionInput &input)
    {
        using SuppressionResult = Result<
            CornerSuppressionResult, TransitionCoordinationError>;
        if (input.current_front.faces.size() !=
                input.current_front.source_face_ids.size() ||
            input.candidate_front.faces.size() !=
                input.candidate_front.source_face_ids.size())
            return SuppressionResult::failure(
                TransitionCoordinationError{MissingTransitionFaceState{0}});

        CornerSuppressionResult result;
        result.face_sets = input.face_sets;
        result.retained_high_faces = input.candidate_front.source_face_ids;
        std::sort(result.retained_high_faces.begin(),
                  result.retained_high_faces.end());
        result.retained_high_faces.erase(std::unique(
            result.retained_high_faces.begin(),
            result.retained_high_faces.end()),
            result.retained_high_faces.end());

        const std::vector<SurfaceFaceId> seeds =
            input.face_sets.corner_suppression_seeds;
        std::unordered_map<SurfaceFaceId, std::size_t> face_indices;
        std::unordered_map<std::uint64_t, std::vector<SurfaceFaceId>>
            edge_faces;
        std::unordered_map<VertexId, std::vector<SurfaceFaceId>>
            vertex_faces;
        for (std::size_t index = 0;
             index < input.current_front.faces.size(); ++index)
        {
            const SurfaceFaceId id =
                input.current_front.source_face_ids[index];
            face_indices[id] = index;
            const auto ids = vertices(input.current_front.faces[index]);
            for (std::size_t edge = 0; edge < ids.size(); ++edge)
                edge_faces[edgeKey(ids[edge], ids[(edge + 1) % ids.size()])]
                    .push_back(id);
            for (const VertexId vertex : ids)
                vertex_faces[vertex].push_back(id);
        }
        std::unordered_map<VertexId, Point3> candidate_points;
        for (const auto &vertex : input.candidate_front.vertices)
            candidate_points.emplace(
                vertex.source_vertex_id, vertex.position);
        for (const SurfaceFaceId seed_id : seeds)
        {
            const auto seed_position = face_indices.find(seed_id);
            if (seed_position == face_indices.end())
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
                const auto uses = edge_faces.find(edgeKey(
                    seed_ids[edge],
                    seed_ids[(edge + 1) % seed_ids.size()]));
                if (uses == edge_faces.end()) continue;
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
                    const auto candidate = candidate_points.find(
                        low_vertex.source_vertex_id);
                    points.push_back(candidate == candidate_points.end()
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
                const auto incident = vertex_faces.find(vertex);
                if (incident != vertex_faces.end())
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
}
