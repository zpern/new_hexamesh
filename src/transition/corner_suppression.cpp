#include <algorithm>
#include <array>
#include <optional>
#include <vector>

#include <boundary_mesh/transition/corner_suppression.hpp>
#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct FaceRecord
        {
            SurfaceFaceId id{};
            const SurfaceFace *face{};
        };

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

        std::optional<std::size_t> sharedLocalEdge(
            const SurfaceFace &low,
            const SurfaceFace &other)
        {
            const auto low_vertices = vertices(low);
            const auto other_vertices = vertices(other);
            for (std::size_t edge = 0; edge < low_vertices.size(); ++edge)
            {
                const VertexId first = low_vertices[edge];
                const VertexId second =
                    low_vertices[(edge + 1) % low_vertices.size()];
                if (std::find(other_vertices.begin(), other_vertices.end(),
                              first) != other_vertices.end() &&
                    std::find(other_vertices.begin(), other_vertices.end(),
                              second) != other_vertices.end())
                    return edge;
            }
            return std::nullopt;
        }

        bool incidentToAny(
            const SurfaceFace &face,
            const std::vector<VertexId> &selected)
        {
            const auto face_vertices = vertices(face);
            for (const VertexId vertex : selected)
                if (std::find(face_vertices.begin(), face_vertices.end(),
                              vertex) != face_vertices.end())
                    return true;
            return false;
        }

        const SurfaceFace *findFace(
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

        Point3 candidatePoint(
            const GrowthFront &candidate,
            VertexId source_vertex,
            const Point3 &fallback)
        {
            for (const auto &vertex : candidate.vertices)
                if (vertex.source_vertex_id == source_vertex)
                    return vertex.position;
            return fallback;
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
        for (const SurfaceFaceId seed_id : seeds)
        {
            const SurfaceFace *seed = findFace(
                input.current_front, seed_id);
            if (seed == nullptr)
                return SuppressionResult::failure(
                    TransitionCoordinationError{
                        MissingTransitionFaceState{seed_id}});

            std::vector<QuadHighNeighbor> quad_highs;
            std::vector<std::pair<std::size_t, SurfaceFaceId>> highs;
            for (std::size_t index = 0;
                 index < input.current_front.faces.size(); ++index)
            {
                const SurfaceFaceId other_id =
                    input.current_front.source_face_ids[index];
                if (other_id == seed_id ||
                    !retained(result.retained_high_faces, other_id))
                    continue;
                const auto edge = sharedLocalEdge(
                    *seed, input.current_front.faces[index]);
                if (edge.has_value())
                    highs.push_back({*edge, other_id});
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
                    points.push_back(candidatePoint(
                        input.candidate_front,
                        low_vertex.source_vertex_id,
                        low_vertex.position));
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

            const std::vector<SurfaceFaceId> retained_snapshot =
                result.retained_high_faces;
            for (const SurfaceFaceId high_id : retained_snapshot)
            {
                if (high_id == seed_id ||
                    std::find_if(highs.begin(), highs.end(),
                        [high_id, &selected_edges](const auto &high)
                        {
                            return high.second == high_id &&
                                std::find(selected_edges.begin(),
                                          selected_edges.end(),
                                          high.first) != selected_edges.end();
                        }) != highs.end())
                    continue;
                const SurfaceFace *high = findFace(
                    input.current_front, high_id);
                if (high != nullptr && incidentToAny(*high, non_contact))
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
