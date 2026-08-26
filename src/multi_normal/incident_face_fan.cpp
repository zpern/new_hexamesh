#include <algorithm>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/multi_normal/incident_face_fan.hpp>

namespace boundary_mesh
{
    namespace
    {
        using FanResult =
            Result<std::vector<IncidentFaceFan>, MultiNormalError>;

        bool validInput(
            const GrowthFront &front,
            const FrontEvaluation &evaluation)
        {
            return front.layer == evaluation.layer &&
                front.faces.size() == front.source_face_ids.size() &&
                front.faces.size() == evaluation.faces.size();
        }
    }

    Result<std::vector<IncidentFaceFan>, MultiNormalError>
    buildIncidentFaceFans(
        const GrowthFront &front,
        const FrontEvaluation &evaluation)
    {
        if (!validInput(front, evaluation))
        {
            return FanResult::failure(MultiNormalInputMismatch{
                front.vertices.size(), front.faces.size()});
        }

        std::vector<std::vector<IncidentFaceSector>> incident(
            front.vertices.size());
        for (std::size_t face_index = 0;
             face_index < front.faces.size();
             ++face_index)
        {
            if (evaluation.faces[face_index].front_face_index != face_index ||
                evaluation.faces[face_index].source_face_id !=
                    front.source_face_ids[face_index])
            {
                return FanResult::failure(MultiNormalInputMismatch{
                    front.vertices.size(), front.faces.size()});
            }

            const auto invalid = std::visit(
                [&](const auto &face) -> std::optional<VertexId>
                {
                    const std::size_t count = face.vertex_ids.size();
                    for (std::size_t local = 0; local < count; ++local)
                    {
                        const VertexId center = face.vertex_ids[local];
                        if (static_cast<std::size_t>(center) >=
                            front.vertices.size())
                        {
                            return center;
                        }
                        incident[static_cast<std::size_t>(center)].push_back(
                            IncidentFaceSector{
                                face_index,
                                face.vertex_ids[(local + count - 1) % count],
                                face.vertex_ids[(local + 1) % count],
                                evaluation.faces[face_index]
                                    .value.unit_normal});
                    }
                    return std::nullopt;
                },
                front.faces[face_index]);
            if (invalid.has_value())
            {
                return FanResult::failure(
                    InvalidMultiNormalTopology{*invalid});
            }
        }

        std::vector<IncidentFaceFan> output;
        output.reserve(front.vertices.size());
        for (std::size_t vertex_index = 0;
             vertex_index < front.vertices.size();
             ++vertex_index)
        {
            auto &sectors = incident[vertex_index];
            IncidentFaceFan fan;
            fan.center_vertex = static_cast<VertexId>(vertex_index);
            if (sectors.empty())
            {
                output.push_back(std::move(fan));
                continue;
            }

            std::map<VertexId, std::size_t> by_previous;
            std::map<VertexId, std::size_t> by_next;
            for (std::size_t index = 0; index < sectors.size(); ++index)
            {
                if (!by_previous.emplace(
                         sectors[index].previous_vertex, index).second ||
                    !by_next.emplace(
                         sectors[index].next_vertex, index).second)
                {
                    return FanResult::failure(
                        InconsistentIncidentFanWinding{
                            static_cast<VertexId>(vertex_index)});
                }
            }

            std::vector<std::size_t> starts;
            for (std::size_t index = 0; index < sectors.size(); ++index)
            {
                if (by_next.find(sectors[index].previous_vertex) ==
                    by_next.end())
                {
                    starts.push_back(index);
                }
            }

            std::size_t current{};
            if (starts.empty())
            {
                fan.closed = true;
                current = std::min_element(
                    sectors.begin(), sectors.end(),
                    [&](const IncidentFaceSector &first,
                        const IncidentFaceSector &second)
                    {
                        const auto first_key = std::make_pair(
                            front.source_face_ids[first.face_index],
                            first.face_index);
                        const auto second_key = std::make_pair(
                            front.source_face_ids[second.face_index],
                            second.face_index);
                        return first_key < second_key;
                    }) - sectors.begin();
            }
            else if (starts.size() == 1)
            {
                current = starts.front();
            }
            else
            {
                return FanResult::failure(DisconnectedIncidentFan{
                    static_cast<VertexId>(vertex_index)});
            }

            const std::size_t start_index = current;
            std::vector<bool> visited(sectors.size(), false);
            while (!visited[current])
            {
                visited[current] = true;
                fan.sectors.push_back(sectors[current]);
                const auto next = by_previous.find(
                    sectors[current].next_vertex);
                if (next == by_previous.end()) break;
                current = next->second;
            }

            if (fan.sectors.size() != sectors.size())
            {
                return FanResult::failure(DisconnectedIncidentFan{
                    static_cast<VertexId>(vertex_index)});
            }
            if (fan.closed && current != start_index)
            {
                return FanResult::failure(DisconnectedIncidentFan{
                    static_cast<VertexId>(vertex_index)});
            }
            output.push_back(std::move(fan));
        }

        return FanResult::success(std::move(output));
    }
}
