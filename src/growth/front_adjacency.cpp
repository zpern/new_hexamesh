#include <algorithm>
#include <cstddef>
#include <optional>
#include <utility>
#include <variant>

#include <boundary_mesh/growth/front_adjacency.hpp>

namespace boundary_mesh
{
    namespace
    {
        void sortAndUnique(std::vector<std::size_t> &values)
        {
            std::sort(values.begin(), values.end());
            values.erase(
                std::unique(values.begin(), values.end()),
                values.end());
        }
    }

    Result<FrontAdjacency, FrontAdjacencyError>
    buildFrontAdjacency(const GrowthFront &front)
    {
        using AdjacencyResult =
            Result<FrontAdjacency, FrontAdjacencyError>;

        if (front.faces.size() != front.source_face_ids.size())
        {
            return AdjacencyResult::failure(
                FrontAdjacencyInputMismatch{
                    front.layer,
                    front.faces.size(),
                    front.source_face_ids.size()});
        }

        FrontAdjacency adjacency;
        adjacency.vertex_neighbors.resize(front.vertices.size());
        adjacency.vertex_incident_faces.resize(front.vertices.size());

        for (std::size_t face_index = 0;
             face_index < front.faces.size();
             ++face_index)
        {
            const auto failure = std::visit(
                [&](const auto &face)
                    -> std::optional<InvalidFrontAdjacencyReference>
                {
                    const std::size_t count = face.vertex_ids.size();
                    for (const VertexId vertex_id : face.vertex_ids)
                    {
                        if (static_cast<std::size_t>(vertex_id) >=
                            front.vertices.size())
                        {
                            return InvalidFrontAdjacencyReference{
                                front.layer,
                                face_index,
                                front.source_face_ids[face_index],
                                vertex_id};
                        }
                    }

                    for (std::size_t local = 0; local < count; ++local)
                    {
                        const std::size_t first =
                            static_cast<std::size_t>(face.vertex_ids[local]);
                        const std::size_t second =
                            static_cast<std::size_t>(
                                face.vertex_ids[(local + 1) % count]);

                        adjacency.vertex_neighbors[first].push_back(second);
                        adjacency.vertex_neighbors[second].push_back(first);
                        adjacency.vertex_incident_faces[first].push_back(
                            face_index);
                    }

                    return std::nullopt;
                },
                front.faces[face_index]);

            if (failure.has_value())
            {
                return AdjacencyResult::failure(*failure);
            }
        }

        for (auto &neighbors : adjacency.vertex_neighbors)
        {
            sortAndUnique(neighbors);
        }
        for (auto &faces : adjacency.vertex_incident_faces)
        {
            sortAndUnique(faces);
        }

        return AdjacencyResult::success(std::move(adjacency));
    }
}
