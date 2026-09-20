#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface_orientation.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct EdgeKey
        {
            VertexId first{};
            VertexId second{};
            bool operator==(const EdgeKey &other) const noexcept
            {
                return first == other.first && second == other.second;
            }
        };

        struct EdgeKeyHash
        {
            std::size_t operator()(const EdgeKey &key) const noexcept
            {
                const auto first = static_cast<std::size_t>(key.first);
                const auto second = static_cast<std::size_t>(key.second);
                return first ^ (second + 0x9e3779b9U +
                                (first << 6U) + (first >> 2U));
            }
        };

        struct EdgeIncidence
        {
            std::array<SurfaceFaceId, 2> face_ids{};
            std::array<bool, 2> canonical_directions{};
            std::size_t count{};
        };

        struct OrientationConstraint
        {
            SurfaceFaceId neighbor{};
            EdgeKey edge{};
            bool flip_parity{};
        };

        void reverseFace(SurfaceFace &face) noexcept
        {
            std::visit(
                [](auto &value)
                {
                    std::reverse(
                        value.vertex_ids.begin() + 1,
                        value.vertex_ids.end());
                },
                face);
        }

        template <std::size_t Count>
        bool hasRepeatedVertex(
            const std::array<VertexId, Count> &vertex_ids) noexcept
        {
            for (std::size_t first = 0; first < Count; ++first)
                for (std::size_t second = first + 1; second < Count; ++second)
                    if (vertex_ids[first] == vertex_ids[second])
                        return true;
            return false;
        }
    }

    Result<std::size_t, SurfaceOrientationError>
    unifySurfaceOrientation(SurfaceMesh &mesh)
    {
        using ResultType = Result<std::size_t, SurfaceOrientationError>;
        std::unordered_map<EdgeKey, EdgeIncidence, EdgeKeyHash> edges;
        std::vector<std::vector<OrientationConstraint>> adjacency(
            mesh.faces.size());

        for (std::size_t face_index = 0;
             face_index < mesh.faces.size();
             ++face_index)
        {
            const auto face_id = static_cast<SurfaceFaceId>(face_index);
            std::optional<SurfaceOrientationError> error;
            std::visit(
                [&](const auto &face)
                {
                    if (hasRepeatedVertex(face.vertex_ids))
                    {
                        error = SurfaceOrientationError{DegenerateFace{face_id}};
                        return;
                    }
                    constexpr std::size_t count =
                        std::tuple_size_v<decltype(face.vertex_ids)>;
                    for (std::size_t edge_index = 0;
                         edge_index < count;
                         ++edge_index)
                    {
                        const VertexId first = face.vertex_ids[edge_index];
                        const VertexId second =
                            face.vertex_ids[(edge_index + 1) % count];
                        const EdgeKey key{
                            std::min(first, second),
                            std::max(first, second)};
                        auto &incidence = edges[key];
                        if (incidence.count >= 2)
                        {
                            error = SurfaceOrientationError{
                                NonManifoldEdge{
                                    {key.first, key.second},
                                    {incidence.face_ids[0],
                                     incidence.face_ids[1],
                                     face_id}}};
                            return;
                        }
                        incidence.face_ids[incidence.count] = face_id;
                        incidence.canonical_directions[incidence.count] =
                            first < second;
                        ++incidence.count;
                        if (incidence.count == 2)
                        {
                            const bool parity =
                                incidence.canonical_directions[0] ==
                                incidence.canonical_directions[1];
                            const auto other = incidence.face_ids[0];
                            adjacency[other].push_back({face_id, key, parity});
                            adjacency[face_id].push_back({other, key, parity});
                        }
                    }
                },
                mesh.faces[face_index]);
            if (error.has_value())
                return ResultType::failure(std::move(*error));
        }

        std::vector<int> flip(mesh.faces.size(), -1);
        std::queue<SurfaceFaceId> pending;
        for (std::size_t seed = 0; seed < mesh.faces.size(); ++seed)
        {
            if (flip[seed] >= 0)
                continue;
            flip[seed] = 0;
            pending.push(static_cast<SurfaceFaceId>(seed));
            while (!pending.empty())
            {
                const auto current = pending.front();
                pending.pop();
                for (const auto &constraint : adjacency[current])
                {
                    const int required =
                        flip[current] ^ static_cast<int>(constraint.flip_parity);
                    auto &neighbor_flip = flip[constraint.neighbor];
                    if (neighbor_flip < 0)
                    {
                        neighbor_flip = required;
                        pending.push(constraint.neighbor);
                    }
                    else if (neighbor_flip != required)
                    {
                        return ResultType::failure(
                            SurfaceOrientationError{
                                NonOrientableSurface{
                                    {constraint.edge.first,
                                     constraint.edge.second},
                                    current,
                                    constraint.neighbor}});
                    }
                }
            }
        }

        std::size_t reversed_count = 0;
        for (std::size_t face = 0; face < mesh.faces.size(); ++face)
        {
            if (flip[face] == 0)
                continue;
            reverseFace(mesh.faces[face]);
            ++reversed_count;
        }
        return ResultType::success(reversed_count);
    }

    void reverseSurfaceOrientation(
        SurfaceMesh &mesh) noexcept
    {
        for (SurfaceFace &face : mesh.faces)
            reverseFace(face);
    }
}
