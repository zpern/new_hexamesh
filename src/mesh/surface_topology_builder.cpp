#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/mesh/surface_topology_builder.hpp>
#include <optional>

namespace boundary_mesh
{
    namespace
    {
        /// 与顶点绕序无关的规范面键，用于识别重复面。
        struct FaceKey
        {
            std::array<VertexId, 4> sorted_vertex_ids{};
            std::uint8_t vertex_count{};

            bool operator==(
                const FaceKey &other) const noexcept
            {
                return vertex_count ==
                           other.vertex_count &&
                       sorted_vertex_ids ==
                           other.sorted_vertex_ids;
            }
        };

        /// 为 FaceKey 提供哈希值。
        struct FaceKeyHash
        {
            std::size_t operator()(
                const FaceKey &key) const noexcept
            {
                std::size_t seed =
                    key.vertex_count;

                for (std::size_t index = 0;
                     index < key.vertex_count;
                     ++index)
                {
                    seed ^=
                        std::hash<VertexId>{}(
                            key.sorted_vertex_ids[index]) +
                        0x9e3779b9U +
                        (seed << 6U) +
                        (seed >> 2U);
                }

                return seed;
            }
        };

        /// 与局部边方向无关的规范边键。
        ///
        /// 始终保证 first < second，因此同一条无向边只会
        /// 在哈希表中出现一次。
        struct EdgeKey
        {
            VertexId first{};
            VertexId second{};

            bool operator==(
                const EdgeKey &other) const noexcept
            {
                return first == other.first &&
                       second == other.second;
            }
        };

        /// 为 EdgeKey 提供哈希值。

        struct EdgeKeyHash
        {
            std::size_t operator()(
                const EdgeKey &key) const noexcept
            {
                std::size_t seed =
                    std::hash<VertexId>{}(
                        key.first);

                seed ^=
                    std::hash<VertexId>{}(
                        key.second) +
                    0x9e3779b9U +
                    (seed << 6U) +
                    (seed >> 2U);

                return seed;
            }
        };

        /// 将局部有向边转换成规范无向边。
        EdgeKey makeEdgeKey(
            VertexId first,
            VertexId second)
        {
            if (first < second)
            {
                return EdgeKey{
                    first,
                    second};
            }

            return EdgeKey{
                second,
                first};
        }

        /// 返回局部边相对于规范边方向的符号。
        ///
        /// Task 4 只记录该信息；Task 5 将使用它检查
        /// 相邻面的方向是否一致。
        int edgeDirection(
            VertexId first,
            VertexId second)
        {
            return first < second ? 1 : -1;
        }

        /// 将三角形或四边形的顶点数组转换为统一序列。
        std::vector<VertexId> faceVertexIds(
            const SurfaceFace &face)
        {
            return std::visit(
                [](const auto &value)
                {
                    return std::vector<VertexId>{
                        value.vertex_ids.begin(),
                        value.vertex_ids.end()};
                },
                face);
        }

        /// 生成与面顶点绕序无关的规范面键。
        FaceKey makeFaceKey(
            const std::vector<VertexId>
                &vertex_ids)
        {
            FaceKey key;

            key.vertex_count =
                static_cast<std::uint8_t>(
                    vertex_ids.size());

            std::copy(
                vertex_ids.begin(),
                vertex_ids.end(),
                key.sorted_vertex_ids.begin());

            std::sort(
                key.sorted_vertex_ids.begin(),
                key.sorted_vertex_ids.begin() +
                    key.vertex_count);

            return key;
        }

        /// 登记一条局部边，并检查共享边是否合法。
        ///
        /// 第三个关联面会形成非流形边；第二个面如果沿共享边
        /// 使用相同方向，则两个面的绕序不一致。
        EdgeId appendEdge(
            VertexId first,
            VertexId second,
            SurfaceFaceId face_id,
            std::unordered_map<
                EdgeKey,
                EdgeId,
                EdgeKeyHash> &edge_ids,
            std::vector<Edge> &edges,
            std::vector<
                std::vector<SurfaceFaceId>>
                &incident_faces,
            std::vector<
                std::vector<int>>
                &incident_directions,
            std::optional<
                SurfaceTopologyError> &error)
        {
            // 当前面前面的局部边已经发现错误时，不再继续登记。
            if (error.has_value())
            {
                return EdgeId{};
            }

            const EdgeKey key =
                makeEdgeKey(
                    first,
                    second);

            const int direction =
                edgeDirection(
                    first,
                    second);

            const auto found =
                edge_ids.find(key);

            // 第一次遇到该边时，分配稳定的 EdgeId。
            if (found == edge_ids.end())
            {
                const auto edge_id =
                    static_cast<EdgeId>(
                        edges.size());

                edge_ids.emplace(
                    key,
                    edge_id);

                edges.push_back(
                    Edge{{key.first,
                          key.second}});

                incident_faces.push_back(
                    {face_id});

                incident_directions.push_back(
                    {direction});

                return edge_id;
            }

            const EdgeId edge_id =
                found->second;

            const auto edge_index =
                static_cast<std::size_t>(
                    edge_id);

            auto &faces =
                incident_faces[edge_index];

            auto &directions =
                incident_directions[edge_index];

            // 已经有两个关联面时，当前面就是第三个关联面。
            if (faces.size() >= 2)
            {
                error =
                    SurfaceTopologyError{
                        NonManifoldEdge{
                            {key.first,
                             key.second},
                            {faces[0],
                             faces[1],
                             face_id}}};

                return edge_id;
            }

            // 封闭且方向一致的表面中，共享边两侧的面必须
            // 使用相反的局部边方向。
            if (directions[0] == direction)
            {
                error =
                    SurfaceTopologyError{
                        InconsistentOrientation{
                            {key.first,
                             key.second},
                            faces[0],
                            face_id}};

                return edge_id;
            }

            faces.push_back(face_id);
            directions.push_back(direction);

            return edge_id;
        }
        /// 登记一个面的所有顶点和局部边。
        template <std::size_t Count>
        std::array<EdgeId, Count> appendFace(
            const std::array<
                VertexId,
                Count> &vertex_ids,
            SurfaceFaceId face_id,
            std::unordered_map<
                EdgeKey,
                EdgeId,
                EdgeKeyHash> &edge_ids,
            std::vector<Edge> &edges,
            std::vector<
                std::vector<SurfaceFaceId>>
                &incident_faces,
            std::vector<
                std::vector<int>>
                &incident_directions,
            std::vector<
                std::vector<SurfaceFaceId>>
                &vertex_faces,
            std::optional<
                SurfaceTopologyError> &error)
        {
            std::array<EdgeId, Count>
                face_edge_ids{};

            for (std::size_t index = 0;
                 index < Count;
                 ++index)
            {
                const VertexId vertex_id =
                    vertex_ids[index];

                vertex_faces[static_cast<std::size_t>(
                                 vertex_id)]
                    .push_back(face_id);

                face_edge_ids[index] =
                    appendEdge(
                        vertex_id,
                        vertex_ids[(index + 1) %
                                   Count],
                        face_id,
                        edge_ids,
                        edges,
                        incident_faces,
                        incident_directions,
                        error);
            }

            return face_edge_ids;
        }

        /// 根据逐边邻接关系生成一个面的相邻面数组。
        ///
        /// 当前成功路径假设每条边至少关联两个面；
        /// Task 6 将在调用本函数前正式验证封闭性。
        template <std::size_t Count>
        std::array<SurfaceFaceId, Count>
        makeNeighbors(
            SurfaceFaceId face_id,
            const std::array<
                EdgeId,
                Count> &face_edge_ids,
            const std::vector<
                EdgeFaceIds> &edge_faces)
        {
            std::array<
                SurfaceFaceId,
                Count>
                neighbors{};

            for (std::size_t index = 0;
                 index < Count;
                 ++index)
            {
                const auto &faces =
                    edge_faces[static_cast<std::size_t>(
                        face_edge_ids[index])];

                neighbors[index] =
                    faces[0] == face_id
                        ? faces[1]
                        : faces[0];
            }

            return neighbors;
        }
    }

    Result<
        SurfaceTopology,
        SurfaceTopologyError>
    SurfaceTopologyBuilder::build(
        const SurfaceMesh &mesh) const
    {
        using BuildResult =
            Result<
                SurfaceTopology,
                SurfaceTopologyError>;

        // 空表面不存在可构建的拓扑。
        if (mesh.faces.empty())
        {
            return BuildResult::failure(
                SurfaceTopologyError{
                    EmptySurface{}});
        }

        // 每个输入面必须拥有一个边界标签。
        if (mesh.faces.size() !=
            mesh.face_tags.size())
        {
            return BuildResult::failure(
                SurfaceTopologyError{
                    FaceTagCountMismatch{
                        mesh.faces.size(),
                        mesh.face_tags.size()}});
        }

        // 第一阶段只验证面自身，不产生任何拓扑结果。
        std::unordered_map<
            FaceKey,
            SurfaceFaceId,
            FaceKeyHash>
            first_face_by_key;

        for (std::size_t face_index = 0;
             face_index < mesh.faces.size();
             ++face_index)
        {
            const auto face_id =
                static_cast<SurfaceFaceId>(
                    face_index);

            const auto vertex_ids =
                faceVertexIds(
                    mesh.faces[face_index]);

            std::unordered_set<VertexId>
                unique_vertex_ids;

            for (const VertexId vertex_id :
                 vertex_ids)
            {
                if (static_cast<std::size_t>(
                        vertex_id) >=
                    mesh.vertices.size())
                {
                    return BuildResult::failure(
                        SurfaceTopologyError{
                            InvalidVertexReference{
                                face_id,
                                vertex_id}});
                }

                if (!unique_vertex_ids
                         .insert(vertex_id)
                         .second)
                {
                    return BuildResult::failure(
                        SurfaceTopologyError{
                            DegenerateFace{
                                face_id}});
                }
            }

            const auto insertion =
                first_face_by_key.emplace(
                    makeFaceKey(vertex_ids),
                    face_id);

            if (!insertion.second)
            {
                return BuildResult::failure(
                    SurfaceTopologyError{
                        DuplicateFace{
                            insertion
                                .first
                                ->second,
                            face_id}});
            }
        }

        // 第二阶段按面和局部边顺序创建稳定 EdgeId。
        std::unordered_map<
            EdgeKey,
            EdgeId,
            EdgeKeyHash>
            edge_ids;

        std::vector<Edge> edges;

        std::vector<
            std::vector<SurfaceFaceId>>
            incident_faces;

        std::vector<
            std::vector<int>>
            incident_directions;

        std::vector<FaceEdgeIds>
            face_edges;

        // 在遍历一个面的局部边期间传递首个边拓扑错误。
        std::optional<SurfaceTopologyError>
            edge_error;

        std::vector<
            std::vector<SurfaceFaceId>>
            vertex_faces(
                mesh.vertices.size());

        face_edges.reserve(
            mesh.faces.size());

        for (std::size_t face_index = 0;
             face_index < mesh.faces.size();
             ++face_index)
        {
            const auto face_id =
                static_cast<SurfaceFaceId>(
                    face_index);

            face_edges.push_back(
                std::visit(
                    [&](const auto &face)
                        -> FaceEdgeIds
                    {
                        return appendFace(
                            face.vertex_ids,
                            face_id,
                            edge_ids,
                            edges,
                            incident_faces,
                            incident_directions,
                            vertex_faces,
                            edge_error);
                    },
                    mesh.faces[face_index]));
            if (edge_error.has_value())
            {
                return BuildResult::failure(
                    std::move(*edge_error));
            }
        }

        // 将构建期动态邻接转换为稠密的边到面数组。
        std::vector<EdgeFaceIds>
            edge_faces(
                edges.size());

        for (std::size_t edge_index = 0;
             edge_index < edges.size();
             ++edge_index)
        {
            if (!incident_faces[edge_index]
                     .empty())
            {
                edge_faces[edge_index][0] =
                    incident_faces[edge_index][0];
            }

            if (incident_faces[edge_index]
                    .size() >= 2)
            {
                edge_faces[edge_index][1] =
                    incident_faces[edge_index][1];
            }
        }

        // 面邻接顺序必须与该面的局部边顺序一致。
        std::vector<FaceNeighborIds>
            face_neighbors;

        face_neighbors.reserve(
            face_edges.size());

        for (std::size_t face_index = 0;
             face_index < face_edges.size();
             ++face_index)
        {
            const auto face_id =
                static_cast<SurfaceFaceId>(
                    face_index);

            face_neighbors.push_back(
                std::visit(
                    [&](const auto &ids)
                        -> FaceNeighborIds
                    {
                        return makeNeighbors(
                            face_id,
                            ids,
                            edge_faces);
                    },
                    face_edges[face_index]));
        }

        return BuildResult::success(
            SurfaceTopology{
                std::move(edges),
                std::move(edge_faces),
                std::move(face_edges),
                std::move(face_neighbors),
                std::move(vertex_faces)});
    }
}