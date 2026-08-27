#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>
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

        struct IncidenceLayer
        {
            std::vector<SurfaceFaceId> faces;
            std::vector<int> directions;
        };

        struct EdgeIncidence
        {
            IncidenceLayer non_internal;
            IncidenceLayer internal;
        };

        bool isInternalFaceTag(
            const SurfaceBoundaryTag &tag) noexcept
        {
            return tag.kind ==
                   SurfaceBoundaryKind::Internal;
        }

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
            bool is_internal,
            std::unordered_map<
                EdgeKey,
                EdgeId,
                EdgeKeyHash> &edge_ids,
            std::vector<Edge> &edges,
            std::vector<EdgeIncidence>
                &edge_incidence,
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

                edge_incidence.emplace_back();

                IncidenceLayer &layer =
                    is_internal
                        ? edge_incidence.back().internal
                        : edge_incidence.back().non_internal;

                layer.faces.push_back(face_id);
                layer.directions.push_back(direction);

                return edge_id;
            }

            const EdgeId edge_id =
                found->second;

            const auto edge_index =
                static_cast<std::size_t>(
                    edge_id);

            IncidenceLayer &layer =
                is_internal
                    ? edge_incidence[edge_index].internal
                    : edge_incidence[edge_index].non_internal;

            auto &faces = layer.faces;
            auto &directions = layer.directions;

            if (faces.empty())
            {
                faces.push_back(face_id);
                directions.push_back(direction);
                return edge_id;
            }

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
            bool is_internal,
            std::unordered_map<
                EdgeKey,
                EdgeId,
                EdgeKeyHash> &edge_ids,
            std::vector<Edge> &edges,
            std::vector<EdgeIncidence>
                &edge_incidence,
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
                        is_internal,
                        edge_ids,
                        edges,
                        edge_incidence,
                        error);
            }

            return face_edge_ids;
        }

        /// 根据逐边邻接关系生成一个面的相邻面数组。
        ///
        /// Internal 与非 Internal 面只在各自拓扑层内寻找邻面。
        /// Internal 开放边没有同层邻面，结果保持为 nullopt。
        template <std::size_t Count>
        std::array<OptionalSurfaceFaceId, Count>
        makeNeighbors(
            SurfaceFaceId face_id,
            bool is_internal,
            const std::array<
                EdgeId,
                Count> &face_edge_ids,
            const std::vector<
                EdgeFaceIds> &edge_faces)
        {
            std::array<
                OptionalSurfaceFaceId,
                Count>
                neighbors{};

            for (std::size_t index = 0;
                 index < Count;
                 ++index)
            {
                const EdgeFaceIds &incidence =
                    edge_faces[static_cast<std::size_t>(
                        face_edge_ids[index])];

                const auto &faces =
                    is_internal
                        ? incidence.internal_faces
                        : incidence.non_internal_faces;

                if (faces[0] == face_id)
                {
                    neighbors[index] = faces[1];
                }
                else if (faces[1] == face_id)
                {
                    neighbors[index] = faces[0];
                }
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

        // 所有输入顶点都必须是有限坐标，
        // 包括当前没有被任何面引用的顶点。
        for (std::size_t vertex_index = 0;
             vertex_index < mesh.vertices.size();
             ++vertex_index)
        {
            const Point3 &point =
                mesh.vertices[vertex_index];

            if (!std::isfinite(point.x()) ||
                !std::isfinite(point.y()) ||
                !std::isfinite(point.z()))
            {
                return BuildResult::failure(
                    SurfaceTopologyError{
                        NonFiniteVertex{
                            static_cast<VertexId>(
                                vertex_index)}});
            }
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

        std::vector<EdgeIncidence>
            edge_incidence;

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

            const bool is_internal =
                isInternalFaceTag(
                    mesh.face_tags[face_index]);

            face_edges.push_back(
                std::visit(
                    [&](const auto &face)
                        -> FaceEdgeIds
                    {
                        return appendFace(
                            face.vertex_ids,
                            face_id,
                            is_internal,
                            edge_ids,
                            edges,
                            edge_incidence,
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
        // 完整输入表面必须封闭。成功扫描到这里时，
        // 非流形边和方向冲突已经被提前拒绝，因此只需
        // 检查是否存在仅关联一个面的开放边。
        for (std::size_t edge_index = 0;
             edge_index < edges.size();
             ++edge_index)
        {
            const auto &non_internal_faces =
                edge_incidence[edge_index]
                    .non_internal.faces;

            if (non_internal_faces.size() == 1)
            {
                return BuildResult::failure(
                    SurfaceTopologyError{
                        BoundaryEdge{
                            edges[edge_index]
                                .vertex_ids,
                            non_internal_faces[0]}});
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
            const EdgeIncidence &incidence =
                edge_incidence[edge_index];

            for (std::size_t index = 0;
                 index < incidence.non_internal.faces.size();
                 ++index)
            {
                edge_faces[edge_index]
                    .non_internal_faces[index] =
                    incidence.non_internal.faces[index];
            }

            for (std::size_t index = 0;
                 index < incidence.internal.faces.size();
                 ++index)
            {
                edge_faces[edge_index]
                    .internal_faces[index] =
                    incidence.internal.faces[index];
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

            const bool is_internal =
                isInternalFaceTag(
                    mesh.face_tags[face_index]);

            face_neighbors.push_back(
                std::visit(
                    [&](const auto &ids)
                        -> FaceNeighborIds
                    {
                        return makeNeighbors(
                            face_id,
                            is_internal,
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
