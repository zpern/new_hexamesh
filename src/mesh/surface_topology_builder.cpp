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

namespace boundary_mesh
{
    namespace
    {
        /// 与面顶点顺序无关的规范面键，用于检测重复面。
        struct FaceKey
        {
            std::array<VertexId, 4> sorted_vertex_ids{};
            std::uint8_t vertex_count{};

            bool operator==(
                const FaceKey &other) const noexcept
            {
                return vertex_count == other.vertex_count &&
                       sorted_vertex_ids ==
                           other.sorted_vertex_ids;
            }
        };

        /// 为 FaceKey 提供 unordered_map 哈希值。
        struct FaceKeyHash
        {
            std::size_t operator()(
                const FaceKey &key) const noexcept
            {
                std::size_t seed = key.vertex_count;

                for (std::size_t i = 0;
                     i < key.vertex_count;
                     ++i)
                {
                    seed ^=
                        std::hash<VertexId>{}(
                            key.sorted_vertex_ids[i]) +
                        0x9e3779b9U +
                        (seed << 6U) +
                        (seed >> 2U);
                }

                return seed;
            }
        };

        /// 将三角形或四边形的定长顶点数组转换为统一序列。
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

        /// 对面顶点编号排序，生成与绕序无关的重复面键。
        FaceKey makeFaceKey(
            const std::vector<VertexId> &vertex_ids)
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
    }

    Result<SurfaceTopology, SurfaceTopologyError>
    SurfaceTopologyBuilder::build(
        const SurfaceMesh &mesh) const
    {
        using BuildResult =
            Result<
                SurfaceTopology,
                SurfaceTopologyError>;

        // 没有面片时不存在可构建的表面拓扑。
        if (mesh.faces.empty())
        {
            return BuildResult::failure(
                SurfaceTopologyError{
                    EmptySurface{}});
        }

        // 每个输入面都必须拥有一个边界标签。
        if (mesh.faces.size() !=
            mesh.face_tags.size())
        {
            return BuildResult::failure(
                SurfaceTopologyError{
                    FaceTagCountMismatch{
                        mesh.faces.size(),
                        mesh.face_tags.size()}});
        }

        // 保存每种规范面键第一次出现的面编号。
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

            // 检测面内重复顶点，同时验证顶点引用范围。
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

            const FaceKey key =
                makeFaceKey(vertex_ids);

            const auto insertion =
                first_face_by_key.emplace(
                    key,
                    face_id);

            if (!insertion.second)
            {
                return BuildResult::failure(
                    SurfaceTopologyError{
                        DuplicateFace{
                            insertion.first->second,
                            face_id}});
            }
        }

        // Task 3 只完成基础输入验证。
        // 稳定 EdgeId 和全部邻接数据将在下一任务中填充。
        return BuildResult::success(
            SurfaceTopology{
                {},
                {},
                {},
                {},
                std::vector<
                    std::vector<SurfaceFaceId>>(
                    mesh.vertices.size())});
    }
}