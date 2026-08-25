#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
#include <array>
#include <map>
#include <optional>
#include <type_traits>

#include <boundary_mesh/growth/multi_normal_split_planner.hpp>

#include "complexnode.h"
#include "geometryfunction.h"
#include "MeshEvaluation.h"

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar pi = Scalar{3.14159265358979323846};
        BLVector toBl(const Vector3 &v) { return {v.x(), v.y(), v.z()}; }
        Vector3 fromBl(const BLVector &v) { return {v.x, v.y, v.z}; }
        Scalar skewness(Scalar cosine)
        {
            if (cosine < Scalar{0}) return Scalar{1};
            return std::acos(std::clamp(cosine, Scalar{-1}, Scalar{1})) /
                (pi / Scalar{2});
        }

        struct BlmeshFan
        {
            bool closed{};
            std::vector<VertexId> neighbors;
            std::vector<std::size_t> face_indices;
            std::vector<Vector3> normals;
        };

        BlmeshFan buildBlmeshFan(const GrowthFront &front, std::size_t center)
        {
            std::vector<std::array<VertexId, 3>> graph;
            std::vector<std::size_t> incident_faces;
            for (std::size_t face_index = 0; face_index < front.faces.size(); ++face_index)
            {
                const Triangle *triangle = std::get_if<Triangle>(&front.faces[face_index]);
                if (triangle == nullptr)
                {
                    const bool contains = std::visit([&](const auto &face) {
                        return std::find(face.vertex_ids.begin(), face.vertex_ids.end(),
                                         static_cast<VertexId>(center)) != face.vertex_ids.end();
                    }, front.faces[face_index]);
                    if (contains) return {};
                    continue;
                }
                auto connector = triangle->vertex_ids;
                bool contains = false;
                for (const VertexId id : connector) contains = contains || id == center;
                if (!contains) continue;
                if (connector[0] == center)
                {
                    connector[0] = connector[1];
                    connector[1] = connector[2];
                }
                else if (connector[1] == center)
                {
                    connector[1] = connector[0];
                    connector[0] = connector[2];
                }
                graph.push_back(connector);
                incident_faces.push_back(face_index);
            }
            if (graph.empty()) return {};

            std::map<VertexId, int> counts;
            for (const auto &edge : graph) ++counts[edge[0]];
            for (const auto &edge : graph)
                if (counts.find(edge[1]) != counts.end()) ++counts[edge[1]];
            VertexId start = graph.front()[0];
            bool closed = true;
            for (const auto &[vertex, count] : counts)
                if (count == 1) { start = vertex; closed = false; break; }

            BlmeshFan result;
            result.closed = closed;
            VertexId position = start;
            do
            {
                bool reached_end = true;
                for (const auto &edge : graph)
                    if (edge[0] == position)
                    {
                        position = edge[1];
                        result.neighbors.push_back(position);
                        reached_end = false;
                        break;
                    }
                if (position == start || reached_end) break;
            } while (result.neighbors.size() <= graph.size());
            if (result.neighbors.size() != graph.size()) return {};

            const Point3 &coordinate = front.vertices[center].position;
            for (std::size_t i = 0; i < result.neighbors.size(); ++i)
            {
                const std::size_t j = (i + 1) % result.neighbors.size();
                const Point3 &first = front.vertices[result.neighbors[j]].position;
                const Point3 &second = front.vertices[result.neighbors[i]].position;
                result.normals.push_back(
                    ((second - first).cross(first - coordinate)).normalized());
                std::array<VertexId, 3> wanted{
                    result.neighbors[i], result.neighbors[j],
                    static_cast<VertexId>(center)};
                std::sort(wanted.begin(), wanted.end());
                for (const std::size_t face_index : incident_faces)
                {
                    auto actual = std::get<Triangle>(front.faces[face_index]).vertex_ids;
                    std::sort(actual.begin(), actual.end());
                    if (actual == wanted) { result.face_indices.push_back(face_index); break; }
                }
            }
            if (result.face_indices.size() != result.neighbors.size()) return {};
            return result;
        }
    }

    Result<std::vector<VertexSplitPlan>, MultiNormalError>
    planMultiNormalSplits(
        const GrowthFront &front,
        const std::vector<IncidentFaceFan> &fans,
        const MultiNormalOptions &options)
    {
        using PlanResult =
            Result<std::vector<VertexSplitPlan>, MultiNormalError>;
        if (fans.size() != front.vertices.size())
            return PlanResult::failure(MultiNormalInputMismatch{
                front.vertices.size(), front.faces.size()});
        if (!options.enabled) return PlanResult::success({});

        std::vector<ComplexNode> nodes(front.vertices.size());
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            nodes[i].coordinate = toBl(front.vertices[i].position);
            nodes[i].node_id_ = static_cast<int>(i);
        }
        MeshEvaluator::GetSingleton().SetDefaultParameter(1e-5, 0.1);

        std::vector<VertexSplitPlan> plans;
        for (std::size_t vertex = 0; vertex < fans.size(); ++vertex)
        {
            const IncidentFaceFan &fan = fans[vertex];
            if (fan.center_vertex != static_cast<VertexId>(vertex) ||
                !fan.closed || fan.sectors.size() < 2)
                continue;

            BlmeshFan blmesh_fan = buildBlmeshFan(front, vertex);
            if (blmesh_fan.neighbors.empty() && fan.closed)
            {
                blmesh_fan.closed = true;
                for (const IncidentFaceSector &sector : fan.sectors)
                {
                    blmesh_fan.neighbors.push_back(sector.previous_vertex);
                    blmesh_fan.face_indices.push_back(sector.face_index);
                    blmesh_fan.normals.push_back(sector.unit_normal);
                }
            }
            if (!blmesh_fan.closed || blmesh_fan.neighbors.size() < 2)
                continue;

            ComplexNode &node = nodes[vertex];
            std::vector<BLVector> normals;
            for (std::size_t sector_index = 0;
                 sector_index < blmesh_fan.neighbors.size(); ++sector_index)
            {
                const std::size_t neighbor =
                    static_cast<std::size_t>(blmesh_fan.neighbors[sector_index]);
                if (neighbor >= nodes.size())
                    return PlanResult::failure(
                        InvalidMultiNormalTopology{
                            front.vertices[vertex].source_vertex_id});
                node.neighbour_node_.push_back(nodes.begin() + neighbor);
                node.neighbour_front_direction_.push_back(
                    toBl(blmesh_fan.normals[sector_index]));
                node.neighbour_front_index_.push_back(
                    static_cast<int>(blmesh_fan.face_indices[sector_index]));
                normals.push_back(toBl(blmesh_fan.normals[sector_index]));
            }
            node.single_normal_ =
                GEEOMETRY_FUNCTION::getMostNormal(normals).normalized();
            node.visible_angle_ =
                node.CaculateVisableAngle(node.single_normal_);
            node.original_skewness_ = skewness(node.visible_angle_);
            if (!std::isfinite(node.original_skewness_) ||
                node.original_skewness_ <= options.split_skewness_threshold)
                continue;

            node.ConfigureSplit(
                options.plane_skewness_threshold,
                options.convex_skewness_threshold,
                static_cast<int>(options.maximum_strategy_count));
            node.SplitNode();
            VirtualSphereMesh &mesh = node.getFinalMesh();
            if (mesh.getExtraPointCount() == 0) continue;

            VertexSplitPlan plan;
            plan.front_vertex_index = vertex;
            plan.source_vertex_id = front.vertices[vertex].source_vertex_id;
            plan.original_visibility_cosine = node.visible_angle_;
            plan.original_skewness = node.original_skewness_;
            plan.selected_skewness = mesh.caculateMaxSkewness();

            std::vector<std::optional<std::size_t>> branch_by_virtual_point(
                mesh.virtual_point_lists_.size());

            for (std::size_t index = 0;
                 index < mesh.virtual_point_lists_.size(); ++index)
            {
                if (mesh.valid.find(static_cast<int>(index)) ==
                    mesh.valid.end()) continue;
                VPoint &point = mesh.virtual_point_lists_[index];
                if (point.isFarNode())
                {
                    const int id = point.getGlobalIndex();
                    if (id >= 0)
                        plan.splitter_neighbors.push_back(
                            static_cast<VertexId>(id));
                    continue;
                }
                const std::vector<int> face_ids =
                    point.getGlobalNeighbourTriIndex();
                const bool inserted_branch =
                    mesh.valid.find(ONE_INSERT) != mesh.valid.end() &&
                    index + 1 == mesh.virtual_point_lists_.size();
                if (face_ids.empty() && !inserted_branch) continue;
                SplitBranch branch;
                branch.direction = fromBl(point.getCoord());
                const Scalar length = branch.direction.norm();
                if (!branch.direction.allFinite() ||
                    !std::isfinite(length) ||
                    length <= std::numeric_limits<Scalar>::epsilon())
                    return PlanResult::failure(
                        InvalidMultiNormalTopology{plan.source_vertex_id});
                branch.direction /= length;
                branch.visibility_cosine = Scalar{1};
                for (const int face_id : face_ids)
                {
                    if (face_id < 0 ||
                        static_cast<std::size_t>(face_id) >=
                            front.faces.size())
                        return PlanResult::failure(
                            InvalidMultiNormalTopology{
                                plan.source_vertex_id});
                    branch.face_indices.push_back(
                        static_cast<std::size_t>(face_id));
                    const auto sector = std::find(
                        blmesh_fan.face_indices.begin(), blmesh_fan.face_indices.end(),
                        static_cast<std::size_t>(face_id));
                    if (sector == blmesh_fan.face_indices.end())
                        return PlanResult::failure(
                            InvalidMultiNormalTopology{
                                plan.source_vertex_id});
                    branch.visibility_cosine = std::min(
                        branch.visibility_cosine,
                        branch.direction.dot(blmesh_fan.normals[
                            static_cast<std::size_t>(sector - blmesh_fan.face_indices.begin())]));
                }
                branch_by_virtual_point[index] = plan.branches.size();
                plan.branches.push_back(std::move(branch));
            }

            std::sort(plan.splitter_neighbors.begin(),
                      plan.splitter_neighbors.end());
            plan.splitter_neighbors.erase(
                std::unique(plan.splitter_neighbors.begin(),
                            plan.splitter_neighbors.end()),
                plan.splitter_neighbors.end());
            const auto point_reference = [&](int virtual_index)
                -> std::optional<SplitVirtualPoint>
            {
                if (virtual_index < 0 ||
                    static_cast<std::size_t>(virtual_index) >=
                        mesh.virtual_point_lists_.size())
                    return std::nullopt;
                VPoint &point = mesh.virtual_point_lists_[virtual_index];
                SplitVirtualPoint reference;
                if (point.isFarNode())
                {
                    const int global = point.getGlobalIndex();
                    if (global < 0) return std::nullopt;
                    reference.kind = SplitVirtualPoint::Kind::FarVertex;
                    reference.far_vertex_id = static_cast<VertexId>(global);
                    return reference;
                }
                const auto branch = branch_by_virtual_point[
                    static_cast<std::size_t>(virtual_index)];
                if (!branch.has_value()) return std::nullopt;
                reference.kind = SplitVirtualPoint::Kind::LocalBranch;
                reference.branch_index = *branch;
                return reference;
            };
            std::vector<std::optional<std::size_t>> stored_triangle_index(
                mesh.triangle_lists_.size());
            for (std::size_t triangle_index = 0;
                 triangle_index < mesh.triangle_lists_.size();
                 ++triangle_index)
            {
                const VTriangle &triangle = mesh.triangle_lists_[triangle_index];
                SplitVirtualTriangle stored;
                bool valid_triangle = true;
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const auto point = point_reference(
                        triangle.point_index_[corner]);
                    if (!point.has_value())
                    {
                        valid_triangle = false;
                        break;
                    }
                    stored.points[corner] = *point;
                }
                if (valid_triangle)
                {
                    stored_triangle_index[triangle_index] =
                        plan.virtual_triangles.size();
                    plan.virtual_triangles.push_back(std::move(stored));
                }
            }
            for (const VertexId neighbor : plan.splitter_neighbors)
            {
                SplitNeighborTriangleChain chain;
                chain.neighbor_vertex_id = neighbor;
                bool invalid_chain = false;
                for (std::size_t triangle_index = 0;
                     triangle_index < mesh.triangle_lists_.size();
                     ++triangle_index)
                {
                    const VTriangle &triangle = mesh.triangle_lists_[triangle_index];
                    for (std::size_t corner = 0; corner < 3; ++corner)
                    {
                        const int virtual_index = triangle.point_index_[corner];
                        VPoint &point = mesh.virtual_point_lists_[virtual_index];
                        if (!point.isFarNode() || point.getGlobalIndex() != neighbor)
                            continue;
                        const auto start = point_reference(
                            triangle.point_index_[(corner + 1) % 3]);
                        const auto end = point_reference(
                            triangle.point_index_[(corner + 2) % 3]);
                        if (!start.has_value() || !end.has_value())
                        {
                            chain.triangles.clear();
                            invalid_chain = true;
                            break;
                        }
                        if (!stored_triangle_index[triangle_index].has_value())
                        {
                            chain.triangles.clear();
                            invalid_chain = true;
                            break;
                        }
                        const BLVector first =
                            mesh.virtual_point_lists_[triangle.point_index_[(corner + 1) % 3]].getCoord() -
                            point.getCoord();
                        const BLVector second =
                            mesh.virtual_point_lists_[triangle.point_index_[(corner + 2) % 3]].getCoord() -
                            mesh.virtual_point_lists_[triangle.point_index_[(corner + 1) % 3]].getCoord();
                        const BLVector normal = (first ^ second).normalized();
                        chain.triangles.push_back(SplitActiveTriangle{
                            *stored_triangle_index[triangle_index], corner,
                            {*start, *end}, fromBl(normal)});
                    }
                    if (invalid_chain) break;
                }
                plan.neighbor_triangle_chains.push_back(std::move(chain));
            }
            if (plan.branches.size() >= 2 &&
                plan.splitter_neighbors.size() <= plan.branches.size())
                plans.push_back(std::move(plan));
        }
        return PlanResult::success(std::move(plans));
    }
}
