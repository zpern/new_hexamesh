#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

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

            ComplexNode &node = nodes[vertex];
            std::vector<BLVector> normals;
            for (const IncidentFaceSector &sector : fan.sectors)
            {
                const std::size_t neighbor =
                    static_cast<std::size_t>(sector.previous_vertex);
                if (neighbor >= nodes.size())
                    return PlanResult::failure(
                        InvalidMultiNormalTopology{
                            front.vertices[vertex].source_vertex_id});
                node.neighbour_node_.push_back(nodes.begin() + neighbor);
                node.neighbour_front_direction_.push_back(
                    toBl(sector.unit_normal));
                node.neighbour_front_index_.push_back(
                    static_cast<int>(sector.face_index));
                normals.push_back(toBl(sector.unit_normal));
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
                if (face_ids.empty()) continue;
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
                    const auto sector = std::find_if(
                        fan.sectors.begin(), fan.sectors.end(),
                        [&](const IncidentFaceSector &item)
                        {
                            return item.face_index ==
                                static_cast<std::size_t>(face_id);
                        });
                    if (sector == fan.sectors.end())
                        return PlanResult::failure(
                            InvalidMultiNormalTopology{
                                plan.source_vertex_id});
                    branch.visibility_cosine = std::min(
                        branch.visibility_cosine,
                        branch.direction.dot(sector->unit_normal));
                }
                plan.branches.push_back(std::move(branch));
            }

            std::sort(plan.splitter_neighbors.begin(),
                      plan.splitter_neighbors.end());
            plan.splitter_neighbors.erase(
                std::unique(plan.splitter_neighbors.begin(),
                            plan.splitter_neighbors.end()),
                plan.splitter_neighbors.end());
            if (plan.branches.size() >= 2 &&
                plan.splitter_neighbors.size() <= plan.branches.size())
                plans.push_back(std::move(plan));
        }
        return PlanResult::success(std::move(plans));
    }
}
