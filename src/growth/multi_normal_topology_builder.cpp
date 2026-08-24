#include <algorithm>
#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/multi_normal_topology_builder.hpp>

namespace boundary_mesh
{
    namespace
    {
        using TopologyResult = Result<MultiNormalTopology, MultiNormalError>;

        std::optional<std::size_t> branchForFace(
            const VertexSplitPlan &plan,
            std::size_t face_index)
        {
            for (std::size_t branch = 0;
                 branch < plan.branches.size();
                 ++branch)
            {
                const auto &faces = plan.branches[branch].face_indices;
                if (std::find(faces.begin(), faces.end(), face_index) !=
                    faces.end())
                {
                    return branch;
                }
            }
            return std::nullopt;
        }

        std::vector<SurfaceFaceId> incidentSourceFaces(
            const GrowthFront &front,
            VertexId first,
            VertexId second)
        {
            std::vector<SurfaceFaceId> result;
            for (std::size_t face_index = 0;
                 face_index < front.faces.size();
                 ++face_index)
            {
                const bool contains = std::visit(
                    [&](const auto &face)
                    {
                        bool has_first = false;
                        bool has_second = false;
                        for (const VertexId id : face.vertex_ids)
                        {
                            has_first = has_first || id == first;
                            has_second = has_second || id == second;
                        }
                        return has_first && has_second;
                    },
                    front.faces[face_index]);
                if (contains)
                {
                    result.push_back(front.source_face_ids[face_index]);
                }
            }
            std::sort(result.begin(), result.end());
            result.erase(std::unique(result.begin(), result.end()), result.end());
            return result;
        }
    }

    Result<MultiNormalTopology, MultiNormalError>
    buildMultiNormalTopology(
        const GrowthFront &front,
        const std::vector<VertexSplitPlan> &plans)
    {
        if (front.faces.size() != front.source_face_ids.size())
        {
            return TopologyResult::failure(MultiNormalInputMismatch{
                front.vertices.size(), front.faces.size()});
        }

        std::vector<const VertexSplitPlan *> plan_by_vertex(
            front.vertices.size(), nullptr);
        for (const VertexSplitPlan &plan : plans)
        {
            if (plan.front_vertex_index >= front.vertices.size() ||
                plan.source_vertex_id !=
                    front.vertices[plan.front_vertex_index].source_vertex_id ||
                plan.branches.size() < 2 ||
                plan.splitter_neighbors.size() > plan.branches.size() ||
                plan_by_vertex[plan.front_vertex_index] != nullptr)
            {
                return TopologyResult::failure(
                    InvalidMultiNormalTopology{plan.source_vertex_id});
            }
            plan_by_vertex[plan.front_vertex_index] = &plan;
        }

        MultiNormalTopology output;
        output.front = front;
        std::vector<std::vector<VertexId>> copies(front.vertices.size());
        for (std::size_t vertex_index = 0;
             vertex_index < front.vertices.size();
             ++vertex_index)
        {
            copies[vertex_index].push_back(
                static_cast<VertexId>(vertex_index));
            GrowthFrontVertex &base = output.front.vertices[vertex_index];
            if (const VertexSplitPlan *plan = plan_by_vertex[vertex_index])
            {
                base.branch_id = 0;
                base.multi_normal_branch = true;
                base.direction = plan->branches[0].direction;
                for (std::size_t branch = 1;
                     branch < plan->branches.size();
                     ++branch)
                {
                    if (output.front.vertices.size() >=
                        static_cast<std::size_t>(
                            std::numeric_limits<VertexId>::max()))
                    {
                        return TopologyResult::failure(
                            InvalidMultiNormalTopology{
                                plan->source_vertex_id});
                    }
                    GrowthFrontVertex copy =
                        front.vertices[vertex_index];
                    copy.branch_id = static_cast<std::uint32_t>(branch);
                    copy.multi_normal_branch = true;
                    copy.direction = plan->branches[branch].direction;
                    const VertexId id = static_cast<VertexId>(
                        output.front.vertices.size());
                    output.front.vertices.push_back(std::move(copy));
                    copies[vertex_index].push_back(id);
                }
            }
        }

        output.vertex_mapping.reserve(output.front.vertices.size());
        for (std::size_t transformed = 0;
             transformed < output.front.vertices.size();
             ++transformed)
        {
            const auto &vertex = output.front.vertices[transformed];
            output.vertex_mapping.push_back(SplitVertexMapping{
                static_cast<VertexId>(transformed),
                vertex.source_vertex_id,
                vertex.branch_id});
        }

        for (std::size_t face_index = 0;
             face_index < output.front.faces.size();
             ++face_index)
        {
            bool valid = true;
            std::visit(
                [&](auto &face)
                {
                    for (VertexId &id : face.vertex_ids)
                    {
                        const std::size_t original =
                            static_cast<std::size_t>(id);
                        if (original >= plan_by_vertex.size())
                        {
                            valid = false;
                            return;
                        }
                        const VertexSplitPlan *plan =
                            plan_by_vertex[original];
                        if (plan == nullptr) continue;
                        const auto branch = branchForFace(*plan, face_index);
                        if (!branch.has_value())
                        {
                            valid = false;
                            return;
                        }
                        id = copies[original][*branch];
                    }
                },
                output.front.faces[face_index]);
            if (!valid)
            {
                return TopologyResult::failure(
                    InvalidMultiNormalTopology{});
            }
        }

        std::set<std::array<VertexId, 2>> stitched_edges;
        for (const VertexSplitPlan &plan : plans)
        {
            const std::size_t vertex = plan.front_vertex_index;
            for (std::size_t split = 0;
                 split < plan.splitter_neighbors.size();
                 ++split)
            {
                const VertexId neighbor_id =
                    plan.splitter_neighbors[split];
                const std::size_t neighbor =
                    static_cast<std::size_t>(neighbor_id);
                if (neighbor >= front.vertices.size())
                {
                    return TopologyResult::failure(
                        InvalidMultiNormalTopology{plan.source_vertex_id});
                }
                const std::array<VertexId, 2> edge{
                    static_cast<VertexId>(std::min(vertex, neighbor)),
                    static_cast<VertexId>(std::max(vertex, neighbor))};
                if (!stitched_edges.insert(edge).second) continue;

                const std::size_t previous =
                    (split + plan.branches.size() - 1) %
                    plan.branches.size();
                const VertexId first_previous = copies[vertex][previous];
                const VertexId first_current = copies[vertex][split];
                const auto origins = incidentSourceFaces(
                    front, static_cast<VertexId>(vertex), neighbor_id);
                if (origins.empty())
                {
                    return TopologyResult::failure(
                        InvalidMultiNormalTopology{plan.source_vertex_id});
                }

                const VertexSplitPlan *neighbor_plan =
                    plan_by_vertex[neighbor];
                if (neighbor_plan == nullptr)
                {
                    output.front.faces.push_back(Triangle{{
                        first_previous,
                        first_current,
                        copies[neighbor][0]}});
                    output.front.source_face_ids.push_back(origins.front());
                    output.transition_face_origins.push_back(
                        TransitionFaceOrigin{
                            output.front.faces.size() - 1,
                            origins,
                            {plan.source_vertex_id,
                             front.vertices[neighbor].source_vertex_id}});
                    continue;
                }

                const auto other_split = std::find(
                    neighbor_plan->splitter_neighbors.begin(),
                    neighbor_plan->splitter_neighbors.end(),
                    static_cast<VertexId>(vertex));
                if (other_split ==
                    neighbor_plan->splitter_neighbors.end())
                {
                    return TopologyResult::failure(
                        InvalidMultiNormalTopology{
                            neighbor_plan->source_vertex_id});
                }
                const std::size_t other = static_cast<std::size_t>(
                    other_split -
                    neighbor_plan->splitter_neighbors.begin());
                const std::size_t other_previous =
                    (other + neighbor_plan->branches.size() - 1) %
                    neighbor_plan->branches.size();
                const VertexId second_previous =
                    copies[neighbor][other_previous];
                const VertexId second_current = copies[neighbor][other];

                const std::array<VertexId, 4> strip{
                    first_previous, first_current,
                    second_current, second_previous};
                const auto minimum = std::min_element(
                    strip.begin(), strip.end()) - strip.begin();
                std::array<Triangle, 2> triangles;
                if (minimum == 0 || minimum == 2)
                {
                    triangles = {
                        Triangle{{strip[0], strip[1], strip[2]}},
                        Triangle{{strip[0], strip[2], strip[3]}}};
                }
                else
                {
                    triangles = {
                        Triangle{{strip[0], strip[1], strip[3]}},
                        Triangle{{strip[1], strip[2], strip[3]}}};
                }
                for (const Triangle &triangle : triangles)
                {
                    output.front.faces.push_back(triangle);
                    output.front.source_face_ids.push_back(origins.front());
                    output.transition_face_origins.push_back(
                        TransitionFaceOrigin{
                            output.front.faces.size() - 1,
                            origins,
                            {plan.source_vertex_id,
                             neighbor_plan->source_vertex_id}});
                }
            }
        }

        return TopologyResult::success(std::move(output));
    }
}
