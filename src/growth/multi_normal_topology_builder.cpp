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
#include <boundary_mesh/growth/blmesh_topology_stitching.hpp>

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

        const SplitNeighborTriangleChain *chainForNeighbor(
            const VertexSplitPlan &plan, VertexId neighbor)
        {
            const auto found = std::find_if(
                plan.neighbor_triangle_chains.begin(),
                plan.neighbor_triangle_chains.end(),
                [&](const SplitNeighborTriangleChain &chain) {
                    return chain.neighbor_vertex_id == neighbor;
                });
            return found == plan.neighbor_triangle_chains.end()
                ? nullptr : &*found;
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
        std::vector<std::set<std::size_t>> consumed_virtual_triangles(
            front.vertices.size());
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
                    // BLMesh does not add a direct strip triangle for a
                    // complex-to-ordinary edge.  The corresponding virtual
                    // triangle remains unconsumed and is emitted in Step 7.
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

                const SplitNeighborTriangleChain *left_chain =
                    chainForNeighbor(plan, neighbor_id);
                const SplitNeighborTriangleChain *right_chain =
                    chainForNeighbor(*neighbor_plan,
                        static_cast<VertexId>(vertex));
                if (left_chain != nullptr && right_chain != nullptr &&
                    !left_chain->triangles.empty() &&
                    !right_chain->triangles.empty())
                {
                    const auto point_key = [](const SplitVirtualPoint &point) {
                        return point.kind == SplitVirtualPoint::Kind::LocalBranch
                            ? static_cast<int>(point.branch_index)
                            : -1 - static_cast<int>(point.far_vertex_id);
                    };
                    const auto order_chain = [&](
                        const SplitNeighborTriangleChain &chain)
                        -> Result<std::vector<SplitActiveTriangle>, MultiNormalError>
                    {
                        std::vector<BlmeshDirectedTriangle> directed;
                        for (std::size_t i = 0; i < chain.triangles.size(); ++i)
                            directed.push_back({static_cast<int>(i),
                                point_key(chain.triangles[i].directed_edge[0]),
                                point_key(chain.triangles[i].directed_edge[1])});
                        const auto ordered = orderBlmeshDirectedTriangleChain(directed);
                        if (!ordered.hasValue())
                            return Result<std::vector<SplitActiveTriangle>, MultiNormalError>::failure(
                                ordered.error());
                        std::vector<SplitActiveTriangle> result;
                        for (const auto &item : ordered.value())
                            result.push_back(chain.triangles[
                                static_cast<std::size_t>(item.triangle_index)]);
                        return Result<std::vector<SplitActiveTriangle>, MultiNormalError>::success(
                            std::move(result));
                    };
                    const auto ordered_left = order_chain(*left_chain);
                    const auto ordered_right = order_chain(*right_chain);
                    if (!ordered_left.hasValue() || !ordered_right.hasValue())
                        return TopologyResult::failure(
                            InvalidMultiNormalTopology{plan.source_vertex_id});
                    std::vector<Vector3> left_normals, right_normals;
                    for (const auto &triangle : ordered_left.value())
                        left_normals.push_back(triangle.unit_normal);
                    for (const auto &triangle : ordered_right.value())
                        right_normals.push_back(triangle.unit_normal);
                    const auto combination = findBlmeshSmoothestInterleaving(
                        left_normals, right_normals);
                    if (!combination.hasValue())
                        return TopologyResult::failure(combination.error());

                    const auto resolve = [&](std::size_t owner,
                                             const SplitVirtualPoint &point)
                        -> std::optional<VertexId>
                    {
                        if (point.kind == SplitVirtualPoint::Kind::LocalBranch)
                        {
                            if (point.branch_index >= copies[owner].size())
                                return std::nullopt;
                            return copies[owner][point.branch_index];
                        }
                        const std::size_t far =
                            static_cast<std::size_t>(point.far_vertex_id);
                        if (far >= copies.size()) return std::nullopt;
                        return copies[far][0];
                    };
                    std::vector<VertexId> left_api, right_api;
                    for (const auto &triangle : ordered_left.value())
                    {
                        const auto point = resolve(vertex, triangle.directed_edge[0]);
                        if (!point) return TopologyResult::failure(
                            InvalidMultiNormalTopology{plan.source_vertex_id});
                        left_api.push_back(*point);
                    }
                    {
                        const auto point = resolve(vertex,
                            ordered_left.value().back().directed_edge[1]);
                        if (!point) return TopologyResult::failure(
                            InvalidMultiNormalTopology{plan.source_vertex_id});
                        left_api.push_back(*point);
                    }
                    for (const auto &triangle : ordered_right.value())
                    {
                        const auto point = resolve(neighbor, triangle.directed_edge[0]);
                        if (!point) return TopologyResult::failure(
                            InvalidMultiNormalTopology{neighbor_plan->source_vertex_id});
                        right_api.push_back(*point);
                    }
                    {
                        const auto point = resolve(neighbor,
                            ordered_right.value().back().directed_edge[1]);
                        if (!point) return TopologyResult::failure(
                            InvalidMultiNormalTopology{neighbor_plan->source_vertex_id});
                        right_api.push_back(*point);
                    }

                    std::size_t left_front = 0;
                    int last_left = 0, last_right = 0;
                    for (const int choice : combination.value())
                    {
                        std::array<VertexId, 3> ids{};
                        const SplitActiveTriangle *triangle = nullptr;
                        std::size_t owner = 0;
                        VertexId api{};
                        if (choice > 0)
                        {
                            while (last_right > 0)
                            {
                                --last_right;
                                right_api.pop_back();
                            }
                            ++last_left;
                            triangle = &ordered_left.value()[
                                static_cast<std::size_t>(choice - 1)];
                            owner = vertex;
                            api = right_api.back();
                            consumed_virtual_triangles[vertex].insert(
                                triangle->virtual_triangle_index);
                        }
                        else
                        {
                            while (last_left > 0)
                            {
                                --last_left;
                                ++left_front;
                            }
                            ++last_right;
                            triangle = &ordered_right.value()[
                                static_cast<std::size_t>(-choice - 1)];
                            owner = neighbor;
                            api = left_api[left_front];
                            consumed_virtual_triangles[neighbor].insert(
                                triangle->virtual_triangle_index);
                        }
                        ids[triangle->far_corner] = api;
                        const auto start = resolve(owner, triangle->directed_edge[0]);
                        const auto end = resolve(owner, triangle->directed_edge[1]);
                        if (!start || !end)
                            return TopologyResult::failure(
                                InvalidMultiNormalTopology{plan.source_vertex_id});
                        ids[(triangle->far_corner + 1) % 3] = *start;
                        ids[(triangle->far_corner + 2) % 3] = *end;
                        output.front.faces.push_back(Triangle{ids});
                        output.front.source_face_ids.push_back(origins.front());
                        output.transition_face_origins.push_back(
                            TransitionFaceOrigin{output.front.faces.size() - 1,
                                origins, {plan.source_vertex_id,
                                    neighbor_plan->source_vertex_id}});
                    }
                    continue;
                }

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

        // BLMesh BuildTopo Step 7: append every virtual-sphere triangle that
        // was not consumed while stitching a complex-complex edge.
        for (const VertexSplitPlan &plan : plans)
        {
            const std::size_t owner = plan.front_vertex_index;
            const auto resolve = [&](const SplitVirtualPoint &point)
                -> std::optional<VertexId>
            {
                if (point.kind == SplitVirtualPoint::Kind::LocalBranch)
                {
                    if (point.branch_index >= copies[owner].size())
                        return std::nullopt;
                    return copies[owner][point.branch_index];
                }
                const std::size_t far =
                    static_cast<std::size_t>(point.far_vertex_id);
                if (far >= copies.size()) return std::nullopt;
                return copies[far][0];
            };
            std::vector<SurfaceFaceId> origins;
            for (const SplitBranch &branch : plan.branches)
                for (const std::size_t face_index : branch.face_indices)
                    if (face_index < front.source_face_ids.size())
                        origins.push_back(front.source_face_ids[face_index]);
            std::sort(origins.begin(), origins.end());
            origins.erase(std::unique(origins.begin(), origins.end()),
                          origins.end());
            if (origins.empty())
                return TopologyResult::failure(
                    InvalidMultiNormalTopology{plan.source_vertex_id});

            for (std::size_t triangle_index = 0;
                 triangle_index < plan.virtual_triangles.size();
                 ++triangle_index)
            {
                if (consumed_virtual_triangles[owner].count(triangle_index))
                    continue;
                std::array<VertexId, 3> ids{};
                bool valid_triangle = true;
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const auto id = resolve(
                        plan.virtual_triangles[triangle_index].points[corner]);
                    if (!id)
                    {
                        valid_triangle = false;
                        break;
                    }
                    ids[corner] = *id;
                }
                if (!valid_triangle)
                    return TopologyResult::failure(
                        InvalidMultiNormalTopology{plan.source_vertex_id});
                output.front.faces.push_back(Triangle{ids});
                output.front.source_face_ids.push_back(origins.front());
                output.transition_face_origins.push_back(
                    TransitionFaceOrigin{output.front.faces.size() - 1,
                        origins, {plan.source_vertex_id}});
            }
        }

        return TopologyResult::success(std::move(output));
    }
}
