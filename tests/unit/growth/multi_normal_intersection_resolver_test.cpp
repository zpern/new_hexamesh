#include <algorithm>
#include <array>
#include <cmath>

#include <boundary_mesh/growth/multi_normal_intersection_resolver.hpp>
#include <boundary_mesh/growth/multi_normal_transition_builder.hpp>
#include <boundary_mesh/growth/front_evaluator.hpp>
#include <boundary_mesh/growth/incident_face_fan.hpp>
#include <boundary_mesh/growth/multi_normal_split_planner.hpp>
#include <boundary_mesh/growth/multi_normal_topology_builder.hpp>
#include <boundary_mesh/growth/multi_normal_quad_triangulator.hpp>

int main()
{
    using namespace boundary_mesh;
    MultiNormalTopology topology;
    topology.front.vertices = {
        {Point3{0, 0, 0}, 0},
        {Point3{1, 0, 0}, 1},
        {Point3{0, 1, 0}, 2}};
    topology.front.vertices[0].multi_normal_branch = true;
    topology.front.vertices[0].direction = Vector3::UnitZ();
    topology.front.faces = {Triangle{{0, 1, 2}}};
    topology.front.source_face_ids = {0};

    MultiNormalOptions options;
    options.enabled = true;
    options.transition_height = Scalar{0.1};
    const std::vector<Scalar> input{Scalar{0.1}, Scalar{0}, Scalar{0}};
    const auto result = resolveMultiNormalLengths(topology, input, options);
    if (!result.hasValue() || result.value().lengths != input ||
        result.value().shrink_iterations != 0 ||
        result.value().used_zero_retry)
    {
        return 1;
    }

    // BLMesh treats geometric shared edges as legal even when one triangle is
    // from the grown surface and the other is from the bottom surface.
    topology.front.vertices.push_back({Point3{1, 1, 0}, 3});
    topology.front.faces.push_back(Triangle{{1, 3, 2}});
    topology.front.source_face_ids.push_back(1);
    const std::vector<Scalar> shared_edge_lengths{
        Scalar{0.1}, Scalar{0}, Scalar{0}, Scalar{0}};
    MultiNormalOptions candidate_options = options;
    candidate_options.resolved_transition_lengths = shared_edge_lengths;
    const auto candidate = buildMultiNormalTransition(topology, candidate_options);
    if (!candidate.hasValue()) return 2;
    if (!findMultiNormalIntersectionBadPoints(topology, candidate.value()).empty())
        return 3;

    std::vector<Scalar> uneven{Scalar{1}, Scalar{2}, Scalar{0}, Scalar{0}};
    smoothMultiNormalLengths(topology, uneven);
    if (std::abs(uneven[1] - Scalar{1.1}) > Scalar{1e-12} ||
        uneven[2] != Scalar{0} || uneven[3] != Scalar{0})
        return 4;

    // Eight-face patch around PLS point 38 in 2dot5_cf.
    GrowthFront patch;
    patch.vertices.resize(51658);
    patch.vertices[37] = {Point3{205.95030, -600.0, 3.3557019}, 37};
    patch.vertices[36] = {Point3{203.33470, -600.0, 3.3557019}, 36};
    patch.vertices[26183] = {Point3{203.76836, -598.20117, 3.3815732}, 26183};
    patch.vertices[4037] = {Point3{208.06746, -600.0, 3.3557019}, 4037};
    patch.vertices[26611] = {Point3{206.06467, -597.64380, 3.3902981}, 26611};
    patch.vertices[5406] = {Point3{206.75081, -600.0, 5.4110594}, 5406};
    patch.vertices[5996] = {Point3{205.21846, -600.0, 1.4766065}, 5996};
    patch.vertices[51652] = {Point3{205.47372, -601.95850, 2.1361139}, 51652};
    patch.vertices[51657] = {Point3{206.27426, -601.97083, 4.1914587}, 51657};
    patch.faces = {
        Triangle{{36, 37, 26183}}, Triangle{{4037, 26611, 37}},
        Triangle{{26183, 37, 26611}}, Triangle{{37, 5406, 4037}},
        Triangle{{5996, 51652, 37}}, Triangle{{5406, 37, 51657}},
        Triangle{{51657, 37, 51652}}, Triangle{{5996, 37, 36}}};
    patch.source_face_ids = {45190, 46183, 46300, 92036,
                             102878, 102886, 102887, 103010};
    MultiNormalOptions patch_options;
    patch_options.enabled = true;
    patch_options.transition_height = Scalar{0.01};
    const auto patch_evaluation = FrontEvaluator{}.evaluate(patch);
    if (!patch_evaluation.hasValue()) return 5;
    const auto patch_fans = buildIncidentFaceFans(patch, patch_evaluation.value());
    if (!patch_fans.hasValue()) return 6;
    const auto patch_plans = planMultiNormalSplits(
        patch, patch_fans.value(), patch_options);
    if (!patch_plans.hasValue()) return 7;
    const auto root_plan = std::find_if(
        patch_plans.value().begin(), patch_plans.value().end(),
        [](const VertexSplitPlan &plan) { return plan.source_vertex_id == 37; });
    if (root_plan == patch_plans.value().end() || root_plan->branches.size() != 3)
        return 8;
    const auto patch_topology = buildMultiNormalTopology(
        patch, patch_plans.value());
    if (!patch_topology.hasValue()) return 9;
    const auto patch_triangles = triangulateMultiNormalQuads(
        patch_topology.value());
    if (!patch_triangles.hasValue()) return 10;
    std::vector<Scalar> patch_lengths(
        patch_triangles.value().front.vertices.size(), Scalar{0});
    std::size_t root_branch_count = 0;
    for (std::size_t i = 0; i < patch_lengths.size(); ++i)
        if (patch_triangles.value().front.vertices[i].source_vertex_id == 37 &&
            patch_triangles.value().front.vertices[i].multi_normal_branch)
        {
            patch_lengths[i] = Scalar{0.01};
            ++root_branch_count;
        }
    if (root_branch_count != 3) return 11;
    MultiNormalOptions patch_candidate_options = patch_options;
    patch_candidate_options.resolved_transition_lengths = patch_lengths;
    const auto patch_candidate = buildMultiNormalTransition(
        patch_triangles.value(), patch_candidate_options);
    if (!patch_candidate.hasValue()) return 12;
    const std::array<Point3, 3> blmesh_reference{
        Point3{205.9478399984, -599.9931529556, 3.3625623938},
        Point3{205.9449854678, -600.0013926147, 3.3640575192},
        Point3{205.9437947976, -600.0070426038, 3.3585451504}};
    std::array<bool, 3> matched{};
    for (std::size_t i = 0; i < patch_lengths.size(); ++i)
        if (patch_lengths[i] > Scalar{0})
        {
            const Point3 &actual =
                patch_candidate.value().transformed_front.vertices[i].position;
            bool found = false;
            for (std::size_t reference = 0; reference < blmesh_reference.size(); ++reference)
                if (!matched[reference] &&
                    (actual - blmesh_reference[reference]).norm() < Scalar{1e-6})
                {
                    matched[reference] = true;
                    found = true;
                    break;
                }
            if (!found) return 13;
        }
    const auto patch_resolved = resolveMultiNormalLengths(
        patch_triangles.value(), patch_lengths, patch_options);
    if (!patch_resolved.hasValue() || patch_resolved.value().shrink_iterations != 0 ||
        patch_resolved.value().fallback_to_single_normal)
        return 14;
    for (std::size_t i = 0; i < patch_lengths.size(); ++i)
        if (patch_lengths[i] > Scalar{0} &&
            patch_resolved.value().lengths[i] <= Scalar{0})
            return 15;
    return 0;
}
