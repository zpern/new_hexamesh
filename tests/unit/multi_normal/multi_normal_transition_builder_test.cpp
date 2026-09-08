#include <cmath>
#include <variant>

#include <boundary_mesh/multi_normal/multi_normal_transition_builder.hpp>
#include <boundary_mesh/multi_normal/multi_normal_transition_generator.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    GrowthFrontVertex branch(
        const Point3 &point,
        VertexId source,
        std::uint32_t branch_id,
        const Vector3 &direction)
    {
        GrowthFrontVertex value{point, source};
        value.branch_id = branch_id;
        value.multi_normal_branch = true;
        value.direction = direction;
        return value;
    }

    bool allPositive(const VolumeMesh &mesh)
    {
        for (const VolumeCell &cell : mesh.cells)
        {
            const auto *tetra = std::get_if<Tetra>(&cell);
            if (tetra == nullptr) continue;
            TetraPoints points{};
            for (std::size_t i = 0; i < 4; ++i)
                points[i] = mesh.vertices[tetra->vertex_ids[i]];
            const auto result = evaluateTetra(points);
            if (!result.hasValue() ||
                result.value().validity != VolumeCellValidity::Valid)
                return false;
        }
        return true;
    }
}

int main()
{
    using namespace boundary_mesh;

    MultiNormalOptions options;
    options.enabled = true;
    options.transition_height = Scalar{0.25};

    MultiNormalTopology same_source;
    same_source.front.vertices = {
        branch(Point3::Zero(), 10, 0, Vector3::UnitX()),
        branch(Point3::Zero(), 10, 1, Vector3::UnitY()),
        branch(Point3::Zero(), 10, 2, Vector3::UnitZ())};
    same_source.front.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}}};
    same_source.front.source_face_ids = {20};
    const auto same_result = buildMultiNormalTransition(same_source, options);
    if (!same_result.hasValue() ||
        same_result.value().transition_cells.cells.size() != 1 ||
        !std::holds_alternative<Tetra>(
            same_result.value().transition_cells.cells[0]))
    {
        return 1;
    }
    if (!allPositive(same_result.value().transition_cells)) return 9;
    if (std::get<Tetra>(same_result.value().transition_cells.cells[0])
            .vertex_ids != std::array<VertexId, 4>{0, 1, 2, 3})
        return 14;

    MultiNormalTopology reversed_same_source = same_source;
    reversed_same_source.front.faces = {
        Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}}};
    const auto reversed_same_result = buildMultiNormalTransition(
        reversed_same_source, options);
    if (!reversed_same_result.hasValue() ||
        reversed_same_result.value().transition_cells.cells.size() != 1 ||
        std::get<Tetra>(reversed_same_result.value().transition_cells.cells[0])
                .vertex_ids != std::array<VertexId, 4>{0, 1, 3, 2})
        return 10;

    MultiNormalOptions per_vertex_options = options;
    per_vertex_options.resolved_transition_lengths = {
        Scalar{0.10}, Scalar{0.20}, Scalar{0.30}};
    const auto per_vertex_result = buildMultiNormalTransition(
        same_source, per_vertex_options);
    if (!per_vertex_result.hasValue()) return 20;
    for (std::size_t i = 0; i < 3; ++i)
    {
        const Point3 expected =
            per_vertex_options.resolved_transition_lengths[i] *
            same_source.front.vertices[i].direction;
        if ((per_vertex_result.value().transformed_front.vertices[i].position -
             expected).norm() > Scalar{1e-12})
        {
            return 21;
        }
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        if ((same_result.value().transformed_front.vertices[i].position -
             options.transition_height *
                 same_source.front.vertices[i].direction).norm() > 1e-12)
        {
            return 2;
        }
    }

    MultiNormalTopology two_sources;
    two_sources.front.vertices = {
        branch(Point3{0, 0, 0}, 30, 0, Vector3::UnitY()),
        branch(Point3{0, 0, 0}, 30, 1, Vector3::UnitZ()),
        GrowthFrontVertex{Point3{1, 0, 0}, 31}};
    two_sources.front.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}}};
    two_sources.front.source_face_ids = {40};
    const auto two_result = buildMultiNormalTransition(two_sources, options);
    if (!two_result.hasValue() ||
        two_result.value().transition_cells.cells.size() != 1 ||
        (two_result.value().transformed_front.vertices[2].position -
         Point3{1, 0, 0}).norm() > 1e-12)
    {
        return 3;
    }
    if (!allPositive(two_result.value().transition_cells)) return 11;
    if (std::get<Tetra>(two_result.value().transition_cells.cells[0])
            .vertex_ids != std::array<VertexId, 4>{0, 2, 3, 4})
        return 15;

    MultiNormalTopology repeated_after_lower_id;
    repeated_after_lower_id.front.vertices = {
        branch(Point3{1, 0, 0}, 41, 0, Vector3::UnitZ()),
        branch(Point3{0, 0, 0}, 42, 0, Vector3::UnitY()),
        branch(Point3{0, 0, 0}, 42, 1, Vector3::UnitZ())};
    repeated_after_lower_id.front.faces = {
        Triangle{{VertexId{1}, VertexId{2}, VertexId{0}}}};
    repeated_after_lower_id.front.source_face_ids = {43};
    const auto repeated_result = buildMultiNormalTransition(
        repeated_after_lower_id, options);
    if (!repeated_result.hasValue() ||
        repeated_result.value().transition_cells.cells.size() != 2)
    {
        return 4;
    }
    if (!allPositive(repeated_result.value().transition_cells)) return 12;
    if (std::get<Tetra>(repeated_result.value().transition_cells.cells[0])
            .vertex_ids != std::array<VertexId, 4>{0, 3, 4, 2} ||
        std::get<Tetra>(repeated_result.value().transition_cells.cells[1])
            .vertex_ids != std::array<VertexId, 4>{1, 3, 4, 0})
        return 16;

    MultiNormalTopology all_distinct;
    all_distinct.front.vertices = {
        branch(Point3{0, 0, 0}, 44, 0, Vector3::UnitZ()),
        GrowthFrontVertex{Point3{1, 0, 0}, 45},
        GrowthFrontVertex{Point3{0, 1, 0}, 46}};
    all_distinct.front.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}}};
    all_distinct.front.source_face_ids = {47};
    const auto distinct_result = buildMultiNormalTransition(
        all_distinct, options);
    if (!distinct_result.hasValue() ||
        distinct_result.value().transition_cells.cells.size() != 1)
    {
        return 5;
    }
    if (!allPositive(distinct_result.value().transition_cells)) return 13;
    if (std::get<Tetra>(distinct_result.value().transition_cells.cells[0])
            .vertex_ids != std::array<VertexId, 4>{0, 3, 4, 5})
        return 17;

    MultiNormalTopology quad;
    quad.front.vertices = {
        branch(Point3{0, 0, 0}, 50, 0, Vector3::UnitZ()),
        GrowthFrontVertex{Point3{1, 0, 0}, 51},
        GrowthFrontVertex{Point3{1, 1, 0}, 52},
        GrowthFrontVertex{Point3{0, 1, 0}, 53}};
    quad.front.faces = {
        Quad{{VertexId{0}, VertexId{1}, VertexId{2}, VertexId{3}}}};
    quad.front.source_face_ids = {60};
    const auto quad_result = buildMultiNormalTransition(quad, options);
    if (!quad_result.hasValue() ||
        !quad_result.value().transition_cells.cells.empty() ||
        quad_result.value().omitted_quad_transitions.size() != 1 ||
        !std::holds_alternative<Quad>(
            quad_result.value().transformed_front.faces[0]) ||
        (quad_result.value().transformed_front.vertices[0].position -
         Point3{0, 0, 0}).norm() <= 1e-12)
    {
        return 6;
    }
    const auto &omitted =
        quad_result.value().omitted_quad_transitions.front();
    if (omitted.source_face_id != 60 || !omitted.moved_corners[0] ||
        omitted.moved_corners[1] || omitted.topology_vertex_ids[3] != 3)
    {
        return 7;
    }

    GrowthFront cube_corner;
    cube_corner.vertices = {
        {Point3{0, 0, 0}, 70}, {Point3{1, 0, 0}, 71},
        {Point3{0, 1, 0}, 72}, {Point3{0, 0, 1}, 73},
        {Point3{1, 1, 0}, 74}, {Point3{1, 0, 1}, 75},
        {Point3{0, 1, 1}, 76}};
    cube_corner.faces = {
        Quad{{0, 2, 4, 1}},
        Quad{{0, 1, 5, 3}},
        Quad{{0, 3, 6, 2}}};
    cube_corner.source_face_ids = {80, 81, 82};
    MultiNormalOptions corner_options = options;
    corner_options.split_skewness_threshold = Scalar{0.5};
    const auto corner_result = generateMultiNormalTransition(
        cube_corner, corner_options);
    // A complex-to-ordinary edge is closed by the remaining virtual-sphere
    // triangle, not by an additional fixed strip face.  The latter used to
    // create a false self-intersection and force a single-normal fallback.
    if (!corner_result.hasValue() || !corner_result.value().applied ||
        corner_result.value().transformed_front.vertices.size() <= 7 ||
        corner_result.value().transformed_front.faces.size() <= 3 ||
        corner_result.value().transition_cells.cells.empty())
    {
        return 8;
    }

    return 0;
}
