#include <boundary_mesh/transition/incremental_transition_templates.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool validIds(
            const std::array<VertexId, 4> &ids,
            const std::vector<Point3> &points)
        {
            for (const VertexId id : ids)
                if (static_cast<std::size_t>(id) >= points.size())
                    return false;
            return true;
        }

        void addMetadata(
            TransitionTemplateOutput &result,
            std::size_t count,
            std::uint32_t layer)
        {
            for (std::size_t index = 0; index < count; ++index)
                result.metadata.push_back({
                    CellRole::LayerTransition,
                    result.source_face_id,
                    layer});
        }
    }

    TransitionTemplateResult
    buildQuadTopCap(const QuadTopCapInput &input)
    {
        using BuildResult = Result<
            TransitionTemplateOutput, TransitionTemplateError>;
        if (input.mesh_vertices == nullptr || input.layer == 0 ||
            !validIds(input.bottom, *input.mesh_vertices) ||
            !validIds(input.top, *input.mesh_vertices))
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});

        TransitionTemplateOutput result;
        result.source_face_id = input.source_face_id;
        result.low_diagonal = input.diagonal;
        Point3 center = Point3::Zero();
        for (const VertexId id : input.bottom)
            center += (*input.mesh_vertices)[id];
        for (const VertexId id : input.top)
            center += (*input.mesh_vertices)[id];
        center /= Scalar{8};
        result.created_vertices.push_back(center);

        const VertexId c = input.center_vertex_id;
        result.volume_cells = {
            Pyramid{{input.bottom[0], input.bottom[1],
                     input.bottom[2], input.bottom[3], c}},
            Pyramid{{input.bottom[0], input.top[0],
                     input.top[1], input.bottom[1], c}},
            Pyramid{{input.bottom[1], input.top[1],
                     input.top[2], input.bottom[2], c}},
            Pyramid{{input.bottom[2], input.top[2],
                     input.top[3], input.bottom[3], c}},
            Pyramid{{input.bottom[3], input.top[3],
                     input.top[0], input.bottom[0], c}}};
        if (input.diagonal == QuadDiagonal::ZeroTwo)
        {
            result.volume_cells.push_back(
                Tetra{{input.top[0], input.top[1], input.top[2], c}});
            result.volume_cells.push_back(
                Tetra{{input.top[0], input.top[2], input.top[3], c}});
            result.top_faces = {
                Triangle{{input.top[0], input.top[1], input.top[2]}},
                Triangle{{input.top[0], input.top[2], input.top[3]}}};
        }
        else
        {
            result.volume_cells.push_back(
                Tetra{{input.top[1], input.top[2], input.top[3], c}});
            result.volume_cells.push_back(
                Tetra{{input.top[1], input.top[3], input.top[0], c}});
            result.top_faces = {
                Triangle{{input.top[1], input.top[2], input.top[3]}},
                Triangle{{input.top[1], input.top[3], input.top[0]}}};
        }
        addMetadata(result, result.volume_cells.size(), input.layer);
        return BuildResult::success(std::move(result));
    }

    TransitionTemplateResult
    buildQuadSideTransition(const QuadSideTransitionInput &input)
    {
        using BuildResult = Result<
            TransitionTemplateOutput, TransitionTemplateError>;
        if (input.high_edge_local_index >= 4)
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});

        TransitionTemplateOutput result;
        result.source_face_id = input.source_face_id;
        result.low_diagonal = input.low_diagonal;
        const std::size_t edge = input.high_edge_local_index;
        const VertexId a = input.low[edge];
        const VertexId b = input.low[(edge + 1) % 4];
        const VertexId d = input.low[(edge + 2) % 4];
        const VertexId c = input.low[(edge + 3) % 4];
        const VertexId e = input.high[edge];
        const VertexId f = input.high[(edge + 1) % 4];
        const bool diagonal_ad =
            (edge % 2 == 0 &&
             input.low_diagonal == QuadDiagonal::ZeroTwo) ||
            (edge % 2 == 1 &&
             input.low_diagonal == QuadDiagonal::OneThree);
        if (diagonal_ad)
        {
            result.volume_cells = {
                Pyramid{{e, f, b, a, d}},
                Tetra{{a, c, d, e}}};
            result.top_faces = {
                Triangle{{a, e, c}}, Triangle{{e, d, c}},
                Triangle{{e, f, d}}, Triangle{{f, b, d}}};
        }
        else
        {
            result.volume_cells = {
                Pyramid{{f, e, a, b, c}},
                Tetra{{b, d, c, f}}};
            result.top_faces = {
                Triangle{{b, f, d}}, Triangle{{f, c, d}},
                Triangle{{f, e, c}}, Triangle{{e, a, c}}};
        }
        addMetadata(
            result, result.volume_cells.size(), input.low_layer + 1);
        return BuildResult::success(std::move(result));
    }

    TransitionTemplateResult
    buildQuadAdjacentSideTransition(
        const QuadAdjacentSideTransitionInput &input)
    {
        using BuildResult = Result<
            TransitionTemplateOutput, TransitionTemplateError>;
        const std::size_t first = input.first_high_edge_local_index;
        const std::size_t second = input.second_high_edge_local_index;
        if (input.mesh_vertices == nullptr || first >= 4 || second >= 4 ||
            !validIds(input.low, *input.mesh_vertices) ||
            !validIds(input.high, *input.mesh_vertices) ||
            ((first + 1) % 4 != second && (second + 1) % 4 != first))
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});
        const std::size_t common =
            (first + 1) % 4 == second ? second : first;
        const QuadDiagonal required = common % 2 == 0
            ? QuadDiagonal::ZeroTwo
            : QuadDiagonal::OneThree;
        if (input.low_diagonal != required)
            return BuildResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});

        const VertexId a = input.low[common];
        const VertexId b = input.low[(common + 1) % 4];
        const VertexId d = input.low[(common + 2) % 4];
        const VertexId c = input.low[(common + 3) % 4];
        const VertexId e = input.high[common];
        const VertexId f = input.high[(common + 1) % 4];
        const VertexId g = input.high[(common + 3) % 4];

        TransitionTemplateOutput result;
        result.source_face_id = input.source_face_id;
        result.low_diagonal = input.low_diagonal;
        result.volume_cells = {
            Pyramid{{a,b,f,e,d}}, Pyramid{{a,e,g,c,d}}};
        result.top_faces = {
            Triangle{{b,f,d}}, Triangle{{e,f,d}},
            Triangle{{e,g,d}}, Triangle{{g,c,d}}};
        addMetadata(result, result.volume_cells.size(),
                    input.low_layer + 1);
        return BuildResult::success(std::move(result));
    }
}
