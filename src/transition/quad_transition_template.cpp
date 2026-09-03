#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include <boundary_mesh/surface/face_skewness.hpp>
#include <boundary_mesh/transition/reserved_layer_growth.hpp>
#include <boundary_mesh/transition/transition_templates.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool validIds(
            const std::array<VertexId, 4> &ids,
            const std::vector<Point3> &points)
        {
            for (const VertexId id : ids)
            {
                if (static_cast<std::size_t>(id) >= points.size())
                    return false;
            }
            return true;
        }

        Result<QuadDiagonalSelection, FaceEvaluationError>
        selectLayerQuad(
            const std::array<VertexId, 4> &ids,
            const std::vector<Point3> &points,
            Scalar tolerance)
        {
            OrientedQuad quad;
            quad.vertex_ids = ids;
            for (std::size_t index = 0; index < 4; ++index)
                quad.points[index] = points[ids[index]];
            return chooseQuadDiagonal(quad, tolerance);
        }

        Result<Scalar, FaceEvaluationError> tetraSkewness(
            const Tetra &tetra,
            const std::vector<Point3> &points,
            Scalar tolerance)
        {
            const auto &ids = tetra.vertex_ids;
            const std::array<std::array<std::size_t, 3>, 4> faces{{
                {{0, 1, 2}},
                {{0, 3, 1}},
                {{1, 3, 2}},
                {{2, 3, 0}}}};
            Scalar worst = Scalar{0};
            for (const auto &face : faces)
            {
                const std::array<Point3, 3> face_points{{
                    points[ids[face[0]]],
                    points[ids[face[1]]],
                    points[ids[face[2]]]}};
                const auto skewness = triangleEquiangularSkewness(
                    face_points, tolerance);
                if (!skewness.hasValue())
                {
                    return Result<Scalar, FaceEvaluationError>::failure(
                        skewness.error());
                }
                worst = std::max(worst, skewness.value());
            }
            return Result<Scalar, FaceEvaluationError>::success(worst);
        }

        Result<Scalar, FaceEvaluationError> tetraPairSkewness(
            const std::array<Tetra, 2> &tetrahedra,
            const std::vector<Point3> &points,
            Scalar tolerance)
        {
            const auto first = tetraSkewness(
                tetrahedra[0], points, tolerance);
            if (!first.hasValue())
                return Result<Scalar, FaceEvaluationError>::failure(
                    first.error());
            const auto second = tetraSkewness(
                tetrahedra[1], points, tolerance);
            if (!second.hasValue())
                return Result<Scalar, FaceEvaluationError>::failure(
                    second.error());
            return Result<Scalar, FaceEvaluationError>::success(
                std::max(first.value(), second.value()));
        }

        void appendSideTransition(
            SourceTransitionResult &result,
            const QuadTransitionInput &input,
            QuadDiagonal diagonal,
            std::uint32_t occupied)
        {
            const std::size_t edge = *input.high_edge_local_index;
            const auto &low = input.layer_vertex_ids[occupied];
            const auto &high = input.layer_vertex_ids[occupied + 1];
            const VertexId a = low[edge];
            const VertexId b = low[(edge + 1) % 4];
            const VertexId d = low[(edge + 2) % 4];
            const VertexId c = low[(edge + 3) % 4];
            const VertexId e = high[edge];
            const VertexId f = high[(edge + 1) % 4];
            const bool diagonal_ad =
                (edge % 2 == 0 && diagonal == QuadDiagonal::ZeroTwo) ||
                (edge % 2 == 1 && diagonal == QuadDiagonal::OneThree);

            Pyramid pyramid;
            Tetra tetra;
            if (diagonal_ad)
            {
                pyramid = Pyramid{{e, f, b, a, d}};
                tetra = Tetra{{a, c, d, e}};
                result.top_faces = {
                    Triangle{{a, e, c}},
                    Triangle{{e, d, c}},
                    Triangle{{e, f, d}},
                    Triangle{{f, b, d}}};
            }
            else
            {
                pyramid = Pyramid{{f, e, a, b, c}};
                tetra = Tetra{{b, d, c, f}};
                result.top_faces = {
                    Triangle{{b, f, d}},
                    Triangle{{f, c, d}},
                    Triangle{{f, e, c}},
                    Triangle{{e, a, c}}};
            }
            result.side_cells.push_back(pyramid);
            result.side_cells.push_back(tetra);
            result.volume_cells.push_back(pyramid);
            result.volume_cells.push_back(tetra);
            result.metadata.push_back(CellMetadata{
                CellRole::ReservedLayerTransition,
                input.source_face_id,
                occupied + 1});
            result.metadata.push_back(CellMetadata{
                CellRole::ReservedLayerTransition,
                input.source_face_id,
                occupied + 1});
        }

        Result<bool, FaceEvaluationError> appendContinuingEdgeTransition(
            SourceTransitionResult &result,
            const QuadTransitionInput &input,
            QuadDiagonal diagonal,
            std::uint32_t occupied)
        {
            const std::size_t edge =
                *input.continuing_edge_local_index;
            const auto &low = input.layer_vertex_ids[occupied];
            const auto &high = input.layer_vertex_ids[occupied + 1];
            const VertexId a = low[edge];
            const VertexId b = low[(edge + 1) % 4];
            const VertexId d = low[(edge + 2) % 4];
            const VertexId c = low[(edge + 3) % 4];
            const VertexId e = high[edge];
            const VertexId f = high[(edge + 1) % 4];
            const VertexId h = high[(edge + 2) % 4];
            const VertexId g = high[(edge + 3) % 4];
            const bool diagonal_ad =
                (edge % 2 == 0 && diagonal == QuadDiagonal::ZeroTwo) ||
                (edge % 2 == 1 && diagonal == QuadDiagonal::OneThree);

            const VertexId apex = diagonal_ad ? d : c;
            const std::array<Tetra, 2> method_one{{
                Tetra{{e, f, h, apex}},
                Tetra{{e, h, g, apex}}}};
            const std::array<Tetra, 2> method_two{{
                Tetra{{e, f, g, apex}},
                Tetra{{f, h, g, apex}}}};
            const auto score_one = tetraPairSkewness(
                method_one, *input.mesh_vertices,
                input.length_tolerance);
            const auto score_two = tetraPairSkewness(
                method_two, *input.mesh_vertices,
                input.length_tolerance);
            if (!score_one.hasValue() && !score_two.hasValue())
                return Result<bool, FaceEvaluationError>::failure(
                    score_two.error());
            const bool use_method_one =
                score_one.hasValue() &&
                (!score_two.hasValue() ||
                 score_one.value() < score_two.value());
            const auto &selected = use_method_one
                ? method_one
                : method_two;

            const Pyramid pyramid = diagonal_ad
                ? Pyramid{{b, f, e, a, d}}
                : Pyramid{{a, e, f, b, c}};
            const Tetra first_tetra = diagonal_ad
                ? Tetra{{a, d, e, c}}
                : Tetra{{b, c, f, d}};
            result.side_cells.push_back(pyramid);
            result.side_cells.push_back(first_tetra);
            result.volume_cells.push_back(pyramid);
            result.volume_cells.push_back(first_tetra);
            for (const Tetra &cell : selected)
            {
                result.side_cells.push_back(cell);
                result.volume_cells.push_back(cell);
            }
            for (std::size_t index = 0; index < 4; ++index)
                result.metadata.push_back(CellMetadata{
                    CellRole::ReservedLayerTransition,
                    input.source_face_id,
                    occupied + 1});

            if (diagonal_ad)
            {
                if (use_method_one)
                    result.top_faces = {
                        Triangle{{b, f, d}},
                        Triangle{{d, c, e}},
                        Triangle{{e, c, a}},
                        Triangle{{e, f, h}},
                        Triangle{{f, d, h}},
                        Triangle{{e, h, g}},
                        Triangle{{h, d, g}},
                        Triangle{{g, d, e}}};
                else
                    result.top_faces = {
                        Triangle{{b, f, d}},
                        Triangle{{d, c, e}},
                        Triangle{{e, c, a}},
                        Triangle{{e, f, g}},
                        Triangle{{g, d, e}},
                        Triangle{{f, h, g}},
                        Triangle{{f, d, h}},
                        Triangle{{h, d, g}}};
            }
            else
            {
                if (use_method_one)
                    result.top_faces = {
                        Triangle{{a, e, c}},
                        Triangle{{c, d, f}},
                        Triangle{{f, d, b}},
                        Triangle{{e, f, h}},
                        Triangle{{f, c, h}},
                        Triangle{{e, h, g}},
                        Triangle{{h, c, g}},
                        Triangle{{g, c, e}}};
                else
                    result.top_faces = {
                        Triangle{{a, e, c}},
                        Triangle{{c, d, f}},
                        Triangle{{f, d, b}},
                        Triangle{{e, f, g}},
                        Triangle{{g, c, e}},
                        Triangle{{f, h, g}},
                        Triangle{{f, c, h}},
                        Triangle{{h, c, g}}};
            }
            return Result<bool, FaceEvaluationError>::success(true);
        }

        Result<bool, FaceEvaluationError> appendDoubleSideTransition(
            SourceTransitionResult &result,
            const QuadTransitionInput &input,
            std::size_t common,
            std::uint32_t occupied)
        {
            const auto &low = input.layer_vertex_ids[occupied];
            const auto &high = input.layer_vertex_ids[occupied + 1];
            const VertexId a = low[common];
            const VertexId b = low[(common + 1) % 4];
            const VertexId d = low[(common + 2) % 4];
            const VertexId c = low[(common + 3) % 4];
            const VertexId e = high[common];
            const VertexId f = high[(common + 1) % 4];
            const VertexId h = high[(common + 2) % 4];
            const VertexId g = high[(common + 3) % 4];

            const std::array<Tetra, 2> method_one{{
                Tetra{{e, g, f, d}},
                Tetra{{g, f, h, d}}}};
            const std::array<Tetra, 2> method_two{{
                Tetra{{e, g, h, d}},
                Tetra{{e, f, h, d}}}};
            const auto score_one = tetraPairSkewness(
                method_one, *input.mesh_vertices,
                input.length_tolerance);
            const auto score_two = tetraPairSkewness(
                method_two, *input.mesh_vertices,
                input.length_tolerance);
            if (!score_one.hasValue() && !score_two.hasValue())
                return Result<bool, FaceEvaluationError>::failure(
                    score_two.error());
            const bool use_method_one =
                score_one.hasValue() &&
                (!score_two.hasValue() ||
                 score_one.value() < score_two.value());
            const auto &selected = use_method_one
                ? method_one
                : method_two;

            const std::array<Pyramid, 2> pyramids{{
                Pyramid{{a, b, f, e, d}},
                Pyramid{{a, e, g, c, d}}}};
            for (const Pyramid &cell : pyramids)
            {
                result.side_cells.push_back(cell);
                result.volume_cells.push_back(cell);
                result.metadata.push_back(CellMetadata{
                CellRole::ReservedLayerTransition,
                    input.source_face_id,
                    occupied + 1});
            }
            for (const Tetra &cell : selected)
            {
                result.side_cells.push_back(cell);
                result.volume_cells.push_back(cell);
                result.metadata.push_back(CellMetadata{
                    CellRole::ReservedLayerTransition,
                    input.source_face_id,
                    occupied + 1});
            }
            if (use_method_one)
            {
                result.top_faces = {
                    Triangle{{b, f, d}},
                    Triangle{{f, h, d}},
                    Triangle{{e, g, f}},
                    Triangle{{g, f, h}},
                    Triangle{{g, h, d}},
                    Triangle{{g, c, d}}};
            }
            else
            {
                result.top_faces = {
                    Triangle{{b, f, d}},
                    Triangle{{f, h, d}},
                    Triangle{{e, f, h}},
                    Triangle{{e, g, h}},
                    Triangle{{g, h, d}},
                    Triangle{{g, c, d}}};
            }
            return Result<bool, FaceEvaluationError>::success(true);
        }
    }

    TransitionTemplateResult buildQuadTransition(
        const QuadTransitionInput &input)
    {
        if (input.mesh_vertices == nullptr ||
            input.layer_vertex_ids.size() !=
                static_cast<std::size_t>(input.trial_layers) + 1 ||
            (input.high_edge_local_index.has_value() &&
             *input.high_edge_local_index >= 4) ||
            (input.second_high_edge_local_index.has_value() &&
             (!input.high_edge_local_index.has_value() ||
              *input.second_high_edge_local_index >= 4)) ||
            (input.third_continuing_vertex_local_index.has_value() &&
             (!input.high_edge_local_index.has_value() ||
              input.second_high_edge_local_index.has_value() ||
              *input.third_continuing_vertex_local_index >= 4)) ||
            (input.continuing_edge_local_index.has_value() &&
             (*input.continuing_edge_local_index >= 4 ||
              input.trial_layers == 0 ||
              input.high_edge_local_index.has_value() ||
              input.second_high_edge_local_index.has_value() ||
              input.third_continuing_vertex_local_index.has_value())))
        {
            return TransitionTemplateResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});
        }
        const auto &points = *input.mesh_vertices;
        for (const auto &ids : input.layer_vertex_ids)
        {
            if (!validIds(ids, points))
            {
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{
                        InvalidTransitionTemplateInput{
                            input.source_face_id}});
            }
        }

        SourceTransitionResult result;
        result.source_face_id = input.source_face_id;
        std::optional<std::size_t> double_high_common;
        if (input.second_high_edge_local_index.has_value())
        {
            const std::size_t first = *input.high_edge_local_index;
            const std::size_t second =
                *input.second_high_edge_local_index;
            if ((first + 1) % 4 == second)
                double_high_common = second;
            else if ((second + 1) % 4 == first)
                double_high_common = first;
            else
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{
                        InvalidTransitionTemplateInput{
                            input.source_face_id}});
        }
        std::optional<std::size_t> three_continuing_common;
        if (input.third_continuing_vertex_local_index.has_value())
        {
            const std::size_t edge = *input.high_edge_local_index;
            const std::size_t third =
                *input.third_continuing_vertex_local_index;
            if (third == (edge + 2) % 4)
                three_continuing_common = (edge + 1) % 4;
            else if (third == (edge + 3) % 4)
                three_continuing_common = edge;
            else
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{
                        InvalidTransitionTemplateInput{
                            input.source_face_id}});
        }
        const std::optional<std::size_t> four_cell_common =
            double_high_common.has_value()
                ? double_high_common
                : three_continuing_common;
        if (input.trial_layers < 2)
        {
            if (input.trial_layers == 0 &&
                input.high_edge_local_index.has_value())
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{
                        InvalidTransitionTemplateInput{
                            input.source_face_id}});
            if (four_cell_common.has_value())
            {
                const auto appended = appendDoubleSideTransition(
                    result, input, *four_cell_common, 0);
                if (!appended.hasValue())
                    return TransitionTemplateResult::failure(
                        TransitionTemplateError{appended.error()});
                return TransitionTemplateResult::success(
                    std::move(result));
            }
            const auto diagonal = selectLayerQuad(
                input.layer_vertex_ids[0],
                points,
                input.length_tolerance);
            if (!diagonal.hasValue())
            {
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{diagonal.error()});
            }
            if (input.continuing_edge_local_index.has_value())
            {
                const auto appended = appendContinuingEdgeTransition(
                    result, input, diagonal.value().diagonal, 0);
                if (!appended.hasValue())
                    return TransitionTemplateResult::failure(
                        TransitionTemplateError{appended.error()});
            }
            else if (input.high_edge_local_index.has_value())
            {
                appendSideTransition(
                    result, input, diagonal.value().diagonal, 0);
            }
            else
            {
                result.top_faces.assign(
                    diagonal.value().triangles.begin(),
                    diagonal.value().triangles.end());
            }
            return TransitionTemplateResult::success(std::move(result));
        }

        const std::uint32_t regular =
            regularLayerCount(input.trial_layers);
        for (std::uint32_t layer = 1; layer <= regular; ++layer)
        {
            const auto &bottom = input.layer_vertex_ids[layer - 1];
            const auto &top = input.layer_vertex_ids[layer];
            result.volume_cells.push_back(Hexa{{
                bottom[0], bottom[1], bottom[2], bottom[3],
                top[0], top[1], top[2], top[3]}});
            result.metadata.push_back(CellMetadata{
                CellRole::RegularLayer,
                input.source_face_id,
                layer});
        }

        const auto &bottom = input.layer_vertex_ids[regular];
        const auto &top = input.layer_vertex_ids[regular + 1];
        Point3 center = Point3::Zero();
        for (const VertexId id : bottom) center += points[id];
        for (const VertexId id : top) center += points[id];
        center /= Scalar{8};
        result.created_vertices.push_back(center);
        const VertexId c = input.center_vertex_id;

        const std::array<Pyramid, 5> pyramids{
            Pyramid{{bottom[0], bottom[1], bottom[2], bottom[3], c}},
            Pyramid{{bottom[0], top[0], top[1], bottom[1], c}},
            Pyramid{{bottom[1], top[1], top[2], bottom[2], c}},
            Pyramid{{bottom[2], top[2], top[3], bottom[3], c}},
            Pyramid{{bottom[3], top[3], top[0], bottom[0], c}}};
        for (const Pyramid &pyramid : pyramids)
        {
            result.volume_cells.push_back(pyramid);
            result.metadata.push_back(CellMetadata{
                CellRole::ReservedLayerTransition,
                input.source_face_id,
                regular + 1});
        }

        QuadDiagonal selected_diagonal{};
        if (four_cell_common.has_value())
        {
            selected_diagonal = *four_cell_common % 2 == 0
                ? QuadDiagonal::ZeroTwo
                : QuadDiagonal::OneThree;
        }
        else
        {
            const auto diagonal = selectLayerQuad(
                top, points, input.length_tolerance);
            if (!diagonal.hasValue())
            {
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{diagonal.error()});
            }
            selected_diagonal = diagonal.value().diagonal;
        }
        if (selected_diagonal == QuadDiagonal::ZeroTwo)
        {
            result.volume_cells.push_back(
                Tetra{{top[0], top[1], top[2], c}});
            result.volume_cells.push_back(
                Tetra{{top[0], top[2], top[3], c}});
        }
        else
        {
            result.volume_cells.push_back(
                Tetra{{top[1], top[2], top[3], c}});
            result.volume_cells.push_back(
                Tetra{{top[1], top[3], top[0], c}});
        }
        result.metadata.push_back(CellMetadata{
            CellRole::ReservedLayerTransition,
            input.source_face_id,
            regular + 1});
        result.metadata.push_back(CellMetadata{
            CellRole::ReservedLayerTransition,
            input.source_face_id,
            regular + 1});
        if (input.continuing_edge_local_index.has_value())
        {
            const auto appended = appendContinuingEdgeTransition(
                result,
                input,
                selected_diagonal,
                regular + 1);
            if (!appended.hasValue())
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{appended.error()});
        }
        else if (four_cell_common.has_value())
        {
            const auto appended = appendDoubleSideTransition(
                result,
                input,
                *four_cell_common,
                regular + 1);
            if (!appended.hasValue())
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{appended.error()});
        }
        else if (input.high_edge_local_index.has_value())
        {
            appendSideTransition(
                result,
                input,
                selected_diagonal,
                regular + 1);
        }
        else
        {
            if (selected_diagonal == QuadDiagonal::ZeroTwo)
                result.top_faces = {
                    Triangle{{top[0], top[1], top[2]}},
                    Triangle{{top[0], top[2], top[3]}}};
            else
                result.top_faces = {
                    Triangle{{top[1], top[2], top[3]}},
                    Triangle{{top[1], top[3], top[0]}}};
        }
        return TransitionTemplateResult::success(std::move(result));
    }
}
