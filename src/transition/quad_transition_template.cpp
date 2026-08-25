#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

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
                CellRole::Transition,
                input.source_face_id,
                occupied + 1});
            result.metadata.push_back(CellMetadata{
                CellRole::Transition,
                input.source_face_id,
                occupied + 1});
        }

        void appendDoubleSideTransition(
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

            const std::array<VolumeCell, 4> cells{
                Pyramid{{a, b, f, e, d}},
                Pyramid{{a, e, g, c, d}},
                Tetra{{e, g, h, d}},
                Tetra{{e, f, h, d}}};
            for (const VolumeCell &cell : cells)
            {
                result.side_cells.push_back(cell);
                result.volume_cells.push_back(cell);
                result.metadata.push_back(CellMetadata{
                    CellRole::Transition,
                    input.source_face_id,
                    occupied + 1});
            }
            result.top_faces = {
                Triangle{{b, f, d}},
                Triangle{{f, h, d}},
                Triangle{{e, f, h}},
                Triangle{{e, g, h}},
                Triangle{{g, h, d}},
                Triangle{{g, c, d}}};
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
              *input.second_high_edge_local_index >= 4)))
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
        if (input.trial_layers < 2)
        {
            if (input.trial_layers == 0 &&
                input.high_edge_local_index.has_value())
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{
                        InvalidTransitionTemplateInput{
                            input.source_face_id}});
            if (double_high_common.has_value())
            {
                appendDoubleSideTransition(
                    result, input, *double_high_common, 0);
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
            if (input.high_edge_local_index.has_value())
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
                CellRole::Transition,
                input.source_face_id,
                regular + 1});
        }

        QuadDiagonal selected_diagonal{};
        if (double_high_common.has_value())
        {
            selected_diagonal = *double_high_common % 2 == 0
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
            CellRole::Transition, input.source_face_id, regular + 1});
        result.metadata.push_back(CellMetadata{
            CellRole::Transition, input.source_face_id, regular + 1});
        if (double_high_common.has_value())
        {
            appendDoubleSideTransition(
                result,
                input,
                *double_high_common,
                regular + 1);
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
