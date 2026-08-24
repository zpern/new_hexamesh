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
    }

    TransitionTemplateResult buildQuadTransition(
        const QuadTransitionInput &input)
    {
        if (input.mesh_vertices == nullptr ||
            input.layer_vertex_ids.size() !=
                static_cast<std::size_t>(input.trial_layers) + 1 ||
            (input.high_edge_local_index.has_value() &&
             *input.high_edge_local_index >= 4))
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
        if (input.trial_layers < 2)
        {
            const auto diagonal = selectLayerQuad(
                input.layer_vertex_ids[0],
                points,
                input.length_tolerance);
            if (!diagonal.hasValue())
            {
                return TransitionTemplateResult::failure(
                    TransitionTemplateError{diagonal.error()});
            }
            result.top_faces.assign(
                diagonal.value().triangles.begin(),
                diagonal.value().triangles.end());
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

        const auto diagonal = selectLayerQuad(
            top, points, input.length_tolerance);
        if (!diagonal.hasValue())
        {
            return TransitionTemplateResult::failure(
                TransitionTemplateError{diagonal.error()});
        }
        if (diagonal.value().diagonal == QuadDiagonal::ZeroTwo)
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
        result.top_faces.assign(
            diagonal.value().triangles.begin(),
            diagonal.value().triangles.end());
        return TransitionTemplateResult::success(std::move(result));
    }
}
