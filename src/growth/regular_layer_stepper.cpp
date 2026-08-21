#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/front_evaluator.hpp>
#include <boundary_mesh/growth/growth_direction.hpp>
#include <boundary_mesh/growth/regular_layer_stepper.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct CompactFront
        {
            GrowthFront front;
            std::vector<std::size_t> previous_vertex_indices;
            std::vector<std::size_t> previous_face_indices;
        };

        std::vector<VertexId> faceVertexIds(const SurfaceFace &face)
        {
            return std::visit(
                [](const auto &value)
                {
                    return std::vector<VertexId>{
                        value.vertex_ids.begin(),
                        value.vertex_ids.end()};
                },
                face);
        }

        bool validFrontShape(const GrowthFront &front)
        {
            if (front.vertices.size() != front.source_vertex_ids.size() ||
                front.vertices.size() != front.vertex_boundaries.size() ||
                front.faces.size() != front.source_face_ids.size())
            {
                return false;
            }
            for (const SurfaceFace &face : front.faces)
            {
                for (const VertexId vertex_id : faceVertexIds(face))
                {
                    if (static_cast<std::size_t>(vertex_id) >=
                        front.vertices.size())
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        SurfaceFace remapFace(
            const SurfaceFace &face,
            const std::vector<VertexId> &old_to_new)
        {
            return std::visit(
                [&](const auto &value) -> SurfaceFace
                {
                    using Face = std::decay_t<decltype(value)>;
                    Face remapped = value;
                    for (VertexId &vertex_id : remapped.vertex_ids)
                    {
                        vertex_id = old_to_new[static_cast<std::size_t>(
                            vertex_id)];
                    }
                    return remapped;
                },
                face);
        }

        CompactFront compactFaces(
            const GrowthFront &source,
            const std::vector<std::size_t> &face_indices,
            std::uint32_t output_layer)
        {
            const VertexId invalid_id =
                std::numeric_limits<VertexId>::max();
            std::vector<bool> used(source.vertices.size(), false);
            for (const std::size_t face_index : face_indices)
            {
                for (const VertexId vertex_id :
                     faceVertexIds(source.faces[face_index]))
                {
                    used[static_cast<std::size_t>(vertex_id)] = true;
                }
            }

            CompactFront result;
            result.front.layer = output_layer;
            std::vector<VertexId> old_to_new(
                source.vertices.size(), invalid_id);
            for (std::size_t old_index = 0;
                 old_index < source.vertices.size();
                 ++old_index)
            {
                if (!used[old_index]) continue;
                const auto new_id = static_cast<VertexId>(
                    result.front.vertices.size());
                old_to_new[old_index] = new_id;
                result.front.vertices.push_back(source.vertices[old_index]);
                result.front.source_vertex_ids.push_back(
                    source.source_vertex_ids[old_index]);
                result.front.vertex_boundaries.push_back(
                    source.vertex_boundaries[old_index]);
                result.previous_vertex_indices.push_back(old_index);
            }

            for (const std::size_t face_index : face_indices)
            {
                result.front.faces.push_back(
                    remapFace(source.faces[face_index], old_to_new));
                result.front.source_face_ids.push_back(
                    source.source_face_ids[face_index]);
                result.previous_face_indices.push_back(face_index);
            }
            return result;
        }

        FaceStopReason rejectionReason(
            const VolumeCellEvaluation &evaluation)
        {
            switch (evaluation.validity)
            {
            case VolumeCellValidity::Degenerate:
                return FaceStopReason::DegenerateCandidate;
            case VolumeCellValidity::Reversed:
                return FaceStopReason::ReversedCandidate;
            case VolumeCellValidity::LocallyInverted:
                return FaceStopReason::LocallyInvertedCandidate;
            case VolumeCellValidity::Valid:
                return FaceStopReason::SkewnessExceeded;
            }
            return FaceStopReason::DegenerateCandidate;
        }
    }

    Result<LayerStepResult, RegularLayerGrowthError>
    RegularLayerStepper::step(
        const GrowthFront &current_front,
        const GrowthProfileTable &profiles,
        const RegularLayerGrowthOptions &options) const
    {
        using StepResult =
            Result<LayerStepResult, RegularLayerGrowthError>;

        if (!validFrontShape(current_front) ||
            current_front.layer ==
                std::numeric_limits<std::uint32_t>::max())
        {
            return StepResult::failure(
                InvalidLayerFrontMapping{current_front.layer});
        }

        const std::uint32_t target_layer = current_front.layer + 1;
        LayerStepResult output;
        output.layer = target_layer;
        output.next_front.layer = target_layer;

        std::vector<std::size_t> eligible_face_indices;
        for (std::size_t face_index = 0;
             face_index < current_front.faces.size();
             ++face_index)
        {
            bool eligible = true;
            for (const VertexId local_id :
                 faceVertexIds(current_front.faces[face_index]))
            {
                const std::size_t local_index =
                    static_cast<std::size_t>(local_id);
                const VertexGrowthProfile *profile = profiles.find(
                    current_front.source_vertex_ids[local_index]);
                if (profile == nullptr ||
                    current_front.layer >= profile->layer_count)
                {
                    eligible = false;
                    break;
                }
            }

            if (eligible)
            {
                eligible_face_indices.push_back(face_index);
            }
            else
            {
                output.completed_faces.push_back(
                    FaceStopEvent{
                        face_index,
                        current_front.source_face_ids[face_index],
                        target_layer,
                        FaceStopReason::VertexLayerLimit});
            }
        }

        if (eligible_face_indices.empty())
        {
            return StepResult::success(std::move(output));
        }

        CompactFront eligible = compactFaces(
            current_front, eligible_face_indices, current_front.layer);
        const auto front_evaluation = FrontEvaluator{}.evaluate(
            eligible.front);
        if (!front_evaluation.hasValue())
        {
            return StepResult::failure(
                FrontEvaluationFailure{
                    target_layer,
                    front_evaluation.error()});
        }

        const auto direction_result = computeGrowthDirections(
            eligible.front, front_evaluation.value());
        if (!direction_result.hasValue())
        {
            return StepResult::failure(
                GrowthDirectionFailure{
                    target_layer,
                    direction_result.error()});
        }

        GrowthFront candidate_front = eligible.front;
        candidate_front.layer = target_layer;
        for (std::size_t vertex_index = 0;
             vertex_index < candidate_front.vertices.size();
             ++vertex_index)
        {
            const VertexId source_id =
                candidate_front.source_vertex_ids[vertex_index];
            const auto height_result = profiles.height(
                source_id, target_layer);
            if (!height_result.hasValue())
            {
                return StepResult::failure(height_result.error());
            }
            candidate_front.vertices[vertex_index] +=
                height_result.value() *
                direction_result.value().values[vertex_index];
            if (!candidate_front.vertices[vertex_index].allFinite())
            {
                return StepResult::failure(
                    NonFiniteLayerHeight{source_id, target_layer});
            }
        }

        std::vector<std::size_t> accepted_eligible_faces;
        for (std::size_t eligible_face_index = 0;
             eligible_face_index < eligible.front.faces.size();
             ++eligible_face_index)
        {
            const auto *triangle = std::get_if<Triangle>(
                &eligible.front.faces[eligible_face_index]);
            if (triangle == nullptr)
            {
                return StepResult::failure(
                    InvalidLayerFrontMapping{current_front.layer});
            }

            PrismPoints points;
            for (std::size_t local = 0; local < 3; ++local)
            {
                const std::size_t vertex_index =
                    static_cast<std::size_t>(
                        triangle->vertex_ids[local]);
                points[local] = eligible.front.vertices[vertex_index];
                points[local + 3] =
                    candidate_front.vertices[vertex_index];
            }
            const auto quality = evaluatePrism(
                points, options.cell_quality);
            const std::size_t previous_face_index =
                eligible.previous_face_indices[eligible_face_index];
            if (!quality.hasValue())
            {
                return StepResult::failure(
                    CellEvaluationFailure{
                        current_front.source_face_ids[previous_face_index],
                        target_layer,
                        quality.error()});
            }
            if (quality.value().acceptable)
            {
                accepted_eligible_faces.push_back(eligible_face_index);
            }
            else
            {
                output.stopped_faces.push_back(
                    FaceStopEvent{
                        previous_face_index,
                        current_front.source_face_ids[previous_face_index],
                        target_layer,
                        rejectionReason(quality.value())});
            }
        }

        CompactFront accepted = compactFaces(
            candidate_front, accepted_eligible_faces, target_layer);
        output.next_front = std::move(accepted.front);
        output.previous_front_vertex_indices.reserve(
            accepted.previous_vertex_indices.size());
        for (const std::size_t eligible_vertex_index :
             accepted.previous_vertex_indices)
        {
            output.previous_front_vertex_indices.push_back(
                eligible.previous_vertex_indices[eligible_vertex_index]);
        }
        output.previous_front_face_indices.reserve(
            accepted.previous_face_indices.size());
        for (const std::size_t eligible_face_index :
             accepted.previous_face_indices)
        {
            output.previous_front_face_indices.push_back(
                eligible.previous_face_indices[eligible_face_index]);
        }

        return StepResult::success(std::move(output));
    }
}
