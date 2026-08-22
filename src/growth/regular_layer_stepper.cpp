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
#include <boundary_mesh/growth/growth_field_smoother.hpp>
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
            if (front.faces.size() != front.source_face_ids.size())
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
        const FaceLayerConstraintTable &constraints,
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
            const SurfaceFaceId source_face_id =
                current_front.source_face_ids[face_index];
            const FaceLayerConstraint *constraint =
                constraints.find(source_face_id);
            if (constraint == nullptr)
            {
                return StepResult::failure(
                    InvalidFaceConstraintState{
                        source_face_id,
                        target_layer});
            }
            if (current_front.layer >= constraint->allowed_layer_count)
            {
                if (constraint->limit_kind ==
                    FaceLayerLimitKind::DirectStop)
                {
                    return StepResult::failure(
                        InvalidFaceConstraintState{
                            source_face_id,
                            target_layer});
                }

                FaceStopEvent event{
                    face_index,
                    source_face_id,
                    target_layer,
                    constraint->limit_kind ==
                            FaceLayerLimitKind::Requested
                        ? FaceStopReason::VertexLayerLimit
                        : FaceStopReason::NeighborLayerConstraint};
                if (constraint->limit_kind ==
                    FaceLayerLimitKind::Requested)
                {
                    output.completed_faces.push_back(event);
                }
                else
                {
                    output.stopped_faces.push_back(event);
                }
                continue;
            }

            bool eligible = true;
            for (const VertexId local_id :
                 faceVertexIds(current_front.faces[face_index]))
            {
                const std::size_t local_index =
                    static_cast<std::size_t>(local_id);
                const VertexGrowthProfile *profile = profiles.find(
                    current_front.vertices[local_index].source_vertex_id);
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
                        source_face_id,
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

        const auto adjacency_result = buildFrontAdjacency(eligible.front);
        if (!adjacency_result.hasValue())
        {
            return StepResult::failure(
                InvalidLayerFrontMapping{target_layer});
        }

        const auto direction_result = computeGrowthDirections(
            eligible.front,
            front_evaluation.value(),
            adjacency_result.value());
        if (!direction_result.hasValue())
        {
            return StepResult::failure(
                GrowthDirectionFailure{
                    target_layer,
                    direction_result.error()});
        }

        std::vector<Scalar> base_heights;
        base_heights.reserve(eligible.front.vertices.size());
        for (const GrowthFrontVertex &vertex : eligible.front.vertices)
        {
            const VertexGrowthProfile *profile =
                profiles.find(vertex.source_vertex_id);
            if (profile == nullptr)
            {
                return StepResult::failure(
                    NonFiniteLayerHeight{
                        vertex.source_vertex_id,
                        target_layer});
            }

            const Scalar base_height = target_layer == 1
                ? profile->first_height
                : vertex.actual_height * profile->growth_ratio;
            if (!std::isfinite(base_height) ||
                base_height <= Scalar{0})
            {
                return StepResult::failure(
                    NonFiniteLayerHeight{
                        vertex.source_vertex_id,
                        target_layer});
            }
            base_heights.push_back(base_height);
        }

        const auto field_result = GrowthFieldSmoother{}.smooth(
            eligible.front,
            front_evaluation.value(),
            adjacency_result.value(),
            direction_result.value(),
            base_heights);
        if (!field_result.hasValue())
        {
            return StepResult::failure(
                GrowthFieldSmoothingFailure{
                    target_layer,
                    field_result.error()});
        }

        GrowthFront candidate_front = eligible.front;
        candidate_front.layer = target_layer;
        for (std::size_t vertex_index = 0;
             vertex_index < candidate_front.vertices.size();
             ++vertex_index)
        {
            GrowthFrontVertex &candidate_vertex =
                candidate_front.vertices[vertex_index];
            const GrowthDirectionSelection &raw_direction =
                direction_result.value().vertices[vertex_index];
            candidate_vertex.direction =
                field_result.value().directions[vertex_index];
            candidate_vertex.actual_height =
                field_result.value().actual_heights[vertex_index];
            candidate_vertex.visibility_cosine =
                raw_direction.visibility_cosine;
            candidate_vertex.complex_corner =
                raw_direction.complex_corner;
            candidate_vertex.position +=
                candidate_vertex.actual_height *
                candidate_vertex.direction;
            if (!candidate_front.vertices[vertex_index].position.allFinite())
            {
                return StepResult::failure(
                    NonFiniteLayerHeight{
                        candidate_vertex.source_vertex_id,
                        target_layer});
            }
        }

        std::vector<std::size_t> accepted_eligible_faces;
        for (std::size_t eligible_face_index = 0;
             eligible_face_index < eligible.front.faces.size();
             ++eligible_face_index)
        {
            const auto quality = std::visit(
                [&](const auto &face)
                    -> Result<VolumeCellEvaluation,
                              VolumeCellEvaluationError>
                {
                    using Face = std::decay_t<decltype(face)>;
                    if constexpr (std::is_same_v<Face, Triangle>)
                    {
                        PrismPoints points;
                        for (std::size_t local = 0; local < 3; ++local)
                        {
                            const std::size_t vertex_index =
                                static_cast<std::size_t>(
                                    face.vertex_ids[local]);
                            points[local] =
                                eligible.front.vertices[vertex_index].position;
                            points[local + 3] =
                                candidate_front.vertices[vertex_index].position;
                        }
                        return evaluatePrism(points, options.cell_quality);
                    }
                    else
                    {
                        HexaPoints points;
                        for (std::size_t local = 0; local < 4; ++local)
                        {
                            const std::size_t vertex_index =
                                static_cast<std::size_t>(
                                    face.vertex_ids[local]);
                            points[local] =
                                eligible.front.vertices[vertex_index].position;
                            points[local + 4] =
                                candidate_front.vertices[vertex_index].position;
                        }
                        return evaluateHexa(points, options.cell_quality);
                    }
                },
                eligible.front.faces[eligible_face_index]);
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
