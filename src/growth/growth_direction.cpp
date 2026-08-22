#include <cmath>
#include <cstddef>
#include <optional>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_direction.hpp>

namespace boundary_mesh
{
    Result<GrowthDirections, GrowthDirectionError>
    computeGrowthDirections(
        const GrowthFront &front,
        const FrontEvaluation &evaluation)
    {
        using DirectionResult = Result<GrowthDirections, GrowthDirectionError>;
        if (front.layer != evaluation.layer ||
            front.faces.size() != evaluation.faces.size() ||
            front.faces.size() != front.source_face_ids.size())
        {
            return DirectionResult::failure(DirectionInputMismatch{
                front.layer, evaluation.layer});
        }
        for (std::size_t index = 0; index < evaluation.faces.size(); ++index)
        {
            if (evaluation.faces[index].front_face_index != index ||
                evaluation.faces[index].source_face_id != front.source_face_ids[index])
            {
                return DirectionResult::failure(DirectionInputMismatch{
                    front.layer, evaluation.layer});
            }
        }

        std::vector<Vector3> accumulated(
            front.vertices.size(), Vector3::Zero());

        for (std::size_t face_index = 0;
             face_index < front.faces.size();
             ++face_index)
        {
            const auto failure = std::visit(
                [&](const auto &face) -> std::optional<DirectionCornerFailure>
                {
                    const std::size_t count = face.vertex_ids.size();
                    for (std::size_t local = 0; local < count; ++local)
                    {
                        const VertexId previous_id =
                            face.vertex_ids[(local + count - 1) % count];
                        const VertexId center_id = face.vertex_ids[local];
                        const VertexId next_id = face.vertex_ids[(local + 1) % count];
                        const std::size_t center_index =
                            static_cast<std::size_t>(center_id);
                        if (static_cast<std::size_t>(previous_id) >= front.vertices.size() ||
                            center_index >= front.vertices.size() ||
                            static_cast<std::size_t>(next_id) >= front.vertices.size())
                        {
                            return DirectionCornerFailure{
                                center_index,
                                center_index < front.vertices.size()
                                    ? front.vertices[center_index].source_vertex_id
                                    : VertexId{},
                                face_index, front.source_face_ids[face_index],
                                front.layer, FaceEvaluationError::DegenerateEdge};
                        }
                        const auto angle = cornerAngle(
                            front.vertices[previous_id].position,
                            front.vertices[center_id].position,
                            front.vertices[next_id].position,
                            evaluation.effective_length_tolerance);
                        if (!angle.hasValue())
                        {
                            return DirectionCornerFailure{
                                center_index,
                                front.vertices[center_index].source_vertex_id,
                                face_index, front.source_face_ids[face_index],
                                front.layer, angle.error()};
                        }
                        accumulated[center_index] +=
                            evaluation.faces[face_index].value.unit_normal * angle.value();
                    }
                    return std::nullopt;
                },
                front.faces[face_index]);

            if (failure.has_value())
            {
                return DirectionResult::failure(*failure);
            }
        }

        GrowthDirections result;
        result.layer = front.layer;
        result.source_vertex_ids.reserve(front.vertices.size());
        for (const GrowthFrontVertex &vertex : front.vertices)
        {
            result.source_vertex_ids.push_back(vertex.source_vertex_id);
        }
        result.values.reserve(accumulated.size());
        for (std::size_t index = 0; index < accumulated.size(); ++index)
        {
            const Scalar length = accumulated[index].norm();
            if (!std::isfinite(length) ||
                length <= evaluation.effective_length_tolerance)
            {
                return DirectionResult::failure(UndefinedGrowthDirection{
                    index, front.vertices[index].source_vertex_id,
                    front.layer});
            }
            result.values.push_back(accumulated[index] / length);
        }
        return DirectionResult::success(std::move(result));
    }
}
