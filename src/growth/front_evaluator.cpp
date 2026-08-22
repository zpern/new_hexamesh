#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <variant>

#include <boundary_mesh/growth/front_evaluator.hpp>

namespace boundary_mesh
{
    Result<FrontEvaluation, FrontEvaluationError>
    FrontEvaluator::evaluate(
        const GrowthFront &front,
        const SurfaceEvaluationOptions &options) const
    {
        using EvaluationResult = Result<FrontEvaluation, FrontEvaluationError>;

        if (!std::isfinite(options.relative_length_tolerance) ||
            options.relative_length_tolerance <= Scalar{0})
        {
            return EvaluationResult::failure(InvalidSurfaceEvaluationOptions{
                options.relative_length_tolerance});
        }
        if (front.vertices.empty() || front.faces.empty())
        {
            return EvaluationResult::failure(EmptyGrowthFront{front.layer});
        }
        if (front.faces.size() != front.source_face_ids.size())
        {
            return EvaluationResult::failure(FrontMappingMismatch{
                front.vertices.size(), front.vertices.size(),
                front.vertices.size(), front.faces.size(),
                front.source_face_ids.size(), front.layer});
        }

        for (std::size_t index = 0; index < front.vertices.size(); ++index)
        {
            if (!front.vertices[index].position.allFinite())
            {
                return EvaluationResult::failure(NonFiniteFrontVertex{
                    index, front.vertices[index].source_vertex_id,
                    front.layer});
            }
        }

        Point3 minimum = front.vertices.front().position;
        Point3 maximum = front.vertices.front().position;
        for (const GrowthFrontVertex &vertex : front.vertices)
        {
            const Point3 &point = vertex.position;
            minimum = minimum.cwiseMin(point);
            maximum = maximum.cwiseMax(point);
        }
        const Scalar characteristic_length = (maximum - minimum).norm();
        if (!std::isfinite(characteristic_length) ||
            characteristic_length <= Scalar{0})
        {
            return EvaluationResult::failure(DegenerateFrontScale{front.layer});
        }

        const Scalar safety_floor =
            std::sqrt(std::numeric_limits<Scalar>::min());
        const Scalar effective_tolerance = std::max(
            characteristic_length * options.relative_length_tolerance,
            safety_floor);
        if (!std::isfinite(effective_tolerance))
        {
            return EvaluationResult::failure(InvalidSurfaceEvaluationOptions{
                options.relative_length_tolerance});
        }

        FrontEvaluation output;
        output.layer = front.layer;
        output.characteristic_length = characteristic_length;
        output.effective_length_tolerance = effective_tolerance;
        output.faces.reserve(front.faces.size());

        for (std::size_t face_index = 0;
             face_index < front.faces.size();
             ++face_index)
        {
            VertexId invalid_id{};
            bool invalid = false;
            std::visit([&](const auto &face)
            {
                for (const VertexId vertex_id : face.vertex_ids)
                {
                    if (static_cast<std::size_t>(vertex_id) >= front.vertices.size())
                    {
                        invalid = true;
                        invalid_id = vertex_id;
                        break;
                    }
                }
            }, front.faces[face_index]);
            if (invalid)
            {
                return EvaluationResult::failure(InvalidFrontVertexReference{
                    face_index, front.source_face_ids[face_index],
                    invalid_id, front.layer});
            }

            const auto local_result = std::visit(
                [&](const auto &face)
                    -> Result<FaceEvaluation, FaceEvaluationError>
                {
                    using Face = std::decay_t<decltype(face)>;
                    if constexpr (std::is_same_v<Face, Triangle>)
                    {
                        return evaluateTriangle(
                            front.vertices[face.vertex_ids[0]].position,
                            front.vertices[face.vertex_ids[1]].position,
                            front.vertices[face.vertex_ids[2]].position,
                            effective_tolerance);
                    }
                    else
                    {
                        return evaluateQuad(
                            front.vertices[face.vertex_ids[0]].position,
                            front.vertices[face.vertex_ids[1]].position,
                            front.vertices[face.vertex_ids[2]].position,
                            front.vertices[face.vertex_ids[3]].position,
                            effective_tolerance);
                    }
                }, front.faces[face_index]);

            if (!local_result.hasValue())
            {
                return EvaluationResult::failure(DegenerateFrontFace{
                    face_index, front.source_face_ids[face_index],
                    front.layer, local_result.error()});
            }
            output.faces.push_back(FrontFaceEvaluation{
                face_index, front.source_face_ids[face_index],
                local_result.value()});
        }
        return EvaluationResult::success(std::move(output));
    }
}
