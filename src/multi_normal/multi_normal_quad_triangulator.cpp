#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/multi_normal/multi_normal_quad_triangulator.hpp>
#include <boundary_mesh/surface/face_skewness.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar score_tolerance = Scalar{1e-12};
        constexpr Scalar length_tolerance = Scalar{1e-12};

        std::optional<Scalar> pairScore(
            const GrowthFront &front,
            const std::array<Triangle, 2> &triangles)
        {
            Scalar score{};
            for (const Triangle &triangle : triangles)
            {
                std::array<Point3, 3> points{};
                for (std::size_t corner = 0; corner < 3; ++corner)
                {
                    const std::size_t index = static_cast<std::size_t>(
                        triangle.vertex_ids[corner]);
                    if (index >= front.vertices.size()) return std::nullopt;
                    points[corner] = front.vertices[index].position;
                }
                const auto skewness = triangleEquiangularSkewness(
                    points, length_tolerance);
                if (!skewness.hasValue()) return std::nullopt;
                score = std::max(score, skewness.value());
            }
            return score;
        }

        void appendOrigins(
            const MultiNormalTopology &input,
            std::size_t input_face_index,
            std::size_t output_face_index,
            std::vector<TransitionFaceOrigin> &output)
        {
            for (const TransitionFaceOrigin &origin :
                 input.transition_face_origins)
            {
                if (origin.transformed_face_index != input_face_index)
                {
                    continue;
                }
                TransitionFaceOrigin copy = origin;
                copy.transformed_face_index = output_face_index;
                output.push_back(std::move(copy));
            }
        }
    }

    Result<MultiNormalTopology, MultiNormalError>
    triangulateMultiNormalQuads(
        const MultiNormalTopology &topology)
    {
        using TriangulationResult =
            Result<MultiNormalTopology, MultiNormalError>;
        if (topology.front.faces.size() !=
            topology.front.source_face_ids.size())
        {
            return TriangulationResult::failure(
                MultiNormalInputMismatch{
                    topology.front.vertices.size(),
                    topology.front.faces.size()});
        }

        MultiNormalTopology output;
        output.front = topology.front;
        output.front.faces.clear();
        output.front.source_face_ids.clear();
        output.vertex_mapping = topology.vertex_mapping;

        const auto appendFace = [&](const SurfaceFace &face,
                                    SurfaceFaceId source_face_id,
                                    std::size_t input_face_index)
        {
            const std::size_t output_index = output.front.faces.size();
            output.front.faces.push_back(face);
            output.front.source_face_ids.push_back(source_face_id);
            appendOrigins(
                topology, input_face_index, output_index,
                output.transition_face_origins);
        };

        for (std::size_t face_index = 0;
             face_index < topology.front.faces.size();
             ++face_index)
        {
            const SurfaceFace &surface_face = topology.front.faces[face_index];
            const Quad *quad = std::get_if<Quad>(&surface_face);
            bool affected = false;
            if (quad != nullptr)
            {
                for (const VertexId id : quad->vertex_ids)
                {
                    const std::size_t index = static_cast<std::size_t>(id);
                    if (index >= topology.front.vertices.size())
                    {
                        return TriangulationResult::failure(
                            InvalidMultiNormalTopology{});
                    }
                    affected = affected ||
                        topology.front.vertices[index].multi_normal_branch;
                }
            }
            if (quad == nullptr || !affected)
            {
                appendFace(
                    surface_face,
                    topology.front.source_face_ids[face_index],
                    face_index);
                continue;
            }

            const auto &v = quad->vertex_ids;
            const std::array<Triangle, 2> diagonal_02{
                Triangle{{v[0], v[1], v[2]}},
                Triangle{{v[0], v[2], v[3]}}};
            const std::array<Triangle, 2> diagonal_13{
                Triangle{{v[0], v[1], v[3]}},
                Triangle{{v[1], v[2], v[3]}}};
            const auto score_02 = pairScore(topology.front, diagonal_02);
            const auto score_13 = pairScore(topology.front, diagonal_13);
            if (!score_02.has_value() && !score_13.has_value())
            {
                return TriangulationResult::failure(
                    InvalidMultiNormalTopology{});
            }

            bool choose_02 = score_02.has_value() && !score_13.has_value();
            if (score_02.has_value() && score_13.has_value())
            {
                const Scalar difference = *score_02 - *score_13;
                if (std::abs(difference) <= score_tolerance)
                {
                    const auto minimum = std::min_element(
                        v.begin(), v.end()) - v.begin();
                    choose_02 = minimum == 0 || minimum == 2;
                }
                else
                {
                    choose_02 = difference < Scalar{0};
                }
            }
            const auto &selected = choose_02 ? diagonal_02 : diagonal_13;
            for (const Triangle &triangle : selected)
            {
                appendFace(
                    triangle,
                    topology.front.source_face_ids[face_index],
                    face_index);
            }
        }

        return TriangulationResult::success(std::move(output));
    }
}
