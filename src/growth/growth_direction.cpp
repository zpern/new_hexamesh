#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/LU>

#include <boundary_mesh/growth/growth_direction.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar pi = 3.14159265358979323846;
        constexpr Scalar cosine30 = 0.86602540378443864676;
        constexpr Scalar cosine10 = 0.98480775301220802032;
        constexpr Scalar cosine1 = 0.99984769515639123916;
        constexpr Scalar cosine25 = 0.90630778703664996324;

        std::optional<Vector3> normalized(const Vector3 &value)
        {
            const Scalar length = value.norm();
            if (!value.allFinite() ||
                !std::isfinite(length) ||
                length <= std::numeric_limits<Scalar>::epsilon())
            {
                return std::nullopt;
            }
            return value / length;
        }

        Scalar visibility(
            const Vector3 &candidate,
            const std::vector<std::size_t> &incident_faces,
            const FrontEvaluation &evaluation)
        {
            Scalar result = Scalar{1};
            for (const std::size_t face_index : incident_faces)
            {
                result = std::min(
                    result,
                    candidate.dot(
                        evaluation.faces[face_index].value.unit_normal));
            }
            return result;
        }

        std::optional<Vector3> simpleNormal(
            const std::vector<std::size_t> &incident_faces,
            const FrontEvaluation &evaluation)
        {
            Vector3 sum = Vector3::Zero();
            for (const std::size_t face_index : incident_faces)
            {
                sum += evaluation.faces[face_index].value.unit_normal;
            }
            return normalized(sum);
        }

        std::optional<Vector3> naiveNormal(
            const std::vector<std::size_t> &incident_faces,
            const FrontEvaluation &evaluation)
        {
            std::vector<std::vector<Vector3>> groups;
            for (const std::size_t face_index : incident_faces)
            {
                const Vector3 normal =
                    evaluation.faces[face_index].value.unit_normal;
                bool assigned = false;
                for (auto &group : groups)
                {
                    if (group.front().dot(normal) > cosine25)
                    {
                        group.push_back(normal);
                        assigned = true;
                        break;
                    }
                }
                if (!assigned)
                {
                    groups.push_back({normal});
                }
            }

            Vector3 sum = Vector3::Zero();
            for (const auto &group : groups)
            {
                Vector3 group_sum = Vector3::Zero();
                for (const Vector3 &normal : group)
                {
                    group_sum += normal;
                }
                const auto group_normal = normalized(group_sum);
                if (group_normal.has_value())
                {
                    sum += *group_normal;
                }
            }

            const auto result = normalized(sum);
            if (result.has_value())
            {
                return result;
            }
            if (!groups.empty())
            {
                return normalized(groups.front().front());
            }
            return std::nullopt;
        }

        struct Intersection
        {
            Vector3 offset{Vector3::Zero()};
            Scalar parameter{};
        };

        std::optional<Intersection> lineAngleIntersection(
            const Vector3 &first_side,
            const Vector3 &second_side,
            const Vector3 &axis,
            const Vector3 &bisector)
        {
            const Vector3 reverse_axis = -axis;
            const Scalar denominator =
                first_side.dot(second_side.cross(reverse_axis));
            if (!std::isfinite(denominator) ||
                std::abs(denominator) < Scalar{1e-7})
            {
                return std::nullopt;
            }

            const Scalar first_coefficient =
                bisector.dot(second_side.cross(reverse_axis)) /
                denominator;
            const Scalar second_coefficient =
                first_side.dot(bisector.cross(reverse_axis)) /
                denominator;
            const Scalar parameter =
                first_side.dot(second_side.cross(bisector)) /
                denominator;

            if (first_coefficient * second_coefficient < Scalar{0} ||
                std::abs(parameter) >= Scalar{50})
            {
                return std::nullopt;
            }

            const Vector3 offset =
                first_coefficient * first_side +
                second_coefficient * second_side;
            if (!offset.allFinite() || !std::isfinite(parameter))
            {
                return std::nullopt;
            }
            return Intersection{offset, parameter};
        }

        std::optional<std::pair<Vector3, Vector3>> faceSides(
            const GrowthFront &front,
            std::size_t face_index,
            std::size_t vertex_index)
        {
            return std::visit(
                [&](const auto &face)
                    -> std::optional<std::pair<Vector3, Vector3>>
                {
                    const std::size_t count = face.vertex_ids.size();
                    for (std::size_t local = 0; local < count; ++local)
                    {
                        if (static_cast<std::size_t>(
                                face.vertex_ids[local]) != vertex_index)
                        {
                            continue;
                        }

                        const std::size_t previous =
                            static_cast<std::size_t>(
                                face.vertex_ids[
                                    (local + count - 1) % count]);
                        const std::size_t next =
                            static_cast<std::size_t>(
                                face.vertex_ids[(local + 1) % count]);
                        const Point3 &center =
                            front.vertices[vertex_index].position;
                        const auto first = normalized(
                            front.vertices[previous].position - center);
                        const auto second = normalized(
                            front.vertices[next].position - center);
                        if (!first.has_value() || !second.has_value())
                        {
                            return std::nullopt;
                        }
                        return std::make_pair(*first, *second);
                    }
                    return std::nullopt;
                },
                front.faces[face_index]);
        }

        std::optional<Vector3> centerNormal(
            const GrowthFront &front,
            std::size_t vertex_index,
            const std::vector<std::size_t> &incident_faces,
            const FrontEvaluation &evaluation,
            const std::optional<Vector3> &simple)
        {
            if (incident_faces.size() < 2)
            {
                return simple;
            }

            std::size_t first_pair = 0;
            std::size_t second_pair = 1;
            Scalar minimum_dot = Scalar{2};
            for (std::size_t first = 0;
                 first < incident_faces.size();
                 ++first)
            {
                for (std::size_t second = first + 1;
                     second < incident_faces.size();
                     ++second)
                {
                    const Scalar dot =
                        evaluation.faces[incident_faces[first]]
                            .value.unit_normal.dot(
                                evaluation.faces[incident_faces[second]]
                                    .value.unit_normal);
                    if (dot < minimum_dot)
                    {
                        minimum_dot = dot;
                        first_pair = first;
                        second_pair = second;
                    }
                }
            }

            const Vector3 first_normal =
                evaluation.faces[incident_faces[first_pair]]
                    .value.unit_normal;
            const Vector3 second_normal =
                evaluation.faces[incident_faces[second_pair]]
                    .value.unit_normal;
            const auto axis = normalized(
                first_normal.cross(second_normal));
            const auto bisector = normalized(
                first_normal + second_normal);
            if (!axis.has_value() || !bisector.has_value())
            {
                return simple;
            }

            Scalar maximum_parameter = -Scalar{1e10};
            Scalar minimum_parameter = Scalar{1e10};
            Vector3 maximum_offset = Vector3::Zero();
            Vector3 minimum_offset = Vector3::Zero();
            bool has_maximum = false;
            bool has_minimum = false;

            for (const std::size_t face_index : incident_faces)
            {
                const auto sides =
                    faceSides(front, face_index, vertex_index);
                if (!sides.has_value())
                {
                    continue;
                }
                const auto intersection = lineAngleIntersection(
                    sides->first,
                    sides->second,
                    *axis,
                    *bisector);
                if (!intersection.has_value())
                {
                    continue;
                }
                if (intersection->parameter > maximum_parameter &&
                    intersection->parameter > Scalar{-0.10})
                {
                    maximum_parameter = intersection->parameter;
                    maximum_offset = intersection->offset;
                    has_maximum = true;
                }
                if (intersection->parameter < minimum_parameter &&
                    intersection->parameter < Scalar{0.10})
                {
                    minimum_parameter = intersection->parameter;
                    minimum_offset = intersection->offset;
                    has_minimum = true;
                }
            }

            if (!has_maximum && !has_minimum)
            {
                return bisector;
            }
            if (has_maximum != has_minimum)
            {
                const Vector3 offset =
                    has_maximum ? maximum_offset : minimum_offset;
                const auto radial = normalized(offset);
                if (!radial.has_value())
                {
                    return bisector;
                }
                const auto first = normalized(*radial + *axis);
                const auto second = normalized(*radial - *axis);
                if (!first.has_value()) return second;
                if (!second.has_value()) return first;
                if (!simple.has_value())
                {
                    return first;
                }
                return first->dot(*simple) > second->dot(*simple)
                    ? first
                    : second;
            }

            const auto first = normalized(maximum_offset);
            const auto second = normalized(minimum_offset);
            if (!first.has_value() || !second.has_value())
            {
                return bisector;
            }
            return normalized(*first + *second);
        }

        std::optional<Vector3> circleCenterDirection(
            const Vector3 &first,
            const Vector3 &second,
            const Vector3 &third)
        {
            const Vector3 first_difference = second - first;
            const Vector3 second_difference = third - first;
            const Vector3 plane_normal =
                first_difference.cross(second_difference);
            if (!normalized(plane_normal).has_value())
            {
                return std::nullopt;
            }

            Eigen::Matrix3d system;
            system.row(0) = first_difference.transpose();
            system.row(1) = second_difference.transpose();
            system.row(2) = plane_normal.transpose();
            const Vector3 right_hand_side{
                (second.squaredNorm() - first.squaredNorm()) /
                    Scalar{2},
                (third.squaredNorm() - first.squaredNorm()) /
                    Scalar{2},
                plane_normal.dot(first)};
            const Eigen::FullPivLU<Eigen::Matrix3d> decomposition(system);
            if (!decomposition.isInvertible())
            {
                return std::nullopt;
            }
            return normalized(decomposition.solve(right_hand_side));
        }

        std::optional<Vector3> geometryNormal(
            const std::vector<std::size_t> &incident_faces,
            const FrontEvaluation &evaluation)
        {
            if (incident_faces.empty())
            {
                return std::nullopt;
            }
            if (incident_faces.size() == 1)
            {
                return normalized(
                    evaluation.faces[incident_faces.front()]
                        .value.unit_normal);
            }

            std::optional<Vector3> best;
            Scalar best_visibility = -std::numeric_limits<Scalar>::infinity();
            const auto consider =
                [&](const std::optional<Vector3> &candidate)
                {
                    if (!candidate.has_value())
                    {
                        return;
                    }
                    const Scalar current = visibility(
                        *candidate,
                        incident_faces,
                        evaluation);
                    if (std::isfinite(current) &&
                        current > best_visibility)
                    {
                        best_visibility = current;
                        best = candidate;
                    }
                };

            for (std::size_t first = 0;
                 first < incident_faces.size();
                 ++first)
            {
                for (std::size_t second = first + 1;
                     second < incident_faces.size();
                     ++second)
                {
                    for (std::size_t third = second + 1;
                         third < incident_faces.size();
                         ++third)
                    {
                        consider(circleCenterDirection(
                            evaluation.faces[incident_faces[first]]
                                .value.unit_normal,
                            evaluation.faces[incident_faces[second]]
                                .value.unit_normal,
                            evaluation.faces[incident_faces[third]]
                                .value.unit_normal));
                    }

                    consider(normalized(
                        evaluation.faces[incident_faces[first]]
                                .value.unit_normal +
                        evaluation.faces[incident_faces[second]]
                                .value.unit_normal));
                }
            }
            return best;
        }

        bool validInputs(
            const GrowthFront &front,
            const FrontEvaluation &evaluation,
            const FrontAdjacency &adjacency)
        {
            if (front.layer != evaluation.layer ||
                front.faces.size() != evaluation.faces.size() ||
                front.faces.size() != front.source_face_ids.size() ||
                adjacency.vertex_neighbors.size() != front.vertices.size() ||
                adjacency.vertex_incident_faces.size() !=
                    front.vertices.size())
            {
                return false;
            }
            for (std::size_t face_index = 0;
                 face_index < evaluation.faces.size();
                 ++face_index)
            {
                if (evaluation.faces[face_index].front_face_index !=
                        face_index ||
                    evaluation.faces[face_index].source_face_id !=
                        front.source_face_ids[face_index])
                {
                    return false;
                }
            }
            for (const auto &incident :
                 adjacency.vertex_incident_faces)
            {
                for (const std::size_t face_index : incident)
                {
                    if (face_index >= evaluation.faces.size())
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }

    Result<GrowthDirections, GrowthDirectionError>
    computeGrowthDirections(
        const GrowthFront &front,
        const FrontEvaluation &evaluation,
        const FrontAdjacency &adjacency)
    {
        using DirectionResult =
            Result<GrowthDirections, GrowthDirectionError>;

        if (!validInputs(front, evaluation, adjacency))
        {
            return DirectionResult::failure(DirectionInputMismatch{
                front.layer,
                evaluation.layer});
        }

        GrowthDirections output;
        output.layer = front.layer;
        output.vertices.reserve(front.vertices.size());

        for (std::size_t vertex_index = 0;
             vertex_index < front.vertices.size();
             ++vertex_index)
        {
            const auto &incident_faces =
                adjacency.vertex_incident_faces[vertex_index];
            if (incident_faces.empty())
            {
                return DirectionResult::failure(
                    UndefinedGrowthDirection{
                        vertex_index,
                        front.vertices[vertex_index].source_vertex_id,
                        front.layer});
            }

            std::optional<Vector3> best;
            Scalar best_visibility =
                -std::numeric_limits<Scalar>::infinity();

            const auto consider =
                [&](const std::optional<Vector3> &candidate,
                    Scalar acceptance_threshold) -> bool
                {
                    if (!candidate.has_value())
                    {
                        return false;
                    }
                    const Scalar current_visibility = visibility(
                        *candidate,
                        incident_faces,
                        evaluation);
                    if (!std::isfinite(current_visibility))
                    {
                        return false;
                    }
                    if (current_visibility > best_visibility)
                    {
                        best = candidate;
                        best_visibility = current_visibility;
                    }
                    return current_visibility > acceptance_threshold;
                };

            if (front.layer > 0)
            {
                const auto previous = normalized(
                    front.vertices[vertex_index].direction);
                if (consider(previous, cosine30))
                {
                    output.vertices.push_back(
                        GrowthDirectionSelection{
                            *previous,
                            visibility(
                                *previous,
                                incident_faces,
                                evaluation),
                            false});
                    continue;
                }
            }

            const auto simple =
                simpleNormal(incident_faces, evaluation);
            if (consider(simple, cosine30))
            {
                output.vertices.push_back(
                    GrowthDirectionSelection{
                        *simple,
                        visibility(*simple, incident_faces, evaluation),
                        false});
                continue;
            }

            const auto naive =
                naiveNormal(incident_faces, evaluation);
            if (consider(naive, cosine30))
            {
                output.vertices.push_back(
                    GrowthDirectionSelection{
                        *naive,
                        visibility(*naive, incident_faces, evaluation),
                        false});
                continue;
            }

            const auto center = centerNormal(
                front,
                vertex_index,
                incident_faces,
                evaluation,
                simple);
            if (consider(center, cosine10))
            {
                output.vertices.push_back(
                    GrowthDirectionSelection{
                        *center,
                        visibility(*center, incident_faces, evaluation),
                        false});
                continue;
            }

            const auto geometry =
                geometryNormal(incident_faces, evaluation);
            if (consider(geometry, cosine1))
            {
                output.vertices.push_back(
                    GrowthDirectionSelection{
                        *geometry,
                        visibility(*geometry, incident_faces, evaluation),
                        false});
                continue;
            }

            if (!best.has_value())
            {
                return DirectionResult::failure(
                    UndefinedGrowthDirection{
                        vertex_index,
                        front.vertices[vertex_index].source_vertex_id,
                        front.layer});
            }

            output.vertices.push_back(
                GrowthDirectionSelection{
                    *best,
                    best_visibility,
                    best_visibility < cosine30});
        }

        return DirectionResult::success(std::move(output));
    }
}
