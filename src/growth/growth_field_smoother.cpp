#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_field_smoother.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar pi = 3.14159265358979323846;
        constexpr Scalar minimum_visible_cosine =
            Scalar{35} * Scalar{0.8} * pi / Scalar{180};
        constexpr Scalar update_alignment = Scalar{0.9985};
        constexpr std::size_t maximum_rounds = 100;
        constexpr std::size_t full_rounds = 4;
        constexpr std::size_t bisection_steps = 40;
        constexpr std::size_t visibility_retries = 11;

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

        Scalar minimumIncidentEdge(
            const GrowthFront &front,
            const std::vector<std::size_t> &incident_faces)
        {
            Scalar minimum = std::numeric_limits<Scalar>::infinity();
            for (const std::size_t face_index : incident_faces)
            {
                std::visit(
                    [&](const auto &face)
                    {
                        const std::size_t count = face.vertex_ids.size();
                        for (std::size_t local = 0; local < count; ++local)
                        {
                            const std::size_t first =
                                static_cast<std::size_t>(
                                    face.vertex_ids[local]);
                            const std::size_t second =
                                static_cast<std::size_t>(
                                    face.vertex_ids[(local + 1) % count]);
                            minimum = std::min(
                                minimum,
                                (front.vertices[second].position -
                                 front.vertices[first].position).norm());
                        }
                    },
                    front.faces[face_index]);
            }
            return minimum;
        }

        bool compatibleInputs(
            const GrowthFront &front,
            const FrontEvaluation &evaluation,
            const FrontAdjacency &adjacency,
            const GrowthDirections &raw_directions,
            const std::vector<Scalar> &base_heights)
        {
            const std::size_t count = front.vertices.size();
            if (front.layer != evaluation.layer ||
                front.layer != raw_directions.layer ||
                front.faces.size() != evaluation.faces.size() ||
                front.faces.size() != front.source_face_ids.size() ||
                adjacency.vertex_neighbors.size() != count ||
                adjacency.vertex_incident_faces.size() != count ||
                raw_directions.vertices.size() != count ||
                base_heights.size() != count)
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
            return true;
        }

        Result<std::vector<Vector3>, GrowthFieldSmoothingError>
        validateAndInitialize(
            const GrowthFront &front,
            const FrontEvaluation &evaluation,
            const FrontAdjacency &adjacency,
            const GrowthDirections &raw_directions,
            const std::vector<Scalar> &base_heights)
        {
            using DirectionBufferResult =
                Result<std::vector<Vector3>, GrowthFieldSmoothingError>;
            std::vector<Vector3> directions;
            directions.reserve(front.vertices.size());

            for (std::size_t index = 0;
                 index < front.vertices.size();
                 ++index)
            {
                const GrowthFrontVertex &vertex = front.vertices[index];
                const GrowthDirectionSelection &selection =
                    raw_directions.vertices[index];
                if (!vertex.position.allFinite() ||
                    !vertex.root_position.allFinite() ||
                    !selection.value.allFinite() ||
                    !std::isfinite(selection.visibility_cosine))
                {
                    return DirectionBufferResult::failure(
                        NonFiniteGrowthFieldInput{
                            index,
                            vertex.source_vertex_id,
                            front.layer});
                }
                if (!std::isfinite(base_heights[index]) ||
                    base_heights[index] <= Scalar{0})
                {
                    return DirectionBufferResult::failure(
                        InvalidGrowthFieldBaseHeight{
                            index,
                            vertex.source_vertex_id,
                            base_heights[index],
                            front.layer});
                }
                if (adjacency.vertex_incident_faces[index].empty())
                {
                    return DirectionBufferResult::failure(
                        UndefinedSmoothedDirection{
                            index,
                            vertex.source_vertex_id,
                            front.layer});
                }
                for (const std::size_t face_index :
                     adjacency.vertex_incident_faces[index])
                {
                    if (face_index >= evaluation.faces.size())
                    {
                        return DirectionBufferResult::failure(
                            UndefinedSmoothedDirection{
                                index,
                                vertex.source_vertex_id,
                                front.layer});
                    }
                }
                for (const std::size_t neighbor :
                     adjacency.vertex_neighbors[index])
                {
                    if (neighbor >= front.vertices.size() ||
                        neighbor == index)
                    {
                        return DirectionBufferResult::failure(
                            DegenerateGrowthFieldNeighbor{
                                index,
                                neighbor,
                                vertex.source_vertex_id,
                                front.layer});
                    }
                    const Scalar squared_distance =
                        (front.vertices[neighbor].position -
                         vertex.position).squaredNorm();
                    if (!std::isfinite(squared_distance) ||
                        squared_distance <=
                            std::numeric_limits<Scalar>::epsilon())
                    {
                        return DirectionBufferResult::failure(
                            DegenerateGrowthFieldNeighbor{
                                index,
                                neighbor,
                                vertex.source_vertex_id,
                                front.layer});
                    }
                }

                const auto direction = normalized(selection.value);
                if (!direction.has_value())
                {
                    return DirectionBufferResult::failure(
                        UndefinedSmoothedDirection{
                            index,
                            vertex.source_vertex_id,
                            front.layer});
                }
                directions.push_back(*direction);
            }
            return DirectionBufferResult::success(
                std::move(directions));
        }

        std::optional<Vector3> smoothOne(
            std::size_t vertex_index,
            const GrowthFront &front,
            const FrontEvaluation &evaluation,
            const FrontAdjacency &adjacency,
            const GrowthDirections &raw_directions,
            const std::vector<Vector3> &current)
        {
            const auto &neighbors =
                adjacency.vertex_neighbors[vertex_index];
            if (neighbors.empty())
            {
                return current[vertex_index];
            }

            const Point3 &position =
                front.vertices[vertex_index].position;
            std::vector<Scalar> squared_distances;
            squared_distances.reserve(neighbors.size());
            Scalar squared_distance_sum = Scalar{0};
            for (const std::size_t neighbor : neighbors)
            {
                const Scalar squared_distance =
                    (front.vertices[neighbor].position -
                     position).squaredNorm();
                squared_distances.push_back(squared_distance);
                squared_distance_sum += squared_distance;
            }
            const Scalar average_squared_distance =
                squared_distance_sum /
                static_cast<Scalar>(neighbors.size());

            Vector3 neighbor_sum = Vector3::Zero();
            for (std::size_t local = 0;
                 local < neighbors.size();
                 ++local)
            {
                const std::size_t neighbor = neighbors[local];
                const Vector3 edge =
                    front.vertices[neighbor].position - position;
                const Vector3 unit_edge =
                    edge / std::sqrt(squared_distances[local]);
                const Scalar distance_ratio =
                    average_squared_distance /
                    squared_distances[local];
                const Scalar alignment =
                    std::abs(unit_edge.dot(current[neighbor]));
                const Scalar visibility_scale =
                    raw_directions.vertices[neighbor]
                        .visibility_cosine *
                    Scalar{2} / pi;
                const Scalar denominator =
                    visibility_scale * visibility_scale;
                const Scalar numerator = std::pow(
                    distance_ratio,
                    Scalar{3} + Scalar{2} * alignment);

                Scalar bounded_influence = Scalar{1};
                if (denominator >
                    std::numeric_limits<Scalar>::epsilon())
                {
                    const Scalar influence = numerator / denominator;
                    bounded_influence = std::isfinite(influence)
                        ? influence / (Scalar{1} + influence)
                        : Scalar{1};
                }
                const Scalar weight =
                    Scalar{0.5} + bounded_influence;
                neighbor_sum += weight * current[neighbor];
            }

            auto smoothed_neighbors = normalized(neighbor_sum);
            if (!smoothed_neighbors.has_value())
            {
                return std::nullopt;
            }
            if (front.layer > 0)
            {
                const auto lower = normalized(
                    front.vertices[vertex_index].direction);
                if (lower.has_value())
                {
                    smoothed_neighbors =
                        normalized(*smoothed_neighbors + *lower);
                    if (!smoothed_neighbors.has_value())
                    {
                        return std::nullopt;
                    }
                }
            }

            const Scalar minimum_edge = minimumIncidentEdge(
                front,
                adjacency.vertex_incident_faces[vertex_index]);
            if (!std::isfinite(minimum_edge) ||
                minimum_edge <=
                    std::numeric_limits<Scalar>::epsilon())
            {
                return std::nullopt;
            }
            const Scalar ratio =
                (position -
                 front.vertices[vertex_index].root_position).norm() /
                minimum_edge;
            const Scalar strength =
                Scalar{2} +
                (std::pow(Scalar{1.7}, ratio) - Scalar{1}) *
                    Scalar{15};
            if (!std::isfinite(strength))
            {
                return std::nullopt;
            }

            const Vector3 original = current[vertex_index];
            const Vector3 contribution =
                strength * *smoothed_neighbors;
            const auto blend =
                [&](Scalar scale) -> std::optional<Vector3>
                {
                    return normalized(
                        original + scale * contribution);
                };

            auto candidate = blend(Scalar{1});
            if (!candidate.has_value())
            {
                return std::nullopt;
            }

            const Scalar original_visibility = std::clamp(
                visibility(
                    original,
                    adjacency.vertex_incident_faces[vertex_index],
                    evaluation),
                Scalar{-1},
                Scalar{1});
            const Scalar original_max_angle =
                std::acos(original_visibility);
            const Scalar maximum_deviation = std::max(
                Scalar{0},
                (pi / Scalar{2} - original_max_angle) *
                    Scalar{2} / Scalar{3});
            const Scalar minimum_alignment =
                std::cos(maximum_deviation);

            if (original.dot(*candidate) < minimum_alignment)
            {
                Scalar low = Scalar{0};
                Scalar high = Scalar{1};
                for (std::size_t iteration = 0;
                     iteration < bisection_steps;
                     ++iteration)
                {
                    const Scalar middle =
                        Scalar{0.5} * (low + high);
                    const auto middle_direction = blend(middle);
                    if (middle_direction.has_value() &&
                        original.dot(*middle_direction) >=
                            minimum_alignment)
                    {
                        low = middle;
                    }
                    else
                    {
                        high = middle;
                    }
                }
                candidate = blend(low);
                if (!candidate.has_value())
                {
                    return std::nullopt;
                }
            }

            std::size_t retry = 0;
            while (visibility(
                       *candidate,
                       adjacency.vertex_incident_faces[vertex_index],
                       evaluation) < minimum_visible_cosine)
            {
                if (retry >= visibility_retries)
                {
                    candidate = original;
                    break;
                }
                candidate = normalized(
                    *candidate + Scalar{0.7} * original);
                if (!candidate.has_value())
                {
                    return std::nullopt;
                }
                ++retry;
            }
            return candidate;
        }
    }

    Result<SmoothedGrowthFields, GrowthFieldSmoothingError>
    GrowthFieldSmoother::smooth(
        const GrowthFront &front,
        const FrontEvaluation &evaluation,
        const FrontAdjacency &adjacency,
        const GrowthDirections &raw_directions,
        const std::vector<Scalar> &base_heights) const
    {
        using SmoothingResult =
            Result<SmoothedGrowthFields, GrowthFieldSmoothingError>;
        if (!compatibleInputs(
                front,
                evaluation,
                adjacency,
                raw_directions,
                base_heights))
        {
            return SmoothingResult::failure(
                GrowthFieldInputMismatch{
                    front.layer,
                    raw_directions.layer,
                    front.vertices.size(),
                    raw_directions.vertices.size(),
                    base_heights.size()});
        }

        auto initialized = validateAndInitialize(
            front,
            evaluation,
            adjacency,
            raw_directions,
            base_heights);
        if (!initialized.hasValue())
        {
            return SmoothingResult::failure(initialized.error());
        }

        std::vector<Vector3> current =
            std::move(initialized.value());
        std::vector<std::size_t> active(current.size());
        for (std::size_t index = 0; index < active.size(); ++index)
        {
            active[index] = index;
        }
        std::vector<std::size_t> active_history;

        for (std::size_t round = 0;
             round < maximum_rounds && !active.empty();
             ++round)
        {
            active_history.push_back(active.size());
            if (active_history.size() > 12 &&
                active_history[active_history.size() - 10] ==
                    active.size())
            {
                break;
            }

            std::vector<Vector3> next = current;
            std::vector<bool> needs_smoothing(current.size(), false);
            for (const std::size_t vertex_index : active)
            {
                const auto candidate = smoothOne(
                    vertex_index,
                    front,
                    evaluation,
                    adjacency,
                    raw_directions,
                    current);
                if (!candidate.has_value())
                {
                    return SmoothingResult::failure(
                        UndefinedSmoothedDirection{
                            vertex_index,
                            front.vertices[vertex_index]
                                .source_vertex_id,
                            front.layer});
                }
                next[vertex_index] = *candidate;
                if (candidate->dot(current[vertex_index]) <
                    update_alignment)
                {
                    needs_smoothing[vertex_index] = true;
                    for (const std::size_t neighbor :
                         adjacency.vertex_neighbors[vertex_index])
                    {
                        needs_smoothing[neighbor] = true;
                    }
                }
            }
            current.swap(next);

            if (round + 1 < full_rounds)
            {
                continue;
            }

            active.clear();
            for (std::size_t index = 0;
                 index < needs_smoothing.size();
                 ++index)
            {
                if (needs_smoothing[index])
                {
                    active.push_back(index);
                }
            }
        }

        return SmoothingResult::success(
            SmoothedGrowthFields{
                front.layer,
                std::move(current),
                base_heights});
    }
}
