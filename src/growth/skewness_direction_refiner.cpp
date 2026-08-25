#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/skewness_direction_refiner.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar pi = 3.14159265358979323846;
        constexpr Scalar minimum_visible_cosine =
            Scalar{35} * Scalar{0.8} * pi / Scalar{180};

        struct LocalObjective
        {
            Scalar maximum{};
            Scalar average{};
            bool valid{};
        };

        bool better(
            const LocalObjective &candidate,
            const LocalObjective &current,
            Scalar candidate_alignment,
            Scalar current_alignment,
            Scalar tolerance)
        {
            if (!candidate.valid) return false;
            if (!current.valid) return true;
            if (candidate.maximum < current.maximum - tolerance) return true;
            if (std::abs(candidate.maximum - current.maximum) > tolerance)
            {
                return false;
            }
            if (candidate.average < current.average - tolerance) return true;
            if (std::abs(candidate.average - current.average) > tolerance)
            {
                return false;
            }
            return candidate_alignment > current_alignment + tolerance;
        }

        bool visible(
            const Vector3 &direction,
            const std::vector<std::size_t> &incident_faces,
            const FrontEvaluation &evaluation)
        {
            for (const std::size_t face_index : incident_faces)
            {
                if (face_index >= evaluation.faces.size() ||
                    direction.dot(
                        evaluation.faces[face_index].value.unit_normal) <
                        minimum_visible_cosine)
                {
                    return false;
                }
            }
            return true;
        }

        Point3 upperPosition(
            std::size_t vertex_index,
            std::size_t trial_vertex,
            const Vector3 &trial_direction,
            const GrowthFront &front,
            const std::vector<Vector3> &directions,
            const std::vector<Scalar> &heights)
        {
            const Vector3 &direction = vertex_index == trial_vertex
                ? trial_direction
                : directions[vertex_index];
            return front.vertices[vertex_index].position +
                heights[vertex_index] * direction;
        }

        LocalObjective evaluateLocal(
            std::size_t vertex_index,
            const Vector3 &trial_direction,
            const GrowthFront &front,
            const FrontAdjacency &adjacency,
            const std::vector<Vector3> &directions,
            const std::vector<Scalar> &heights)
        {
            LocalObjective objective;
            Scalar sum = Scalar{0};
            std::size_t count = 0;

            for (const std::size_t face_index :
                 adjacency.vertex_incident_faces[vertex_index])
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
                                const std::size_t current =
                                    static_cast<std::size_t>(
                                        face.vertex_ids[local]);
                                points[local] =
                                    front.vertices[current].position;
                                points[local + 3] = upperPosition(
                                    current,
                                    vertex_index,
                                    trial_direction,
                                    front,
                                    directions,
                                    heights);
                            }
                            return evaluatePrism(points);
                        }
                        else
                        {
                            HexaPoints points;
                            for (std::size_t local = 0; local < 4; ++local)
                            {
                                const std::size_t current =
                                    static_cast<std::size_t>(
                                        face.vertex_ids[local]);
                                points[local] =
                                    front.vertices[current].position;
                                points[local + 4] = upperPosition(
                                    current,
                                    vertex_index,
                                    trial_direction,
                                    front,
                                    directions,
                                    heights);
                            }
                            return evaluateHexa(points);
                        }
                    },
                    front.faces[face_index]);

                if (!quality.hasValue() ||
                    quality.value().validity != VolumeCellValidity::Valid ||
                    !std::isfinite(quality.value().skewness))
                {
                    return objective;
                }
                objective.maximum = std::max(
                    objective.maximum,
                    quality.value().skewness);
                sum += quality.value().skewness;
                ++count;
            }

            if (count == 0) return objective;
            objective.average = sum / static_cast<Scalar>(count);
            objective.valid = std::isfinite(objective.average);
            return objective;
        }

        std::pair<Vector3, Vector3> tangentBasis(
            const Vector3 &direction)
        {
            const std::array<Vector3, 3> axes{
                Vector3::UnitX(),
                Vector3::UnitY(),
                Vector3::UnitZ()};
            const auto axis = *std::min_element(
                axes.begin(),
                axes.end(),
                [&](const Vector3 &first, const Vector3 &second)
                {
                    return std::abs(direction.dot(first)) <
                        std::abs(direction.dot(second));
                });
            const Vector3 tangent = direction.cross(axis).normalized();
            return {tangent, direction.cross(tangent).normalized()};
        }

        Scalar globalMaximum(
            const GrowthFront &front,
            const FrontAdjacency &adjacency,
            const std::vector<Vector3> &directions,
            const std::vector<Scalar> &heights)
        {
            Scalar maximum = Scalar{0};
            for (std::size_t vertex = 0;
                 vertex < front.vertices.size();
                 ++vertex)
            {
                const LocalObjective objective = evaluateLocal(
                    vertex,
                    directions[vertex],
                    front,
                    adjacency,
                    directions,
                    heights);
                if (objective.valid)
                {
                    maximum = std::max(maximum, objective.maximum);
                }
            }
            return maximum;
        }
    }

    SkewnessDirectionRefinement refineDirectionsForSkewness(
        const GrowthFront &front,
        const FrontEvaluation &evaluation,
        const FrontAdjacency &adjacency,
        const std::vector<Vector3> &baseline_directions,
        const std::vector<Scalar> &fixed_actual_heights,
        const SkewnessNormalOptimizationOptions &options)
    {
        SkewnessDirectionRefinement output;
        output.directions = baseline_directions;
        if (!options.enabled ||
            baseline_directions.size() != front.vertices.size() ||
            fixed_actual_heights.size() != front.vertices.size())
        {
            return output;
        }

        output.diagnostics.maximum_skewness_before = globalMaximum(
            front,
            adjacency,
            baseline_directions,
            fixed_actual_heights);

        std::vector<bool> active(front.vertices.size(), false);
        for (std::size_t vertex = 0;
             vertex < front.vertices.size();
             ++vertex)
        {
            const LocalObjective objective = evaluateLocal(
                vertex,
                baseline_directions[vertex],
                front,
                adjacency,
                baseline_directions,
                fixed_actual_heights);
            active[vertex] = objective.valid &&
                objective.maximum > options.activation_skewness;
            if (active[vertex])
            {
                ++output.diagnostics.activated_vertices;
            }
        }

        std::vector<bool> ever_updated(front.vertices.size(), false);
        const std::array<Scalar, 2> angles{
            options.first_angle_degrees,
            options.second_angle_degrees};
        const Scalar maximum_deviation =
            (options.first_angle_degrees + options.second_angle_degrees) *
            pi / Scalar{180};
        const Scalar minimum_baseline_alignment =
            std::cos(maximum_deviation);

        const std::size_t levels = std::min<std::size_t>(
            options.maximum_levels,
            angles.size());
        for (std::size_t level = 0; level < levels; ++level)
        {
            const std::vector<Vector3> level_start = output.directions;
            std::vector<Vector3> level_next = level_start;
            std::vector<bool> next_active(front.vertices.size(), false);

            for (std::size_t vertex = 0;
                 vertex < front.vertices.size();
                 ++vertex)
            {
                if (!active[vertex]) continue;
                const Vector3 center = level_start[vertex];
                const auto basis = tangentBasis(center);
                const LocalObjective original = evaluateLocal(
                    vertex,
                    center,
                    front,
                    adjacency,
                    level_start,
                    fixed_actual_heights);
                LocalObjective best = original;
                Vector3 best_direction = center;
                Scalar best_alignment = center.dot(
                    baseline_directions[vertex]);
                const Scalar angle = angles[level] * pi / Scalar{180};

                for (std::size_t sample = 0;
                     sample < options.azimuth_samples;
                     ++sample)
                {
                    const Scalar azimuth = Scalar{2} * pi *
                        static_cast<Scalar>(sample) /
                        static_cast<Scalar>(options.azimuth_samples);
                    Vector3 candidate =
                        std::cos(angle) * center +
                        std::sin(angle) *
                            (std::cos(azimuth) * basis.first +
                             std::sin(azimuth) * basis.second);
                    candidate.normalize();
                    const Scalar alignment = candidate.dot(
                        baseline_directions[vertex]);
                    if (!candidate.allFinite() ||
                        alignment < minimum_baseline_alignment ||
                        !visible(
                            candidate,
                            adjacency.vertex_incident_faces[vertex],
                            evaluation))
                    {
                        continue;
                    }
                    const LocalObjective trial = evaluateLocal(
                        vertex,
                        candidate,
                        front,
                        adjacency,
                        level_start,
                        fixed_actual_heights);
                    if (better(
                            trial,
                            best,
                            alignment,
                            best_alignment,
                            options.improvement_tolerance))
                    {
                        best = trial;
                        best_direction = candidate;
                        best_alignment = alignment;
                    }
                }

                if (best.valid && original.valid &&
                    best.maximum < original.maximum -
                        options.improvement_tolerance)
                {
                    level_next[vertex] = best_direction;
                    next_active[vertex] = true;
                    ever_updated[vertex] = true;
                }
            }
            output.directions.swap(level_next);
            active.swap(next_active);
        }

        output.diagnostics.updated_vertices =
            static_cast<std::size_t>(std::count(
                ever_updated.begin(),
                ever_updated.end(),
                true));
        output.diagnostics.maximum_skewness_after = globalMaximum(
            front,
            adjacency,
            output.directions,
            fixed_actual_heights);
        return output;
    }
}
