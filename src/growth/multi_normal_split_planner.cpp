#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <set>
#include <utility>
#include <vector>

#include <Eigen/LU>

#include <boundary_mesh/growth/multi_normal_split_planner.hpp>

namespace boundary_mesh
{
    namespace
    {
        constexpr Scalar pi = 3.14159265358979323846;

        std::optional<Vector3> normalized(const Vector3 &value)
        {
            const Scalar length = value.norm();
            if (!value.allFinite() || !std::isfinite(length) ||
                length <= std::numeric_limits<Scalar>::epsilon())
            {
                return std::nullopt;
            }
            return value / length;
        }

        Scalar visibility(
            const Vector3 &direction,
            const std::vector<Vector3> &normals)
        {
            Scalar result = Scalar{1};
            for (const Vector3 &normal : normals)
            {
                result = std::min(result, direction.dot(normal));
            }
            return result;
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
            if (!normalized(plane_normal).has_value()) return std::nullopt;

            Eigen::Matrix3d system;
            system.row(0) = first_difference.transpose();
            system.row(1) = second_difference.transpose();
            system.row(2) = plane_normal.transpose();
            const Vector3 right_hand_side{
                (second.squaredNorm() - first.squaredNorm()) / Scalar{2},
                (third.squaredNorm() - first.squaredNorm()) / Scalar{2},
                plane_normal.dot(first)};
            const Eigen::FullPivLU<Eigen::Matrix3d> decomposition(system);
            if (!decomposition.isInvertible()) return std::nullopt;
            return normalized(decomposition.solve(right_hand_side));
        }

        std::optional<Vector3> mostNormal(
            const std::vector<Vector3> &normals)
        {
            if (normals.empty()) return std::nullopt;
            if (normals.size() == 1) return normalized(normals.front());

            std::optional<Vector3> best;
            Scalar best_visibility = -std::numeric_limits<Scalar>::infinity();
            const auto consider = [&](const std::optional<Vector3> &candidate)
            {
                if (!candidate.has_value()) return;
                const Scalar value = visibility(*candidate, normals);
                if (std::isfinite(value) && value > best_visibility)
                {
                    best_visibility = value;
                    best = candidate;
                }
            };

            for (std::size_t first = 0; first < normals.size(); ++first)
            {
                for (std::size_t second = first + 1;
                     second < normals.size();
                     ++second)
                {
                    consider(normalized(normals[first] + normals[second]));
                    for (std::size_t third = second + 1;
                         third < normals.size();
                         ++third)
                    {
                        consider(circleCenterDirection(
                            normals[first], normals[second], normals[third]));
                    }
                }
            }
            return best;
        }

        Scalar skewness(Scalar cosine)
        {
            return std::acos(std::clamp(cosine, Scalar{-1}, Scalar{1})) /
                (pi / Scalar{2});
        }

        std::vector<std::vector<std::size_t>> groupsFromSplitters(
            std::size_t sector_count,
            const std::vector<std::size_t> &splitters)
        {
            std::vector<std::vector<std::size_t>> groups;
            if (splitters.empty()) return groups;
            for (std::size_t split_index = 0;
                 split_index < splitters.size();
                 ++split_index)
            {
                const std::size_t begin = splitters[split_index];
                const std::size_t end =
                    splitters[(split_index + 1) % splitters.size()];
                groups.emplace_back();
                for (std::size_t sector = begin;;
                     sector = (sector + 1) % sector_count)
                {
                    groups.back().push_back(sector);
                    if ((sector + 1) % sector_count == end) break;
                }
            }
            return groups;
        }
    }

    Result<std::vector<VertexSplitPlan>, MultiNormalError>
    planMultiNormalSplits(
        const GrowthFront &front,
        const std::vector<IncidentFaceFan> &fans,
        const MultiNormalOptions &options)
    {
        using PlanResult =
            Result<std::vector<VertexSplitPlan>, MultiNormalError>;
        if (fans.size() != front.vertices.size())
        {
            return PlanResult::failure(MultiNormalInputMismatch{
                front.vertices.size(), front.faces.size()});
        }
        if (!options.enabled)
        {
            return PlanResult::success({});
        }

        std::vector<VertexSplitPlan> plans;
        for (std::size_t vertex_index = 0;
             vertex_index < fans.size();
             ++vertex_index)
        {
            const IncidentFaceFan &fan = fans[vertex_index];
            if (fan.center_vertex != static_cast<VertexId>(vertex_index) ||
                !fan.closed || fan.sectors.size() < 2)
            {
                continue;
            }

            std::vector<Vector3> all_normals;
            for (const IncidentFaceSector &sector : fan.sectors)
            {
                all_normals.push_back(sector.unit_normal);
            }
            const auto single = mostNormal(all_normals);
            if (!single.has_value())
            {
                return PlanResult::failure(InvalidMultiNormalTopology{
                    front.vertices[vertex_index].source_vertex_id});
            }
            const Scalar original_visibility =
                visibility(*single, all_normals);
            const Scalar original_skewness =
                skewness(original_visibility);
            if (original_skewness <= options.split_skewness_threshold)
            {
                continue;
            }

            std::vector<std::size_t> convex_splitters;
            for (std::size_t sector = 0;
                 sector < fan.sectors.size();
                 ++sector)
            {
                const std::size_t previous =
                    (sector + fan.sectors.size() - 1) % fan.sectors.size();
                const Scalar ridge =
                    (Scalar{1} - fan.sectors[sector].unit_normal.dot(
                        fan.sectors[previous].unit_normal)) / Scalar{2};
                if (ridge > options.convex_skewness_threshold)
                {
                    convex_splitters.push_back(sector);
                }
            }
            if (convex_splitters.size() < 2 ||
                convex_splitters.size() > 16)
            {
                continue;
            }

            const auto groups = groupsFromSplitters(
                fan.sectors.size(), convex_splitters);
            VertexSplitPlan plan;
            plan.front_vertex_index = vertex_index;
            plan.source_vertex_id =
                front.vertices[vertex_index].source_vertex_id;
            plan.original_visibility_cosine = original_visibility;
            plan.original_skewness = original_skewness;
            plan.selected_skewness = Scalar{0};

            bool valid = true;
            for (std::size_t group_index = 0;
                 group_index < groups.size();
                 ++group_index)
            {
                std::vector<Vector3> group_normals;
                SplitBranch branch;
                for (const std::size_t sector : groups[group_index])
                {
                    group_normals.push_back(
                        fan.sectors[sector].unit_normal);
                    branch.face_indices.push_back(
                        fan.sectors[sector].face_index);
                }
                const auto direction = mostNormal(group_normals);
                if (!direction.has_value())
                {
                    valid = false;
                    break;
                }
                branch.direction = *direction;
                branch.visibility_cosine =
                    visibility(*direction, group_normals);
                plan.selected_skewness = std::max(
                    plan.selected_skewness,
                    skewness(branch.visibility_cosine));
                plan.branches.push_back(std::move(branch));
                plan.splitter_neighbors.push_back(
                    fan.sectors[convex_splitters[group_index]]
                        .previous_vertex);
            }
            if (valid &&
                plan.selected_skewness * Scalar{1.01} <
                    plan.original_skewness)
            {
                plans.push_back(std::move(plan));
            }
        }

        return PlanResult::success(std::move(plans));
    }
}
