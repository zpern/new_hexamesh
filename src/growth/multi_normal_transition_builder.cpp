#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <set>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/multi_normal_transition_builder.hpp>

namespace boundary_mesh
{
    namespace
    {
        using TransitionResult =
            Result<MultiNormalTransitionResult, MultiNormalError>;

        Scalar signedVolume(
            const Point3 &a,
            const Point3 &b,
            const Point3 &c,
            const Point3 &d)
        {
            return (b - a).dot((c - a).cross(d - a)) / Scalar{6};
        }
    }

    Result<MultiNormalTransitionResult, MultiNormalError>
    buildMultiNormalTransition(
        const MultiNormalTopology &topology,
        const MultiNormalOptions &options)
    {
        if (topology.front.faces.size() !=
                topology.front.source_face_ids.size() ||
            !std::isfinite(options.transition_height) ||
            options.transition_height < Scalar{0})
        {
            return TransitionResult::failure(MultiNormalInputMismatch{
                topology.front.vertices.size(),
                topology.front.faces.size()});
        }

        MultiNormalTransitionResult output;
        output.transformed_front = topology.front;
        output.vertex_mapping = topology.vertex_mapping;
        output.transition_face_origins =
            topology.transition_face_origins;

        std::map<VertexId, VertexId> source_to_bottom;
        std::vector<VertexId> bottom_ids(
            topology.front.vertices.size());
        for (std::size_t topology_id = 0;
             topology_id < topology.front.vertices.size();
             ++topology_id)
        {
            const GrowthFrontVertex &vertex =
                topology.front.vertices[topology_id];
            const auto found = source_to_bottom.find(
                vertex.source_vertex_id);
            if (found == source_to_bottom.end())
            {
                const VertexId bottom_id = static_cast<VertexId>(
                    output.transition_cells.vertices.size());
                source_to_bottom.emplace(
                    vertex.source_vertex_id, bottom_id);
                output.transition_cells.vertices.push_back(
                    vertex.root_position);
                bottom_ids[topology_id] = bottom_id;
            }
            else
            {
                bottom_ids[topology_id] = found->second;
            }
        }

        std::vector<VertexId> upper_ids(topology.front.vertices.size());
        std::vector<bool> moved(topology.front.vertices.size(), false);
        for (std::size_t topology_id = 0;
             topology_id < topology.front.vertices.size();
             ++topology_id)
        {
            const GrowthFrontVertex &source =
                topology.front.vertices[topology_id];
            GrowthFrontVertex &target =
                output.transformed_front.vertices[topology_id];
            moved[topology_id] = source.multi_normal_branch;
            if (moved[topology_id])
            {
                const Scalar direction_length = source.direction.norm();
                if (!source.direction.allFinite() ||
                    !std::isfinite(direction_length) ||
                    direction_length <=
                        std::numeric_limits<Scalar>::epsilon())
                {
                    return TransitionResult::failure(
                        NonFiniteMultiNormalDisplacement{
                            source.source_vertex_id});
                }
                target.position += options.transition_height *
                    (source.direction / direction_length);
            }
            if (!target.position.allFinite())
            {
                return TransitionResult::failure(
                    NonFiniteMultiNormalDisplacement{
                        source.source_vertex_id});
            }
            upper_ids[topology_id] = static_cast<VertexId>(
                output.transition_cells.vertices.size());
            output.transition_cells.vertices.push_back(target.position);
        }

        const auto addTet = [&](std::array<VertexId, 4> ids,
                                SurfaceFaceId source_face_id)
        {
            std::set<VertexId> distinct_ids(ids.begin(), ids.end());
            if (distinct_ids.size() != 4) return;
            const Point3 &a = output.transition_cells.vertices[ids[0]];
            const Point3 &b = output.transition_cells.vertices[ids[1]];
            const Point3 &c = output.transition_cells.vertices[ids[2]];
            const Point3 &d = output.transition_cells.vertices[ids[3]];
            Scalar volume = signedVolume(a, b, c, d);
            if (!std::isfinite(volume) ||
                std::abs(volume) <= Scalar{1e-14})
            {
                return;
            }
            if (volume < Scalar{0}) std::swap(ids[1], ids[2]);
            output.transition_cells.cells.push_back(Tetra{ids});
            output.transition_cells.metadata.push_back(CellMetadata{
                CellRole::Transition, source_face_id, 0});
        };

        for (std::size_t face_index = 0;
             face_index < topology.front.faces.size();
             ++face_index)
        {
            const SurfaceFaceId source_face_id =
                topology.front.source_face_ids[face_index];
            std::visit(
                [&](const auto &face)
                {
                    using Face = std::decay_t<decltype(face)>;
                    if constexpr (std::is_same_v<Face, Quad>)
                    {
                        OmittedQuadTransition omitted;
                        omitted.source_face_id = source_face_id;
                        bool any_moved = false;
                        for (std::size_t corner = 0; corner < 4; ++corner)
                        {
                            const std::size_t topology_id =
                                static_cast<std::size_t>(
                                    face.vertex_ids[corner]);
                            if (topology_id >= bottom_ids.size()) return;
                            omitted.topology_vertex_ids[corner] =
                                face.vertex_ids[corner];
                            omitted.bottom_vertex_ids[corner] =
                                bottom_ids[topology_id];
                            omitted.moved_corners[corner] = moved[topology_id];
                            any_moved = any_moved || moved[topology_id];
                        }
                        if (any_moved)
                        {
                            output.omitted_quad_transitions.push_back(
                                omitted);
                        }
                    }
                    else
                    {
                        std::array<std::size_t, 3> topology_ids{};
                        std::array<VertexId, 3> lower{};
                        std::array<VertexId, 3> upper{};
                        for (std::size_t corner = 0; corner < 3; ++corner)
                        {
                            topology_ids[corner] =
                                static_cast<std::size_t>(
                                    face.vertex_ids[corner]);
                            if (topology_ids[corner] >= bottom_ids.size())
                            {
                                return;
                            }
                            lower[corner] = bottom_ids[topology_ids[corner]];
                            upper[corner] = upper_ids[topology_ids[corner]];
                        }

                        if (lower[0] == lower[1] &&
                            lower[1] == lower[2])
                        {
                            addTet({lower[0], upper[0], upper[1], upper[2]},
                                   source_face_id);
                            return;
                        }

                        if (lower[0] != lower[1] &&
                            lower[1] != lower[2] &&
                            lower[0] != lower[2])
                        {
                            std::size_t k1 = 0;
                            for (std::size_t corner = 1; corner < 3; ++corner)
                            {
                                if (lower[corner] < lower[k1]) k1 = corner;
                            }
                            const bool reverse =
                                lower[(k1 + 2) % 3] < lower[(k1 + 1) % 3];
                            const std::size_t k2 =
                                reverse ? (k1 + 2) % 3 : (k1 + 1) % 3;
                            const std::size_t k3 =
                                reverse ? (k1 + 1) % 3 : (k1 + 2) % 3;
                            const int ordinary_count =
                                static_cast<int>(!moved[topology_ids[k1]]) +
                                static_cast<int>(!moved[topology_ids[k2]]) +
                                static_cast<int>(!moved[topology_ids[k3]]);
                            if (ordinary_count == 3) return;

                            if (!reverse)
                            {
                                addTet({lower[k1], upper[k2], lower[k2], upper[k3]}, source_face_id);
                                addTet({lower[k1], upper[k1], upper[k2], upper[k3]}, source_face_id);
                                addTet({lower[k1], lower[k2], lower[k3], upper[k3]}, source_face_id);
                            }
                            else
                            {
                                addTet({lower[k1], lower[k2], upper[k2], upper[k3]}, source_face_id);
                                addTet({lower[k1], upper[k2], upper[k1], upper[k3]}, source_face_id);
                                addTet({lower[k1], lower[k3], lower[k2], upper[k3]}, source_face_id);
                            }
                            return;
                        }

                        std::size_t k1 = 0;
                        for (std::size_t corner = 0; corner < 3; ++corner)
                        {
                            if (lower[corner] == lower[(corner + 1) % 3])
                            {
                                k1 = corner;
                                break;
                            }
                        }
                        const std::size_t k2 = (k1 + 1) % 3;
                        const std::size_t k3 = (k1 + 2) % 3;
                        if (lower[k1] < lower[k3])
                        {
                            addTet({lower[k1], upper[k1], upper[k2], upper[k3]}, source_face_id);
                        }
                        else
                        {
                            if (moved[topology_ids[k3]])
                            {
                                addTet({lower[k3], upper[k1], upper[k2], upper[k3]}, source_face_id);
                            }
                            addTet({lower[k1], upper[k1], upper[k2], lower[k3]}, source_face_id);
                        }
                    }
                },
                topology.front.faces[face_index]);
        }

        return TransitionResult::success(std::move(output));
    }
}
