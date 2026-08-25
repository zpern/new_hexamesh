#include <boundary_mesh/growth/isotropic_stop_evaluator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
        struct FaceScales
        {
            Scalar average{};
            Scalar geometric{};
            Scalar minimum{};
        };

        template <class Face>
        FaceScales faceScales(
            const Face &face,
            const GrowthFront &front)
        {
            Scalar sum = Scalar{0};
            Scalar product = Scalar{1};
            Scalar minimum =
                std::numeric_limits<Scalar>::max();
            for (std::size_t local = 0;
                 local < face.vertex_ids.size();
                 ++local)
            {
                const std::size_t first = static_cast<std::size_t>(
                    face.vertex_ids[local]);
                const std::size_t second = static_cast<std::size_t>(
                    face.vertex_ids[
                        (local + 1) % face.vertex_ids.size()]);
                const Scalar length =
                    (front.vertices[second].position -
                     front.vertices[first].position)
                        .norm();
                sum += length;
                product *= length;
                minimum = std::min(minimum, length);
            }
            const Scalar count = static_cast<Scalar>(
                face.vertex_ids.size());
            return FaceScales{
                sum / count,
                std::pow(product, Scalar{1} / count),
                minimum};
        }

        bool validFaceMapping(
            const SurfaceFace &current,
            const SurfaceFace &candidate,
            std::size_t vertex_count)
        {
            return std::visit(
                [&](const auto &current_face)
                {
                    using Face = std::decay_t<decltype(current_face)>;
                    const Face *candidate_face =
                        std::get_if<Face>(&candidate);
                    if (candidate_face == nullptr ||
                        candidate_face->vertex_ids !=
                            current_face.vertex_ids)
                    {
                        return false;
                    }
                    return std::all_of(
                        current_face.vertex_ids.begin(),
                        current_face.vertex_ids.end(),
                        [&](const VertexId vertex_id)
                        {
                            return static_cast<std::size_t>(vertex_id) <
                                vertex_count;
                        });
                },
                current);
        }

        bool validAdjacency(
            const FrontAdjacency &adjacency,
            std::size_t vertex_count,
            std::size_t face_count)
        {
            if (adjacency.vertex_neighbors.size() != vertex_count ||
                adjacency.vertex_incident_faces.size() != vertex_count)
            {
                return false;
            }
            for (std::size_t vertex = 0;
                 vertex < vertex_count;
                 ++vertex)
            {
                if (std::any_of(
                        adjacency.vertex_neighbors[vertex].begin(),
                        adjacency.vertex_neighbors[vertex].end(),
                        [&](const std::size_t neighbor)
                        {
                            return neighbor >= vertex_count;
                        }) ||
                    std::any_of(
                        adjacency.vertex_incident_faces[vertex].begin(),
                        adjacency.vertex_incident_faces[vertex].end(),
                        [&](const std::size_t face)
                        {
                            return face >= face_count;
                        }))
                {
                    return false;
                }
            }
            return true;
        }
    }

    Result<IsotropicStopEvaluation, InvalidLayerFrontMapping>
    IsotropicStopEvaluator::evaluate(
        const GrowthFront &current_front,
        const GrowthFront &candidate_front,
        const FrontAdjacency &adjacency,
        Scalar isotropic_height) const
    {
        using EvaluationResult = Result<
            IsotropicStopEvaluation,
            InvalidLayerFrontMapping>;

        const auto invalid = [&]
        {
            return EvaluationResult::failure(
                InvalidLayerFrontMapping{candidate_front.layer});
        };
        if (!std::isfinite(isotropic_height) ||
            isotropic_height <= Scalar{0} ||
            current_front.faces.empty() ||
            current_front.vertices.size() !=
                candidate_front.vertices.size() ||
            current_front.faces.size() != candidate_front.faces.size() ||
            current_front.source_face_ids !=
                candidate_front.source_face_ids ||
            !validAdjacency(
                adjacency,
                current_front.vertices.size(),
                current_front.faces.size()))
        {
            return invalid();
        }
        for (std::size_t face_index = 0;
             face_index < current_front.faces.size();
             ++face_index)
        {
            if (!validFaceMapping(
                    current_front.faces[face_index],
                    candidate_front.faces[face_index],
                    current_front.vertices.size()))
            {
                return invalid();
            }
        }

        std::vector<FaceScales> scales;
        scales.reserve(current_front.faces.size());
        Scalar global_average = Scalar{0};
        for (const SurfaceFace &face : current_front.faces)
        {
            const FaceScales value = std::visit(
                [&](const auto &typed_face)
                {
                    return faceScales(typed_face, current_front);
                },
                face);
            scales.push_back(value);
            global_average += value.average;
            if (!std::isfinite(value.average) ||
                !std::isfinite(value.geometric) ||
                !std::isfinite(value.minimum) ||
                value.average <= Scalar{0} ||
                value.geometric <= Scalar{0} ||
                value.minimum <= Scalar{0})
            {
                return invalid();
            }
        }
        global_average /= static_cast<Scalar>(scales.size());
        if (!std::isfinite(global_average) ||
            global_average <= Scalar{0})
        {
            return invalid();
        }

        IsotropicStopEvaluation output;
        output.vertex_candidates.resize(
            current_front.vertices.size(), true);
        output.vertex_stops.resize(
            current_front.vertices.size(), false);
        output.face_stops.resize(current_front.faces.size(), false);

        for (std::size_t vertex = 0;
             vertex < current_front.vertices.size();
             ++vertex)
        {
            const Scalar scaled_height =
                (candidate_front.vertices[vertex].position -
                 current_front.vertices[vertex].position)
                    .norm() /
                isotropic_height;
            if (!std::isfinite(scaled_height) ||
                scaled_height <= Scalar{0} ||
                adjacency.vertex_incident_faces[vertex].empty())
            {
                return invalid();
            }
            bool continue_requested = false;
            bool forced_stop = false;
            for (const std::size_t face_index :
                 adjacency.vertex_incident_faces[vertex])
            {
                const FaceScales &local = scales[face_index];
                const Scalar blended = std::max(
                    Scalar{0.1} * global_average +
                        Scalar{0.9} * local.average,
                    Scalar{0.3} * global_average +
                        Scalar{0.7} * local.average);
                if (blended > Scalar{0.95} * scaled_height)
                {
                    continue_requested = true;
                }
                if (scaled_height > Scalar{1.3} * local.geometric ||
                    scaled_height > Scalar{1.8} * local.minimum)
                {
                    forced_stop = true;
                }
            }
            output.vertex_candidates[vertex] =
                forced_stop || !continue_requested;
        }

        for (std::size_t vertex = 0;
             vertex < current_front.vertices.size();
             ++vertex)
        {
            output.vertex_stops[vertex] =
                output.vertex_candidates[vertex] &&
                std::all_of(
                    adjacency.vertex_neighbors[vertex].begin(),
                    adjacency.vertex_neighbors[vertex].end(),
                    [&](const std::size_t neighbor)
                    {
                        return output.vertex_candidates[neighbor];
                    });
        }

        for (std::size_t face_index = 0;
             face_index < current_front.faces.size();
             ++face_index)
        {
            output.face_stops[face_index] = std::visit(
                [&](const auto &face)
                {
                    return std::any_of(
                        face.vertex_ids.begin(),
                        face.vertex_ids.end(),
                        [&](const VertexId vertex_id)
                        {
                            return output.vertex_stops[
                                static_cast<std::size_t>(vertex_id)];
                        });
                },
                current_front.faces[face_index]);
        }

        return EvaluationResult::success(std::move(output));
    }
}
