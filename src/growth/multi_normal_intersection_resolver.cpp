#include <array>
#include <cmath>
#include <cstddef>
#include <set>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/multi_normal_intersection_resolver.hpp>
#include <boundary_mesh/growth/multi_normal_transition_builder.hpp>
#include <boundary_mesh/spatial/collision_index.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct TaggedTriangle
        {
            CollisionTriangle triangle;
            std::size_t face_index{};
            bool top{};
        };

        bool nonDegenerate(const TrianglePoints &points)
        {
            return (points[1] - points[0]).cross(points[2] - points[0])
                       .squaredNorm() > Scalar{1e-24};
        }

        template <typename PointGetter, typename KeyGetter>
        void appendTriangle(
            const std::array<VertexId, 3> &ids,
            std::size_t face_index,
            bool top,
            PointGetter point,
            KeyGetter key,
            std::vector<TaggedTriangle> &output)
        {
            TaggedTriangle tagged;
            tagged.face_index = face_index;
            tagged.top = top;
            tagged.triangle.owner_kind = top
                ? CollisionOwnerKind::LayerCandidate
                : CollisionOwnerKind::OriginalSurface;
            tagged.triangle.owner_id = static_cast<std::uint32_t>(face_index);
            tagged.triangle.boundary_vertex_count = 3;
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                tagged.triangle.points[corner] = point(ids[corner]);
                tagged.triangle.vertex_keys[corner] = key(ids[corner]);
                tagged.triangle.boundary_points[corner] =
                    tagged.triangle.points[corner];
                tagged.triangle.boundary_vertex_keys[corner] =
                    tagged.triangle.vertex_keys[corner];
            }
            if (nonDegenerate(tagged.triangle.points))
                output.push_back(std::move(tagged));
        }

        template <typename PointGetter, typename KeyGetter>
        void appendFace(
            const SurfaceFace &surface_face,
            std::size_t face_index,
            bool top,
            PointGetter point,
            KeyGetter key,
            std::vector<TaggedTriangle> &output)
        {
            std::visit(
                [&](const auto &face)
                {
                    using Face = std::decay_t<decltype(face)>;
                    if constexpr (std::is_same_v<Face, Triangle>)
                    {
                        appendTriangle(face.vertex_ids, face_index, top,
                                       point, key, output);
                    }
                    else
                    {
                        appendTriangle(
                            {face.vertex_ids[0], face.vertex_ids[1],
                             face.vertex_ids[2]},
                            face_index, top, point, key, output);
                        appendTriangle(
                            {face.vertex_ids[0], face.vertex_ids[2],
                             face.vertex_ids[3]},
                            face_index, top, point, key, output);
                    }
                },
                surface_face);
        }

        std::set<std::size_t> intersectingFaces(
            const MultiNormalTopology &topology,
            const MultiNormalTransitionResult &candidate)
        {
            std::vector<TaggedTriangle> tagged;
            tagged.reserve(topology.front.faces.size() * 3);
            const auto bottom_point = [&](VertexId id)
            {
                return topology.front.vertices[static_cast<std::size_t>(id)]
                    .root_position;
            };
            const auto top_point = [&](VertexId id)
            {
                return candidate.transformed_front
                    .vertices[static_cast<std::size_t>(id)].position;
            };
            const auto bottom_key = [&](VertexId id)
            {
                return CollisionVertexKey{id, 0};
            };
            const auto top_key = [&](VertexId id)
            {
                return CollisionVertexKey{id, 1};
            };
            for (std::size_t face = 0;
                 face < topology.front.faces.size(); ++face)
            {
                appendFace(topology.front.faces[face], face, false,
                           bottom_point, bottom_key, tagged);
                appendFace(topology.front.faces[face], face, true,
                           top_point, top_key, tagged);
            }

            std::vector<CollisionTriangle> primitives;
            primitives.reserve(tagged.size());
            for (const TaggedTriangle &item : tagged)
                primitives.push_back(item.triangle);
            const auto built = CollisionIndex::build(std::move(primitives));
            if (!built.hasValue()) return {};
            const CollisionIndex &index = built.value();

            std::set<std::size_t> bad;
            for (std::size_t query_index = 0;
                 query_index < tagged.size(); ++query_index)
            {
                const TaggedTriangle &query = tagged[query_index];
                if (!query.top) continue;
                bool affected = false;
                bool displaced = false;
                std::visit(
                    [&](const auto &face)
                    {
                        for (const VertexId id : face.vertex_ids)
                        {
                            const std::size_t vertex =
                                static_cast<std::size_t>(id);
                            affected = affected || topology.front.vertices[
                                vertex].multi_normal_branch;
                            displaced = displaced ||
                                (candidate.transformed_front.vertices[vertex]
                                     .position -
                                 topology.front.vertices[vertex].root_position)
                                        .squaredNorm() > Scalar{1e-24};
                        }
                    },
                    topology.front.faces[query.face_index]);
                if (!affected || !displaced) continue;

                for (const std::size_t contact :
                     index.queryIllegalContacts(query.triangle))
                {
                    if (contact >= tagged.size()) continue;
                    const TaggedTriangle &other = tagged[contact];
                    if (other.face_index == query.face_index) continue;
                    const auto kind = classifyTriangleContact(
                        query.triangle.points, other.triangle.points);
                    if (!kind.hasValue())
                    {
                        bad.insert(query.face_index);
                        break;
                    }
                    if (kind.value() == TriangleContactKind::ProperIntersect ||
                        kind.value() == TriangleContactKind::CoplanarOverlap)
                    {
                        bad.insert(query.face_index);
                        break;
                    }
                }
            }
            return bad;
        }

        void changeBadLengths(
            const MultiNormalTopology &topology,
            const std::set<std::size_t> &bad_faces,
            Scalar factor,
            std::vector<Scalar> &lengths)
        {
            for (const std::size_t face_index : bad_faces)
            {
                std::visit(
                    [&](const auto &face)
                    {
                        for (const VertexId id : face.vertex_ids)
                        {
                            const std::size_t index =
                                static_cast<std::size_t>(id);
                            if (index < lengths.size() &&
                                topology.front.vertices[index]
                                    .multi_normal_branch)
                                lengths[index] *= factor;
                        }
                    },
                    topology.front.faces[face_index]);
            }
        }
    }

    Result<ResolvedMultiNormalLengths, MultiNormalError>
    resolveMultiNormalLengths(
        const MultiNormalTopology &topology,
        std::vector<Scalar> lengths,
        const MultiNormalOptions &options)
    {
        using ResolveResult =
            Result<ResolvedMultiNormalLengths, MultiNormalError>;
        if (lengths.size() != topology.front.vertices.size())
            return ResolveResult::failure(MultiNormalInputMismatch{
                topology.front.vertices.size(), topology.front.faces.size()});

        bool zero_retry = false;
        std::size_t shrink_count = 0;
        for (std::size_t iteration = 0;; ++iteration)
        {
            MultiNormalOptions candidate_options = options;
            candidate_options.resolved_transition_lengths = lengths;
            const auto candidate = buildMultiNormalTransition(
                topology, candidate_options);
            if (!candidate.hasValue())
                return ResolveResult::failure(candidate.error());
            const std::set<std::size_t> bad =
                intersectingFaces(topology, candidate.value());
            if (bad.empty())
                return ResolveResult::success(ResolvedMultiNormalLengths{
                    std::move(lengths), shrink_count, zero_retry});

            if (iteration < 20)
            {
                changeBadLengths(topology, bad, Scalar{0.8}, lengths);
                ++shrink_count;
            }
            else if (!zero_retry)
            {
                changeBadLengths(topology, bad, Scalar{0}, lengths);
                zero_retry = true;
            }
            else
            {
                return ResolveResult::failure(
                    UnresolvedMultiNormalIntersection{bad.size()});
            }
        }
    }
}
