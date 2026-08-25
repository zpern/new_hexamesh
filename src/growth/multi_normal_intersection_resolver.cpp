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
#include <boundary_mesh/growth/blmesh_intersection_checker.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct TaggedTriangle
        {
            TrianglePoints triangle;
            std::size_t face_index{};
            bool top{};
            std::array<VertexId, 3> vertex_ids{};
        };

        bool nonDegenerate(const TrianglePoints &points)
        {
            return (points[1] - points[0]).cross(points[2] - points[0])
                       .squaredNorm() > Scalar{1e-24};
        }

        template <typename PointGetter>
        void appendTriangle(
            const std::array<VertexId, 3> &ids,
            std::size_t face_index,
            bool top,
            PointGetter point,
            std::vector<TaggedTriangle> &output)
        {
            TaggedTriangle tagged;
            tagged.face_index = face_index;
            tagged.top = top;
            tagged.vertex_ids = ids;
            for (std::size_t corner = 0; corner < 3; ++corner)
            {
                tagged.triangle[corner] = point(ids[corner]);
            }
            if (nonDegenerate(tagged.triangle))
                output.push_back(std::move(tagged));
        }

        template <typename PointGetter>
        void appendFace(
            const SurfaceFace &surface_face,
            std::size_t face_index,
            bool top,
            PointGetter point,
            std::vector<TaggedTriangle> &output)
        {
            std::visit(
                [&](const auto &face)
                {
                    using Face = std::decay_t<decltype(face)>;
                    if constexpr (std::is_same_v<Face, Triangle>)
                    {
                        appendTriangle(face.vertex_ids, face_index, top,
                                       point, output);
                    }
                    else
                    {
                        appendTriangle(
                            {face.vertex_ids[0], face.vertex_ids[1],
                             face.vertex_ids[2]},
                            face_index, top, point, output);
                        appendTriangle(
                            {face.vertex_ids[0], face.vertex_ids[2],
                             face.vertex_ids[3]},
                            face_index, top, point, output);
                    }
                },
                surface_face);
        }

        void changeBadLengths(
            const MultiNormalTopology &topology,
            const std::set<std::size_t> &bad_points,
            Scalar factor,
            std::vector<Scalar> &lengths)
        {
            for (const std::size_t index : bad_points)
            {
                if (index < lengths.size() &&
                    topology.front.vertices[index].multi_normal_branch)
                    lengths[index] *= factor;
            }
        }
    }

    std::set<std::size_t> findMultiNormalIntersectionBadPoints(
        const MultiNormalTopology &topology,
        const MultiNormalTransitionResult &candidate)
    {
        std::vector<TaggedTriangle> triangles;
        const auto bottom_point = [&](VertexId id) {
            return topology.front.vertices[static_cast<std::size_t>(id)].root_position;
        };
        const auto top_point = [&](VertexId id) {
            return candidate.transformed_front.vertices[static_cast<std::size_t>(id)].position;
        };
        for (std::size_t face = 0; face < topology.front.faces.size(); ++face)
        {
            appendFace(topology.front.faces[face], face, false, bottom_point, triangles);
            appendFace(topology.front.faces[face], face, true, top_point, triangles);
        }

        std::set<std::size_t> bad_points;
        for (const TaggedTriangle &query : triangles)
        {
            if (!query.top) continue;
            for (const TaggedTriangle &other : triangles)
            {
                if (!blmeshTrianglesIntersect(query.triangle, other.triangle))
                    continue;
                for (const VertexId id : query.vertex_ids)
                    bad_points.insert(static_cast<std::size_t>(id));
                break;
            }
        }
        return bad_points;
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
                findMultiNormalIntersectionBadPoints(topology, candidate.value());
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
