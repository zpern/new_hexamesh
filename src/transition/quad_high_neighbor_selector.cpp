#include <algorithm>
#include <limits>

#include <boundary_mesh/surface/face_skewness.hpp>
#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>

namespace boundary_mesh
{
    namespace
    {
        bool adjacent(std::size_t first, std::size_t second)
        {
            return (first + 1) % 4 == second ||
                   (second + 1) % 4 == first;
        }

        std::optional<QuadDiagonal> forcedDiagonal(
            const std::vector<std::size_t> &edges)
        {
            if (edges.size() != 2) return std::nullopt;
            const std::size_t common =
                (edges[0] + 1) % 4 == edges[1]
                    ? edges[1]
                    : edges[0];
            return common % 2 == 0
                ? QuadDiagonal::ZeroTwo
                : QuadDiagonal::OneThree;
        }

        std::vector<std::vector<std::size_t>> candidates(
            const std::vector<QuadHighNeighbor> &highs)
        {
            std::vector<std::size_t> edges;
            for (const auto &high : highs) edges.push_back(high.local_edge);
            std::sort(edges.begin(), edges.end());
            edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
            if (edges.size() <= 1) return {edges};
            if (edges.size() == 2 && adjacent(edges[0], edges[1]))
                return {edges};
            if (edges.size() == 2)
                return {{edges[0]}, {edges[1]}};

            std::vector<std::vector<std::size_t>> result;
            for (std::size_t first = 0; first < edges.size(); ++first)
                for (std::size_t second = first + 1;
                     second < edges.size(); ++second)
                    if (adjacent(edges[first], edges[second]))
                        result.push_back({edges[first], edges[second]});
            return result;
        }

        SurfaceFaceId neighborFor(
            const std::vector<QuadHighNeighbor> &highs,
            std::size_t edge)
        {
            for (const auto &high : highs)
                if (high.local_edge == edge)
                    return high.neighbor_face_id;
            return std::numeric_limits<SurfaceFaceId>::max();
        }

        Result<Scalar, FaceEvaluationError> scoreTriangles(
            const std::vector<Triangle> &faces,
            const std::vector<Point3> &points,
            Scalar tolerance)
        {
            Scalar worst = 0;
            for (const Triangle &face : faces)
            {
                const std::array<Point3, 3> triangle_points{{
                    points[face.vertex_ids[0]],
                    points[face.vertex_ids[1]],
                    points[face.vertex_ids[2]]}};
                const auto score = triangleEquiangularSkewness(
                    triangle_points, tolerance);
                if (!score.hasValue())
                    return Result<Scalar, FaceEvaluationError>::failure(
                        score.error());
                worst = std::max(worst, score.value());
            }
            return Result<Scalar, FaceEvaluationError>::success(worst);
        }
    }

    QuadHighNeighborSelectionResult selectQuadHighNeighbors(
        const QuadHighNeighborSelectionInput &input)
    {
        using SelectResult = QuadHighNeighborSelectionResult;
        if (input.mesh_vertices == nullptr)
            return SelectResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});
        for (const auto &high : input.high_neighbors)
            if (high.local_edge >= 4)
                return SelectResult::failure(
                    TransitionTemplateError{
                        InvalidTransitionTemplateInput{
                            input.source_face_id}});

        const auto options = candidates(input.high_neighbors);
        std::vector<QuadHighNeighborSelection> viable;
        for (const auto &edges : options)
        {
            QuadHighNeighborSelection selection;
            selection.retained_local_edges = edges;
            selection.required_low_diagonal = forcedDiagonal(edges);
            for (const std::size_t edge : edges)
                selection.retained_neighbor_faces.push_back(
                    neighborFor(input.high_neighbors, edge));
            std::sort(
                selection.retained_neighbor_faces.begin(),
                selection.retained_neighbor_faces.end());
            for (const auto &high : input.high_neighbors)
                if (std::find(edges.begin(), edges.end(), high.local_edge) ==
                    edges.end())
                    selection.suppressed_neighbor_faces.push_back(
                        high.neighbor_face_id);
            std::sort(
                selection.suppressed_neighbor_faces.begin(),
                selection.suppressed_neighbor_faces.end());

            std::vector<Triangle> top_faces;
            if (edges.size() == 1)
            {
                OrientedQuad quad;
                quad.vertex_ids = input.low;
                for (std::size_t index = 0; index < 4; ++index)
                    quad.points[index] =
                        (*input.mesh_vertices)[input.low[index]];
                const auto diagonal = chooseQuadDiagonal(
                    quad, input.length_tolerance);
                if (!diagonal.hasValue()) continue;
                const auto built = buildQuadSideTransition({
                    input.source_face_id, input.low_layer,
                    input.low, input.high, edges[0],
                    diagonal.value().diagonal});
                if (!built.hasValue()) continue;
                top_faces = built.value().top_faces;
            }
            else if (edges.size() == 2)
            {
                const auto built = buildQuadAdjacentSideTransition({
                    input.source_face_id, input.low_layer,
                    input.low, input.high, edges[0], edges[1],
                    *selection.required_low_diagonal,
                    input.mesh_vertices, input.length_tolerance});
                if (!built.hasValue()) continue;
                top_faces = built.value().top_faces;
            }
            const auto score = scoreTriangles(
                top_faces, *input.mesh_vertices,
                input.length_tolerance);
            if (!score.hasValue() && !top_faces.empty()) continue;
            selection.worst_skewness = top_faces.empty()
                ? Scalar{0}
                : score.value();
            viable.push_back(std::move(selection));
        }
        if (viable.empty())
            return SelectResult::failure(
                TransitionTemplateError{
                    InvalidTransitionTemplateInput{
                        input.source_face_id}});
        std::sort(
            viable.begin(), viable.end(),
            [](const auto &left, const auto &right)
            {
                if (left.worst_skewness != right.worst_skewness)
                    return left.worst_skewness < right.worst_skewness;
                if (left.retained_neighbor_faces !=
                    right.retained_neighbor_faces)
                    return left.retained_neighbor_faces <
                           right.retained_neighbor_faces;
                return left.retained_local_edges <
                       right.retained_local_edges;
            });
        return SelectResult::success(std::move(viable.front()));
    }
}
