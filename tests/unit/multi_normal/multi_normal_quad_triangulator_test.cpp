#include <algorithm>
#include <array>
#include <cmath>
#include <type_traits>
#include <variant>

#include <boundary_mesh/multi_normal/multi_normal_quad_triangulator.hpp>
#include <boundary_mesh/surface/face_skewness.hpp>

namespace
{
    using namespace boundary_mesh;

    Scalar triangleSkewness(
        const GrowthFront &front,
        const Triangle &triangle)
    {
        std::array<Point3, 3> points{};
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            points[corner] = front.vertices[
                triangle.vertex_ids[corner]].position;
        }
        const auto value = triangleEquiangularSkewness(points, Scalar{1e-12});
        return value.hasValue() ? value.value() : Scalar{2};
    }

    bool isTriangle(const SurfaceFace &face)
    {
        return std::holds_alternative<Triangle>(face);
    }
}

int main()
{
    using namespace boundary_mesh;

    MultiNormalTopology unaffected;
    unaffected.front.vertices = {
        {Point3{0, 0, 0}, 0}, {Point3{1, 0, 0}, 1},
        {Point3{1, 1, 0}, 2}, {Point3{0, 1, 0}, 3}};
    unaffected.front.faces = {Quad{{0, 1, 2, 3}}};
    unaffected.front.source_face_ids = {10};
    const auto unchanged = triangulateMultiNormalQuads(unaffected);
    if (!unchanged.hasValue() || unchanged.value().front.faces.size() != 1 ||
        !std::holds_alternative<Quad>(unchanged.value().front.faces[0]))
    {
        return 1;
    }

    MultiNormalTopology skewed = unaffected;
    skewed.front.vertices[0].multi_normal_branch = true;
    skewed.front.vertices[0].position = Point3{0, 0, 0};
    skewed.front.vertices[1].position = Point3{3, 0, 0};
    skewed.front.vertices[2].position = Point3{2, 1, 0};
    skewed.front.vertices[3].position = Point3{0, 2, 0};
    skewed.transition_face_origins = {{0, {10, 11}, {0, 1, 2, 3}}};
    const auto split = triangulateMultiNormalQuads(skewed);
    if (!split.hasValue() || split.value().front.faces.size() != 2 ||
        !isTriangle(split.value().front.faces[0]) ||
        !isTriangle(split.value().front.faces[1]) ||
        split.value().front.source_face_ids !=
            std::vector<SurfaceFaceId>{10, 10} ||
        split.value().transition_face_origins.size() != 2 ||
        split.value().transition_face_origins[0].transformed_face_index != 0 ||
        split.value().transition_face_origins[1].transformed_face_index != 1 ||
        split.value().transition_face_origins[1].source_face_ids !=
            std::vector<SurfaceFaceId>{10, 11})
    {
        return 2;
    }
    const Scalar chosen_score = std::max(
        triangleSkewness(
            split.value().front,
            std::get<Triangle>(split.value().front.faces[0])),
        triangleSkewness(
            split.value().front,
            std::get<Triangle>(split.value().front.faces[1])));
    const Scalar diagonal_02_score = std::max(
        triangleSkewness(skewed.front, Triangle{{0, 1, 2}}),
        triangleSkewness(skewed.front, Triangle{{0, 2, 3}}));
    const Scalar diagonal_13_score = std::max(
        triangleSkewness(skewed.front, Triangle{{0, 1, 3}}),
        triangleSkewness(skewed.front, Triangle{{1, 2, 3}}));
    if (std::abs(chosen_score -
            std::min(diagonal_02_score, diagonal_13_score)) > 1e-12)
    {
        return 3;
    }

    MultiNormalTopology tied;
    tied.front.vertices = {
        {Point3{1, 0, 0}, 0}, {Point3{0, 1, 0}, 1},
        {Point3{0, 0, 0}, 2}, {Point3{1, 1, 0}, 3}};
    tied.front.vertices[2].multi_normal_branch = true;
    tied.front.faces = {Quad{{2, 0, 3, 1}}};
    tied.front.source_face_ids = {20};
    const auto tie_result = triangulateMultiNormalQuads(tied);
    if (!tie_result.hasValue() || tie_result.value().front.faces.size() != 2 ||
        std::get<Triangle>(tie_result.value().front.faces[0]).vertex_ids !=
            std::array<VertexId, 3>{2, 0, 1} ||
        std::get<Triangle>(tie_result.value().front.faces[1]).vertex_ids !=
            std::array<VertexId, 3>{0, 3, 1})
    {
        return 4;
    }

    return 0;
}
