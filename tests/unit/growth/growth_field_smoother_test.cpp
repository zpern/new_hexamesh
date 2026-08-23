#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/growth_field_smoother.hpp>

namespace
{
    using namespace boundary_mesh;

    GrowthFront makeFan()
    {
        GrowthFront front;
        front.layer = 0;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{100}},
            {Point3{1, 0, 0.2}, VertexId{101}},
            {Point3{0, 1, -0.2}, VertexId{102}},
            {Point3{-1, 0, 0.2}, VertexId{103}},
            {Point3{0, -1, -0.2}, VertexId{104}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Triangle{{VertexId{0}, VertexId{2}, VertexId{3}}},
            Triangle{{VertexId{0}, VertexId{3}, VertexId{4}}},
            Triangle{{VertexId{0}, VertexId{4}, VertexId{1}}}};
        front.source_face_ids = {
            SurfaceFaceId{200}, SurfaceFaceId{201},
            SurfaceFaceId{202}, SurfaceFaceId{203}};
        return front;
    }

    FrontEvaluation makeEvaluation(const GrowthFront &front)
    {
        FrontEvaluation evaluation;
        evaluation.layer = front.layer;
        evaluation.characteristic_length = Scalar{2};
        evaluation.effective_length_tolerance = Scalar{1e-12};
        for (std::size_t index = 0; index < front.faces.size(); ++index)
        {
            evaluation.faces.push_back(FrontFaceEvaluation{
                index,
                front.source_face_ids[index],
                FaceEvaluation{
                    Point3::Zero(),
                    Vector3::UnitZ(),
                    Vector3::UnitZ(),
                    Scalar{1},
                    Scalar{0}}});
        }
        return evaluation;
    }

    GrowthDirections makeDirections(std::uint32_t layer)
    {
        const Scalar z = std::sqrt(Scalar{0.96});
        GrowthDirections directions;
        directions.layer = layer;
        directions.vertices = {
            {Vector3::UnitZ(), Scalar{1}, false},
            {Vector3{0.2, 0, z}, Scalar{1}, false},
            {Vector3{0, 0.2, z}, Scalar{1}, false},
            {Vector3{-0.2, 0, z}, Scalar{1}, false},
            {Vector3{0, -0.2, z}, Scalar{1}, false}};
        return directions;
    }

    bool sameDirections(
        const std::vector<Vector3> &first,
        const std::vector<Vector3> &second)
    {
        if (first.size() != second.size()) return false;
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            if ((first[index] - second[index]).norm() > Scalar{1e-12})
            {
                return false;
            }
        }
        return true;
    }

    Scalar expectedHeight(
        std::size_t vertex_index,
        const GrowthFront &front,
        const FrontAdjacency &adjacency,
        const std::vector<Vector3> &directions,
        const std::vector<Scalar> &base_heights)
    {
        Scalar predicted = Scalar{0};
        for (const std::size_t neighbor :
             adjacency.vertex_neighbors[vertex_index])
        {
            predicted +=
                ((front.vertices[neighbor].position +
                  base_heights[neighbor] * directions[neighbor]) -
                 front.vertices[vertex_index].position)
                    .dot(directions[vertex_index]);
        }
        predicted /= static_cast<Scalar>(
            adjacency.vertex_neighbors[vertex_index].size());
        const Scalar relative =
            (predicted - base_heights[vertex_index]) /
            base_heights[vertex_index];
        const Scalar correction =
            Scalar{1} /
                (Scalar{1} + std::exp(Scalar{-0.5} * relative)) -
            Scalar{0.5};
        return std::clamp(
            base_heights[vertex_index] /
                (Scalar{1} + correction),
            Scalar{0.5} * base_heights[vertex_index],
            Scalar{1.5} * base_heights[vertex_index]);
    }
}

int main()
{
    using namespace boundary_mesh;

    const GrowthFront front = makeFan();
    const auto adjacency = buildFrontAdjacency(front);
    if (!adjacency.hasValue()) return 1;
    const FrontEvaluation evaluation = makeEvaluation(front);
    const GrowthDirections raw = makeDirections(front.layer);
    const std::vector<Scalar> base_heights{
        Scalar{0.10}, Scalar{0.08}, Scalar{0.12},
        Scalar{0.09}, Scalar{0.11}};

    const auto result = GrowthFieldSmoother{}.smooth(
        front,
        evaluation,
        adjacency.value(),
        raw,
        base_heights);
    if (!result.hasValue() ||
        result.value().layer != front.layer ||
        result.value().directions.size() != front.vertices.size() ||
        result.value().actual_heights.size() != front.vertices.size())
    {
        return 2;
    }
    if ((result.value().directions[0] - Vector3::UnitZ()).norm() >
        Scalar{1e-12})
    {
        return 3;
    }
    for (const Vector3 &direction : result.value().directions)
    {
        if (!direction.allFinite() ||
            std::abs(direction.norm() - Scalar{1}) > Scalar{1e-12})
        {
            return 4;
        }
    }
    for (std::size_t index = 0;
         index < front.vertices.size();
         ++index)
    {
        const Scalar expected = expectedHeight(
            index,
            front,
            adjacency.value(),
            result.value().directions,
            base_heights);
        if (std::abs(
                result.value().actual_heights[index] - expected) >
            Scalar{1e-12})
        {
            return 10;
        }
    }

    GrowthFront permuted = front;
    std::swap(permuted.faces[0], permuted.faces[2]);
    std::swap(
        permuted.source_face_ids[0],
        permuted.source_face_ids[2]);
    const auto permuted_adjacency = buildFrontAdjacency(permuted);
    if (!permuted_adjacency.hasValue()) return 5;
    const auto permuted_result = GrowthFieldSmoother{}.smooth(
        permuted,
        makeEvaluation(permuted),
        permuted_adjacency.value(),
        raw,
        base_heights);
    if (!permuted_result.hasValue() ||
        !sameDirections(
            result.value().directions,
            permuted_result.value().directions))
    {
        return 6;
    }

    GrowthFront coincident = front;
    coincident.vertices[1].position =
        coincident.vertices[0].position;
    const auto coincident_adjacency =
        buildFrontAdjacency(coincident);
    if (!coincident_adjacency.hasValue()) return 7;
    const auto coincident_result = GrowthFieldSmoother{}.smooth(
        coincident,
        evaluation,
        coincident_adjacency.value(),
        raw,
        base_heights);
    if (coincident_result.hasValue() ||
        std::get_if<DegenerateGrowthFieldNeighbor>(
            &coincident_result.error()) == nullptr)
    {
        return 8;
    }

    GrowthDirections nonfinite = raw;
    nonfinite.vertices[0].value.x() =
        std::numeric_limits<Scalar>::quiet_NaN();
    const auto nonfinite_result = GrowthFieldSmoother{}.smooth(
        front,
        evaluation,
        adjacency.value(),
        nonfinite,
        base_heights);
    if (nonfinite_result.hasValue() ||
        std::get_if<NonFiniteGrowthFieldInput>(
            &nonfinite_result.error()) == nullptr)
    {
        return 9;
    }

    return 0;
}
