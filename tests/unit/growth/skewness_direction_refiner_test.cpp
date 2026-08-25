#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/skewness_direction_refiner.hpp>
#include <boundary_mesh/quality/volume_cell_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    GrowthFront makeTriangle()
    {
        GrowthFront front;
        front.layer = 0;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{100}},
            {Point3{1, 0, 0}, VertexId{101}},
            {Point3{0, 1, 0}, VertexId{102}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}}};
        front.source_face_ids = {SurfaceFaceId{200}};
        return front;
    }

    FrontEvaluation makeEvaluation(const GrowthFront &front)
    {
        FrontEvaluation evaluation;
        evaluation.layer = front.layer;
        evaluation.characteristic_length = Scalar{1};
        evaluation.effective_length_tolerance = Scalar{1e-12};
        evaluation.faces.push_back(FrontFaceEvaluation{
            0,
            front.source_face_ids[0],
            FaceEvaluation{
                Point3{Scalar{1} / Scalar{3}, Scalar{1} / Scalar{3}, 0},
                Vector3::UnitZ(),
                Vector3::UnitZ(),
                Scalar{0.5},
                Scalar{0}}});
        return evaluation;
    }

    Scalar incidentMaximum(
        const GrowthFront &front,
        const std::vector<Vector3> &directions,
        const std::vector<Scalar> &heights,
        std::size_t)
    {
        const auto &face = std::get<Triangle>(front.faces[0]);
        PrismPoints points;
        for (std::size_t local = 0; local < 3; ++local)
        {
            const std::size_t vertex =
                static_cast<std::size_t>(face.vertex_ids[local]);
            points[local] = front.vertices[vertex].position;
            points[local + 3] =
                front.vertices[vertex].position +
                heights[vertex] * directions[vertex];
        }
        const auto quality = evaluatePrism(points);
        return quality.hasValue()
            ? quality.value().skewness
            : std::numeric_limits<Scalar>::infinity();
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
}

int main()
{
    using namespace boundary_mesh;

    const GrowthFront front = makeTriangle();
    const auto adjacency = buildFrontAdjacency(front);
    if (!adjacency.hasValue()) return 10;
    const FrontEvaluation evaluation = makeEvaluation(front);

    const Scalar z = std::sqrt(Scalar{0.75});
    const std::vector<Vector3> baseline{
        Vector3{Scalar{0.5}, 0, z},
        Vector3::UnitZ(),
        Vector3::UnitZ()};
    const std::vector<Scalar> fixed_heights(3, Scalar{0.4});
    SkewnessNormalOptimizationOptions options;
    options.activation_skewness = Scalar{0};

    const auto refined = refineDirectionsForSkewness(
        front,
        evaluation,
        adjacency.value(),
        baseline,
        fixed_heights,
        options);

    if (refined.diagnostics.activated_vertices == 0 ||
        refined.diagnostics.updated_vertices == 0 ||
        !(refined.diagnostics.maximum_skewness_after <
          refined.diagnostics.maximum_skewness_before) ||
        !(incidentMaximum(front, refined.directions, fixed_heights, 0) <
          incidentMaximum(front, baseline, fixed_heights, 0)))
    {
        return 1;
    }

    SkewnessNormalOptimizationOptions inactive = options;
    inactive.activation_skewness = Scalar{1};
    const auto unchanged = refineDirectionsForSkewness(
        front,
        evaluation,
        adjacency.value(),
        baseline,
        fixed_heights,
        inactive);
    if (!sameDirections(unchanged.directions, baseline) ||
        unchanged.diagnostics.activated_vertices != 0)
    {
        return 2;
    }

    SkewnessNormalOptimizationOptions disabled = options;
    disabled.enabled = false;
    const auto bypassed = refineDirectionsForSkewness(
        front,
        evaluation,
        adjacency.value(),
        baseline,
        fixed_heights,
        disabled);
    if (!sameDirections(bypassed.directions, baseline)) return 3;

    return 0;
}
