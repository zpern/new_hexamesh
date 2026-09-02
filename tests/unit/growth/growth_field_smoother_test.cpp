#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/growth_field_smoother.hpp>
#include <boundary_mesh/growth/sliding_constraint_builder.hpp>
#include <boundary_mesh/growth/sliding_surface_builder.hpp>

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
        const std::vector<Scalar> &reference_heights,
        const std::vector<Scalar> &provisional_heights)
    {
        Scalar predicted = Scalar{0};
        for (const std::size_t neighbor :
             adjacency.vertex_neighbors[vertex_index])
        {
            predicted +=
                ((front.vertices[neighbor].position +
                  provisional_heights[neighbor] *
                      directions[neighbor]) -
                 front.vertices[vertex_index].position)
                    .dot(directions[vertex_index]);
        }
        predicted /= static_cast<Scalar>(
            adjacency.vertex_neighbors[vertex_index].size());
        const Scalar relative =
            (predicted - reference_heights[vertex_index]) /
            reference_heights[vertex_index];
        const Scalar correction =
            Scalar{1} /
                (Scalar{1} + std::exp(Scalar{-0.5} * relative)) -
            Scalar{0.5};
        return std::clamp(
            reference_heights[vertex_index] *
                (Scalar{1} + correction),
            Scalar{0.5} * reference_heights[vertex_index],
            Scalar{1.5} * reference_heights[vertex_index]);
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
    const std::vector<Scalar> reference_heights{
        Scalar{0.10}, Scalar{0.08}, Scalar{0.12},
        Scalar{0.09}, Scalar{0.11}};
    const std::vector<Scalar> provisional_heights{
        Scalar{0.15}, Scalar{0.12}, Scalar{0.18},
        Scalar{0.135}, Scalar{0.165}};

    const auto result = GrowthFieldSmoother{}.smooth(
        front,
        evaluation,
        adjacency.value(),
        raw,
        reference_heights,
        provisional_heights);
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
            reference_heights,
            provisional_heights);
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
        reference_heights,
        provisional_heights);
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
        reference_heights,
        provisional_heights);
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
        reference_heights,
        provisional_heights);
    if (nonfinite_result.hasValue() ||
        std::get_if<NonFiniteGrowthFieldInput>(
            &nonfinite_result.error()) == nullptr)
    {
        return 9;
    }

    std::vector<Scalar> missing_provisional =
        provisional_heights;
    missing_provisional.pop_back();
    const auto mismatch_result = GrowthFieldSmoother{}.smooth(
        front,
        evaluation,
        adjacency.value(),
        raw,
        reference_heights,
        missing_provisional);
    if (mismatch_result.hasValue() ||
        std::get_if<GrowthFieldInputMismatch>(
            &mismatch_result.error()) == nullptr)
    {
        return 11;
    }

    std::vector<Scalar> invalid_provisional =
        provisional_heights;
    invalid_provisional[0] = Scalar{0};
    const auto invalid_height_result =
        GrowthFieldSmoother{}.smooth(
            front,
            evaluation,
            adjacency.value(),
            raw,
            reference_heights,
            invalid_provisional);
    if (invalid_height_result.hasValue() ||
        std::get_if<InvalidGrowthFieldBaseHeight>(
            &invalid_height_result.error()) == nullptr)
    {
        return 12;
    }

    GrowthFieldSmoothingOptions disabled;
    disabled.skewness.enabled = false;
    const auto baseline_result = GrowthFieldSmoother{}.smooth(
        front,
        evaluation,
        adjacency.value(),
        raw,
        reference_heights,
        provisional_heights,
        disabled);

    GrowthFieldSmoothingOptions enabled;
    enabled.skewness.activation_skewness = Scalar{0};
    const auto refined_result = GrowthFieldSmoother{}.smooth(
        front,
        evaluation,
        adjacency.value(),
        raw,
        reference_heights,
        provisional_heights,
        enabled);
    if (!baseline_result.hasValue() || !refined_result.hasValue())
    {
        return 13;
    }
    if (baseline_result.value().actual_heights !=
        refined_result.value().actual_heights)
    {
        return 14;
    }
    if (baseline_result.value().diagnostics.activated_vertices != 0 ||
        baseline_result.value().diagnostics.updated_vertices != 0 ||
        refined_result.value().diagnostics.activated_vertices == 0)
    {
        return 15;
    }

    GrowthFront sliding_front = front;
    sliding_front.vertices[0].boundary.sliding_region_ids = {70};
    SurfaceMesh sliding_mesh;
    sliding_mesh.vertices = {
        Point3{-2, 0, -2}, Point3{2, 0, -2},
        Point3{2, 0, 2}, Point3{-2, 0, 2}};
    sliding_mesh.faces = {
        Triangle{{0, 1, 2}}, Triangle{{0, 2, 3}}};
    sliding_mesh.face_tags = {
        {SurfaceBoundaryKind::Internal, 70},
        {SurfaceBoundaryKind::Internal, 70}};
    const auto sliding_surfaces =
        SlidingSurfaceBuilder{}.build(sliding_mesh);
    if (!sliding_surfaces.hasValue()) return 17;
    const auto sliding_constraints = SlidingConstraintBuilder{}.build(
        sliding_surfaces.value(), sliding_front, evaluation);
    if (!sliding_constraints.hasValue()) return 18;

    GrowthDirections sliding_raw = raw;
    sliding_raw.vertices[0].value = Vector3{0, 0.6, 0.8};
    const auto iterative_sliding = GrowthFieldSmoother{}.smooth(
        sliding_front,
        evaluation,
        adjacency.value(),
        sliding_raw,
        reference_heights,
        provisional_heights,
        {},
        &sliding_constraints.value());
    if (!iterative_sliding.hasValue()) return 19;
    const auto center_reconstrained =
        sliding_constraints.value().constrainDirection(
            0,
            sliding_front.vertices[0].position,
            iterative_sliding.value().directions[0]);
    if (!center_reconstrained.hasValue() ||
        (center_reconstrained.value() -
         iterative_sliding.value().directions[0]).norm() > Scalar{1e-12})
        return 20;

    const auto post_only = GrowthFieldSmoother{}.smooth(
        sliding_front,
        evaluation,
        adjacency.value(),
        sliding_raw,
        reference_heights,
        provisional_heights);
    if (!post_only.hasValue() ||
        (post_only.value().directions[1] -
         iterative_sliding.value().directions[1]).norm() <= Scalar{1e-6})
        return 21;

    GrowthDirections normal_only = raw;
    normal_only.vertices[0].value = Vector3::UnitY();
    const auto constraint_failure = GrowthFieldSmoother{}.smooth(
        sliding_front,
        evaluation,
        adjacency.value(),
        normal_only,
        reference_heights,
        provisional_heights,
        {},
        &sliding_constraints.value());
    const auto *failure = constraint_failure.hasValue()
        ? nullptr
        : std::get_if<SlidingGrowthFieldConstraintFailure>(
              &constraint_failure.error());
    if (failure == nullptr ||
        failure->front_vertex_index != 0 ||
        failure->source_vertex_id != VertexId{100} ||
        failure->layer != sliding_front.layer ||
        std::get_if<UndefinedConstrainedDirection>(
            &failure->cause) == nullptr)
        return 22;

    GrowthFieldSmoothingOptions invalid = enabled;
    invalid.skewness.activation_skewness = Scalar{1.1};
    const auto invalid_options_result = GrowthFieldSmoother{}.smooth(
        front,
        evaluation,
        adjacency.value(),
        raw,
        reference_heights,
        provisional_heights,
        invalid);
    if (invalid_options_result.hasValue() ||
        std::get_if<InvalidSkewnessNormalOptimizationOptions>(
            &invalid_options_result.error()) == nullptr)
    {
        return 16;
    }

    return 0;
}
