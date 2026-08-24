#include <cmath>
#include <vector>

#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/isotropic_stop_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    GrowthFront regularMixedFront()
    {
        const Scalar triangle_height = std::sqrt(Scalar{3}) / Scalar{2};
        GrowthFront front;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{0}},
            {Point3{1, 0, 0}, VertexId{1}},
            {Point3{0.5, triangle_height, 0}, VertexId{2}},
            {Point3{3, 0, 0}, VertexId{3}},
            {Point3{4, 0, 0}, VertexId{4}},
            {Point3{4, 1, 0}, VertexId{5}},
            {Point3{3, 1, 0}, VertexId{6}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Quad{{VertexId{3}, VertexId{4}, VertexId{5}, VertexId{6}}}};
        front.source_face_ids = {SurfaceFaceId{10}, SurfaceFaceId{20}};
        return front;
    }

    GrowthFront displaced(const GrowthFront &current, Scalar height)
    {
        GrowthFront candidate = current;
        candidate.layer = current.layer + 1;
        for (GrowthFrontVertex &vertex : candidate.vertices)
        {
            vertex.position.z() += height;
            vertex.actual_height = height;
        }
        return candidate;
    }

    GrowthFront guardFront(
        Scalar short_edge,
        Scalar second_edge,
        Scalar third_edge)
    {
        const Scalar x =
            (short_edge * short_edge + third_edge * third_edge -
             second_edge * second_edge) /
            (Scalar{2} * short_edge);
        const Scalar y = std::sqrt(
            third_edge * third_edge - x * x);
        GrowthFront front;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{0}},
            {Point3{short_edge, 0, 0}, VertexId{1}},
            {Point3{x, y, 0}, VertexId{2}},
            {Point3{10, 0, 0}, VertexId{3}},
            {Point3{5, Scalar{5} * std::sqrt(Scalar{3}), 0}, VertexId{4}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Triangle{{VertexId{0}, VertexId{3}, VertexId{4}}}};
        front.source_face_ids = {SurfaceFaceId{30}, SurfaceFaceId{31}};
        return front;
    }

    GrowthFront quadrilateralGuardFront()
    {
        GrowthFront front;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{0}},
            {Point3{0.5, 0, 0}, VertexId{1}},
            {Point3{0.5, 1, 0}, VertexId{2}},
            {Point3{0, 1, 0}, VertexId{3}},
            {Point3{10, 0, 0}, VertexId{4}},
            {Point3{10, 10, 0}, VertexId{5}},
            {Point3{0, 10, 0}, VertexId{6}}};
        front.faces = {
            Quad{{VertexId{0}, VertexId{1}, VertexId{2}, VertexId{3}}},
            Quad{{VertexId{0}, VertexId{4}, VertexId{5}, VertexId{6}}}};
        front.source_face_ids = {SurfaceFaceId{50}, SurfaceFaceId{51}};
        return front;
    }

    IsotropicStopEvaluation evaluate(
        const GrowthFront &current,
        const GrowthFront &candidate)
    {
        const auto adjacency = buildFrontAdjacency(current);
        if (!adjacency.hasValue()) return {};
        const auto result = IsotropicStopEvaluator{}.evaluate(
            current, candidate, adjacency.value(), Scalar{1});
        return result.hasValue()
            ? result.value()
            : IsotropicStopEvaluation{};
    }
}

int main()
{
    using namespace boundary_mesh;

    const GrowthFront current = regularMixedFront();
    const GrowthFront candidate = displaced(current, Scalar{1.1});
    const auto adjacency = buildFrontAdjacency(current);
    if (!adjacency.hasValue()) return 1;

    const auto evaluation = IsotropicStopEvaluator{}.evaluate(
        current, candidate, adjacency.value(), Scalar{1});
    if (!evaluation.hasValue()) return 2;
    if (evaluation.value().face_stops !=
        std::vector<bool>({true, true}))
    {
        return 3;
    }

    const IsotropicStopEvaluation continuing = evaluate(
        current, displaced(current, Scalar{0.5}));
    if (continuing.face_stops != std::vector<bool>({false, false}))
    {
        return 4;
    }

    const GrowthFront geometric_current = guardFront(
        Scalar{1}, Scalar{1}, Scalar{1});
    const IsotropicStopEvaluation geometric = evaluate(
        geometric_current,
        displaced(geometric_current, Scalar{1.4}));
    if (geometric.vertex_candidates.size() != 5 ||
        !geometric.vertex_candidates[0])
    {
        return 5;
    }

    const GrowthFront minimum_current = guardFront(
        Scalar{0.5}, Scalar{1}, Scalar{1});
    const Scalar minimum_geometric = std::pow(Scalar{0.5}, Scalar{1} / 3);
    if (!(Scalar{1} <= Scalar{1.3} * minimum_geometric) ||
        !(Scalar{1} > Scalar{1.8} * Scalar{0.5}))
    {
        return 6;
    }
    const IsotropicStopEvaluation minimum = evaluate(
        minimum_current,
        displaced(minimum_current, Scalar{1}));
    if (minimum.vertex_candidates.size() != 5 ||
        !minimum.vertex_candidates[0])
    {
        return 7;
    }

    const GrowthFront quad_minimum_current = quadrilateralGuardFront();
    const Scalar quad_height = Scalar{0.91};
    const Scalar quad_geometric = std::pow(Scalar{0.25}, Scalar{0.25});
    if (!(quad_height <= Scalar{1.3} * quad_geometric) ||
        !(quad_height > Scalar{1.8} * Scalar{0.5}))
    {
        return 10;
    }
    const IsotropicStopEvaluation quad_minimum = evaluate(
        quad_minimum_current,
        displaced(quad_minimum_current, quad_height));
    if (quad_minimum.vertex_candidates.size() != 7 ||
        !quad_minimum.vertex_candidates[0])
    {
        return 11;
    }

    GrowthFront consensus_current;
    consensus_current.vertices = {
        {Point3{0, 0, 0}, VertexId{0}},
        {Point3{1, 0, 0}, VertexId{1}},
        {Point3{0.5, std::sqrt(Scalar{3}) / Scalar{2}, 0}, VertexId{2}}};
    consensus_current.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}}};
    consensus_current.source_face_ids = {SurfaceFaceId{40}};
    GrowthFront consensus_candidate = displaced(
        consensus_current, Scalar{0.5});
    consensus_candidate.vertices[0].position.z() = Scalar{1.1};
    const IsotropicStopEvaluation consensus = evaluate(
        consensus_current, consensus_candidate);
    if (consensus.vertex_candidates.size() != 3 ||
        !consensus.vertex_candidates[0] ||
        consensus.vertex_stops[0] ||
        consensus.face_stops[0])
    {
        return 8;
    }

    GrowthFront mismatched = candidate;
    mismatched.vertices.push_back(
        GrowthFrontVertex{Point3{9, 9, 9}, VertexId{99}});
    const auto invalid = IsotropicStopEvaluator{}.evaluate(
        current, mismatched, adjacency.value(), Scalar{1});
    if (invalid.hasValue() || invalid.error().layer != mismatched.layer)
    {
        return 9;
    }
    return 0;
}
