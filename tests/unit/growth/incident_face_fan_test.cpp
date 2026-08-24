#include <variant>

#include <boundary_mesh/growth/front_evaluator.hpp>
#include <boundary_mesh/growth/incident_face_fan.hpp>

namespace
{
    using namespace boundary_mesh;

    GrowthFront mixedClosedFan()
    {
        GrowthFront front;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{0}},
            {Point3{1, -1, 0}, VertexId{1}},
            {Point3{0, 1, 0}, VertexId{2}},
            {Point3{-1, 1, 0}, VertexId{3}},
            {Point3{-1, -1, 0}, VertexId{4}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Quad{{VertexId{0}, VertexId{2}, VertexId{3}, VertexId{4}}},
            Triangle{{VertexId{0}, VertexId{4}, VertexId{1}}}};
        front.source_face_ids = {10, 11, 12};
        return front;
    }
}

int main()
{
    using namespace boundary_mesh;

    const GrowthFront mixed = mixedClosedFan();
    const auto evaluation = FrontEvaluator{}.evaluate(mixed);
    if (!evaluation.hasValue()) return 1;
    const auto result = buildIncidentFaceFans(mixed, evaluation.value());
    if (!result.hasValue() || result.value().size() != mixed.vertices.size())
    {
        return 2;
    }

    const IncidentFaceFan &center = result.value()[0];
    if (!center.closed || center.sectors.size() != 3 ||
        center.sectors[0].face_index != 0 ||
        center.sectors[1].face_index != 2 ||
        center.sectors[2].face_index != 1)
    {
        return 3;
    }
    if (center.sectors[2].previous_vertex != VertexId{4} ||
        center.sectors[2].next_vertex != VertexId{2})
    {
        return 4; // Quad uses its perimeter neighbors, not vertex 3 diagonal.
    }

    GrowthFront disconnected;
    disconnected.vertices = mixed.vertices;
    disconnected.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
        Triangle{{VertexId{0}, VertexId{3}, VertexId{4}}}};
    disconnected.source_face_ids = {20, 21};
    const auto disconnected_evaluation = FrontEvaluator{}.evaluate(disconnected);
    if (!disconnected_evaluation.hasValue()) return 5;
    const auto disconnected_result = buildIncidentFaceFans(
        disconnected, disconnected_evaluation.value());
    if (disconnected_result.hasValue() ||
        !std::holds_alternative<DisconnectedIncidentFan>(
            disconnected_result.error()))
    {
        return 6;
    }

    GrowthFront inconsistent;
    inconsistent.vertices = mixed.vertices;
    inconsistent.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
        Triangle{{VertexId{0}, VertexId{3}, VertexId{2}}}};
    inconsistent.source_face_ids = {30, 31};
    const auto inconsistent_evaluation = FrontEvaluator{}.evaluate(inconsistent);
    if (!inconsistent_evaluation.hasValue()) return 7;
    const auto inconsistent_result = buildIncidentFaceFans(
        inconsistent, inconsistent_evaluation.value());
    if (inconsistent_result.hasValue() ||
        !std::holds_alternative<InconsistentIncidentFanWinding>(
            inconsistent_result.error()))
    {
        return 8;
    }

    return 0;
}
