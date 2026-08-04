#include <variant>

#include <boundary_mesh/growth/front_evaluator.hpp>
#include <boundary_mesh/growth/growth_direction.hpp>

int main()
{
    using namespace boundary_mesh;
    GrowthFront front;
    front.layer = 2;
    front.vertices = {
        Point3{0, 0, 0}, Point3{1, 0, 0},
        Point3{0, 1, 0}, Point3{0, 0, 1}};
    front.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
        Triangle{{VertexId{0}, VertexId{3}, VertexId{1}}}};
    front.source_vertex_ids = {
        VertexId{30}, VertexId{31}, VertexId{32}, VertexId{33}};
    front.source_face_ids = {SurfaceFaceId{40}, SurfaceFaceId{41}};
    front.vertex_boundaries.resize(front.vertices.size());

    const auto evaluation = FrontEvaluator{}.evaluate(front);
    if (!evaluation.hasValue()) return 1;
    const auto directions = computeGrowthDirections(front, evaluation.value());
    if (!directions.hasValue() || directions.value().values.size() != 4 ||
        directions.value().source_vertex_ids != front.source_vertex_ids)
    {
        return 2;
    }

    const Vector3 expected = Vector3{0, 1, 1}.normalized();
    if ((directions.value().values[0] - expected).norm() > 1e-12 ||
        (directions.value().values[2] - Vector3{0, 0, 1}).norm() > 1e-12 ||
        (directions.value().values[3] - Vector3{0, 1, 0}).norm() > 1e-12)
    {
        return 3;
    }

    FrontEvaluation wrong = evaluation.value();
    wrong.layer = 3;
    const auto mismatch = computeGrowthDirections(front, wrong);
    if (mismatch.hasValue() ||
        !std::holds_alternative<DirectionInputMismatch>(mismatch.error()))
    {
        return 4;
    }

    GrowthFront cancelled;
    cancelled.layer = 5;
    cancelled.vertices = {
        Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{0, 1, 0}};
    cancelled.faces = {
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
        Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}}};
    cancelled.source_vertex_ids = {VertexId{50}, VertexId{51}, VertexId{52}};
    cancelled.source_face_ids = {SurfaceFaceId{60}, SurfaceFaceId{61}};
    cancelled.vertex_boundaries.resize(3);
    const auto cancelled_evaluation = FrontEvaluator{}.evaluate(cancelled);
    if (!cancelled_evaluation.hasValue()) return 5;
    const auto cancelled_direction = computeGrowthDirections(
        cancelled, cancelled_evaluation.value());
    const auto *undefined = cancelled_direction.hasValue() ? nullptr :
        std::get_if<UndefinedGrowthDirection>(&cancelled_direction.error());
    if (undefined == nullptr || undefined->front_vertex_index != 0 ||
        undefined->source_vertex_id != VertexId{50} || undefined->layer != 5)
    {
        return 6;
    }
    return 0;
}
