#include <cmath>
#include <limits>
#include <variant>

#include <boundary_mesh/growth/front_evaluator.hpp>

namespace
{
    using namespace boundary_mesh;

    GrowthFront makeFront()
    {
        GrowthFront front;
        front.layer = 4;
        front.vertices = {
            Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{1, 1, 0},
            Point3{0, 1, 0}, Point3{2, 0, 0}, Point3{2, 1, 0}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Quad{{VertexId{1}, VertexId{4}, VertexId{5}, VertexId{2}}}};
        front.source_vertex_ids = {
            VertexId{10}, VertexId{11}, VertexId{12},
            VertexId{13}, VertexId{14}, VertexId{15}};
        front.source_face_ids = {SurfaceFaceId{20}, SurfaceFaceId{21}};
        front.vertex_boundaries.resize(front.vertices.size());
        return front;
    }
}

int main()
{
    using namespace boundary_mesh;
    const GrowthFront front = makeFront();
    const auto result = FrontEvaluator{}.evaluate(front);
    if (!result.hasValue() || result.value().layer != 4 ||
        result.value().faces.size() != 2 ||
        std::abs(result.value().faces[0].value.area - 0.5) > 1e-12 ||
        std::abs(result.value().faces[1].value.area - 1.0) > 1e-12 ||
        result.value().faces[0].source_face_id != SurfaceFaceId{20} ||
        result.value().faces[1].source_face_id != SurfaceFaceId{21})
    {
        return 1;
    }

    GrowthFront non_finite = front;
    non_finite.vertices[3].z() =
        std::numeric_limits<Scalar>::quiet_NaN();
    const auto non_finite_result = FrontEvaluator{}.evaluate(non_finite);
    const auto *vertex_error = non_finite_result.hasValue() ? nullptr :
        std::get_if<NonFiniteFrontVertex>(&non_finite_result.error());
    if (vertex_error == nullptr || vertex_error->front_vertex_index != 3 ||
        vertex_error->source_vertex_id != VertexId{13} ||
        vertex_error->layer != 4)
    {
        return 2;
    }

    GrowthFront invalid_reference = front;
    std::get<Triangle>(invalid_reference.faces[0]).vertex_ids[2] = VertexId{99};
    const auto reference_result = FrontEvaluator{}.evaluate(invalid_reference);
    if (reference_result.hasValue() ||
        !std::holds_alternative<InvalidFrontVertexReference>(
            reference_result.error()))
    {
        return 3;
    }

    GrowthFront degenerate = front;
    degenerate.vertices[2] = degenerate.vertices[1];
    const auto degenerate_result = FrontEvaluator{}.evaluate(degenerate);
    const auto *face_error = degenerate_result.hasValue() ? nullptr :
        std::get_if<DegenerateFrontFace>(&degenerate_result.error());
    if (face_error == nullptr || face_error->front_face_index != 0 ||
        face_error->source_face_id != SurfaceFaceId{20} ||
        face_error->layer != 4)
    {
        return 4;
    }

    GrowthFront mapping = front;
    mapping.vertex_boundaries.pop_back();
    const auto mapping_result = FrontEvaluator{}.evaluate(mapping);
    if (mapping_result.hasValue() ||
        !std::holds_alternative<FrontMappingMismatch>(mapping_result.error()))
    {
        return 5;
    }

    const auto bad_options = FrontEvaluator{}.evaluate(
        front,
        SurfaceEvaluationOptions{Scalar{0}});
    if (bad_options.hasValue() ||
        !std::holds_alternative<InvalidSurfaceEvaluationOptions>(
            bad_options.error()))
    {
        return 6;
    }

    GrowthFront empty;
    empty.layer = 7;
    const auto empty_result = FrontEvaluator{}.evaluate(empty);
    if (empty_result.hasValue() ||
        !std::holds_alternative<EmptyGrowthFront>(empty_result.error()))
    {
        return 7;
    }
    return 0;
}
