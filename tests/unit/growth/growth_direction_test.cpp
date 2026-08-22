#include <cmath>
#include <cstddef>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/growth_direction.hpp>

namespace
{
    using namespace boundary_mesh;

    constexpr Scalar pi = 3.14159265358979323846;

    Vector3 rotatedNormal(Scalar degrees)
    {
        const Scalar radians = degrees * pi / Scalar{180};
        return Vector3{std::sin(radians), Scalar{0}, std::cos(radians)};
    }

    FrontEvaluation makeEvaluation(
        const GrowthFront &front,
        const std::vector<Vector3> &normals)
    {
        FrontEvaluation evaluation;
        evaluation.layer = front.layer;
        evaluation.characteristic_length = Scalar{2};
        evaluation.effective_length_tolerance = Scalar{1e-12};
        for (std::size_t index = 0; index < normals.size(); ++index)
        {
            evaluation.faces.push_back(FrontFaceEvaluation{
                index,
                front.source_face_ids[index],
                FaceEvaluation{
                    Point3::Zero(),
                    normals[index],
                    normals[index],
                    Scalar{1},
                    Scalar{0}}});
        }
        return evaluation;
    }

    GrowthFront makeMixedPlanarFront()
    {
        GrowthFront front;
        front.layer = 0;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{30}},
            {Point3{1, 0, 0}, VertexId{31}},
            {Point3{0, 1, 0}, VertexId{32}},
            {Point3{-1, 0, 0}, VertexId{33}},
            {Point3{0, -1, 0}, VertexId{34}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Quad{{VertexId{0}, VertexId{2}, VertexId{4}, VertexId{3}}}};
        front.source_face_ids = {SurfaceFaceId{40}, SurfaceFaceId{41}};
        return front;
    }

    GrowthFront makeFourFaceFan()
    {
        GrowthFront front;
        front.layer = 2;
        front.vertices = {
            {Point3{0, 0, 0}, VertexId{50}},
            {Point3{1, 0, 0}, VertexId{51}},
            {Point3{0, 1, 0}, VertexId{52}},
            {Point3{-1, 0, 0}, VertexId{53}},
            {Point3{0, -1, 0}, VertexId{54}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Triangle{{VertexId{0}, VertexId{2}, VertexId{3}}},
            Triangle{{VertexId{0}, VertexId{3}, VertexId{4}}},
            Triangle{{VertexId{0}, VertexId{4}, VertexId{1}}}};
        front.source_face_ids = {
            SurfaceFaceId{60}, SurfaceFaceId{61},
            SurfaceFaceId{62}, SurfaceFaceId{63}};
        return front;
    }
}

int main()
{
    using namespace boundary_mesh;

    GrowthFront planar = makeMixedPlanarFront();
    const auto planar_adjacency = buildFrontAdjacency(planar);
    if (!planar_adjacency.hasValue()) return 1;
    const FrontEvaluation planar_evaluation = makeEvaluation(
        planar,
        {Vector3::UnitZ(), Vector3::UnitZ()});
    const auto planar_result = computeGrowthDirections(
        planar,
        planar_evaluation,
        planar_adjacency.value());
    if (!planar_result.hasValue() ||
        planar_result.value().vertices.size() != planar.vertices.size() ||
        (planar_result.value().vertices[0].value - Vector3::UnitZ()).norm() >
            Scalar{1e-12} ||
        planar_result.value().vertices[0].complex_corner)
    {
        return 2;
    }

    planar.layer = 1;
    planar.vertices[0].direction = Vector3{0.1, 0, std::sqrt(0.99)};
    const Vector3 previous = planar.vertices[0].direction.normalized();
    FrontEvaluation previous_evaluation = planar_evaluation;
    previous_evaluation.layer = planar.layer;
    const auto previous_result = computeGrowthDirections(
        planar,
        previous_evaluation,
        planar_adjacency.value());
    if (!previous_result.hasValue() ||
        (previous_result.value().vertices[0].value - previous).norm() >
            Scalar{1e-12})
    {
        return 3;
    }

    const GrowthFront fan = makeFourFaceFan();
    const auto fan_adjacency = buildFrontAdjacency(fan);
    if (!fan_adjacency.hasValue()) return 4;
    const Vector3 separated = rotatedNormal(50);
    const FrontEvaluation grouped_evaluation = makeEvaluation(
        fan,
        {Vector3::UnitZ(), Vector3::UnitZ(),
         Vector3::UnitZ(), separated});
    const auto grouped_result = computeGrowthDirections(
        fan,
        grouped_evaluation,
        fan_adjacency.value());
    const Vector3 grouped_expected =
        (Vector3::UnitZ() + separated).normalized();
    if (!grouped_result.hasValue() ||
        (grouped_result.value().vertices[0].value - grouped_expected).norm() >
            Scalar{1e-12} ||
        grouped_result.value().vertices[0].visibility_cosine <=
            std::cos(Scalar{30} * pi / Scalar{180}))
    {
        return 5;
    }

    const GrowthFront opposed = makeMixedPlanarFront();
    const auto opposed_adjacency = buildFrontAdjacency(opposed);
    if (!opposed_adjacency.hasValue()) return 6;
    const FrontEvaluation opposed_evaluation = makeEvaluation(
        opposed,
        {Vector3::UnitZ(), -Vector3::UnitZ()});
    const auto opposed_result = computeGrowthDirections(
        opposed,
        opposed_evaluation,
        opposed_adjacency.value());
    if (!opposed_result.hasValue() ||
        !opposed_result.value().vertices[0].complex_corner ||
        opposed_result.value().vertices[0].visibility_cosine != Scalar{-1} ||
        !opposed_result.value().vertices[0].value.allFinite())
    {
        return 7;
    }

    FrontEvaluation wrong = planar_evaluation;
    wrong.layer = 9;
    const auto mismatch = computeGrowthDirections(
        planar,
        wrong,
        planar_adjacency.value());
    if (mismatch.hasValue() ||
        !std::holds_alternative<DirectionInputMismatch>(mismatch.error()))
    {
        return 8;
    }

    return 0;
}
