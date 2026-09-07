#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/sliding_constraint_builder.hpp>
#include <boundary_mesh/growth/sliding_surface_builder.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceMesh makeSymmetryMesh()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0},
            Point3{0.0, 0.0, 1.0}, Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, -1.0}, Point3{-1.0, 0.0, 0.0}};
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{0}, VertexId{3}, VertexId{2}}},
            Triangle{{VertexId{0}, VertexId{1}, VertexId{3}}},
            Triangle{{VertexId{0}, VertexId{5}, VertexId{4}}}};
        mesh.face_tags = {
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 7},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 8},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 9},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 10}};
        return mesh;
    }

    GrowthFront makeFront(
        std::vector<std::uint32_t> first_regions)
    {
        GrowthFront front;
        front.layer = 2;
        front.vertices = {
            {Point3{0.0, 0.0, 0.0}, VertexId{20},
             FrontVertexBoundary{std::move(first_regions)}},
            {Point3{1.0, 0.0, 0.0}, VertexId{21}},
            {Point3{0.0, 1.0, 0.0}, VertexId{22}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}}};
        front.source_face_ids = {SurfaceFaceId{30}};
        return front;
    }

    FrontEvaluation makeEvaluation()
    {
        FrontEvaluation evaluation;
        evaluation.layer = 2;
        evaluation.characteristic_length = 2.0;
        evaluation.effective_length_tolerance = 1e-12;
        return evaluation;
    }

    Result<SlidingConstraints, SlidingError> buildConstraints(
        const SurfaceMesh &mesh,
        const GrowthFront &front,
        const FrontEvaluation &evaluation)
    {
        auto surfaces = SlidingSurfaceBuilder{}.build(mesh);
        if (!surfaces.hasValue())
            return Result<SlidingConstraints, SlidingError>::failure(
                surfaces.error());
        std::vector<SlidingVertexInput> inputs;
        inputs.reserve(front.vertices.size());
        for (std::size_t index = 0; index < front.vertices.size(); ++index)
        {
            const auto &vertex = front.vertices[index];
            inputs.push_back({index, vertex.source_vertex_id,
                              vertex.boundary.sliding_region_ids});
        }
        return SlidingConstraintBuilder{}.build(
            surfaces.value(), inputs,
            evaluation.characteristic_length,
            evaluation.effective_length_tolerance,
            front.layer);
    }

    bool nearlyEqual(
        const Vector3 &first,
        const Vector3 &second,
        Scalar tolerance = 1e-12)
    {
        return (first - second).norm() <= tolerance;
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceMesh mesh = makeSymmetryMesh();
    const FrontEvaluation evaluation = makeEvaluation();

    // 单平面约束会去除法向分量；无约束顶点只做归一化。
    const GrowthFront single_front = makeFront({7});
    const auto single = buildConstraints(
        mesh, single_front, evaluation);
    if (!single.hasValue() ||
        single.value().planes().size() != 4 ||
        single.value().vertices().size() != single_front.vertices.size())
    {
        return 1;
    }
    const auto single_direction = single.value().apply(
        0, Vector3{1.0, 2.0, 3.0});
    if (!single_direction.hasValue() ||
        std::abs(single_direction.value().dot(Vector3{0.0, 1.0, 0.0})) > 1e-12 ||
        std::abs(single_direction.value().norm() - 1.0) > 1e-12)
    {
        return 2;
    }
    const auto unconstrained = single.value().apply(
        1, Vector3{3.0, 0.0, 4.0});
    if (!unconstrained.hasValue() ||
        !nearlyEqual(unconstrained.value(), Vector3{0.6, 0.0, 0.8}))
    {
        return 3;
    }

    // 两个独立平面的允许方向只能沿其交线，并尽量保持原方向符号。
    const GrowthFront line_front = makeFront({8, 7});
    const auto line = buildConstraints(
        mesh, line_front, evaluation);
    if (!line.hasValue())
    {
        return 4;
    }
    const Vector3 raw_line{1.0, 1.0, 1.0};
    const auto line_direction = line.value().apply(0, raw_line);
    if (!line_direction.hasValue() ||
        std::abs(line_direction.value().dot(Vector3{1.0, 0.0, 0.0})) > 1e-12 ||
        std::abs(line_direction.value().dot(Vector3{0.0, 1.0, 0.0})) > 1e-12 ||
        line_direction.value().dot(raw_line) < 0.0)
    {
        return 5;
    }

    // 平行或反向的 region 法向属于同一个独立约束。
    const GrowthFront redundant_front = makeFront({10, 7});
    const auto redundant = buildConstraints(
        mesh, redundant_front, evaluation);
    if (!redundant.hasValue() ||
        redundant.value().vertices()[0].plane_indices.size() != 1)
    {
        return 6;
    }

    // region 输入顺序不影响平面顺序和约束结果。
    const auto ordered = buildConstraints(
        mesh, makeFront({7, 8}), evaluation);
    const auto reversed = buildConstraints(
        mesh, makeFront({8, 7}), evaluation);
    if (!ordered.hasValue() || !reversed.hasValue())
    {
        return 7;
    }
    const auto ordered_direction = ordered.value().apply(0, raw_line);
    const auto reversed_direction = reversed.value().apply(0, raw_line);
    if (!ordered_direction.hasValue() || !reversed_direction.hasValue() ||
        !nearlyEqual(ordered_direction.value(), reversed_direction.value()))
    {
        return 8;
    }

    // 三个线性独立的平面会完全锁死顶点，构建阶段必须拒绝。
    const GrowthFront locked_front = makeFront({7, 8, 9});
    SurfaceMesh locked_mesh = mesh;
    locked_mesh.face_tags[1].kind =
        SurfaceBoundaryKind::Internal;
    const auto locked = buildConstraints(
        locked_mesh, locked_front, evaluation);
    const auto *locked_error = locked.hasValue()
        ? nullptr
        : std::get_if<OverConstrainedGrowthVertex>(&locked.error());
    if (locked_error == nullptr ||
        locked_error->front_vertex_index != 0 ||
        locked_error->source_vertex_id != VertexId{20} ||
        locked_error->layer != 2)
    {
        return 9;
    }

    // 前沿引用不存在的 region 时返回输入不匹配错误。
    const auto missing = buildConstraints(
        mesh, makeFront({99}), evaluation);
    const auto *missing_error = missing.hasValue()
        ? nullptr
        : std::get_if<SlidingInputMismatch>(&missing.error());
    if (missing_error == nullptr || missing_error->region_id != 99)
    {
        return 10;
    }

    // 非轴对齐或不共面的 region 由曲面投影处理。
    SurfaceMesh bent = mesh;
    bent.faces.push_back(
        Triangle{{VertexId{0}, VertexId{1}, VertexId{3}}});
    bent.face_tags.push_back(
        SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 7});
    const auto invalid = buildConstraints(
        bent, single_front, evaluation);
    if (!invalid.hasValue())
    {
        return 11;
    }

    // 投影后没有剩余切向分量时返回可诊断错误。
    const auto undefined = single.value().apply(
        0, Vector3{0.0, 1.0, 0.0});
    const auto *undefined_error = undefined.hasValue()
        ? nullptr
        : std::get_if<UndefinedConstrainedDirection>(&undefined.error());
    if (undefined_error == nullptr ||
        undefined_error->front_vertex_index != 0 ||
        undefined_error->source_vertex_id != VertexId{20} ||
        undefined_error->layer != 2)
    {
        return 12;
    }

    // Internal region 与 Symmetry region 使用相同的单平面滑移投影。
    SurfaceMesh internal_mesh = mesh;
    internal_mesh.face_tags[0].kind =
        SurfaceBoundaryKind::Internal;
    const auto internal_single = buildConstraints(
        internal_mesh, single_front, evaluation);
    if (!internal_single.hasValue())
    {
        return 13;
    }
    const auto internal_direction = internal_single.value().apply(
        0, Vector3{1.0, 2.0, 3.0});
    if (!internal_direction.hasValue() ||
        std::abs(internal_direction.value().dot(
            Vector3{0.0, 1.0, 0.0})) > 1e-12)
    {
        return 14;
    }

    // Internal 与 Symmetry 两个独立平面共同约束到交线。
    SurfaceMesh mixed_mesh = mesh;
    mixed_mesh.face_tags[1].kind =
        SurfaceBoundaryKind::Internal;
    const auto mixed_line = buildConstraints(
        mixed_mesh, line_front, evaluation);
    if (!mixed_line.hasValue())
    {
        return 15;
    }
    const auto mixed_direction =
        mixed_line.value().apply(0, raw_line);
    if (!mixed_direction.hasValue() ||
        std::abs(mixed_direction.value().dot(
            Vector3{1.0, 0.0, 0.0})) > 1e-12 ||
        std::abs(mixed_direction.value().dot(
            Vector3{0.0, 1.0, 0.0})) > 1e-12)
    {
        return 16;
    }

    // 非共面的 Internal region 同样作为曲面处理。
    SurfaceMesh bent_internal = internal_mesh;
    bent_internal.faces.push_back(
        Triangle{{VertexId{0}, VertexId{1}, VertexId{3}}});
    bent_internal.face_tags.push_back(
        {SurfaceBoundaryKind::Internal, 7});
    const auto invalid_internal = buildConstraints(
        bent_internal, single_front, evaluation);
    if (!invalid_internal.hasValue())
    {
        return 17;
    }

    const auto surfaces = SlidingSurfaceBuilder{}.build(mesh);
    if (!surfaces.hasValue()) return 18;
    const auto final_constraints = buildConstraints(
        mesh, single_front, evaluation);
    if (!final_constraints.hasValue()) return 19;
    const Point3 current{0.0, 0.0, 0.0};
    const auto constrained = final_constraints.value().constrainDirection(
        0, current, Vector3{1.0, 2.0, 3.0});
    if (!constrained.hasValue() ||
        std::abs(constrained.value().y()) > 1e-12)
        return 20;
    const auto projected = final_constraints.value().projectPosition(
        0, Point3{0.2, 0.4, 0.6});
    if (!projected.hasValue() ||
        std::abs(projected.value().position.y()) > 1e-12 ||
        projected.value().iterations != 1)
        return 21;

    SurfaceMesh curved_mesh;
    curved_mesh.vertices = {
        Point3{0,0,0}, Point3{1,0,1}, Point3{0,1,1}};
    curved_mesh.faces = {Triangle{{0,1,2}}};
    curved_mesh.face_tags = {{SurfaceBoundaryKind::Internal, 50}};
    const auto curved_surfaces = SlidingSurfaceBuilder{}.build(curved_mesh);
    GrowthFront curved_front = makeFront({50});
    curved_front.vertices[0].position = Point3{0.2,0.2,0.4};
    const auto curved_constraints = buildConstraints(
        curved_mesh, curved_front, evaluation);
    const auto curved_position = curved_constraints.value().projectPosition(
        0, Point3{0.2,0.2,1.4});
    if (!curved_position.hasValue() ||
        std::abs(curved_position.value().position.z() -
                 curved_position.value().position.x() -
                 curved_position.value().position.y()) > 1e-12)
        return 22;

    return 0;
}
