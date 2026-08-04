#include <cmath>
#include <cstddef>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/front_evaluator.hpp>
#include <boundary_mesh/growth/growth_direction.hpp>
#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/symmetry_constraint_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    /// 构造封闭三棱柱：底面生长、顶面远场、三个侧面分别为对称区域。
    SurfaceMesh makePipelineMesh()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0}, Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0}, Point3{0.0, 1.0, 1.0}};
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{3}, VertexId{4}, VertexId{5}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{4}, VertexId{3}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{2}, VertexId{0}, VertexId{3}, VertexId{5}}}};
        mesh.face_tags = {
            SurfaceBoundaryTag{SurfaceBoundaryKind::Wall, 10},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Farfield, 20},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 30},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 31},
            SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 32}};
        return mesh;
    }

    bool sameBoundaries(
        const std::vector<FrontVertexBoundary> &first,
        const std::vector<FrontVertexBoundary> &second)
    {
        if (first.size() != second.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            if (first[index].symmetry_region_ids !=
                second[index].symmetry_region_ids)
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

    const SurfaceMesh mesh = makePipelineMesh();
    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue())
    {
        return 1;
    }
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue())
    {
        return 2;
    }
    const auto layer0 = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    if (!layer0.hasValue())
    {
        return 3;
    }
    const auto evaluation0 = FrontEvaluator{}.evaluate(layer0.value());
    if (!evaluation0.hasValue())
    {
        return 4;
    }
    const auto directions0 = computeGrowthDirections(
        layer0.value(), evaluation0.value());
    if (!directions0.hasValue())
    {
        return 5;
    }
    const auto constraints0 = SymmetryConstraintBuilder{}.build(
        mesh, layer0.value(), evaluation0.value());
    if (!constraints0.hasValue())
    {
        return 6;
    }
    for (std::size_t vertex_index = 0;
         vertex_index < directions0.value().values.size();
         ++vertex_index)
    {
        if (!constraints0.value().apply(
                vertex_index,
                directions0.value().values[vertex_index]).hasValue())
        {
            return 7;
        }
    }

    const Scalar layer0_area = evaluation0.value().faces[0].value.area;
    const Vector3 layer0_normal =
        evaluation0.value().faces[0].value.unit_normal;
    const std::vector<VertexId> source_vertices =
        layer0.value().source_vertex_ids;
    const std::vector<SurfaceFaceId> source_faces =
        layer0.value().source_face_ids;
    const std::vector<FrontVertexBoundary> source_boundaries =
        layer0.value().vertex_boundaries;

    // 阶段 03 不生成新层；这里只人工移动坐标，验证算法没有缓存第 0 层几何。
    GrowthFront layer1 = layer0.value();
    layer1.layer = 1;
    layer1.vertices[1].z() += 0.20;
    layer1.vertices[2].z() += 0.10;

    const auto evaluation1 = FrontEvaluator{}.evaluate(layer1);
    if (!evaluation1.hasValue())
    {
        return 8;
    }
    const auto directions1 = computeGrowthDirections(
        layer1, evaluation1.value());
    if (!directions1.hasValue())
    {
        return 9;
    }
    const auto constraints1 = SymmetryConstraintBuilder{}.build(
        mesh, layer1, evaluation1.value());
    if (!constraints1.hasValue())
    {
        return 10;
    }

    if (evaluation1.value().layer != 1 ||
        std::abs(evaluation1.value().faces[0].value.area - layer0_area) <= 1e-12 ||
        (evaluation1.value().faces[0].value.unit_normal - layer0_normal).norm()
            <= 1e-12 ||
        (directions1.value().values[0] - directions0.value().values[0]).norm()
            <= 1e-12)
    {
        return 11;
    }
    if (layer1.source_vertex_ids != source_vertices ||
        layer1.source_face_ids != source_faces ||
        !sameBoundaries(layer1.vertex_boundaries, source_boundaries))
    {
        return 12;
    }
    for (std::size_t vertex_index = 0;
         vertex_index < directions1.value().values.size();
         ++vertex_index)
    {
        if (!constraints1.value().apply(
                vertex_index,
                directions1.value().values[vertex_index]).hasValue())
        {
            return 13;
        }
    }

    // 中间层面片折叠时必须报告当前层号、局部面和源 Wall 面。
    GrowthFront collapsed = layer1;
    const Triangle &first = std::get<Triangle>(collapsed.faces[0]);
    collapsed.vertices[first.vertex_ids[1]] =
        collapsed.vertices[first.vertex_ids[0]];
    const auto failure = FrontEvaluator{}.evaluate(collapsed);
    const auto *error = failure.hasValue()
        ? nullptr
        : std::get_if<DegenerateFrontFace>(&failure.error());
    if (error == nullptr ||
        error->layer != 1 ||
        error->front_face_index != 0 ||
        error->source_face_id != collapsed.source_face_ids[0])
    {
        return 14;
    }

    return 0;
}
