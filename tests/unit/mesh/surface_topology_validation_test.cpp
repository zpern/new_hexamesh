#include <variant>
#include <limits>

#include <boundary_mesh/mesh/surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    /// 创建测试使用的壁面标签。
    SurfaceBoundaryTag wallTag()
    {
        return SurfaceBoundaryTag{
            SurfaceBoundaryKind::Wall,
            1};
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceTopologyBuilder builder;

    // 空表面必须返回 EmptySurface。
    {
        const SurfaceMesh mesh;
        const auto result = builder.build(mesh);

        if (result.hasValue() ||
            !std::holds_alternative<EmptySurface>(
                result.error()))
        {
            return 1;
        }
    }

    // 面数量与标签数量不一致。
    {
        SurfaceMesh mesh;
        mesh.vertices.resize(
            3,
            Point3::Zero());

        mesh.faces.emplace_back(
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{2}}});

        const auto result = builder.build(mesh);

        if (result.hasValue())
        {
            return 2;
        }

        const auto *error =
            std::get_if<FaceTagCountMismatch>(
                &result.error());

        if (error == nullptr ||
            error->face_count != 1 ||
            error->face_tag_count != 0)
        {
            return 3;
        }
    }

    // 面片引用越界的顶点编号。
    {
        SurfaceMesh mesh;
        mesh.vertices.resize(
            3,
            Point3::Zero());

        mesh.faces.emplace_back(
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{3}}});

        mesh.face_tags.push_back(wallTag());

        const auto result = builder.build(mesh);

        const auto *error =
            result.hasValue()
                ? nullptr
                : std::get_if<InvalidVertexReference>(
                      &result.error());

        if (error == nullptr ||
            error->face_id != SurfaceFaceId{0} ||
            error->vertex_id != VertexId{3})
        {
            return 4;
        }
    }

    // 面片内部重复使用同一个顶点。
    {
        SurfaceMesh mesh;
        mesh.vertices.resize(
            3,
            Point3::Zero());

        mesh.faces.emplace_back(
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{1}}});

        mesh.face_tags.push_back(wallTag());

        const auto result = builder.build(mesh);

        const auto *error =
            result.hasValue()
                ? nullptr
                : std::get_if<DegenerateFace>(
                      &result.error());

        if (error == nullptr ||
            error->face_id != SurfaceFaceId{0})
        {
            return 5;
        }
    }

    // 顶点顺序不同但顶点集合相同，仍然属于重复面。
    {
        SurfaceMesh mesh;
        mesh.vertices.resize(
            3,
            Point3::Zero());

        mesh.faces.emplace_back(
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{2}}});

        mesh.faces.emplace_back(
            Triangle{{VertexId{2},
                      VertexId{1},
                      VertexId{0}}});

        mesh.face_tags = {
            wallTag(),
            wallTag()};

        const auto result = builder.build(mesh);

        const auto *error =
            result.hasValue()
                ? nullptr
                : std::get_if<DuplicateFace>(
                      &result.error());

        if (error == nullptr ||
            error->first_face_id != SurfaceFaceId{0} ||
            error->duplicate_face_id != SurfaceFaceId{1})
        {
            return 6;
        }
    }

    // 输入顶点包含 NaN 时，必须返回 NonFiniteVertex。
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0},
            Point3{
                std::numeric_limits<Scalar>::quiet_NaN(),
                0.0,
                0.0},
            Point3{0.0, 1.0, 0.0}};

        mesh.faces.emplace_back(
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{2}}});

        mesh.face_tags.push_back(wallTag());

        const auto result = builder.build(mesh);

        const auto *error =
            result.hasValue()
                ? nullptr
                : std::get_if<NonFiniteVertex>(
                      &result.error());

        if (error == nullptr ||
            error->vertex_id != VertexId{1})
        {
            return 7;
        }
    }

    // 即使顶点没有被任何面引用，非有限坐标也必须被拒绝。
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{
                0.0,
                -std::numeric_limits<Scalar>::infinity(),
                0.0}};

        mesh.faces.emplace_back(
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{2}}});

        mesh.face_tags.push_back(wallTag());

        const auto result = builder.build(mesh);

        const auto *error =
            result.hasValue()
                ? nullptr
                : std::get_if<NonFiniteVertex>(
                      &result.error());

        if (error == nullptr ||
            error->vertex_id != VertexId{3})
        {
            return 8;
        }
    }

    return 0;
}
