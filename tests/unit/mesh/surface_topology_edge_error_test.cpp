#include <array>
#include <variant>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    /// 创建测试用的壁面标签。
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

    // 三个面共享边 (0,1)，应当返回非流形边错误。
    {
        SurfaceMesh mesh;

        mesh.vertices.resize(
            5,
            Point3::Zero());

        mesh.faces = {
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{2}}},
            Triangle{{VertexId{1},
                      VertexId{0},
                      VertexId{3}}},
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{4}}}};

        mesh.face_tags = {
            wallTag(),
            wallTag(),
            wallTag()};

        const auto result =
            builder.build(mesh);

        const auto *error =
            result.hasValue()
                ? nullptr
                : std::get_if<NonManifoldEdge>(
                      &result.error());

        if (error == nullptr)
        {
            return 1;
        }

        if (error->edge_vertices !=
                std::array<VertexId, 2>{
                    VertexId{0},
                    VertexId{1}} ||
            error->face_ids !=
                std::array<SurfaceFaceId, 3>{
                    SurfaceFaceId{0},
                    SurfaceFaceId{1},
                    SurfaceFaceId{2}})
        {
            return 2;
        }
    }

    // 两个面沿共享边 (0,1) 使用相同方向，
    // 说明两个面的绕序不一致。
    {
        SurfaceMesh mesh;

        mesh.vertices.resize(
            4,
            Point3::Zero());

        mesh.faces = {
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{2}}},
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{3}}}};

        mesh.face_tags = {
            wallTag(),
            wallTag()};

        const auto result =
            builder.build(mesh);

        const auto *error =
            result.hasValue()
                ? nullptr
                : std::get_if<
                      InconsistentOrientation>(
                      &result.error());

        if (error == nullptr)
        {
            return 3;
        }

        if (error->edge_vertices !=
                std::array<VertexId, 2>{
                    VertexId{0},
                    VertexId{1}} ||
            error->first_face_id !=
                SurfaceFaceId{0} ||
            error->second_face_id !=
                SurfaceFaceId{1})
        {
            return 4;
        }
    }
    // 单个三角形的三条边都只关联一个面，
    // 因此完整输入表面没有封闭。
    {
        SurfaceMesh mesh;

        mesh.vertices.resize(
            3,
            Point3::Zero());

        mesh.faces = {
            Triangle{{VertexId{0},
                      VertexId{1},
                      VertexId{2}}}};

        mesh.face_tags = {
            wallTag()};

        const auto result =
            builder.build(mesh);

        const auto *error =
            result.hasValue()
                ? nullptr
                : std::get_if<BoundaryEdge>(
                      &result.error());

        if (error == nullptr)
        {
            return 5;
        }

        // EdgeId 按面和局部边的扫描顺序产生，
        // 因此第一条开放边确定为 (0,1)。
        if (error->edge_vertices !=
                std::array<VertexId, 2>{
                    VertexId{0},
                    VertexId{1}} ||
            error->face_id !=
                SurfaceFaceId{0})
        {
            return 6;
        }
    }
    return 0;
}