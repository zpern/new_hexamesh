#include <array>
#include <cstddef>
#include <variant>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    /// 创建一个封闭且方向一致的三棱柱混合表面。
    ///
    /// 两个端面为三角形，三个侧面为四边形。
    SurfaceMesh makePrismSurface()
    {
        SurfaceMesh mesh;

        mesh.vertices = {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0},
            Point3{0.0, 1.0, 1.0}};

        // 所有面从体单元外部观察时均采用一致绕序。
        mesh.faces = {
            Triangle{{VertexId{0},
                      VertexId{2},
                      VertexId{1}}},
            Triangle{{VertexId{3},
                      VertexId{4},
                      VertexId{5}}},
            Quad{{VertexId{0},
                  VertexId{1},
                  VertexId{4},
                  VertexId{3}}},
            Quad{{VertexId{1},
                  VertexId{2},
                  VertexId{5},
                  VertexId{4}}},
            Quad{{VertexId{2},
                  VertexId{0},
                  VertexId{3},
                  VertexId{5}}}};

        // 拓扑构建与边界类别无关，但完整表面允许混合标签。
        mesh.face_tags = {
            {SurfaceBoundaryKind::Wall, 1},
            {SurfaceBoundaryKind::Farfield, 2},
            {SurfaceBoundaryKind::Symmetry, 3},
            {SurfaceBoundaryKind::Wall, 1},
            {SurfaceBoundaryKind::Wall, 1}};

        return mesh;
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceMesh mesh = makePrismSurface();
    const auto result =
        SurfaceTopologyBuilder{}.build(mesh);

    if (!result.hasValue())
    {
        return 1;
    }

    const SurfaceTopology &topology =
        result.value();

    // EdgeId 必须按照“面编号 + 面内局部边”的扫描顺序稳定生成。
    const std::vector<Edge> expected_edges = {
        Edge{{VertexId{0}, VertexId{2}}},
        Edge{{VertexId{1}, VertexId{2}}},
        Edge{{VertexId{0}, VertexId{1}}},
        Edge{{VertexId{3}, VertexId{4}}},
        Edge{{VertexId{4}, VertexId{5}}},
        Edge{{VertexId{3}, VertexId{5}}},
        Edge{{VertexId{1}, VertexId{4}}},
        Edge{{VertexId{0}, VertexId{3}}},
        Edge{{VertexId{2}, VertexId{5}}}};

    if (topology.edges().size() !=
        expected_edges.size())
    {
        return 2;
    }

    for (std::size_t index = 0;
         index < expected_edges.size();
         ++index)
    {
        if (topology.edges()[index].vertex_ids !=
            expected_edges[index].vertex_ids)
        {
            return 3;
        }
    }

    const std::vector<EdgeFaceIds>
        expected_edge_faces = {
            {{{SurfaceFaceId{0}, SurfaceFaceId{4}}}, {}},
            {{{SurfaceFaceId{0}, SurfaceFaceId{3}}}, {}},
            {{{SurfaceFaceId{0}, SurfaceFaceId{2}}}, {}},
            {{{SurfaceFaceId{1}, SurfaceFaceId{2}}}, {}},
            {{{SurfaceFaceId{1}, SurfaceFaceId{3}}}, {}},
            {{{SurfaceFaceId{1}, SurfaceFaceId{4}}}, {}},
            {{{SurfaceFaceId{2}, SurfaceFaceId{3}}}, {}},
            {{{SurfaceFaceId{2}, SurfaceFaceId{4}}}, {}},
            {{{SurfaceFaceId{3}, SurfaceFaceId{4}}}, {}}};

    if (topology.edgeFaces() !=
        expected_edge_faces)
    {
        return 4;
    }

    const auto *face0_edges =
        std::get_if<TriangleEdgeIds>(
            &topology.faceEdges()[0]);

    const auto *face2_edges =
        std::get_if<QuadEdgeIds>(
            &topology.faceEdges()[2]);

    if (face0_edges == nullptr ||
        *face0_edges != TriangleEdgeIds{
                            EdgeId{0},
                            EdgeId{1},
                            EdgeId{2}})
    {
        return 5;
    }

    if (face2_edges == nullptr ||
        *face2_edges != QuadEdgeIds{
                            EdgeId{2},
                            EdgeId{6},
                            EdgeId{3},
                            EdgeId{7}})
    {
        return 6;
    }

    const auto *face0_neighbors =
        std::get_if<TriangleNeighborIds>(
            &topology.faceNeighbors()[0]);

    const auto *face2_neighbors =
        std::get_if<QuadNeighborIds>(
            &topology.faceNeighbors()[2]);

    if (face0_neighbors == nullptr ||
        *face0_neighbors != TriangleNeighborIds{
                                OptionalSurfaceFaceId{SurfaceFaceId{4}},
                                OptionalSurfaceFaceId{SurfaceFaceId{3}},
                                OptionalSurfaceFaceId{SurfaceFaceId{2}}})
    {
        return 7;
    }

    if (face2_neighbors == nullptr ||
        *face2_neighbors != QuadNeighborIds{
                                OptionalSurfaceFaceId{SurfaceFaceId{0}},
                                OptionalSurfaceFaceId{SurfaceFaceId{3}},
                                OptionalSurfaceFaceId{SurfaceFaceId{1}},
                                OptionalSurfaceFaceId{SurfaceFaceId{4}}})
    {
        return 8;
    }

    const std::vector<
        std::vector<SurfaceFaceId>>
        expected_vertex_faces = {
            {SurfaceFaceId{0},
             SurfaceFaceId{2},
             SurfaceFaceId{4}},
            {SurfaceFaceId{0},
             SurfaceFaceId{2},
             SurfaceFaceId{3}},
            {SurfaceFaceId{0},
             SurfaceFaceId{3},
             SurfaceFaceId{4}},
            {SurfaceFaceId{1},
             SurfaceFaceId{2},
             SurfaceFaceId{4}},
            {SurfaceFaceId{1},
             SurfaceFaceId{2},
             SurfaceFaceId{3}},
            {SurfaceFaceId{1},
             SurfaceFaceId{3},
             SurfaceFaceId{4}}};

    if (topology.vertexFaces() !=
        expected_vertex_faces)
    {
        return 9;
    }

    // 构建器不能修改输入表面。
    if (mesh.faces.size() != 5 ||
        mesh.face_tags.size() != 5)
    {
        return 10;
    }

    // 仅包含 Internal 面的网格允许开放边，且 point-to-face 仍保留该面。
    {
        SurfaceMesh internal_mesh;
        internal_mesh.vertices = {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0}};
        internal_mesh.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}}};
        internal_mesh.face_tags = {
            {SurfaceBoundaryKind::Internal, 7}};

        const auto internal_result =
            SurfaceTopologyBuilder{}.build(internal_mesh);
        if (!internal_result.hasValue())
        {
            return 11;
        }

        const auto &internal_topology = internal_result.value();
        for (const EdgeFaceIds &edge_faces :
             internal_topology.edgeFaces())
        {
            if (edge_faces.non_internal_faces[0].has_value() ||
                edge_faces.non_internal_faces[1].has_value() ||
                edge_faces.internal_faces[0] != SurfaceFaceId{0} ||
                edge_faces.internal_faces[1].has_value())
            {
                return 12;
            }
        }

        const auto *neighbors =
            std::get_if<TriangleNeighborIds>(
                &internal_topology.faceNeighbors()[0]);
        if (neighbors == nullptr ||
            (*neighbors)[0].has_value() ||
            (*neighbors)[1].has_value() ||
            (*neighbors)[2].has_value())
        {
            return 13;
        }

        if (internal_topology.vertexFaces()[0] !=
            std::vector<SurfaceFaceId>{SurfaceFaceId{0}})
        {
            return 14;
        }
    }

    // 同一几何边允许两个 Non-Internal 面和两个 Internal 面。
    {
        SurfaceMesh layered_mesh = makePrismSurface();
        layered_mesh.vertices.push_back(Point3{-1.0, 0.0, 0.0});
        layered_mesh.vertices.push_back(Point3{-1.0, 1.0, 0.0});
        layered_mesh.faces.push_back(
            Triangle{{VertexId{0}, VertexId{2}, VertexId{6}}});
        layered_mesh.face_tags.push_back(
            {SurfaceBoundaryKind::Internal, 8});

        const auto attached_result =
            SurfaceTopologyBuilder{}.build(layered_mesh);
        if (!attached_result.hasValue())
        {
            return 15;
        }

        const auto &attached_topology = attached_result.value();
        const EdgeFaceIds &attached_shared =
            attached_topology.edgeFaces()[0];
        const auto *attached_internal_neighbors =
            std::get_if<TriangleNeighborIds>(
                &attached_topology.faceNeighbors()[5]);
        if (attached_shared.non_internal_faces !=
                std::array<OptionalSurfaceFaceId, 2>{
                    SurfaceFaceId{0}, SurfaceFaceId{4}} ||
            attached_shared.internal_faces !=
                std::array<OptionalSurfaceFaceId, 2>{
                    SurfaceFaceId{5}, std::nullopt} ||
            attached_internal_neighbors == nullptr ||
            (*attached_internal_neighbors)[0].has_value())
        {
            return 16;
        }

        layered_mesh.faces.push_back(
            Triangle{{VertexId{2}, VertexId{0}, VertexId{7}}});
        layered_mesh.face_tags.push_back(
            {SurfaceBoundaryKind::Internal, 8});

        const auto layered_result =
            SurfaceTopologyBuilder{}.build(layered_mesh);
        if (!layered_result.hasValue())
        {
            return 17;
        }

        const auto &layered_topology = layered_result.value();
        const EdgeFaceIds &shared = layered_topology.edgeFaces()[0];
        if (shared.non_internal_faces !=
                std::array<OptionalSurfaceFaceId, 2>{
                    SurfaceFaceId{0}, SurfaceFaceId{4}} ||
            shared.internal_faces !=
                std::array<OptionalSurfaceFaceId, 2>{
                    SurfaceFaceId{5}, SurfaceFaceId{6}})
        {
            return 18;
        }

        const auto *wall_neighbors =
            std::get_if<TriangleNeighborIds>(
                &layered_topology.faceNeighbors()[0]);
        const auto *internal_neighbors =
            std::get_if<TriangleNeighborIds>(
                &layered_topology.faceNeighbors()[5]);
        if (wall_neighbors == nullptr ||
            internal_neighbors == nullptr ||
            (*wall_neighbors)[0] != SurfaceFaceId{4} ||
            (*internal_neighbors)[0] != SurfaceFaceId{6})
        {
            return 19;
        }

        if (layered_topology.vertexFaces()[0] !=
            std::vector<SurfaceFaceId>{
                SurfaceFaceId{0}, SurfaceFaceId{2},
                SurfaceFaceId{4}, SurfaceFaceId{5},
                SurfaceFaceId{6}})
        {
            return 20;
        }
    }

    return 0;
}
