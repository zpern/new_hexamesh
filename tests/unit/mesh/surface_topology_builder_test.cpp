#include <array>
#include <cstddef>
#include <variant>
#include <vector>

#include <boundary_mesh/mesh/surface_topology_builder.hpp>

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
            {SurfaceFaceId{0}, SurfaceFaceId{4}},
            {SurfaceFaceId{0}, SurfaceFaceId{3}},
            {SurfaceFaceId{0}, SurfaceFaceId{2}},
            {SurfaceFaceId{1}, SurfaceFaceId{2}},
            {SurfaceFaceId{1}, SurfaceFaceId{3}},
            {SurfaceFaceId{1}, SurfaceFaceId{4}},
            {SurfaceFaceId{2}, SurfaceFaceId{3}},
            {SurfaceFaceId{2}, SurfaceFaceId{4}},
            {SurfaceFaceId{3}, SurfaceFaceId{4}}};

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
                                SurfaceFaceId{4},
                                SurfaceFaceId{3},
                                SurfaceFaceId{2}})
    {
        return 7;
    }

    if (face2_neighbors == nullptr ||
        *face2_neighbors != QuadNeighborIds{
                                SurfaceFaceId{0},
                                SurfaceFaceId{3},
                                SurfaceFaceId{1},
                                SurfaceFaceId{4}})
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

    return 0;
}