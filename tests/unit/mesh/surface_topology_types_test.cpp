#include <array>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface_topology.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_error.hpp>

int main()
{
    using namespace boundary_mesh;

    static_assert(
        !std::is_default_constructible_v<SurfaceTopology>);

    static_assert(std::is_same_v<
                  decltype(std::declval<const SurfaceTopology &>().edges()),
                  const std::vector<Edge> &>);

    static_assert(std::is_same_v<
                  decltype(std::declval<const SurfaceTopology &>().edgeFaces()),
                  const std::vector<EdgeFaceIds> &>);

    static_assert(std::is_same_v<
                  decltype(std::declval<const SurfaceTopology &>().faceEdges()),
                  const std::vector<FaceEdgeIds> &>);

    static_assert(std::is_same_v<
                  decltype(std::declval<const SurfaceTopology &>()
                               .faceNeighbors()),
                  const std::vector<FaceNeighborIds> &>);

    static_assert(std::is_same_v<
                  decltype(std::declval<const SurfaceTopology &>()
                               .vertexFaces()),
                  const std::vector<
                      std::vector<SurfaceFaceId>> &>);

    static_assert(std::is_same_v<
                  OptionalSurfaceFaceId,
                  std::optional<SurfaceFaceId>>);

    static_assert(std::is_same_v<
                  TriangleNeighborIds,
                  std::array<OptionalSurfaceFaceId, 3>>);

    static_assert(std::is_same_v<
                  QuadNeighborIds,
                  std::array<OptionalSurfaceFaceId, 4>>);

    const EdgeFaceIds incidence{
        {SurfaceFaceId{1}, SurfaceFaceId{2}},
        {SurfaceFaceId{3}, std::nullopt}};

    if (incidence.non_internal_faces[1] !=
            SurfaceFaceId{2} ||
        incidence.internal_faces[0] !=
            SurfaceFaceId{3} ||
        incidence.internal_faces[1].has_value())
    {
        return 4;
    }

    const Edge edge{{VertexId{2},
                     VertexId{5}}};

    if (edge.vertex_ids !=
        std::array<VertexId, 2>{
            VertexId{2},
            VertexId{5}})
    {
        return 1;
    }

    const FaceEdgeIds face_edges =
        TriangleEdgeIds{
            EdgeId{0},
            EdgeId{1},
            EdgeId{2}};

    if (!std::holds_alternative<TriangleEdgeIds>(
            face_edges))
    {
        return 2;
    }

    const SurfaceTopologyError error =
        BoundaryEdge{
            {VertexId{1}, VertexId{4}},
            SurfaceFaceId{7}};

    if (!std::holds_alternative<BoundaryEdge>(error))
    {
        return 3;
    }

    return 0;
}
