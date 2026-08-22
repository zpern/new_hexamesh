#include <cstddef>
#include <vector>

#include <boundary_mesh/growth/front_adjacency.hpp>

using namespace boundary_mesh;

namespace
{
    GrowthFront makeMixedFront()
    {
        GrowthFront front;
        front.layer = 3;
        front.vertices = {
            GrowthFrontVertex{Point3{0, 0, 0}, VertexId{10}},
            GrowthFrontVertex{Point3{1, 0, 0}, VertexId{11}},
            GrowthFrontVertex{Point3{0, 1, 0}, VertexId{12}},
            GrowthFrontVertex{Point3{-1, 0, 0}, VertexId{13}},
            GrowthFrontVertex{Point3{0, -1, 0}, VertexId{14}}};
        front.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Quad{{VertexId{0}, VertexId{2}, VertexId{4}, VertexId{3}}}};
        front.source_face_ids = {SurfaceFaceId{20}, SurfaceFaceId{21}};
        return front;
    }
}

int main()
{
    const GrowthFront mixed = makeMixedFront();
    const auto mixed_result = buildFrontAdjacency(mixed);
    if (!mixed_result.hasValue())
    {
        return 1;
    }

    const FrontAdjacency &mixed_adjacency = mixed_result.value();
    if (mixed_adjacency.vertex_neighbors[0] !=
            std::vector<std::size_t>{1, 2, 3} ||
        mixed_adjacency.vertex_incident_faces[0] !=
            std::vector<std::size_t>{0, 1})
    {
        return 2;
    }

    GrowthFront triangle_only = mixed;
    triangle_only.faces.resize(1);
    triangle_only.source_face_ids.resize(1);

    const auto triangle_result =
        buildFrontAdjacency(triangle_only);
    if (!triangle_result.hasValue())
    {
        return 3;
    }
    if (triangle_result.value().vertex_neighbors[0] !=
            std::vector<std::size_t>{1, 2} ||
        triangle_result.value().vertex_incident_faces[0] !=
            std::vector<std::size_t>{0})
    {
        return 4;
    }

    GrowthFront invalid = mixed;
    invalid.faces[0] = Triangle{
        {VertexId{0}, VertexId{1}, VertexId{99}}};

    const auto invalid_result = buildFrontAdjacency(invalid);
    if (invalid_result.hasValue())
    {
        return 5;
    }

    const auto *error = std::get_if<
        InvalidFrontAdjacencyReference>(
        &invalid_result.error());
    if (error == nullptr ||
        error->layer != 3 ||
        error->front_face_index != 0 ||
        error->source_face_id != SurfaceFaceId{20} ||
        error->vertex_id != VertexId{99})
    {
        return 6;
    }

    return 0;
}
