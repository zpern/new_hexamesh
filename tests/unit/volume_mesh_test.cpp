#include <variant>

#include <boundary_mesh/mesh/volume_mesh.hpp>

int main()
{
    using namespace boundary_mesh;

    VolumeMesh mesh;
    mesh.vertices.resize(8, Point3::Zero());

    mesh.cells.emplace_back(
        Tetra{{VertexId{0},
               VertexId{1},
               VertexId{2},
               VertexId{3}}});

    mesh.cells.emplace_back(
        Pyramid{{VertexId{0},
                 VertexId{1},
                 VertexId{2},
                 VertexId{3},
                 VertexId{4}}});

    mesh.cells.emplace_back(
        Prism{{VertexId{0},
               VertexId{1},
               VertexId{2},
               VertexId{3},
               VertexId{4},
               VertexId{5}}});

    mesh.cells.emplace_back(
        Hexa{{VertexId{0},
              VertexId{1},
              VertexId{2},
              VertexId{3},
              VertexId{4},
              VertexId{5},
              VertexId{6},
              VertexId{7}}});

    mesh.metadata = {
        CellMetadata{
            CellRole::Transition,
            SurfaceFaceId{0},
            0},
        CellMetadata{
            CellRole::Transition,
            SurfaceFaceId{1},
            0},
        CellMetadata{
            CellRole::RegularLayer,
            SurfaceFaceId{2},
            1},
        CellMetadata{
            CellRole::RegularLayer,
            SurfaceFaceId{3},
            1}};

    if (mesh.cells.size() != 4)
    {
        return 1;
    }

    if (!std::holds_alternative<Tetra>(mesh.cells[0]))
    {
        return 2;
    }

    if (cellType(mesh.cells[0]) != CellType::Tetra)
    {
        return 3;
    }

    if (cellType(mesh.cells[1]) != CellType::Pyramid)
    {
        return 4;
    }

    if (cellType(mesh.cells[2]) != CellType::Prism)
    {
        return 5;
    }

    if (cellType(mesh.cells[3]) != CellType::Hexa)
    {
        return 6;
    }

    if (mesh.metadata[2].role != CellRole::RegularLayer)
    {
        return 7;
    }

    if (mesh.metadata[2].layer != 1)
    {
        return 8;
    }

    if (
        mesh.metadata[2].source_face_id != SurfaceFaceId{2})
    {
        return 9;
    }

    const CellMetadata default_metadata{};

    if (default_metadata.role != CellRole::RegularLayer)
    {
        return 10;
    }

    return 0;
}