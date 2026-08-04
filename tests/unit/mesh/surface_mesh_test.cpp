#include <variant>

#include <boundary_mesh/mesh/surface_mesh.hpp>

int main()
{
    using namespace boundary_mesh;

    SurfaceMesh mesh;

    mesh.vertices = {
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0}};

    mesh.faces.emplace_back(
        Triangle{{VertexId{0},
                  VertexId{1},
                  VertexId{2}}});

    mesh.faces.emplace_back(
        Quad{{VertexId{0},
              VertexId{1},
              VertexId{2},
              VertexId{3}}});

    mesh.face_tags = {
        SurfaceBoundaryTag{
            SurfaceBoundaryKind::Wall,
            1},
        SurfaceBoundaryTag{
            SurfaceBoundaryKind::Symmetry,
            2}};

    if (mesh.vertices.size() != 4)
    {
        return 1;
    }

    if (!std::holds_alternative<Triangle>(mesh.faces[0]))
    {
        return 2;
    }

    if (!std::holds_alternative<Quad>(mesh.faces[1]))
    {
        return 3;
    }

    if (mesh.face_tags[0].kind != SurfaceBoundaryKind::Wall)
    {
        return 4;
    }

    if (mesh.face_tags[1].kind != SurfaceBoundaryKind::Symmetry)
    {
        return 5;
    }

    if (mesh.face_tags[1].region_id != 2)
    {
        return 6;
    }

    return 0;
}