#include <array>
#include <cstddef>
#include <variant>

#include <boundary_mesh/mesh/mesh_surface_orientation.hpp>

int main()
{
    using namespace boundary_mesh;

    SurfaceMesh mesh;
    mesh.vertices = {
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0}};
    mesh.faces = {
        Triangle{{0, 1, 2}},
        Quad{{0, 1, 2, 3}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Wall, 7},
        {SurfaceBoundaryKind::Farfield, 9}};

    const auto original_vertices = mesh.vertices;
    const auto original_tags = mesh.face_tags;

    reverseSurfaceOrientation(mesh);

    const auto *triangle = std::get_if<Triangle>(&mesh.faces[0]);
    const auto *quad = std::get_if<Quad>(&mesh.faces[1]);
    if (triangle == nullptr ||
        triangle->vertex_ids != std::array<VertexId, 3>{0, 2, 1})
    {
        return 1;
    }
    if (quad == nullptr ||
        quad->vertex_ids != std::array<VertexId, 4>{0, 3, 2, 1})
    {
        return 2;
    }
    if (mesh.vertices.size() != original_vertices.size())
    {
        return 3;
    }
    for (std::size_t index = 0; index < mesh.vertices.size(); ++index)
    {
        if (!mesh.vertices[index].isApprox(original_vertices[index]))
        {
            return 4;
        }
    }
    if (mesh.face_tags.size() != original_tags.size())
    {
        return 5;
    }
    for (std::size_t index = 0; index < mesh.face_tags.size(); ++index)
    {
        if (mesh.face_tags[index].kind != original_tags[index].kind ||
            mesh.face_tags[index].region_id != original_tags[index].region_id)
        {
            return 6;
        }
    }

    reverseSurfaceOrientation(mesh);
    triangle = std::get_if<Triangle>(&mesh.faces[0]);
    quad = std::get_if<Quad>(&mesh.faces[1]);
    if (triangle == nullptr ||
        triangle->vertex_ids != std::array<VertexId, 3>{0, 1, 2} ||
        quad == nullptr ||
        quad->vertex_ids != std::array<VertexId, 4>{0, 1, 2, 3})
    {
        return 7;
    }

    SurfaceMesh empty;
    reverseSurfaceOrientation(empty);
    if (!empty.vertices.empty() ||
        !empty.faces.empty() ||
        !empty.face_tags.empty())
    {
        return 8;
    }

    return 0;
}
