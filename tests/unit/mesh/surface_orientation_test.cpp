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

    SurfaceMesh inconsistent;
    inconsistent.vertices = {
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{0.0, 1.0, 0.0},
        Point3{1.0, 1.0, 0.0}};
    inconsistent.faces = {
        Triangle{{0, 1, 2}},
        Triangle{{1, 2, 3}}};
    inconsistent.face_tags = {
        {SurfaceBoundaryKind::Wall, 7},
        {SurfaceBoundaryKind::Farfield, 9}};
    const auto inconsistent_vertices = inconsistent.vertices;
    const auto inconsistent_tags = inconsistent.face_tags;

    const auto unified = unifySurfaceOrientation(inconsistent);
    if (!unified.hasValue() || unified.value() != 1)
        return 9;
    const auto *unchanged = std::get_if<Triangle>(&inconsistent.faces[0]);
    const auto *flipped = std::get_if<Triangle>(&inconsistent.faces[1]);
    if (unchanged == nullptr ||
        unchanged->vertex_ids != std::array<VertexId, 3>{0, 1, 2} ||
        flipped == nullptr ||
        flipped->vertex_ids != std::array<VertexId, 3>{1, 3, 2})
        return 10;
    if (inconsistent.vertices.size() != inconsistent_vertices.size() ||
        inconsistent.face_tags.size() != inconsistent_tags.size())
        return 11;
    for (std::size_t index = 0; index < inconsistent.vertices.size(); ++index)
        if (!inconsistent.vertices[index].isApprox(inconsistent_vertices[index]))
            return 12;
    for (std::size_t index = 0; index < inconsistent.face_tags.size(); ++index)
        if (inconsistent.face_tags[index].kind != inconsistent_tags[index].kind ||
            inconsistent.face_tags[index].region_id != inconsistent_tags[index].region_id)
            return 13;

    const auto repeated = unifySurfaceOrientation(inconsistent);
    if (!repeated.hasValue() || repeated.value() != 0)
        return 14;

    SurfaceMesh consistent;
    consistent.vertices = inconsistent.vertices;
    consistent.faces = {
        Triangle{{0, 1, 2}},
        Triangle{{2, 1, 3}}};
    consistent.face_tags.resize(2);
    const auto already_unified = unifySurfaceOrientation(consistent);
    if (!already_unified.hasValue() || already_unified.value() != 0)
        return 15;

    SurfaceMesh disconnected;
    disconnected.vertices.resize(9);
    disconnected.faces = {
        Triangle{{0, 1, 2}},
        Triangle{{1, 2, 3}},
        Quad{{4, 5, 6, 7}},
        Triangle{{5, 6, 8}}};
    disconnected.face_tags.resize(4);
    const auto disconnected_result = unifySurfaceOrientation(disconnected);
    if (!disconnected_result.hasValue() || disconnected_result.value() != 2)
        return 21;
    const auto *disconnected_triangle =
        std::get_if<Triangle>(&disconnected.faces[1]);
    const auto *mixed_triangle =
        std::get_if<Triangle>(&disconnected.faces[3]);
    if (disconnected_triangle == nullptr ||
        disconnected_triangle->vertex_ids !=
            std::array<VertexId, 3>{1, 3, 2} ||
        mixed_triangle == nullptr ||
        mixed_triangle->vertex_ids !=
            std::array<VertexId, 3>{5, 8, 6})
        return 22;

    SurfaceMesh degenerate;
    degenerate.vertices = inconsistent.vertices;
    degenerate.faces = {Triangle{{0, 0, 1}}};
    degenerate.face_tags = {{SurfaceBoundaryKind::Wall, 4}};
    const auto degenerate_result = unifySurfaceOrientation(degenerate);
    const auto *degenerate_error = degenerate_result.hasValue()
        ? nullptr
        : std::get_if<DegenerateFace>(&degenerate_result.error());
    const auto *degenerate_face = std::get_if<Triangle>(&degenerate.faces[0]);
    if (degenerate_error == nullptr || degenerate_error->face_id != 0 ||
        degenerate_face == nullptr ||
        degenerate_face->vertex_ids != std::array<VertexId, 3>{0, 0, 1} ||
        degenerate.face_tags[0].region_id != 4)
        return 16;

    SurfaceMesh non_manifold;
    non_manifold.vertices = {
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0},
        Point3{0.0, 1.0, 0.0}, Point3{0.0, -1.0, 0.0},
        Point3{0.0, 0.0, 1.0}};
    non_manifold.faces = {
        Triangle{{0, 1, 2}},
        Triangle{{1, 0, 3}},
        Triangle{{0, 1, 4}}};
    non_manifold.face_tags.resize(3);
    const auto non_manifold_result = unifySurfaceOrientation(non_manifold);
    const auto *non_manifold_error = non_manifold_result.hasValue()
        ? nullptr
        : std::get_if<NonManifoldEdge>(&non_manifold_result.error());
    if (non_manifold_error == nullptr ||
        non_manifold_error->edge_vertices !=
            std::array<VertexId, 2>{0, 1} ||
        non_manifold_error->face_ids !=
            std::array<SurfaceFaceId, 3>{0, 1, 2})
        return 17;
    for (std::size_t face = 0; face < non_manifold.faces.size(); ++face)
    {
        const auto *value = std::get_if<Triangle>(&non_manifold.faces[face]);
        const std::array<std::array<VertexId, 3>, 3> expected{{
            {{0, 1, 2}}, {{1, 0, 3}}, {{0, 1, 4}}}};
        if (value == nullptr || value->vertex_ids != expected[face])
            return 18;
    }

    SurfaceMesh mobius;
    mobius.vertices.resize(6);
    mobius.faces = {
        Quad{{0, 2, 3, 1}},
        Quad{{2, 4, 5, 3}},
        Quad{{4, 1, 0, 5}}};
    mobius.face_tags.resize(3);
    const auto mobius_result = unifySurfaceOrientation(mobius);
    const auto *orientation_error = mobius_result.hasValue()
        ? nullptr
        : std::get_if<NonOrientableSurface>(&mobius_result.error());
    if (orientation_error == nullptr ||
        orientation_error->edge_vertices !=
            std::array<VertexId, 2>{4, 5} ||
        orientation_error->first_face_id != 1 ||
        orientation_error->second_face_id != 2)
        return 19;
    const std::array<std::array<VertexId, 4>, 3> mobius_faces{{
        {{0, 2, 3, 1}}, {{2, 4, 5, 3}}, {{4, 1, 0, 5}}}};
    for (std::size_t face = 0; face < mobius.faces.size(); ++face)
    {
        const auto *value = std::get_if<Quad>(&mobius.faces[face]);
        if (value == nullptr || value->vertex_ids != mobius_faces[face])
            return 20;
    }

    return 0;
}
