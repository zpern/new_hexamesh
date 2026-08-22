#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceMesh makePrismMesh()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{0, 1, 0},
            Point3{0, 0, 1}, Point3{1, 0, 1}, Point3{0, 1, 1}};
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{3}, VertexId{4}, VertexId{5}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{4}, VertexId{3}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{2}, VertexId{0}, VertexId{3}, VertexId{5}}}};
        mesh.face_tags = {
            {SurfaceBoundaryKind::Wall, 10},
            {SurfaceBoundaryKind::Farfield, 20},
            {SurfaceBoundaryKind::Symmetry, 30},
            {SurfaceBoundaryKind::Symmetry, 31},
            {SurfaceBoundaryKind::Symmetry, 32}};
        return mesh;
    }
}

int main()
{
    using namespace boundary_mesh;
    const SurfaceMesh mesh = makePrismMesh();
    const auto topology = SurfaceTopologyBuilder{}.build(mesh);
    if (!topology.hasValue()) return 1;
    const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
    if (!patch.hasValue()) return 2;

    const auto front = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
    if (!front.hasValue()) return 3;

    const GrowthFront &value = front.value();
    if (value.layer != 0 || value.vertices.size() != 3 ||
        value.faces.size() != 1 || value.source_face_ids !=
            std::vector<SurfaceFaceId>{SurfaceFaceId{0}} ||
        value.vertices[0].source_vertex_id != VertexId{0} ||
        value.vertices[1].source_vertex_id != VertexId{1} ||
        value.vertices[2].source_vertex_id != VertexId{2})
    {
        return 4;
    }

    for (std::size_t index = 0; index < value.vertices.size(); ++index)
    {
        const GrowthFrontVertex &vertex = value.vertices[index];
        if ((vertex.position - mesh.vertices[index]).norm() > 1e-12 ||
            (vertex.root_position - vertex.position).norm() > 1e-12 ||
            vertex.source_vertex_id != static_cast<VertexId>(index) ||
            vertex.boundary.symmetry_region_ids !=
                patch.value().vertices()[index].symmetry_region_ids)
        {
            return 5;
        }
        if (vertex.direction.norm() != Scalar{0} ||
            vertex.actual_height != Scalar{0} ||
            vertex.visibility_cosine != Scalar{1} ||
            vertex.complex_corner)
        {
            return 10;
        }
    }

    const auto *triangle = std::get_if<Triangle>(&value.faces[0]);
    if (triangle == nullptr || triangle->vertex_ids !=
        std::array<VertexId, 3>{VertexId{0}, VertexId{2}, VertexId{1}})
    {
        return 6;
    }

    SurfaceMesh missing_vertex = mesh;
    missing_vertex.vertices.resize(2);
    const auto invalid_vertex = GrowthFrontBuilder{}.buildInitial(
        missing_vertex, patch.value());
    const auto *vertex_error = invalid_vertex.hasValue() ? nullptr :
        std::get_if<InvalidPatchVertex>(&invalid_vertex.error());
    if (vertex_error == nullptr || vertex_error->source_vertex_id != VertexId{2})
    {
        return 7;
    }

    SurfaceMesh missing_face = mesh;
    missing_face.faces.clear();
    const auto invalid_face = GrowthFrontBuilder{}.buildInitial(
        missing_face, patch.value());
    const auto *face_error = invalid_face.hasValue() ? nullptr :
        std::get_if<InvalidPatchFace>(&invalid_face.error());
    if (face_error == nullptr || face_error->source_face_id != SurfaceFaceId{0})
    {
        return 8;
    }

    SurfaceMesh unknown_vertex = mesh;
    unknown_vertex.faces[0] = Triangle{{VertexId{0}, VertexId{3}, VertexId{1}}};
    const auto unknown = GrowthFrontBuilder{}.buildInitial(
        unknown_vertex, patch.value());
    const auto *unknown_error = unknown.hasValue() ? nullptr :
        std::get_if<PatchFaceUsesUnknownVertex>(&unknown.error());
    if (unknown_error == nullptr ||
        unknown_error->source_face_id != SurfaceFaceId{0} ||
        unknown_error->source_vertex_id != VertexId{3})
    {
        return 9;
    }
    return 0;
}
