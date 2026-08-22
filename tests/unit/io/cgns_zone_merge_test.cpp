#include <array>
#include <cassert>
#include <filesystem>
#include <fstream>
#include <set>
#include <vector>
#include <variant>

#include <boundary_mesh/io/cgns_surface_reader.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

#include <helpers/cgns_fixture.hpp>

namespace
{
    std::set<boundary_mesh::VertexId> vertexIds(
        const boundary_mesh::SurfaceFace &face)
    {
        return std::visit(
            [](const auto &value)
            {
                return std::set<boundary_mesh::VertexId>{
                    value.vertex_ids.begin(),
                    value.vertex_ids.end()};
            },
            face);
    }

    std::vector<boundary_mesh::VertexId> orderedVertexIds(
        const boundary_mesh::SurfaceFace &face)
    {
        return std::visit(
            [](const auto &value)
            {
                return std::vector<boundary_mesh::VertexId>{
                    value.vertex_ids.begin(),
                    value.vertex_ids.end()};
            },
            face);
    }
}

int main()
{
    using namespace boundary_mesh;

    const auto directory =
        std::filesystem::temp_directory_path() /
        "boundary_mesh_cgns_zone_merge_test";
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    const auto path = directory / "connected.cgns";
    boundary_mesh::test::writeTwoZoneConnectedSurface(path);
    std::ofstream(directory / "connected.bc.txt")
        << "Wall:\n1\n2\n";

    const auto result = readCgnsSurface(path);
    assert(result.hasValue());
    const auto &mesh = result.value();
    assert(mesh.vertices.size() == 6);
    assert(mesh.faces.size() == 2);

    const auto first = vertexIds(mesh.faces[0]);
    const auto second = vertexIds(mesh.faces[1]);
    std::size_t shared{};
    for (const auto vertex_id : first)
    {
        shared += second.count(vertex_id);
    }
    assert(shared == 2);

    const auto reversed_path = directory / "reversed.cgns";
    boundary_mesh::test::writeTwoZoneConnectedSurface(
        reversed_path,
        true);
    std::ofstream(directory / "reversed.bc.txt")
        << "Wall:\n1\n2\n";
    const auto reversed = readCgnsSurface(reversed_path);
    assert(reversed.hasValue());
    assert(reversed.value().vertices.size() == mesh.vertices.size());
    assert(reversed.value().faces.size() == mesh.faces.size());
    for (std::size_t vertex = 0;
         vertex < mesh.vertices.size();
         ++vertex)
    {
        assert(reversed.value().vertices[vertex].x() ==
               mesh.vertices[vertex].x());
        assert(reversed.value().vertices[vertex].y() ==
               mesh.vertices[vertex].y());
        assert(reversed.value().vertices[vertex].z() ==
               mesh.vertices[vertex].z());
    }
    for (std::size_t face = 0; face < mesh.faces.size(); ++face)
    {
        assert(orderedVertexIds(reversed.value().faces[face]) ==
               orderedVertexIds(mesh.faces[face]));
    }

    const auto mismatch_path = directory / "mismatch.cgns";
    boundary_mesh::test::writeTwoZoneConnectedSurface(
        mismatch_path,
        false,
        true);
    std::ofstream(directory / "mismatch.bc.txt")
        << "Wall:\n1\n2\n";
    const auto mismatch = readCgnsSurface(mismatch_path);
    assert(!mismatch.hasValue());
    assert(mismatch.error().code ==
           CgnsSurfaceErrorCode::ConnectivityCoordinateMismatch);

    std::filesystem::remove_all(directory);
    return 0;
}
