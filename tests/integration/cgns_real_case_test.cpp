#include <cstddef>
#include <filesystem>
#include <iostream>
#include <variant>

#include <cgnslib.h>

#include <boundary_mesh/io/cgns_surface_reader.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace
{
    std::size_t localVertexCount(
        const std::filesystem::path &path)
    {
        int file{};
        const auto path_utf8 = path.u8string();
        if (cg_open(path_utf8.c_str(), CG_MODE_READ, &file) != CG_OK)
        {
            return 0;
        }
        int zone_count{};
        if (cg_nzones(file, 1, &zone_count) != CG_OK)
        {
            cg_close(file);
            return 0;
        }
        std::size_t count{};
        for (int zone = 1; zone <= zone_count; ++zone)
        {
            char name[33]{};
            cgsize_t size[9]{};
            if (cg_zone_read(file, 1, zone, name, size) != CG_OK)
            {
                cg_close(file);
                return 0;
            }
            count += static_cast<std::size_t>(size[0]);
        }
        cg_close(file);
        return count;
    }
}

int main(int argc, char **argv)
{
    using namespace boundary_mesh;

    if (argc != 2)
    {
        std::cerr << "usage: boundary_mesh_cgns_real_case_test FILE.cgns\n";
        return 2;
    }

    const std::filesystem::path path(argv[1]);
    const auto result = readCgnsSurface(path);
    if (!result.hasValue())
    {
        std::cerr << "reader_error="
                  << static_cast<int>(result.error().code)
                  << " zone=" << result.error().zone_id
                  << " element=" << result.error().element_id
                  << " detail=" << result.error().detail << '\n';
        return 1;
    }

    const auto &mesh = result.value();
    std::size_t triangles{};
    std::size_t quads{};
    std::size_t farfield{};
    std::size_t wall{};
    for (const auto &face : mesh.faces)
    {
        triangles += std::holds_alternative<Triangle>(face);
        quads += std::holds_alternative<Quad>(face);
    }
    for (const auto &tag : mesh.face_tags)
    {
        farfield += tag.kind == SurfaceBoundaryKind::Farfield;
        wall += tag.kind == SurfaceBoundaryKind::Wall;
    }

    const auto local_vertices = localVertexCount(path);
    std::cout << "local_vertices=" << local_vertices << '\n'
              << "merged_vertices=" << mesh.vertices.size() << '\n'
              << "triangles=" << triangles << '\n'
              << "quads=" << quads << '\n'
              << "farfield=" << farfield << '\n'
              << "wall=" << wall << '\n';

    const bool matches =
        local_vertices == 52232 &&
        mesh.vertices.size() == 52010 &&
        triangles == 13186 &&
        quads == 45413 &&
        farfield == 422 &&
        wall == 58177;
    return matches ? 0 : 1;
}
