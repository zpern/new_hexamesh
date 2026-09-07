#include "boundary_meshing_fixture.hpp"

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace boundary_mesh::test
{
    namespace
    {
        void requireKey(const std::string &actual, const char *expected)
        {
            if (actual != expected)
                throw std::runtime_error(
                    "Expected " + std::string(expected) + ", got " + actual);
        }

        SurfaceBoundaryKind boundaryKind(int value)
        {
            switch (value)
            {
            case 0: return SurfaceBoundaryKind::Farfield;
            case 1: return SurfaceBoundaryKind::Wall;
            case 2: return SurfaceBoundaryKind::Symmetry;
            case 3: return SurfaceBoundaryKind::Internal;
            default:
                throw std::runtime_error(
                    "Unsupported boundary type: " + std::to_string(value));
            }
        }
    }

    BoundaryMeshingFixture readBoundaryMeshingFixture(
        const std::filesystem::path &path)
    {
        std::ifstream file(path);
        if (!file)
            throw std::runtime_error("Cannot open input file: " + path.string());

        BoundaryMeshingFixture fixture;
        std::string key;

        file >> key >> fixture.layer_count;
        requireKey(key, "nLN:");
        file >> key >> fixture.first_height;
        requireKey(key, "dLen:");
        file >> key >> fixture.growth_ratio;
        requireKey(key, "dRto:");
        file >> key >> fixture.maximum_prism_skewness;
        requireKey(key, "max_prism_skewness:");
        file >> key >> fixture.maximum_pyramid_skewness;
        requireKey(key, "max_pyramid_skewness:");
        file >> key >> fixture.maximum_ratio_difference;
        requireKey(key, "max_ratio_diff:");
        file >> key >> fixture.isotropic_stop;
        requireKey(key, "bisostop:");

        int multiple_normals{};
        file >> key >> multiple_normals;
        requireKey(key, "b_use_multiple_normals:");
        fixture.use_multiple_normals = multiple_normals != 0;

        file >> key;
        requireKey(key, "boundary_info:");

        std::map<std::uint32_t, SurfaceBoundaryKind> boundary_kinds;
        while (file >> key && key != "variable_para:")
        {
            const auto face_id = static_cast<std::uint32_t>(std::stoul(key));
            int face_type{};
            file >> face_type;
            boundary_kinds[face_id] = boundaryKind(face_type);
        }
        requireKey(key, "variable_para:");

        std::size_t parameter_count{};
        file >> parameter_count;
        for (std::size_t i = 0; i < parameter_count; ++i)
        {
            std::uint32_t face_id{};
            FaceGrowthParameters parameters;
            file >> face_id >> parameters.layer_count >> parameters.first_height >>
                parameters.growth_ratio;
            fixture.face_parameters[face_id] = parameters;
        }

        file >> key;
        requireKey(key, "PointsAndCells:");

        std::size_t triangle_count{};
        std::size_t point_count{};
        file >> triangle_count >> point_count;
        if (!file || triangle_count == 0 || point_count == 0)
            throw std::runtime_error("Invalid PointsAndCells size");

        fixture.mesh.vertices.resize(point_count);
        std::vector<bool> point_seen(point_count, false);
        for (std::size_t i = 0; i < point_count; ++i)
        {
            std::size_t point_id{};
            Scalar x{}, y{}, z{};
            file >> point_id >> x >> y >> z;
            if (!file || point_id == 0 || point_id > point_count ||
                point_seen[point_id - 1])
                throw std::runtime_error("Invalid point record");
            fixture.mesh.vertices[point_id - 1] = Point3{x, y, z};
            point_seen[point_id - 1] = true;
        }

        fixture.mesh.faces.resize(triangle_count);
        fixture.mesh.face_tags.resize(triangle_count);
        std::vector<bool> triangle_seen(triangle_count, false);
        for (std::size_t i = 0; i < triangle_count; ++i)
        {
            std::size_t triangle_id{};
            std::size_t v0{}, v1{}, v2{};
            std::uint32_t face_id{};
            file >> triangle_id >> v0 >> v1 >> v2 >> face_id;
            if (!file || triangle_id == 0 || triangle_id > triangle_count ||
                v0 >= point_count || v1 >= point_count || v2 >= point_count ||
                triangle_seen[triangle_id - 1])
                throw std::runtime_error(
                    "Invalid triangle record at input row " + std::to_string(i) +
                    ": id=" + std::to_string(triangle_id) +
                    ", vertices=" + std::to_string(v0) + "," +
                    std::to_string(v1) + "," + std::to_string(v2));
            const auto kind = boundary_kinds.find(face_id);
            if (kind == boundary_kinds.end())
                throw std::runtime_error("Missing boundary type for face region");

            fixture.mesh.faces[triangle_id - 1] = Triangle{{
                static_cast<VertexId>(v0),
                static_cast<VertexId>(v1),
                static_cast<VertexId>(v2)}};
            fixture.mesh.face_tags[triangle_id - 1] = {kind->second, face_id};
            triangle_seen[triangle_id - 1] = true;
        }

        return fixture;
    }
}
