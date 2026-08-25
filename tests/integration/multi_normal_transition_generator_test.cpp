#include <filesystem>
#include <fstream>
#include <string>
#include <variant>

#include <boundary_mesh/growth/multi_normal_transition_generator.hpp>

namespace
{
    boundary_mesh::GrowthFront cubeCorner()
    {
        using namespace boundary_mesh;
        GrowthFront front;
        front.vertices = {
            {Point3{0, 0, 0}, 70}, {Point3{1, 0, 0}, 71},
            {Point3{0, 1, 0}, 72}, {Point3{0, 0, 1}, 73},
            {Point3{1, 1, 0}, 74}, {Point3{1, 0, 1}, 75},
            {Point3{0, 1, 1}, 76}};
        front.faces = {
            Quad{{0, 2, 4, 1}},
            Quad{{0, 1, 5, 3}},
            Quad{{0, 3, 6, 2}}};
        front.source_face_ids = {80, 81, 82};
        return front;
    }

    bool containsVtkHeader(const std::filesystem::path &path)
    {
        std::ifstream input(path);
        const std::string content{
            std::istreambuf_iterator<char>(input),
            std::istreambuf_iterator<char>()};
        return content.find("DATASET UNSTRUCTURED_GRID") !=
            std::string::npos;
    }
}

int main()
{
    using namespace boundary_mesh;
    const auto output_directory =
        std::filesystem::temp_directory_path() /
        "boundary_mesh_multi_normal_debug_test";
    std::error_code error;
    std::filesystem::remove_all(output_directory, error);

    MultiNormalOptions options;
    options.enabled = true;
    options.transition_height = Scalar{0.05};
    options.split_skewness_threshold = Scalar{0.5};
    const auto default_result = generateMultiNormalTransition(
        cubeCorner(), options);
    if (!default_result.hasValue() ||
        std::filesystem::exists(output_directory))
    {
        return 1;
    }

    options.debug_output.enabled = true;
    options.debug_output.directory = output_directory;
    const auto debug_result = generateMultiNormalTransition(
        cubeCorner(), options);
    const auto volume_path =
        output_directory / "multi_normal_transition.vtk";
    const auto surface_path =
        output_directory / "multi_normal_front.vtk";
    if (!debug_result.hasValue() ||
        !containsVtkHeader(volume_path) ||
        !containsVtkHeader(surface_path))
    {
        return 2;
    }

    std::filesystem::remove_all(output_directory, error);
    return 0;
}
